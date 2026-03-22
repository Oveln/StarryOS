//! IPI (Inter-Processor Interrupt) character device
//!
//! - Write to device: Send IPI to hart 0 via SBI
//! - IRQ handler: Add message when IPI is received
//! - Read from device: Get IPI event messages

use alloc::{collections::VecDeque, format, string::String};
use core::any::Any;
use core::sync::atomic::{AtomicBool, Ordering, compiler_fence};

use axerrno::AxError;
use axfs_ng_vfs::{NodeFlags, VfsResult};
use axsync::Mutex;

use crate::pseudofs::DeviceOps;

/// IPI IRQ number for RISC-V (software interrupt)
const IPI_IRQ: usize = 0x8000_0000_0000_0001;

/// Hart 同步标志的物理地址
/// 这个地址用于两个 hart 之间同步启动状态
pub const HART_SYNC_PHYS_ADDR: usize = 0xc8000000;

/// Hart 同步标志结构体
/// 放置在固定物理地址，确保两个 hart 都能访问
#[repr(C)]
pub struct HartSyncFlags {
    /// 魔数，用于验证结构体有效性
    pub magic_number: u16,
    /// Hart1 上的 OS 是否已经启动
    /// true = 已启动, false = 未启动
    pub hart1_os_ready: AtomicBool,
    /// Hart0 是否发送了 IPI（Inter-Processor Interrupt）
    /// true = 已发送, false = 未发送
    pub hart0_ipi_sent: AtomicBool,
}

/// 获取 Hart 同步标志的引用
/// 通过物理地址映射访问
pub fn get_hart_sync() -> &'static HartSyncFlags {
    let vaddr = axhal::mem::phys_to_virt(memory_addr::PhysAddr::from(HART_SYNC_PHYS_ADDR));
    unsafe { &*(vaddr.as_ptr() as *const HartSyncFlags) }
}

/// 标记 Hart1 OS 已准备好
pub fn mark_hart1_os_ready() {
    let sync = get_hart_sync();

    // 验证魔数
    if sync.magic_number != 0x0721 {
        warn!("ipi_device: invalid magic number {:#x}, expected 0x0721", sync.magic_number);
    }

    info!("marking hart1_os_ready = true");

    sync.hart1_os_ready.store(true, Ordering::Release);
    compiler_fence(Ordering::Release);
    info!("ipi_device: marked hart1_os_ready = true");
}

/// Maximum number of interrupt messages to store
const MAX_MESSAGES: usize = 32;

/// Global IPI device state
static IPI_MESSAGES: Mutex<VecDeque<String>> = Mutex::new(VecDeque::new());

/// Add a message to the IPI message buffer (called by IRQ handler)
fn add_ipi_message(msg: String) {
    info!("ipi_device: {}", msg);
    let mut messages = IPI_MESSAGES.lock();
    if messages.len() >= MAX_MESSAGES {
        messages.pop_front();
    }
    messages.push_back(msg);
}

/// IRQ handler - called when IPI interrupt is received
#[cfg(target_arch = "riscv64")]
fn ipi_irq_handler() {
    unsafe {
        core::arch::asm!("csrc sip, {}", const 2_usize);
    }
    add_ipi_message(format!("Soft interrupt (IPI) received on current hart"));
}

/// Send an IPI to hart 0 via SBI call (does NOT add message)
#[cfg(target_arch = "riscv64")]
fn send_ipi_to_hart0() -> Result<(), AxError> {
    debug!("ipi_device: sending IPI to hart 0");

    let hart_mask: usize = 0x1; // Only hart 0
    let hart_mask_base: usize = 0; // Start from hart 0

    // SBI Extension ID for IPI (0x735049 = ASCII "sPI")
    const SBI_EXT_IPI: usize = 0x735049;
    // SBI Function ID for sbi_send_ipi
    const SBI_FUNC_SEND_IPI: usize = 0x00;

    let mut error: usize;
    let mut _value: usize;

    unsafe {
        core::arch::asm!(
            "ecall",
            inlateout("a0") hart_mask => error,
            inlateout("a1") hart_mask_base => _value,
            in("a6") SBI_FUNC_SEND_IPI,
            in("a7") SBI_EXT_IPI,
        );
    }

    if error == 0 {
        info!("ipi_device: successfully sent IPI to hart 0");
        // Message will be added by IRQ handler when interrupt is received
        Ok(())
    } else {
        warn!("ipi_device: failed to send IPI to hart 0: SBI error {}", error);
        Err(AxError::Unsupported)
    }
}

/// IPI Character Device
pub struct IpiDevice;

impl IpiDevice {
    pub fn new() -> Self {
        // Register IRQ handler for IPI when device is created
        #[cfg(target_arch = "riscv64")]
        {
            use axhal::irq::register;
            if register(IPI_IRQ, ipi_irq_handler) {
                info!("ipi_device: IRQ handler registered for IRQ {}", IPI_IRQ);
            } else {
                warn!("ipi_device: Failed to register IRQ handler for IRQ {}", IPI_IRQ);
            }
        }

        // IPI 设备初始化完成后，标记 Hart1 OS 已准备好
        mark_hart1_os_ready();

        Self
    }
}

impl Default for IpiDevice {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceOps for IpiDevice {
    fn read_at(&self, buf: &mut [u8], _offset: u64) -> VfsResult<usize> {
        let mut messages = IPI_MESSAGES.lock();
        if messages.is_empty() {
            let msg = b"No IPI messages pending\n";
            let len = core::cmp::min(buf.len(), msg.len());
            buf[..len].copy_from_slice(&msg[..len]);
            return Ok(len);
        }

        if let Some(msg) = messages.pop_front() {
            let msg_bytes = msg.as_bytes();
            let len = core::cmp::min(buf.len(), msg_bytes.len());
            buf[..len].copy_from_slice(&msg_bytes[..len]);
            if len < buf.len() {
                buf[len] = b'\n';
                Ok(len + 1)
            } else {
                Ok(len)
            }
        } else {
            Ok(0)
        }
    }

    fn write_at(&self, buf: &[u8], _offset: u64) -> VfsResult<usize> {
        debug!("ipi_device: write called with {} bytes", buf.len());

        #[cfg(target_arch = "riscv64")]
        {
            send_ipi_to_hart0()?;
            Ok(buf.len())
        }

        #[cfg(not(target_arch = "riscv64"))]
        {
            info!("ipi_device: IPI only supported on RISC-V");
            Err(AxError::Unsupported)
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn flags(&self) -> NodeFlags {
        NodeFlags::NON_CACHEABLE | NodeFlags::STREAM
    }
}
