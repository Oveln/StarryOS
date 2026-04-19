//! IPI (Inter-Processor Interrupt) character device
//!
//! - Write to device: Send message/trigger IPI to hart 0
//!   - "notify <id>" - Send notification
//!   - "hello" - Call hello_world function
//! - IRQ handler: Process messages from hart 0
//! - Read from device: Get received messages

use alloc::{collections::VecDeque, format, string::String, vec::Vec};
use core::any::Any;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering, compiler_fence};
use core::str::FromStr;

use axerrno::AxError;
use axfs_ng_vfs::{NodeFlags, VfsResult};
use axsync::Mutex;

use crate::pseudofs::DeviceOps;

// Import ov_channal for inter-system communication
use ov_channal::{ChannelId, Message, MsgType, SharedMemory};

/// Response type for deserialization strategy
#[derive(Clone, Copy, Debug, PartialEq)]
enum ResponseType {
    /// No response data (just requestid)
    Empty,
    /// Usize
    Usize,
    /// i32 response
    I32,
    /// (i32, i32) tuple response
    I32Tuple,
}

/// Pending request tracking entry
struct PendingRequest {
    request_id: u64,
    response_type: ResponseType,
}

/// Global request ID counter (atomic for thread safety)
static NEXT_REQUEST_ID: AtomicU64 = AtomicU64::new(1);

/// Pending requests table (maps request_id to response type)
static PENDING_REQUESTS: Mutex<Vec<PendingRequest>> = Mutex::new(Vec::new());

/// IPI IRQ number for RISC-V (software interrupt)
const IPI_IRQ: usize = 0x8000_0000_0000_0001;

/// Hart 同步标志的物理地址
/// 这个地址用于两个 hart 之间同步启动状态
pub const HART_SYNC_PHYS_ADDR: usize = 0xc8000000;

/// Shared memory base address for inter-system communication: 0xc8000000 + 256
const INTERCOM_SHM_BASE: usize = 0xc8000100;

/// Hart 同步标志结构体
/// 放置在固定物理地址，确保两个 hart 都能访问
#[repr(C)]
pub struct HartSyncFlags {
    /// 魔数，用于验证结构体有效性
    pub magic_number: u16,
    /// Hart0 上的 OS 是否已经启动
    /// true = 已启动, false = 未启动
    pub hart0_os_ready: AtomicBool,
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

    // Initialize shared memory for inter-system communication
    init_intercom();
}

/// Initialize inter-system communication
fn init_intercom() {
    unsafe {
        let shm = SharedMemory::at(axhal::mem::phys_to_virt(memory_addr::PhysAddr::from(INTERCOM_SHM_BASE)).as_ptr() as usize);
        if !shm.is_valid() {
            panic!("ipi_device: failed to initialize shared memory at {:#x}", INTERCOM_SHM_BASE);
        }
    }
    info!("ipi_device: Inter-system communication initialized at {:#x}", INTERCOM_SHM_BASE);
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

/// Generate a new unique request ID and register the response type
fn register_request(response_type: ResponseType) -> u64 {
    let request_id = NEXT_REQUEST_ID.fetch_add(1, Ordering::Relaxed);

    let mut pending = PENDING_REQUESTS.lock();
    pending.push(PendingRequest {
        request_id,
        response_type,
    });

    // Limit pending requests size to prevent unbounded growth
    if pending.len() > 128 {
        pending.remove(0);
    }

    request_id
}

/// Look up and remove the response type for a given request ID
fn lookup_and_remove_request(request_id: u64) -> Option<ResponseType> {
    let mut pending = PENDING_REQUESTS.lock();
    let pos = pending.iter().position(|r| r.request_id == request_id)?;
    Some(pending.remove(pos).response_type)
}

/// IRQ handler - called when IPI interrupt is received
#[cfg(target_arch = "riscv64")]
fn ipi_irq_handler() {
    unsafe {
        core::arch::asm!("csrc sip, {}", const 2_usize);
    }

    // Process messages from embassy_preempt
    unsafe {
        let shm = SharedMemory::at(axhal::mem::phys_to_virt(memory_addr::PhysAddr::from(INTERCOM_SHM_BASE)).as_ptr() as usize);
        if let Ok(rx) = shm.receiver(ChannelId::new(1)) {
            while let Some(msg) = rx.try_recv() {
                match msg.ty() {
                    Some(MsgType::Notification) => {
                        if let Some(id) = msg.as_notification() {
                            add_ipi_message(format!("Notification from embassy: {}", id));
                        }
                    }
                    Some(MsgType::Response) => {
                        let rid = msg.request_id().unwrap();
                        if let Some(resp_type) = lookup_and_remove_request(rid) {
                            match resp_type {
                                ResponseType::Empty => {
                                    if let Some((_, ())) = msg.as_response::<()>() {
                                        add_ipi_message(format!("Response {}: OK", rid));
                                    }
                                }
                                ResponseType::Usize => {
                                    if let Some((_, result)) = msg.as_response::<usize>() {
                                        add_ipi_message(format!("Response {}: {}", rid, result));
                                    }
                                }
                                ResponseType::I32 => {
                                    if let Some((_, result)) = msg.as_response::<i32>() {
                                        add_ipi_message(format!("Response {}: {}", rid, result));
                                    }
                                }
                                ResponseType::I32Tuple => {
                                    if let Some((_, (a, b))) = msg.as_response::<(i32, i32)>() {
                                        add_ipi_message(format!("Response {}: ({}, {})", rid, a, b));
                                    }
                                }
                            }
                        } else {
                            // Unknown request_id, fallback to i32
                            if let Some((_, result)) = msg.as_response::<i32>() {
                                add_ipi_message(format!("Response{}: {}", rid, result));
                            }
                        }
                    }
                    _ => {
                        add_ipi_message(format!("Got message from embassy"));
                    }
                }
            }
        }
    }
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
            return Ok(0);
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

        // Parse command from buffer
        use crate::alloc::string::ToString;
        let cmd = String::from_utf8_lossy(buf).trim().to_string();
        info!("ipi_device: Received command: '{}'", cmd);

        #[cfg(target_arch = "riscv64")]
        {
            if cmd.starts_with("notify ") {
                // Parse: notify <id>
                if let Some(id_str) = cmd.strip_prefix("notify ") {
                    if let Ok(id) = id_str.parse::<u32>() {
                        // Send notification to embassy_preempt
                        unsafe {
                            let shm = SharedMemory::at(axhal::mem::phys_to_virt(memory_addr::PhysAddr::from(INTERCOM_SHM_BASE)).as_ptr() as usize);
                            if let Ok(tx) = shm.sender(ChannelId::new(0)) {
                                let msg = Message::notification(id);
                                if tx.try_send(&msg).is_ok() {
                                    send_ipi_to_hart0()?;
                                    add_ipi_message(format!("Sent notification {}", id));
                                    return Ok(buf.len());
                                }
                            }
                        }
                        return Err(AxError::Io);
                    }
                }
            } else if cmd == "hello" {
                // Call hello_world function
                let rid = register_request(ResponseType::Usize);
                unsafe {
                    let shm = SharedMemory::at(axhal::mem::phys_to_virt(memory_addr::PhysAddr::from(INTERCOM_SHM_BASE)).as_ptr() as usize);
                    if let Ok(tx) = shm.sender(ChannelId::new(0)) {
                        const METHOD_HELLO: u64 = 0;
                        let msg = Message::request(rid, METHOD_HELLO, &()).unwrap();
                        if tx.try_send(&msg).is_ok() {
                            send_ipi_to_hart0()?;
                            add_ipi_message(format!("Called hello_world (rid={})", rid));
                            return Ok(buf.len());
                        }
                    }
                }
                return Err(AxError::Io);
            } else if cmd.starts_with("add ") {
                use alloc::vec::Vec;
                // Parse: add <a> <b>
                let parts: Vec<&str> = cmd.split_whitespace().collect();
                if parts.len() == 3 {
                    if let (Ok(a), Ok(b)) = (i32::from_str(parts[1]), i32::from_str(parts[2])) {
                        let rid = register_request(ResponseType::I32);
                        unsafe {
                            let shm = SharedMemory::at(axhal::mem::phys_to_virt(memory_addr::PhysAddr::from(INTERCOM_SHM_BASE)).as_ptr() as usize);
                            if let Ok(tx) = shm.sender(ChannelId::new(0)) {
                                const METHOD_ADD: u64 = 1;
                                let msg = Message::request(rid, METHOD_ADD, &(a, b)).unwrap();
                                if tx.try_send(&msg).is_ok() {
                                    send_ipi_to_hart0()?;
                                    add_ipi_message(format!("Sent add request: {} + {} (rid={})", a, b, rid));
                                    return Ok(buf.len());
                                }
                            }
                        }
                        return Err(AxError::Io);
                    }
                }
            }

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
