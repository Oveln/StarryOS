//! Shared memory device for AMP inter-core communication.
//!
//! Exposes the ov_channal shared memory region at physical address 0x8800_0000
//! as `/dev/rt_shm`. User processes can:
//!
//! - `open("/dev/rt_shm", O_RDWR)` — exclusive access (one process only)
//! - `mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0)` — maps the
//!   shared memory directly into user space
//! - `ioctl(fd, RT_SHM_IOC_NOTIFY, 0)` — trigger SBI IPI to hart 0 (rt-async)
//! - `ioctl(fd, RT_SHM_IOC_AWAIT, 0)` — block until an IPI interrupt arrives
//!   from hart 0, then return
//!
//! The shared memory layout is `ov_channal::SharedMemory` (67,072 bytes),
//! containing two SPSC channels:
//!   - Channel 0: StarryOS (user) → rt-async (sender)
//!   - Channel 1: rt-async → StarryOS (user) (receiver)

use core::any::Any;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Context;

use axfs_ng_vfs::{NodeFlags, VfsError, VfsResult};
use axpoll::{IoEvents, PollSet, Pollable};
use memory_addr::PhysAddrRange;

use crate::pseudofs::{DeviceMmap, DeviceOps};

/// ioctl command: send IPI notification to hart 0 (rt-async).
pub const RT_SHM_IOC_NOTIFY: u32 = 0x7350_01;

/// ioctl command: block until IPI interrupt from hart 0 arrives.
pub const RT_SHM_IOC_AWAIT: u32 = 0x7350_02;

/// Physical base address of the shared memory region.
const SHM_PHYS_BASE: usize = 0x8800_0000;

/// Size of `ov_channal::SharedMemory`: 2 channels × 131 × 256 bytes = 67,072.
const SHM_SIZE: usize = 67072;

/// RISC-V Supervisor Software Interrupt cause (used as IRQ number).
const IPI_IRQ: usize = 0x8000_0000_0000_0001;

/// SBI IPI extension ID (standard: 0x735049).
const SBI_EXT_IPI: usize = 0x735049;
const SBI_FUNC_SEND_IPI: usize = 0x00;

static OPENED: AtomicBool = AtomicBool::new(false);

static IPC_PENDING: AtomicBool = AtomicBool::new(false);

static IPC_POLLSET: PollSet = PollSet::new();

/// Send IPI to hart 0 via standard SBI ecall.
#[cfg(target_arch = "riscv64")]
fn sbi_send_ipi_to_hart0() -> VfsResult<usize> {
    let hart_mask: usize = 0x1;
    let hart_mask_base: usize = 0x0;

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
        Ok(0)
    } else {
        warn!("rt_shm: SBI IPI failed with error {}", error);
        Err(VfsError::Io)
    }
}

#[cfg(not(target_arch = "riscv64"))]
fn sbi_send_ipi_to_hart0() -> VfsResult<usize> {
    Err(VfsError::Unsupported)
}

/// IPI interrupt handler — called when hart 0 sends an IPI to us.
#[cfg(target_arch = "riscv64")]
fn ipi_irq_handler() {
    unsafe {
        core::arch::asm!("csrc sip, {}", const 2_usize);
    }
    IPC_PENDING.store(true, Ordering::Release);
    IPC_POLLSET.wake();
}

pub struct RtShmDevice {
    _private: (),
}

impl RtShmDevice {
    pub fn new() -> Self {
        #[cfg(target_arch = "riscv64")]
        {
            use axhal::irq::register;
            if register(IPI_IRQ, ipi_irq_handler) {
                info!("rt_shm: IPI IRQ handler registered");
            } else {
                warn!("rt_shm: failed to register IPI IRQ handler");
            }
        }

        info!(
            "rt_shm: device initialized, phys base {:#x}, size {} bytes",
            SHM_PHYS_BASE, SHM_SIZE
        );

        Self { _private: () }
    }
}

impl DeviceOps for RtShmDevice {
    fn read_at(&self, _buf: &mut [u8], _offset: u64) -> VfsResult<usize> {
        Err(VfsError::Unsupported)
    }

    fn write_at(&self, _buf: &[u8], _offset: u64) -> VfsResult<usize> {
        Err(VfsError::Unsupported)
    }

    fn ioctl(&self, cmd: u32, _arg: usize) -> VfsResult<usize> {
        match cmd {
            RT_SHM_IOC_NOTIFY => sbi_send_ipi_to_hart0(),
            RT_SHM_IOC_AWAIT => {
                use axtask::future::{block_on, interruptible};
                use core::future::poll_fn;
                use core::task::Poll;

                block_on(interruptible(poll_fn(|cx| {
                    IPC_POLLSET.register(cx.waker());
                    if IPC_PENDING.swap(false, Ordering::AcqRel) {
                        Poll::Ready(0usize)
                    } else {
                        Poll::Pending
                    }
                })))
                .map_err(|_| VfsError::Interrupted)
            }
            _ => Err(VfsError::InvalidInput),
        }
    }

    fn mmap(&self) -> DeviceMmap {
        DeviceMmap::Physical(PhysAddrRange::from_start_size(
            memory_addr::PhysAddr::from(SHM_PHYS_BASE),
            SHM_SIZE,
        ))
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }

    fn flags(&self) -> NodeFlags {
        NodeFlags::NON_CACHEABLE
    }
}

impl Pollable for RtShmDevice {
    fn poll(&self) -> IoEvents {
        let mut events = IoEvents::OUT;
        if IPC_PENDING.load(Ordering::Acquire) {
            events |= IoEvents::IN;
        }
        events
    }

    fn register(&self, context: &mut Context<'_>, _events: IoEvents) {
        IPC_POLLSET.register(context.waker());
    }
}

/// Try to atomically claim exclusive ownership of the rt_shm device.
/// Returns `true` on success, `false` if already opened.
pub fn try_claim_device() -> bool {
    OPENED
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_ok()
}

/// Release exclusive ownership (called on close).
pub fn release_device() {
    OPENED.store(false, Ordering::Release);
}
