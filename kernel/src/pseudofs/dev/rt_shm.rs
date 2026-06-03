//! Shared memory device for AMP inter-core communication.
//!
//! Exposes the ov_channels shared memory region as `/dev/rt_shm`.
//!
//! All physical addresses and constants below are generated from `amp.toml`
//! at the repository root via `build.rs`.

use core::any::Any;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use core::task::{Context, Waker};

use axfs_ng_vfs::{NodeFlags, VfsError, VfsResult};
use axpoll::{IoEvents, Pollable};
use kspin::SpinNoIrq;
use memory_addr::PhysAddrRange;

use crate::pseudofs::{DeviceMmap, DeviceOps};

mod amp {
    include!(concat!(env!("OUT_DIR"), "/amp_gen.rs"));
}

/// ioctl command: send IPI notification to hart 1 (rt-async).
pub const RT_SHM_IOC_NOTIFY: u32 = 0x7350_01;

/// ioctl command: block until CH1 has pending messages.
pub const RT_SHM_IOC_AWAIT: u32 = 0x7350_02;

/// ioctl command: no-op (kept for ABI compatibility).
pub const RT_SHM_IOC_CLR_PENDING: u32 = 0x7350_03;

/// Physical base address of the shared memory region (from amp.toml: SHMBASE).
const SHM_PHYS_BASE: usize = amp::SHMBASE;

/// Size of `ov_channels::SharedMemory` (from amp.toml: SHMSIZE).
const SHM_SIZE: usize = amp::SHMSIZE;

/// RISC-V Supervisor Software Interrupt cause (used as IRQ number).
const IPI_IRQ: usize = 0x8000_0000_0000_0001;

/// CLINT base address (from amp.toml: CLINTBASE).
const CLINT_BASE: usize = amp::CLINTBASE;
const CLINT_MSIP1_OFFSET: usize = 0x4;

const CH1_RING_READ_OFFSET: usize = 0x8400;
const CH1_RING_WRITE_OFFSET: usize = 0x8408;

static OPENED: AtomicBool = AtomicBool::new(false);

/// IRQ-safe waker storage. `SpinNoIrq` disables local IRQs during lock,
/// preventing the IPI handler from deadlocking on the same hart.
static IPC_WAKER: SpinNoIrq<Option<Waker>> = SpinNoIrq::new(None);

fn shm_vaddr() -> usize {
    axhal::mem::phys_to_virt(memory_addr::PhysAddr::from(SHM_PHYS_BASE)).as_ptr() as usize
}

fn ch1_has_pending() -> bool {
    let base = shm_vaddr();
    let r = unsafe {
        (*((base + CH1_RING_READ_OFFSET) as *const AtomicUsize))
            .load(Ordering::Acquire)
    };
    let w = unsafe {
        (*((base + CH1_RING_WRITE_OFFSET) as *const AtomicUsize))
            .load(Ordering::Acquire)
    };
    trace!("rt_shm: ch1 read {}, write {}", r, w);
    r != w
}

#[cfg(target_arch = "riscv64")]
fn send_ipi_to_rt_async() -> VfsResult<usize> {
    core::sync::atomic::fence(core::sync::atomic::Ordering::Release);
    let vaddr = axhal::mem::phys_to_virt(memory_addr::PhysAddr::from(
        CLINT_BASE + CLINT_MSIP1_OFFSET,
    ));
    unsafe {
        core::ptr::write_volatile(vaddr.as_ptr() as *mut u32, 1);
    }
    Ok(0)
}

#[cfg(not(target_arch = "riscv64"))]
fn send_ipi_to_rt_async() -> VfsResult<usize> {
    Err(VfsError::Unsupported)
}

/// IPI interrupt handler — called when hart 1 (rt-async) sends an IPI to us.
///
/// Wakes the task blocked in `AWAIT`. The waker is stored in [`IPC_WAKER`];
/// message availability is determined directly from CH1's ring buffer, so
/// a spurious wakeup is harmless.
#[cfg(target_arch = "riscv64")]
fn ipi_irq_handler() {
    if let Some(waker) = IPC_WAKER.lock().take() {
        waker.wake();
    }
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
            RT_SHM_IOC_NOTIFY => send_ipi_to_rt_async(),
            RT_SHM_IOC_AWAIT => {
                use axtask::future::{block_on, interruptible};
                use core::future::poll_fn;
                use core::task::Poll;
                block_on(interruptible(poll_fn(|cx| {
                    if ch1_has_pending() {
                        debug!("rt_shm: pending message detected in AWAIT");
                        return Poll::Ready(0usize);
                    }
                    debug!("rt_shm: no pending message, blocking in AWAIT");
                    // SpinNoIrq disables local IRQs; register and re-check
                    // are atomic w.r.t. the IPI handler on this hart.
                    let mut guard = IPC_WAKER.lock();
                    if ch1_has_pending() {
                        debug!("rt_shm: pending message detected after register in AWAIT");
                        Poll::Ready(0usize)
                    } else {
                        *guard = Some(cx.waker().clone());
                        Poll::Pending
                    }
                })))
                .map_err(|_| VfsError::Interrupted)
            }
            RT_SHM_IOC_CLR_PENDING => Ok(0),
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
        if ch1_has_pending() {
            events |= IoEvents::IN;
        }
        events
    }

    fn register(&self, context: &mut Context<'_>, _events: IoEvents) {
        *IPC_WAKER.lock() = Some(context.waker().clone());
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
