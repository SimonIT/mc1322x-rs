use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

use critical_section::{CriticalSection, Mutex};

/// A single `critical_section`-guarded waker slot.
///
/// Shared shape used by every peripheral driver in this crate that arms a hardware interrupt
/// and waits to be woken by it (I2C, SPI, UART, the RTC delay): [`Self::set`] stores the
/// current task's waker right before arming the interrupt (called from inside the same
/// [`critical_section::with`] as the status check that decided to wait, so a completion
/// landing in between isn't missed), and [`Self::wake`] takes and wakes it from interrupt
/// context. A [`Waker`] left behind by a cancelled (dropped) async future is woken like any
/// other by whichever ISR next runs - harmless, since [`Waker::wake`] on a waker whose task no
/// longer exists is required by the `core` contract to do nothing.
///
/// Every user of this type can rely on `critical_section::with` unconditionally: this crate
/// (`mc1322x-hal`) provides its own `critical_section::Impl` (see `crate::critical_section_impl`),
/// so every binary that links any part of this crate already has a working provider, without
/// extra wiring.
pub(crate) struct WakerCell(Mutex<RefCell<Option<Waker>>>);

impl WakerCell {
    pub(crate) const fn new() -> Self {
        Self(Mutex::new(RefCell::new(None)))
    }

    /// Store `waker`, replacing whatever was there. Call from inside the `critical_section`
    /// scope that just decided to wait.
    pub(crate) fn set(&self, cs: CriticalSection, waker: &Waker) {
        *self.0.borrow(cs).borrow_mut() = Some(waker.clone());
    }

    /// Take and wake whichever waker is currently stored, if any. A no-op if nothing was
    /// waiting.
    pub(crate) fn wake(&self) {
        critical_section::with(|cs| {
            if let Some(waker) = self.0.borrow(cs).borrow_mut().take() {
                waker.wake();
            }
        });
    }
}

/// Yield to the executor once, then resume.
///
/// Used by [`crate::i2c::I2c0::start_async`] for the initial bus-not-busy wait: `I2C_MBB`
/// reflects other masters' bus activity, which has no interrupt of its own on this
/// peripheral, so that wait polls the status register in a loop instead, yielding between
/// polls with this future rather than spinning.
pub(crate) fn yield_now() -> YieldNow {
    YieldNow(false)
}

pub(crate) struct YieldNow(bool);

impl Future for YieldNow {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if self.0 {
            Poll::Ready(())
        } else {
            self.0 = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}
