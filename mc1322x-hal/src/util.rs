//! Async helpers shared by the peripheral drivers.

use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

use critical_section::{CriticalSection, Mutex};

/// A single `critical_section`-guarded waker slot.
///
/// Used by every driver that waits for a hardware interrupt: the task calls [`Self::set`] in
/// the same critical section as the status check that decided to wait (so a completion in
/// between isn't missed), and the ISR calls [`Self::wake`]. A waker left behind by a dropped
/// future is woken spuriously, which executors handle.
pub(crate) struct WakerCell(Mutex<RefCell<Option<Waker>>>);

impl WakerCell {
    pub(crate) const fn new() -> Self {
        Self(Mutex::new(RefCell::new(None)))
    }

    /// Store `waker`, replacing whatever was there. Call from the critical section that decided
    /// to wait.
    pub(crate) fn set(&self, cs: CriticalSection, waker: &Waker) {
        *self.0.borrow(cs).borrow_mut() = Some(waker.clone());
    }

    /// Take and wake the stored waker, if any.
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
/// Used by `I2c0::start_async` to poll `I2C_MBB` (bus busy), which has no interrupt.
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
