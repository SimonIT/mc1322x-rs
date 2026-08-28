use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

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
