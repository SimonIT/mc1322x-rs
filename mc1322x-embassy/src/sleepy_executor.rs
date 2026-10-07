//! A sleep-aware executor that enters CRM `Doze` between polls when nothing is runnable,
//! instead of busy-polling like `embassy-executor`'s `platform-spin`.
//!
//! # Limitations
//!
//! [`SleepyExecutor`] only sleeps while
//! [`SleepInhibitGuard::count`](mc1322x_hal::sleep::SleepInhibitGuard::count) is zero. Every
//! async peripheral wait in `mc1322x-hal` (`uart`, `spi`, `i2c`, `delay`, `aes`, `adc`, `gpio`'s
//! `KbiInput`) holds a guard while it is in flight, because the peripheral loses its clock in
//! `Doze` and could never raise the interrupt the wait depends on. Tasks may therefore mix
//! `embassy-time` timers with those waits. Any other interrupt-driven wait (a custom driver or
//! hand-written ISR/waker) is not covered and hangs if the executor sleeps while it is pending.
//!
//! # `platform-spin` conflict
//!
//! [`SleepyExecutor`] registers its own [`embassy_executor::pender::Pender`] with
//! `pender_impl!`, which may only happen once per binary. A binary using it must not enable any
//! `embassy-executor` `platform-*` feature (e.g. `platform-spin`) anywhere in its dependency
//! graph, or the two registrations conflict at link time.
//!
//! # Wake-ups between polling and sleeping
//!
//! An interrupt can wake a task after `raw::Executor::poll()` has returned but before the run
//! loop decides to sleep - most often the 1 kHz TMR0 tick waking a timer that just came due.
//! Sleeping then would leave that task unpolled for the whole sleep. To prevent this, the
//! [`Pender`] (called by `embassy-executor` whenever a wake makes a task runnable) sets a flag
//! that is cleared before every poll, and the run loop checks that flag, picks the sleep
//! duration and calls [`mc1322x_hal::sleep::sleep_with`] inside one critical section.
//! `sleep_with` busy-waits on CRM status bits rather than on an interrupt, so masking interrupts
//! across it is fine; anything that becomes pending meanwhile is serviced when the critical
//! section ends.
//!
//! # TMR0 resync
//!
//! TMR0 (the `embassy-time` tick source) stops in `Doze`; only the CRM sleep timer keeps running
//! (RM §5.2.3, §5.3). After every sleep, TMR0 is reconfigured from scratch and the tick count is
//! advanced by the time the sleep timer measured, which includes the wake-up sequence that runs
//! past the programmed timeout.

use core::ptr;
use core::sync::atomic::Ordering;

use embassy_executor::pender::Pender;
use embassy_executor::{Spawner, pender_impl, raw};
use mc1322x_hal::sleep::{RamRetention, Retention, SleepInhibitGuard, SleepMode, WakeSources, sleep_with};
use portable_atomic::AtomicBool;

use crate::time_driver;

/// Set whenever a task was woken since the start of the last poll.
static PENDED: AtomicBool = AtomicBool::new(false);

struct SleepyPender;

impl Pender for SleepyPender {
    fn pend(_context: *mut ()) {
        PENDED.store(true, Ordering::Release);
    }
}

pender_impl!(SleepyPender);

/// Minimum idle stretch worth actually sleeping for, in `embassy-time` ticks (milliseconds).
///
/// Shorter naps aren't worth the CRM sleep/wake handshake overhead (RM §5.3.1, §5.3.2) or the
/// extra exposure to `mc1322x_hal::sleep`'s peripheral-clocking caveat after the first
/// sleep/wake cycle.
const MIN_SLEEP_TICKS: u64 = 20;

/// Executor that enters CRM `Doze` while idle. See the module docs for its limitations.
pub struct SleepyExecutor {
    inner: raw::Executor,
}

impl SleepyExecutor {
    /// Create a new executor.
    pub fn new() -> Self {
        Self {
            inner: raw::Executor::new(ptr::null_mut()),
        }
    }

    /// Run the executor.
    ///
    /// Like `embassy_executor::Executor::run`: `init` spawns the initial tasks, then this polls
    /// forever. Between polls it enters `Doze` whenever no task is runnable, no `mc1322x-hal`
    /// async peripheral wait is in flight, and the next `embassy-time` timer is at least 20 ms
    /// away. It never sleeps without a timer scheduled. Get the `&'static mut self` from e.g. a
    /// `static_cell::StaticCell`.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        loop {
            PENDED.store(false, Ordering::Release);
            unsafe { self.inner.poll() };

            // Interrupts stay masked from the `PENDED` check until after the resync, so nothing
            // can wake a task in between (see the module docs), and no TMR0 interrupt left over
            // from before the sleep runs against TMR0's meaningless post-wake state.
            critical_section::with(|cs| {
                if PENDED.load(Ordering::Acquire) || SleepInhibitGuard::count() != 0 {
                    return;
                }
                let Some(ticks) = time_driver::ticks_until_next_wake(cs) else {
                    return;
                };
                if ticks < MIN_SLEEP_TICKS {
                    return;
                }
                let Some(doze_ticks) = time_driver::doze_ticks_until_next_wake(cs) else {
                    return;
                };

                let mut counts_before_sleep = 0;
                // `Doze` rather than `Hibernate`: its sleep timer runs off the reference
                // oscillator, so the wake delay is accurate without a 32 kHz crystal.
                sleep_with(
                    SleepMode::Doze,
                    WakeSources {
                        timer: Some(doze_ticks),
                        ..Default::default()
                    },
                    // Full retention: every task's stack and state must survive the sleep.
                    Retention {
                        mcu: true,
                        gpio_pads: true,
                        ram: RamRetention::Kb96,
                    },
                    // TMR0's position in the current tick, taken at the last moment before the
                    // clocks stop so no time between it and power-down goes uncounted.
                    || counts_before_sleep = time_driver::counts_before_sleep(cs),
                );
                // Resync regardless of the `WakeReason`: the sleep timer measures the actual sleep
                // whatever ended it, and TMR0 must be reconfigured after any wake.
                time_driver::resync_after_sleep(counts_before_sleep);
            });
        }
    }
}

impl Default for SleepyExecutor {
    fn default() -> Self {
        Self::new()
    }
}
