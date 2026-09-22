//! A sleep-aware `embassy-executor`: enters CRM `Doze` between polls when nothing is
//! immediately runnable, instead of spin-polling like `embassy-executor`'s `platform-spin`.
//!
//! # Only sound for pure-timer workloads
//!
//! [`SleepyExecutor`] only checks `mc1322x_hal::sleep::SleepInhibitGuard::count()` before
//! sleeping — every async peripheral wait in `mc1322x-hal` (`uart`, `spi`, `i2c`, `delay`,
//! `aes`, `adc`, `gpio`'s `KbiInput`) holds one for as long as its wait is in flight, so sleep
//! is correctly refused whenever one of those is pending (sleeping would cut the peripheral's
//! clock and it could never raise the completion interrupt the wait depends on). This makes
//! [`SleepyExecutor`] safe to use with tasks that mix `embassy-time` timers and those async
//! peripheral waits — but note what the guard does *not* cover: any interrupt-driven wait
//! written *outside* `mc1322x-hal` (a future board-specific driver, or hand-rolled ISR/waker
//! code) that doesn't also hold a `SleepInhibitGuard` would hang exactly the same way, silently.
//!
//! # `platform-spin` conflict
//!
//! [`SleepyExecutor`] registers its own no-op [`embassy_executor::pender::Pender`] (the
//! "pend" callback only matters for waking a *sleeping* executor thread from another context,
//! which never happens here — this run loop always re-polls unconditionally, the same reason
//! `platform-spin`'s own `SpinPender` is a no-op). `embassy-executor`'s `pender_impl!` may only
//! run once in the whole crate tree: a binary using [`SleepyExecutor`] must **not** also enable
//! any `platform-*` Cargo feature (e.g. `platform-spin`, which every other example in this
//! workspace uses for the plain `embassy_executor::Executor`) elsewhere in its dependency
//! graph, or the two `Pender` registrations conflict at link time.
//!
//! # Known race: a benign latency window, not a hang
//!
//! Between `raw::Executor::poll()` returning "nothing runnable" and [`mc1322x_hal::sleep::sleep`]
//! actually committing to low-power mode, interrupts stay fully enabled — a peripheral
//! interrupt landing in that narrow window is serviced normally (nothing is lost; the
//! peripheral's own hardware FIFO/latch holds whatever it received, and the corresponding
//! waker fires, making its task ready), but this run loop has no way to notice that before
//! calling `sleep()` anyway, so that now-ready task doesn't actually get polled again until
//! the `Doze` timer wakes the CPU. Bounded by whatever sleep duration was chosen (at most
//! [`ticks_until_next_wake`]'s value), not unbounded, and not a correctness bug — just added
//! latency for that one task, in a narrow, rarely-hit window. Not fixed here: doing so would
//! need holding a critical section across the idle-check-and-sleep sequence, which is
//! unverified and out of scope for a first version — see the module's git history/PR
//! description if this ever needs revisiting.
//!
//! # TMR0 resync
//!
//! TMR0 (the `embassy-time` tick source) does not survive `Doze`/`Hibernate` — only the CRM's
//! own dedicated sleep timer does (RM §5.2.3/§5.3) — so every sleep cycle here reconfigures it
//! from scratch and advances the time driver's tick count by exactly the duration this code
//! itself chose and armed as the wake-up timeout, rather than trying to re-derive elapsed time
//! from TMR0's meaningless post-wake state. See `crate::time_driver::resync_after_sleep`.

use core::ptr;

use embassy_executor::pender::Pender;
use embassy_executor::{Spawner, pender_impl, raw};
use mc1322x_hal::sleep::{
    RamRetention, Retention, SleepInhibitGuard, SleepMode, WakeSources, sleep,
};

use crate::time_driver;

struct SleepyPender;

impl Pender for SleepyPender {
    fn pend(_context: *mut ()) {}
}

pender_impl!(SleepyPender);

/// `Doze` runs off the reference oscillator ÷128 (RM §5.2.3 - see `mc1322x_hal::sleep`'s module
/// doc for the Hibernate/Doze trade-off; `Doze` is used here for its accurate wake delay, no
/// crystal needed).
const DOZE_CLOCK_HZ: u64 = 24_000_000 / 128;
/// `embassy-time`'s tick rate (matches `mc1322x-embassy::time_driver`'s own `TICK_HZ`).
const TICK_HZ: u64 = 1_000;

/// Minimum idle stretch worth actually sleeping for, in `embassy-time` ticks (milliseconds).
///
/// Below this, the CRM sleep/wake handshake's own overhead (RM §5.3.1/§5.3.2: hardware
/// synchronizing `SLEEP_SYNC` on entry and exit, up to a couple of sleep-clock cycles each way)
/// and the accumulated exposure to further sleep/wake cycles (`mc1322x_hal::sleep`'s
/// documented, permanent UART-clocking quirk after the *first* post-boot cycle) aren't worth it
/// for a nap this short. Not exposed as a tuning knob for this first version.
const MIN_SLEEP_TICKS: u64 = 20;

/// A sleep-aware executor. See the module docs for what this is and isn't safe for.
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
    /// Same shape as `embassy_executor::Executor::run` (see its docs for why this needs
    /// `&'static mut self` and how to obtain that, e.g. via a `static_cell::StaticCell`): the
    /// `init` closure spawns the initial task(s), then this polls forever, entering `Doze`
    /// between polls whenever nothing is immediately runnable, no async peripheral wait is in
    /// flight, and a `embassy-time` timer is scheduled at least [`MIN_SLEEP_TICKS`] away. Never
    /// returns.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        loop {
            unsafe { self.inner.poll() };

            if SleepInhibitGuard::count() != 0 {
                continue;
            }
            let Some(ticks) = time_driver::ticks_until_next_wake() else {
                continue;
            };
            if ticks < MIN_SLEEP_TICKS {
                continue;
            }

            let doze_ticks = ((ticks * DOZE_CLOCK_HZ) / TICK_HZ).min(u32::MAX as u64) as u32;
            sleep(
                SleepMode::Doze,
                WakeSources {
                    timer: Some(doze_ticks),
                    ..Default::default()
                },
                // Full retention: an executor sleeping mid-task must resume every task's stack
                // and state exactly, not cold-restart - not exposed as a tuning knob for this
                // first version.
                Retention {
                    mcu: true,
                    gpio_pads: true,
                    ram: RamRetention::Kb96,
                },
            );
            // Resynced unconditionally, not gated on the returned `WakeReason`: `timer` is the
            // only wake source armed above, so whatever woke `sleep` (barring some
            // undocumented hardware fluke misclassifying it) did so after approximately
            // `ticks` elapsed - and leaving TMR0/`base` un-resynced on a misclassification
            // would silently break `now()` forever after, which is worse than a rare,
            // approximately-correct resync.
            time_driver::resync_after_sleep(ticks);
        }
    }
}

impl Default for SleepyExecutor {
    fn default() -> Self {
        Self::new()
    }
}
