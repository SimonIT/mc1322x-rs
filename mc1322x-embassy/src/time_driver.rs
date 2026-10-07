//! `embassy-time` driver on TMR0, with a 1 kHz tick.
//!
//! TMR0 runs free from the 24 MHz reference oscillator prescaled by 64, as a 16-bit counter. A
//! compare interrupt fires every 375 counts, i.e. exactly 1000 times per second, and advances a
//! 64-bit tick count. `now()` adds the whole periods the live counter has run past the last
//! tick boundary, so it stays correct while the interrupt is delayed by a critical section.
//!
//! The interrupt is dispatched by the boot ROM's handler, which calls the weak `tmr0_isr` symbol
//! declared in libmc1322x's `isr.h`. The Rust handler is Thumb code, so like the C ISRs it relies
//! on that call being an interworking `bx`.
//!
//! # Limitations
//!
//! Interrupts must not stay disabled for longer than one wrap of the 16-bit counter (65536 / 375
//! kHz, about 175 ms). Beyond that, missed periods can no longer be recovered and the clock
//! silently falls behind.

use core::cell::{Cell, RefCell};
use core::sync::atomic::Ordering;
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use mc1322x_sys::{
    INTBASE, INTENNUM_OFF, TMR_REGOFF_CNTR, TMR_REGOFF_COMP1, TMR_REGOFF_CSCTRL, TMR_REGOFF_CTRL, TMR_REGOFF_ENBL,
    TMR_REGOFF_SCTRL, TMR0_BASE, interrupt_nums_INT_NUM_TMR,
};
use portable_atomic::AtomicBool;

/// Reference oscillator frequency (Hz).
const REF_OSC_HZ: u32 = 24_000_000;
/// Tick rate of the timebase (matches the `tick-hz-1_000` feature).
const TICK_HZ: u32 = 1_000;
/// `log2` of the TMR0 input prescaler.
///
/// 64 is the largest power of two dividing `REF_OSC_HZ / TICK_HZ` (24_000): `PERIOD` stays an
/// exact integer (no drift) while the 16-bit counter's wrap margin is as long as possible.
const PRESCALE_SHIFT: u32 = 6;
/// TMR0 input prescaler: the counter runs at `REF_OSC_HZ / PRESCALE`.
const PRESCALE: u32 = 1 << PRESCALE_SHIFT;
/// Counter counts per tick.
const PERIOD: u32 = REF_OSC_HZ / PRESCALE / TICK_HZ;

/// TMR0 counts per CRM sleep timer tick in `Doze`, which runs off the reference oscillator ÷ 128
/// (RM §5.2.3.6). An exact integer, so a sleep measured by the sleep timer converts to TMR0
/// counts without rounding. A power of two, so dividing a 64-bit count by it is a shift rather than
/// a 64-bit division libcall.
#[cfg(feature = "sleepy-executor")]
const COUNTS_PER_DOZE_TICK: u32 = 128 / PRESCALE;
#[cfg(feature = "sleepy-executor")]
const _: () = assert!(COUNTS_PER_DOZE_TICK.is_power_of_two());

/// `TMRx_CSCTRL.TCF1EN`: compare-1 interrupt enable (RM §12.6.9). Writing just this bit also
/// writes 0 to `TCF1`, which is what clears a pending compare-1 flag.
const CSCTRL_TCF1EN: u16 = 1 << 6;
/// `TMRx_CTRL.COUNT_MODE` (bits 15-13) = 1: count rising edges of the primary source.
const CTRL_COUNT_MODE_RISING: u16 = 1 << 13;
/// `TMRx_CTRL.PRIMARY_CNT_SOURCE` (bits 12-9) = `8 + n`: peripheral clock divided by `2.pow(n)`.
const CTRL_PRIMARY_CNT_SOURCE: u16 = (8 + PRESCALE_SHIFT as u16) << 9;

/// Read a 16-bit TMR0 register.
#[inline]
unsafe fn read16(offset: u32) -> u16 {
    unsafe { core::ptr::read_volatile((TMR0_BASE + offset) as *const u16) }
}

/// Write a 16-bit TMR0 register.
#[inline]
unsafe fn write16(offset: u32, value: u16) {
    unsafe { core::ptr::write_volatile((TMR0_BASE + offset) as *mut u16, value) };
}

struct TmrDriver {
    /// Number of whole ticks (1 ms periods) completed at `last_boundary`.
    base: Mutex<Cell<u64>>,
    /// The TMR0 counter value at `base`. Tick `t` is current whenever the free-running counter has
    /// passed `last_boundary + t * PERIOD` (mod 65536).
    last_boundary: Mutex<Cell<u16>>,
    /// Pending timers.
    queue: Mutex<RefCell<Queue>>,
    /// Whether `init()` has run.
    started: AtomicBool,
    /// Next queue deadline (`None` if nothing is scheduled), for [`ticks_until_next_wake`].
    /// Recomputed in [`TmrDriver::process_queue`] every tick and after every sleep, and
    /// tightened (never loosened) by [`Driver::schedule_wake`] so a sooner deadline is visible
    /// at once. `Option` instead of the queue's `u64::MAX` sentinel keeps `DRIVER` all-zero, so it
    /// lands in `.bss` instead of `.data`.
    next_alarm: Mutex<Cell<Option<u64>>>,
}

impl TmrDriver {
    const fn new() -> Self {
        Self {
            base: Mutex::new(Cell::new(0)),
            last_boundary: Mutex::new(Cell::new(0)),
            queue: Mutex::new(RefCell::new(Queue::new())),
            started: AtomicBool::new(false),
            next_alarm: Mutex::new(Cell::new(None)),
        }
    }
}

impl Driver for TmrDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| {
            let boundary = self.last_boundary.borrow(cs).get();
            let base = self.base.borrow(cs).get();
            let cntr = unsafe { read16(TMR_REGOFF_CNTR) };
            let since = (cntr.wrapping_sub(boundary)) as u32;
            base + u64::from(since / PERIOD as u16 as u32)
        })
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            // The TMR0 ISR fires once per tick and recomputes the timer queue, so there is
            // nothing to reprogram in hardware. `Queue::schedule_wake` also wakes wakers that
            // are already past due.
            queue.schedule_wake(at, waker);
            // Tighten the cached deadline now; the next tick recomputes the exact value, but the
            // sleepy executor may read it before then.
            let next_alarm = self.next_alarm.borrow(cs);
            next_alarm.set(Some(next_alarm.get().map_or(at, |next| next.min(at))));
        })
    }
}

/// Reset and configure TMR0 for a free-running 1 kHz compare interrupt, and reset
/// [`TmrDriver::last_boundary`] to match. Used by [`init`] and by [`resync_after_sleep`], since
/// TMR0 does not keep running through `Doze`/`Hibernate`.
fn configure_tmr0(cs: critical_section::CriticalSection) {
    unsafe {
        // Reset the timer first.
        write16(TMR_REGOFF_ENBL, 0);
        write16(TMR_REGOFF_SCTRL, 0);
        // Enable the compare-1 interrupt and clear the compare flag.
        write16(TMR_REGOFF_CSCTRL, CSCTRL_TCF1EN);
        write16(TMR_REGOFF_CNTR, 0);

        // LENGTH=0 (free-running): count past the compare value instead of reloading.
        write16(TMR_REGOFF_CTRL, CTRL_COUNT_MODE_RISING | CTRL_PRIMARY_CNT_SOURCE);

        // Free-run counter, armed to fire every PERIOD counts.
        let boundary = read16(TMR_REGOFF_CNTR);
        DRIVER.last_boundary.borrow(cs).set(boundary);
        write16(TMR_REGOFF_COMP1, boundary.wrapping_add(PERIOD as u16));

        // `TMR_ENBL` is shared by all four TMR channels. Like libmc1322x's `tmr-ints.c`, enable
        // all of them: writing bit 0 alone leaves the counter stopped.
        write16(TMR_REGOFF_ENBL, 0x0f);
    }
}

/// Start the time driver.
///
/// Configures TMR0 for a 1 kHz compare interrupt and enables the TMR interrupt in the ITC. Call
/// this before using `embassy-time`; calling it again does nothing. [`crate::init`] calls it.
pub fn init() {
    if DRIVER.started.swap(true, Ordering::AcqRel) {
        return;
    }

    critical_section::with(configure_tmr0);

    // Outside the critical section: `mc1322x-hal`'s `critical_section` restores the saved
    // `INTENABLE` on exit, which would undo an enable made inside it.
    unsafe {
        core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, interrupt_nums_INT_NUM_TMR);
    }
}

/// Ticks until the earliest scheduled wake, if any is currently pending.
///
/// Used by the sleepy executor to decide whether to sleep. See [`TmrDriver::next_alarm`].
#[cfg(feature = "sleepy-executor")]
pub(crate) fn ticks_until_next_wake(cs: critical_section::CriticalSection) -> Option<u64> {
    let next_alarm = DRIVER.next_alarm.borrow(cs).get()?;
    Some(next_alarm.saturating_sub(DRIVER.now()))
}

/// CRM sleep timer ticks in `Doze` from now until the earliest scheduled wake, if any is pending.
///
/// Measured from TMR0's current position rather than from the last tick boundary, so a sleep
/// armed with this ends at the deadline instead of up to one tick after it.
#[cfg(feature = "sleepy-executor")]
pub(crate) fn doze_ticks_until_next_wake(cs: critical_section::CriticalSection) -> Option<u32> {
    let next_alarm = DRIVER.next_alarm.borrow(cs).get()?;
    let ticks = next_alarm.saturating_sub(DRIVER.base.borrow(cs).get());
    // TMR0 counts from now to the deadline's tick boundary. `counts_before_sleep` includes any
    // whole ticks a pending TMR0 interrupt has not added to `base` yet.
    let counts = ticks
        .saturating_mul(u64::from(PERIOD))
        .saturating_sub(u64::from(counts_before_sleep(cs)));
    Some(u32::try_from(counts / u64::from(COUNTS_PER_DOZE_TICK)).unwrap_or(u32::MAX))
}

/// TMR0 counts since the last tick boundary, to hand to [`resync_after_sleep`].
///
/// Read this in `mc1322x_hal::sleep::sleep_with`'s `before_power_down` hook, as close as possible
/// to the clocks stopping: the partial tick (and any whole ticks a pending TMR0 interrupt has not
/// counted yet) belongs to the time the resync accounts for, and time between this read and
/// power-down is lost.
#[cfg(feature = "sleepy-executor")]
pub(crate) fn counts_before_sleep(cs: critical_section::CriticalSection) -> u16 {
    let cntr = unsafe { read16(TMR_REGOFF_CNTR) };
    cntr.wrapping_sub(DRIVER.last_boundary.borrow(cs).get())
}

/// Reconcile the time driver's tick count after CRM `Doze`, and reconfigure TMR0 from scratch.
///
/// `Doze` stops TMR0 along with every other clock except the sleep timer (RM §5.2.3, §5.3). The
/// sleep timer restarts from zero on sleep entry and keeps counting after wake, so its count
/// (`mc1322x_hal::sleep::timer_count()`) covers the whole sleep including the wake-up sequence.
/// Together with `counts_before_sleep` (from [`counts_before_sleep`]) it gives the time since the
/// last tick boundary: whole ticks are added to `base` and the remainder becomes the new TMR0
/// phase, so nothing is rounded away.
///
/// Call this right after `mc1322x_hal::sleep::sleep_with(SleepMode::Doze, ..)` returns, before
/// anything else reads the time.
///
/// Also wakes the timers that came due during the sleep right away. Leaving them to the next
/// TMR0 interrupt could let the executor read [`ticks_until_next_wake`] after that interrupt, see
/// the deadline after, and sleep through it with the woken task never polled.
#[cfg(feature = "sleepy-executor")]
pub(crate) fn resync_after_sleep(counts_before_sleep: u16) {
    critical_section::with(|cs| {
        // Restart TMR0 first and only then sample the sleep timer, together with TMR0's own
        // counter: the arithmetic below then runs while TMR0 is already counting, instead of
        // being time neither counter sees.
        configure_tmr0(cs);
        let cntr = unsafe { read16(TMR_REGOFF_CNTR) };
        let doze_ticks = mc1322x_hal::sleep::timer_count();

        // Time from the boundary before the sleep up to these two samples, in TMR0 counts, split
        // into whole ticks and the remainder (the new phase, so `last_boundary` lands that far
        // behind `cntr`). The sleep timer's count is rounded down to whole Doze ticks: add half a
        // tick so it is right on average. 32-bit division only (no 64-bit libcall).
        let rest =
            (doze_ticks % PERIOD) * COUNTS_PER_DOZE_TICK + COUNTS_PER_DOZE_TICK / 2 + u32::from(counts_before_sleep);
        let ticks = u64::from(doze_ticks / PERIOD) * u64::from(COUNTS_PER_DOZE_TICK) + u64::from(rest / PERIOD);
        let base = DRIVER.base.borrow(cs);
        base.set(base.get().wrapping_add(ticks));
        DRIVER
            .last_boundary
            .borrow(cs)
            .set(cntr.wrapping_sub((rest % PERIOD) as u16));
        DRIVER.arm_next_compare(cs);

        DRIVER.process_queue(cs);
    });
}

/// TMR0 interrupt handler, called by the ROM's interrupt dispatcher.
///
/// Overrides the weak `tmr0_isr` symbol declared in libmc1322x's `isr.h`.
#[unsafe(no_mangle)]
extern "C" fn tmr0_isr() {
    unsafe {
        // Clear the compare flag (writing `TCF1EN` alone writes 0 to `TCF1`).
        write16(TMR_REGOFF_SCTRL, 0);
        write16(TMR_REGOFF_CSCTRL, CSCTRL_TCF1EN);
    }

    DRIVER.on_tick();
}

impl TmrDriver {
    fn on_tick(&self) {
        critical_section::with(|cs| {
            let cntr = unsafe { read16(TMR_REGOFF_CNTR) };
            let base = self.base.borrow(cs);
            let last_boundary = self.last_boundary.borrow(cs);

            // Advance `base` by the number of whole periods elapsed since the last boundary. This
            // recovers ticks lost if the ISR was delayed by a held critical section. The boundary
            // moves by whole periods only, never to `cntr` itself: the counts between the real
            // boundary and this ISR (interrupt latency) belong to the next tick, and dropping
            // them every tick would make the clock run slow by latency / `PERIOD`.
            let elapsed = cntr.wrapping_sub(last_boundary.get()) / PERIOD as u16;
            base.set(base.get().wrapping_add(u64::from(elapsed)));
            last_boundary.set(last_boundary.get().wrapping_add(elapsed * PERIOD as u16));

            self.arm_next_compare(cs);
            self.process_queue(cs);
        });
    }

    /// Arm the compare for the boundary after `last_boundary`. If the counter is already past it
    /// by the time the write lands, the match is missed and the next interrupt would only come
    /// after a full 16-bit wrap: count that period here and arm the one after instead.
    fn arm_next_compare(&self, cs: critical_section::CriticalSection) {
        let base = self.base.borrow(cs);
        let last_boundary = self.last_boundary.borrow(cs);
        loop {
            let next = last_boundary.get().wrapping_add(PERIOD as u16);
            unsafe { write16(TMR_REGOFF_COMP1, next) };
            let cntr = unsafe { read16(TMR_REGOFF_CNTR) };
            if cntr.wrapping_sub(last_boundary.get()) < PERIOD as u16 {
                break;
            }
            base.set(base.get().wrapping_add(1));
            last_boundary.set(next);
        }
    }

    /// Wake every timer that has expired, and cache the (possibly now-updated) next deadline for
    /// [`ticks_until_next_wake`].
    fn process_queue(&self, cs: critical_section::CriticalSection) {
        let now = self.base.borrow(cs).get();
        let mut queue = self.queue.borrow(cs).borrow_mut();
        let next_alarm = queue.next_expiration(now);
        self.next_alarm
            .borrow(cs)
            .set((next_alarm != u64::MAX).then_some(next_alarm));
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: TmrDriver = TmrDriver::new());
