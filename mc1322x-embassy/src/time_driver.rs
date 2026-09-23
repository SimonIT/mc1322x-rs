//! `embassy-time` driver based on the TMR0 timer, with a 1 kHz tick.
//!
//! TMR0 runs from the 24 MHz reference oscillator, prescaled by [`PRESCALE`], and is configured
//! as a free-running 16-bit counter. A compare interrupt is re-armed for every [`PERIOD`] counts,
//! i.e. exactly 1000 times per second. Each interrupt advances a 64-bit tick counter (`base`).
//!
//! `now()` is derived from the live counter value so that it stays correct even when the ISR is
//! delayed by a held critical section: if the ISR was late, the number of elapsed whole periods
//! since the last interrupt is recovered from the counter delta.
//!
//! # Maximum interrupt-disabled duration
//!
//! Recovering missed periods from the counter delta only works up to one full wrap of the
//! 16-bit counter: if interrupts stay disabled for longer than `65536 / (REF_OSC_HZ / PRESCALE)`,
//! the delta aliases and both `now()` and the missed-tick catch-up in [`TmrDriver::on_tick`]
//! silently return a too-small elapsed time. [`PRESCALE`] is chosen to keep that bound at
//! `65536 * PRESCALE / REF_OSC_HZ` ≈ 175 ms (see its doc comment), comfortably above any
//! critical section held anywhere in this dependency graph (packet copies, register pokes), while
//! keeping `PERIOD` an exact integer (no long-run drift).
//!
//! The ISR is delivered through the boot ROM's interrupt dispatcher, which calls the weak symbol
//! `tmr0_isr` (declared in the reference `isr.h`). We mirror the register writes of the reference
//! test `tmr-ints.c` exactly (`*TMR0_SCTRL = 0; *TMR0_CSCTRL = 0x0040;`) to clear the compare
//! flag.
//!
//! # Caveats
//!
//! The ROM dispatcher must use interworking (`bx`) to call the ISR, because Rust code is compiled
//! to Thumb while the ROM runs ARM state. This is the same requirement the reference C ISRs have.

use core::cell::{Cell, RefCell};
use core::sync::atomic::Ordering;
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use mc1322x_sys::{
    INTBASE, TMR_REGOFF_CNTR, TMR_REGOFF_COMP1, TMR_REGOFF_CSCTRL, TMR_REGOFF_CTRL,
    TMR_REGOFF_ENBL, TMR_REGOFF_SCTRL, TMR0_BASE,
};
use portable_atomic::AtomicBool;

/// Reference oscillator frequency (Hz).
const REF_OSC_HZ: u32 = 24_000_000;
/// Tick rate of the timebase (matches the `tick-hz-1_000` feature).
const TICK_HZ: u32 = 1_000;
/// `log2` of the TMR0 input prescaler, i.e. [`PRESCALE`] `= 2.pow(PRESCALE_SHIFT)`.
///
/// The largest power-of-two divisor of `REF_OSC_HZ / TICK_HZ` (24_000) is 64: this both keeps
/// `PERIOD` an exact integer (no rounding drift between the tick rate and real time) and
/// maximizes the 16-bit counter's wraparound margin (see the module docs), at 64x the margin a
/// prescaler of 1 would give.
const PRESCALE_SHIFT: u32 = 6;
/// TMR0 input prescaler: the counter runs at `REF_OSC_HZ / PRESCALE`.
const PRESCALE: u32 = 1 << PRESCALE_SHIFT;
/// Counter counts per tick.
const PERIOD: u32 = REF_OSC_HZ / PRESCALE / TICK_HZ;

/// Interrupt number of the TMR block in the ITC (see `isr.h` `enum interrupt_nums`).
const INT_NUM_TMR: u32 = 5;
/// `INTENNUM` register offset in the ITC: write an interrupt number to enable it.
const INTENNUM_OFF: u32 = 0x8;

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
    /// Cached result of the queue's own `next_expiration` (`u64::MAX` if nothing is scheduled) -
    /// recomputed every tick in [`TmrDriver::on_tick`], and speculatively tightened (never
    /// loosened) by [`Driver::schedule_wake`] itself so a newly-scheduled, sooner deadline is
    /// visible immediately rather than waiting up to one tick. Backs [`ticks_until_next_wake`],
    /// which a sleep-aware executor (`mc1322x_embassy::SleepyExecutor`) uses to decide how long
    /// it's safe to sleep.
    next_alarm: Mutex<Cell<u64>>,
}

impl TmrDriver {
    const fn new() -> Self {
        Self {
            base: Mutex::new(Cell::new(0)),
            last_boundary: Mutex::new(Cell::new(0)),
            queue: Mutex::new(RefCell::new(Queue::new())),
            started: AtomicBool::new(false),
            next_alarm: Mutex::new(Cell::new(u64::MAX)),
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
            // Tighten (never loosen) the cached next-alarm estimate immediately: `on_tick`
            // will recompute the true value from the queue within one tick regardless, but a
            // sleep-aware executor calling `ticks_until_next_wake` between now and then should
            // still see this new, sooner deadline rather than a stale, larger one.
            let next_alarm = self.next_alarm.borrow(cs);
            next_alarm.set(next_alarm.get().min(at));
        })
    }
}

/// Reset and (re)configure TMR0 for a free-running 1 kHz compare interrupt, and reset
/// [`TmrDriver::last_boundary`] to match. Shared by [`init`] (first bring-up) and
/// [`resync_after_sleep`] (TMR0 does not survive `Doze`/`Hibernate` - only the dedicated sleep
/// timer does - so its registers need reconfiguring from scratch on every wake, the same as at
/// boot). Must run inside a `critical_section` (both callers already are one).
fn configure_tmr0(cs: critical_section::CriticalSection) {
    unsafe {
        // Reset the timer first.
        write16(TMR_REGOFF_ENBL, 0);
        write16(TMR_REGOFF_SCTRL, 0);
        // Enable the compare-1 interrupt and clear the compare flag (mirrors tmr-ints.c).
        write16(TMR_REGOFF_CSCTRL, 0x0040);
        write16(TMR_REGOFF_CNTR, 0);

        // CTRL = COUNT_MODE=1 (count rising edges of primary source)
        //      | PRIMARY_CNT_SOURCE=8+PRESCALE_SHIFT (prescaler /2.pow(PRESCALE_SHIFT))
        //      | LENGTH=0 (free-running)
        write16(
            TMR_REGOFF_CTRL,
            (1 << 13) | ((8 + PRESCALE_SHIFT as u16) << 9),
        );

        // Free-run counter, armed to fire every PERIOD counts.
        let boundary = read16(TMR_REGOFF_CNTR);
        DRIVER.last_boundary.borrow(cs).set(boundary);
        write16(TMR_REGOFF_COMP1, boundary.wrapping_add(PERIOD as u16));

        // Enable TMR0. `TMR_ENBL` is a single shared register for all four TMR channels
        // ("one enable register to rule them all", per libmc1322x's tmr.h) - the reference
        // `tmr-ints.c` test writes 0xf (all four channels) rather than just this channel's
        // bit; matching that here matters (bit 0 alone leaves the counter not running).
        write16(TMR_REGOFF_ENBL, 0x0f);
    }
}

/// Initialize the time driver (idempotent).
///
/// This configures TMR0 for a 1 kHz compare interrupt and enables the TMR interrupt in the ITC.
/// It must be called once, with interrupts in a known state, before using `embassy-time`.
/// [`crate::init`] does this.
pub fn init() {
    if DRIVER.started.swap(true, Ordering::AcqRel) {
        return;
    }

    critical_section::with(configure_tmr0);

    // Enable the TMR interrupt in the ITC - deliberately *outside* the critical section above:
    // this crate's `critical_section` impl (`mc1322x_hal::critical_section_impl`) saves
    // `INTENABLE` on entry and unconditionally restores that saved value on exit, so a source
    // enabled from inside a critical section gets silently un-enabled the moment it ends.
    unsafe {
        core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_TMR);
    }
}

/// Ticks until the earliest scheduled wake, if any is currently pending.
///
/// Backs a sleep-aware executor's (`mc1322x_embassy::SleepyExecutor`) decision of whether, and
/// for how long, it's safe to enter CRM sleep - see [`TmrDriver::next_alarm`]'s doc comment for
/// how this stays fresh (recomputed every tick, tightened immediately on every new
/// `schedule_wake`).
pub(crate) fn ticks_until_next_wake() -> Option<u64> {
    let next_alarm = critical_section::with(|cs| DRIVER.next_alarm.borrow(cs).get());
    if next_alarm == u64::MAX {
        return None;
    }
    Some(next_alarm.saturating_sub(DRIVER.now()))
}

/// Reconcile the time driver's tick count after CRM sleep, and reconfigure TMR0 from scratch.
///
/// Doze/Hibernate power down everything except the dedicated sleep timer (RM §5.2.3/§5.3) -
/// TMR0 itself does not survive, so its registers are meaningless on wake and its counter does
/// not reflect elapsed time. `elapsed_ticks` (in the same 1 kHz units as [`Driver::now`]) must
/// therefore come from the sleep duration the caller itself chose and armed as the CRM wake-up
/// timeout, not be re-derived from TMR0's post-wake state. Call this immediately after
/// `mc1322x_hal::sleep::sleep(..)` returns `WakeReason::Timer`, before anything else reads the
/// time - `base` is advanced and TMR0 reconfigured in one critical section so no `now()`/
/// `schedule_wake()` caller can observe a "time went backwards" or "no time passed" window.
pub(crate) fn resync_after_sleep(elapsed_ticks: u64) {
    critical_section::with(|cs| {
        let base = DRIVER.base.borrow(cs);
        base.set(base.get().wrapping_add(elapsed_ticks));
        configure_tmr0(cs);
    });
}

/// TMR0 interrupt handler, called by the ROM's interrupt dispatcher.
///
/// This is the weak symbol declared in the reference `isr.h`; the `#[unsafe(no_mangle)]`
/// definition here overrides it.
#[unsafe(no_mangle)]
extern "C" fn tmr0_isr() {
    unsafe {
        // Clear the compare flag, exactly like the reference tmr-ints.c ISR.
        write16(TMR_REGOFF_SCTRL, 0);
        write16(TMR_REGOFF_CSCTRL, 0x0040);
    }

    DRIVER.on_tick();
}

impl TmrDriver {
    fn on_tick(&self) {
        critical_section::with(|cs| {
            let cntr = unsafe { read16(TMR_REGOFF_CNTR) };
            let boundary = self.last_boundary.borrow(cs).get();

            // Advance `base` by the number of whole periods elapsed since the last ISR. This
            // recovers ticks lost if the ISR was delayed by a held critical section.
            let since = (cntr.wrapping_sub(boundary)) as u32;
            let elapsed = since / (PERIOD as u16 as u32);
            if elapsed > 0 {
                let base = self.base.borrow(cs);
                base.set(base.get().wrapping_add(u64::from(elapsed)));
                self.last_boundary.borrow(cs).set(cntr);
            }

            // Re-arm the compare for the next period.
            unsafe { write16(TMR_REGOFF_COMP1, cntr.wrapping_add(PERIOD as u16)) };

            // Wake every timer that has expired, and cache the (possibly now-updated) next
            // deadline for `ticks_until_next_wake`.
            let now = self.base.borrow(cs).get();
            let mut queue = self.queue.borrow(cs).borrow_mut();
            let next_alarm = queue.next_expiration(now);
            self.next_alarm.borrow(cs).set(next_alarm);
        });
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: TmrDriver = TmrDriver::new());
