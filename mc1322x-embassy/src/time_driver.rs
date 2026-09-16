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
//! to Thumb while the ROM runs ARM state. This is the same requirement the reference C ISRs have
//! and must be validated on hardware.

use core::cell::{Cell, RefCell};
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use mc1322x_sys::{
    INTBASE, TMR_REGOFF_CNTR, TMR_REGOFF_COMP1, TMR_REGOFF_CSCTRL, TMR_REGOFF_CTRL,
    TMR_REGOFF_ENBL, TMR_REGOFF_SCTRL, TMR0_BASE,
};

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
    started: Mutex<Cell<bool>>,
}

impl TmrDriver {
    const fn new() -> Self {
        Self {
            base: Mutex::new(Cell::new(0)),
            last_boundary: Mutex::new(Cell::new(0)),
            queue: Mutex::new(RefCell::new(Queue::new())),
            started: Mutex::new(Cell::new(false)),
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
        })
    }
}

/// Initialize the time driver (idempotent).
///
/// This configures TMR0 for a 1 kHz compare interrupt and enables the TMR interrupt in the ITC.
/// It must be called once, with interrupts in a known state, before using `embassy-time`.
/// [`crate::init`] does this.
pub fn init() {
    let already_started = critical_section::with(|cs| {
        let started = DRIVER.started.borrow(cs);
        if started.get() {
            return true;
        }
        started.set(true);

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
            // bit; matching that here mattered on real hardware (bit 0 alone left the counter
            // not running).
            write16(TMR_REGOFF_ENBL, 0x0f);
        }
        false
    });
    if already_started {
        return;
    }

    // Enable the TMR interrupt in the ITC - deliberately *outside* the critical section above:
    // this crate's `critical_section` impl (`mc1322x_hal::critical_section_impl`) saves
    // `INTENABLE` on entry and unconditionally restores that saved value on exit, so a source
    // enabled from inside a critical section gets silently un-enabled the moment it ends.
    unsafe {
        core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_TMR);
    }
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

            // Wake every timer that has expired.
            let now = self.base.borrow(cs).get();
            let mut queue = self.queue.borrow(cs).borrow_mut();
            queue.next_expiration(now);
        });
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: TmrDriver = TmrDriver::new());
