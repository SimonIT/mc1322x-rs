//! The CRM's COP ("Computer Operating Properly") watchdog timer (RM §5.6, §5.9.5-5.9.6).
//!
//! Once [`Watchdog::start`]ed, software must [`Watchdog::feed`] it more often than the chosen
//! [`Timeout`] (87 ms to ~11.2 s, in 128 steps of ~87 ms at the 24 MHz `REF_OSC`), or the chip
//! goes through a complete power-on reset - including the ROM boot sequence, so what runs
//! afterwards is whatever the ROM boots next (the NVM image, or UART/SPI boot), not whatever was
//! loaded into RAM over JTAG.
//!
//! [`Watchdog::lock`] sets `COP_WP`, after which `COP_CNTL` is read-only until the next reset:
//! the watchdog can then no longer be stopped or reconfigured, even by code that has gone
//! astray - the type it returns ([`LockedWatchdog`]) only offers [`LockedWatchdog::feed`].
//!
//! # Only the reset action is supported
//!
//! The hardware can alternatively raise `STATUS.COP_EVT` as a CRM interrupt instead of
//! resetting (`COP_OUT = 1`). This driver always programs `COP_OUT = 0` (reset): `libmc1322x`'s
//! `isr.c` dispatches the `INT_NUM_CRM` interrupt to RTC, KBI and calibration handlers but has
//! no hook for `COP_EVT`, so with that interrupt enabled in the ITC (as [`crate::delay`] and
//! [`crate::gpio`] do) a COP interrupt would never be acknowledged.
//!
//! # Caveats
//!
//! - The RM states the COP "is disabled for ARM debug mode", i.e. it does not count while the
//!   core is halted by a debugger. Merely having a debugger attached doesn't stop it while
//!   the core runs.
//! - The COP does not run during [`crate::sleep::sleep`]. Without `Retention::mcu`, `COP_CNTL`
//!   is reset on wake like every other register, so the watchdog comes back stopped. With
//!   `Retention::mcu` the counter is frozen during sleep and carries on from where it stopped
//!   on wake, so feed it right before sleeping and right after waking. (RM §5.6)

use mc1322x_sys::{CRM_BASE, REF_OSC};

const COP_CNTL: *mut u32 = (CRM_BASE as usize + 0x10) as *mut u32;
const COP_SERVICE: *mut u32 = (CRM_BASE as usize + 0x14) as *mut u32;

/// Value that must be written (as one 32-bit access) to `COP_SERVICE` to restart the count.
const SERVICE_KEY: u32 = 0xC0DE_5AFE;

bitflags::bitflags! {
    /// `COP_CNTL` plain flag bits (RM Table 5-10). `COP_TIMEOUT[6:0]` and `COP_COUNT[6:0]` are
    /// 7-bit sub-fields rather than flags - see [`TIMEOUT_SHIFT`]/[`COUNT_SHIFT`].
    #[derive(Clone, Copy, PartialEq, Eq)]
    struct CopCntl: u32 {
        const COP_EN = 1 << 0;
        const COP_OUT = 1 << 1;
        const COP_WP = 1 << 2;
    }
}
const TIMEOUT_SHIFT: u32 = 8;
const COUNT_SHIFT: u32 = 16;
const FIELD_MASK: u32 = 0x7F;

/// `REF_OSC` cycles per millisecond. Each COP step is `2^21` `REF_OSC` cycles (RM Table 5-11:
/// period = `(COP_TIMEOUT + 1) * 2^21 / RefXTALFreq`).
const REF_CYCLES_PER_MS: u32 = REF_OSC / 1000;
const _: () = assert!(
    REF_OSC.is_multiple_of(1000),
    "Timeout math assumes a whole-kHz REF_OSC"
);
const STEP_SHIFT: u32 = 21;

#[inline]
fn read_cntl() -> u32 {
    unsafe { COP_CNTL.read_volatile() }
}

#[inline]
fn write_cntl(value: u32) {
    unsafe { COP_CNTL.write_volatile(value) }
}

#[inline]
fn service() {
    unsafe { COP_SERVICE.write_volatile(SERVICE_KEY) }
}

/// A COP time-out period: one of the hardware's 128 `COP_TIMEOUT` settings.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd)]
pub struct Timeout(u8);

impl Timeout {
    /// Shortest period, ~87 ms.
    pub const MIN: Self = Self(0);
    /// Longest period, ~11.18 s.
    pub const MAX: Self = Self(FIELD_MASK as u8);

    /// The shortest period that is at least `ms` milliseconds, or `None` if `ms` is longer than
    /// [`Self::MAX`]. `ms` shorter than [`Self::MIN`] gives [`Self::MIN`].
    pub const fn from_millis(ms: u32) -> Option<Self> {
        // Early bound keeps `ms * REF_CYCLES_PER_MS` from overflowing; anything above it is
        // far past `MAX` anyway.
        if ms > 2 * Self::MAX.as_millis() {
            return None;
        }
        let cycles = ms * REF_CYCLES_PER_MS;
        let steps = cycles.div_ceil(1 << STEP_SHIFT);
        let steps = if steps == 0 { 1 } else { steps };
        if steps > FIELD_MASK + 1 {
            None
        } else {
            Some(Self((steps - 1) as u8))
        }
    }

    /// Like [`Self::from_millis`], but anything longer than [`Self::MAX`] gives [`Self::MAX`].
    pub const fn from_millis_saturating(ms: u32) -> Self {
        match Self::from_millis(ms) {
            Some(timeout) => timeout,
            None => Self::MAX,
        }
    }

    /// From a raw `COP_TIMEOUT[6:0]` value (0..=127), or `None` if out of range.
    pub const fn from_raw(raw: u8) -> Option<Self> {
        if raw as u32 > FIELD_MASK {
            None
        } else {
            Some(Self(raw))
        }
    }

    /// The raw `COP_TIMEOUT[6:0]` value.
    pub const fn raw(self) -> u8 {
        self.0
    }

    /// The period in milliseconds, rounded down.
    pub const fn as_millis(self) -> u32 {
        ((self.0 as u32 + 1) << STEP_SHIFT) / REF_CYCLES_PER_MS
    }
}

/// `COP_CNTL` has been write-protected by [`Watchdog::lock`] and can't be changed until the
/// next reset.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Locked;

/// The COP watchdog. See the module docs.
///
/// [`Self::new`] doesn't touch the hardware: the watchdog stays in whatever state it was in
/// (stopped after any reset).
pub struct Watchdog {
    _private: (),
}

impl Watchdog {
    /// Create a handle to the COP. Doesn't touch the hardware.
    pub fn new() -> Self {
        Self { _private: () }
    }

    /// (Re)start the watchdog with `timeout`, set to reset the chip when it expires. The count
    /// starts from zero.
    ///
    /// If it's already running, it's stopped first, following the RM's procedure for changing
    /// `COP_TIMEOUT` (RM §5.9.5: disable, write the time-out, re-enable - writing the time-out
    /// alone doesn't reset the counter).
    ///
    /// # Errors
    ///
    /// Returns [`Locked`] if [`Self::lock`] has been called since the last reset.
    pub fn start(&mut self, timeout: Timeout) -> Result<(), Locked> {
        self.stop()?;
        let timeout = (timeout.0 as u32) << TIMEOUT_SHIFT;
        write_cntl(timeout);
        // Restart the count before enabling too, in case the stopped counter still holds an
        // old value past the new time-out.
        service();
        write_cntl(timeout | CopCntl::COP_EN.bits());
        service();
        Ok(())
    }

    /// Stop the watchdog. Does nothing if it's already stopped.
    ///
    /// # Errors
    ///
    /// Returns [`Locked`] if [`Self::lock`] has been called since the last reset.
    pub fn stop(&mut self) -> Result<(), Locked> {
        let cntl = read_cntl();
        if cntl & CopCntl::COP_WP.bits() != 0 {
            return Err(Locked);
        }
        if cntl & CopCntl::COP_EN.bits() != 0 {
            write_cntl(cntl & (FIELD_MASK << TIMEOUT_SHIFT));
        }
        Ok(())
    }

    /// Restart the count. Must be called more often than the configured [`Timeout`] while the
    /// watchdog is running; harmless while stopped.
    pub fn feed(&mut self) {
        service();
    }

    /// Whether the watchdog is currently running.
    pub fn is_running(&self) -> bool {
        read_cntl() & CopCntl::COP_EN.bits() != 0
    }

    /// Whether [`Self::lock`] has been called since the last reset.
    pub fn is_locked(&self) -> bool {
        read_cntl() & CopCntl::COP_WP.bits() != 0
    }

    /// The configured time-out.
    pub fn timeout(&self) -> Timeout {
        Timeout(((read_cntl() >> TIMEOUT_SHIFT) & FIELD_MASK) as u8)
    }

    /// Current `COP_COUNT[6:0]`: whole ~87 ms steps elapsed since the last feed.
    pub fn count(&self) -> u8 {
        count()
    }

    /// Write-protect `COP_CNTL` until the next reset, freezing the watchdog's current
    /// configuration - running or not. Once locked, a running watchdog can only be fed.
    pub fn lock(self) -> LockedWatchdog {
        write_cntl(read_cntl() | CopCntl::COP_WP.bits());
        LockedWatchdog { _private: () }
    }
}

impl Default for Watchdog {
    fn default() -> Self {
        Self::new()
    }
}

/// A watchdog whose configuration [`Watchdog::lock`] has frozen until the next reset.
pub struct LockedWatchdog {
    _private: (),
}

impl LockedWatchdog {
    /// See [`Watchdog::feed`].
    pub fn feed(&mut self) {
        service();
    }

    /// See [`Watchdog::count`].
    pub fn count(&self) -> u8 {
        count()
    }
}

fn count() -> u8 {
    ((read_cntl() >> COUNT_SHIFT) & FIELD_MASK) as u8
}

/// [`task-watchdog`](https://docs.rs/task-watchdog) backend (feature = "task-watchdog"): lets
/// its task multiplexer drive the COP, on any [`task_watchdog::Clock`] whose durations convert
/// to [`core::time::Duration`] - e.g. `embassy_time::Duration`, as `mc1322x-embassy`'s
/// `task_watchdog` module uses.
///
/// The trait's `start`/`feed` share names with [`Watchdog::start`]/[`Watchdog::feed`]; method
/// syntax on a `Watchdog` still picks the inherent ones.
#[cfg(feature = "task-watchdog")]
impl<C> task_watchdog::HardwareWatchdog<C> for Watchdog
where
    C: task_watchdog::Clock,
    C::Duration: Into<core::time::Duration>,
{
    /// # Panics
    ///
    /// If `timeout` is longer than [`Timeout::MAX`], or the COP has been [locked](Self::lock) -
    /// both configuration errors that would otherwise silently leave the chip resetting on a
    /// different schedule than asked for.
    fn start(&mut self, timeout: C::Duration) {
        let timeout: core::time::Duration = timeout.into();
        // Via seconds rather than `as_millis()`, which is `u128` arithmetic.
        let ms = u32::try_from(timeout.as_secs())
            .ok()
            .and_then(|secs| secs.checked_mul(1000))
            .and_then(|ms| ms.checked_add(timeout.subsec_millis()))
            .unwrap_or(u32::MAX);
        let timeout = Timeout::from_millis(ms).expect("watchdog timeout longer than Timeout::MAX");
        Watchdog::start(self, timeout).expect("COP watchdog is locked");
    }

    fn feed(&mut self) {
        Watchdog::feed(self);
    }

    fn trigger_reset(&mut self) -> ! {
        crate::reset::software_reset()
    }

    /// Always `None`: the MC1322x has no reset-cause register.
    fn reset_reason(&self) -> Option<task_watchdog::ResetReason> {
        None
    }
}
