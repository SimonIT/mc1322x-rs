//! The RTC: a free-running 32-bit tick counter (`CRM->RTC_COUNT`), clocked by either the
//! internal ~2 kHz ring oscillator or an external 32 kHz crystal.
//!
//! This is a counter, not a calendar: it only counts ticks since [`RtcRingOscillator::new`] or
//! [`RtcCrystal::new`] started the oscillator.
//!
//! - [`RtcCrystal`] ticks at the fixed [`RtcCrystal::FREQUENCY_HZ`].
//! - [`RtcRingOscillator`] ticks at a rate calibrated against the reference oscillator at
//!   startup. It's only known at runtime ([`RtcRingOscillator::frequency_hz`]) and varies with
//!   temperature, voltage and part tolerance.
//!
//! # Features
//!
//! - `fugit`: `now()` on both types, returning a wrapping `fugit` instant (the 32-bit counter
//!   wraps about every 1.5 days at 32 kHz, or every 25 days at ~2 kHz).
//! - `embedded-time`: `embedded_time::Clock` for [`RtcCrystal`] only, since the trait's
//!   `SCALING_FACTOR` must be a compile-time constant.

use mc1322x_sys::{CRM_BASE, rtc_calibrate, rtc_freq, rtc_init_osc};

/// `RTC_COUNT` register offset within the CRM block (see `libmc1322x`'s `crm.h`).
const RTC_COUNT: *const u32 = (CRM_BASE as usize + 0x28) as *const u32;

fn read_ticks() -> u32 {
    unsafe { RTC_COUNT.read_volatile() }
}

/// RTC clocked by the internal ~2 kHz ring oscillator.
///
/// [`Self::new`] calibrates the oscillator against `REF_OSC` (as [`crate::delay::Delay::new`]
/// does), which takes a few milliseconds. The resulting [`Self::frequency_hz`] is close to, but
/// not exactly, 2000.
pub struct RtcRingOscillator {
    _private: (),
}

impl RtcRingOscillator {
    /// Start and calibrate the ring oscillator.
    ///
    /// Doesn't give a working RTC once [`RtcCrystal::new`] has started the crystal oscillator;
    /// see the caveat on [`RtcCrystal`].
    pub fn new() -> Self {
        unsafe { rtc_init_osc(0) };
        Self { _private: () }
    }

    /// Re-run calibration (e.g. after a temperature change).
    pub fn recalibrate(&mut self) {
        unsafe { rtc_calibrate() };
    }

    /// The raw tick count. Wraps about every 25 days at ~2 kHz.
    pub fn ticks(&self) -> u32 {
        read_ticks()
    }

    /// The calibrated tick frequency, in Hz.
    pub fn frequency_hz(&self) -> u32 {
        unsafe { rtc_freq as u32 }
    }
}

impl Default for RtcRingOscillator {
    fn default() -> Self {
        Self::new()
    }
}

/// RTC clocked by an external 32 kHz crystal. Needs no calibration.
///
/// # Caveats
///
/// Per `libmc1322x`'s `rtc.c`: once started, the crystal oscillator can only be stopped by a
/// full chip reset (hard or soft) — there is no way to switch back to the ring oscillator, or to
/// construct a working [`RtcRingOscillator`], afterwards without one.
pub struct RtcCrystal {
    _private: (),
}

impl RtcCrystal {
    /// Tick frequency in Hz (the value `libmc1322x` uses for the crystal).
    pub const FREQUENCY_HZ: u32 = 32_000;

    /// Start the crystal oscillator. See the caveat on [`RtcCrystal`] before calling this.
    pub fn new() -> Self {
        unsafe { rtc_init_osc(1) };
        Self { _private: () }
    }

    /// The raw tick count. Wraps about every 1.5 days.
    pub fn ticks(&self) -> u32 {
        read_ticks()
    }
}

impl Default for RtcCrystal {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "fugit")]
impl RtcRingOscillator {
    /// The tick count as a wrapping `fugit` instant at a **nominal** 2 kHz.
    ///
    /// The calibrated rate ([`Self::frequency_hz`]) differs by several percent and can't be a
    /// const generic. For precise timing, scale [`Self::ticks`] by [`Self::frequency_hz`]
    /// instead.
    pub fn now(&self) -> fugit::WrappingTimerInstantU32<2_000> {
        fugit::WrappingTimerInstantU32::from_ticks(self.ticks())
    }
}

#[cfg(feature = "fugit")]
impl RtcCrystal {
    /// The tick count as a wrapping `fugit` instant at [`Self::FREQUENCY_HZ`].
    pub fn now(&self) -> fugit::WrappingTimerInstantU32<32_000> {
        fugit::WrappingTimerInstantU32::from_ticks(self.ticks())
    }
}

#[cfg(feature = "embedded-time")]
impl embedded_time::Clock for RtcCrystal {
    type T = u32;

    const SCALING_FACTOR: embedded_time::fraction::Fraction =
        embedded_time::fraction::Fraction::new(1, Self::FREQUENCY_HZ);

    fn try_now(&self) -> Result<embedded_time::Instant<Self>, embedded_time::clock::Error> {
        Ok(embedded_time::Instant::new(self.ticks()))
    }
}

// No `embedded_time::Clock` impl for `RtcRingOscillator`: `SCALING_FACTOR` must be a
// compile-time constant, and the ring oscillator's calibrated rate isn't one.
