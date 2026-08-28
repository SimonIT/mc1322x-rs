//! The MC1322x's RTC block: a free-running 32-bit tick counter (`CRM->RTC_COUNT`), clocked by
//! either the internal ~2 kHz ring oscillator (calibrated against `REF_OSC` at startup) or an
//! external 32 kHz crystal.
//!
//! This is a counter, not a calendar: there is no year/month/day/hour/minute/second concept in
//! the hardware, only ticks since [`RtcRingOscillator::new`]/[`RtcCrystal::new`] was called. The
//! two types here mirror that split because it matters for how precisely each can report time:
//!
//! - [`RtcCrystal`] runs at an exact, compile-time-known 32 kHz.
//! - [`RtcRingOscillator`] runs at a *calibrated* rate nominally near 2 kHz, but the exact value
//!   ([`RtcRingOscillator::frequency_hz`]) is only known at runtime (it varies with temperature,
//!   voltage and part tolerance) and isn't available until after calibration.
//!
//! That distinction is why `embedded-time`'s `Clock` trait (feature = "embedded-time") is only
//! implemented for [`RtcCrystal`]: its `SCALING_FACTOR` must be a compile-time constant, which
//! the ring oscillator's calibrated rate isn't. `fugit` (feature = "fugit") supports both, since
//! it only needs the rate as a `now()` argument at the type level (a `Wrapping`-kind instant,
//! matching the fact that this 32-bit counter really does wrap: roughly every 1.5 days at 32 kHz,
//! or every 25 days at ~2 kHz).
//!
//! Both features only add conversions on top of the always-available raw
//! [`RtcCrystal::ticks`]/[`RtcRingOscillator::ticks`] API; neither is required to use this
//! module.

use mc1322x_sys::{CRM_BASE, rtc_calibrate, rtc_freq, rtc_init_osc};

/// `RTC_COUNT` register offset within the CRM block (see `libmc1322x`'s `crm.h`).
const RTC_COUNT: *const u32 = (CRM_BASE as usize + 0x28) as *const u32;

fn read_ticks() -> u32 {
    unsafe { RTC_COUNT.read_volatile() }
}

/// RTC clocked by the internal ~2 kHz ring oscillator.
///
/// [`Self::new`] calibrates the oscillator against `REF_OSC` (the same calibration
/// [`crate::delay::Delay`] runs), which takes a handful of milliseconds; the resulting
/// [`Self::frequency_hz`] is usually close to, but not exactly, 2000.
pub struct RtcRingOscillator {
    _private: (),
}

impl RtcRingOscillator {
    /// Start and calibrate the ring oscillator.
    pub fn new() -> Self {
        unsafe { rtc_init_osc(0) };
        Self { _private: () }
    }

    /// Re-run calibration (e.g. after a temperature change).
    pub fn recalibrate(&mut self) {
        unsafe { rtc_calibrate() };
    }

    /// The raw tick count. Wraps every `u32::MAX / `[`Self::frequency_hz`]` seconds — about 25
    /// days at the nominal ~2 kHz.
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

/// RTC clocked by an external 32 kHz crystal, exact (no calibration needed).
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
    /// Exact tick frequency: the crystal needs no calibration.
    pub const FREQUENCY_HZ: u32 = 32_000;

    /// Start the crystal oscillator. See the caveat on [`RtcCrystal`] before calling this.
    pub fn new() -> Self {
        unsafe { rtc_init_osc(1) };
        Self { _private: () }
    }

    /// The raw tick count. Wraps every `u32::MAX / `[`Self::FREQUENCY_HZ`]` seconds — about 1.5
    /// days.
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
    /// The raw counter as a `fugit` instant on a wrapping (not monotonic) timeline, at a
    /// **nominal** 2 kHz.
    ///
    /// The real calibrated rate ([`Self::frequency_hz`]) varies by several percent and can't be
    /// a `fugit` const generic; if you need the precise rate, scale [`Self::ticks`] by
    /// [`Self::frequency_hz`] yourself instead of using this.
    pub fn now(&self) -> fugit::WrappingTimerInstantU32<2_000> {
        fugit::WrappingTimerInstantU32::from_ticks(self.ticks())
    }
}

#[cfg(feature = "fugit")]
impl RtcCrystal {
    /// The raw counter as a `fugit` instant on a wrapping (not monotonic) timeline, at the
    /// exact crystal rate ([`Self::FREQUENCY_HZ`]).
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
// compile-time constant, and the ring oscillator's calibrated rate isn't one — see the module
// docs. Use [`RtcRingOscillator::now`] (feature = "fugit") or
// [`RtcRingOscillator::ticks`]/[`RtcRingOscillator::frequency_hz`] instead.
