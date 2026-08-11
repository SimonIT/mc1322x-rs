use core::hint::black_box;
use embedded_hal::delay::DelayNs;
use mc1322x_sys::{rtc_delay_ms, rtc_init_osc};

/// Core clock frequency in Hz. The MC1322x core runs at `REF_OSC`
/// (24 MHz) unless the PLL is enabled.
const CORE_CLOCK_HZ: u32 = 24_000_000;

/// Approximate number of core cycles spent per busy-wait iteration,
/// based on the compiled loop body.
const CYCLES_PER_ITERATION: u32 = 2;

/// RTC-based delay.
///
/// Milliseconds are measured against the RTC (started on the ~2 kHz ring
/// oscillator and calibrated by `rtc_init_osc`). Microsecond and nanosecond
/// delays use an approximate busy-wait derived from the core clock and may
/// therefore over-delay.
pub struct Delay;

impl Delay {
    /// Start and calibrate the RTC on the ring oscillator.
    pub fn new() -> Self {
        unsafe { rtc_init_osc(0) };
        Delay
    }
}

impl Default for Delay {
    fn default() -> Self {
        Self::new()
    }
}

/// Busy-wait for `iterations` loop iterations.
///
/// Kept in a separate function and `#[inline(never)]` so the loop body is
/// stable across call sites.
#[inline(never)]
fn spin(iterations: u32) {
    let mut n = iterations;
    while n > 0 {
        black_box(n);
        n -= 1;
    }
}

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // One core cycle at 24 MHz is ~41.7 ns; a loop iteration costs
        // CYCLES_PER_ITERATION cycles, so one iteration is ~83 ns.
        let cycles = (ns as u64 * CORE_CLOCK_HZ as u64) / 1_000_000_000;
        spin((cycles / CYCLES_PER_ITERATION as u64).min(u32::MAX as u64) as u32);
    }

    fn delay_us(&mut self, us: u32) {
        // 24 MHz / 2 cycles per iteration = 12 iterations per us.
        spin(us.saturating_mul(CORE_CLOCK_HZ / CYCLES_PER_ITERATION / 1_000_000));
    }

    fn delay_ms(&mut self, ms: u32) {
        unsafe { rtc_delay_ms(ms) };
    }
}
