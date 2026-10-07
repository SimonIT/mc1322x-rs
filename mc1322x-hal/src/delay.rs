//! Blocking and async delays based on the RTC and core-clock busy-waits.

use core::hint::black_box;
use core::task::Poll;
use embedded_hal::delay::DelayNs;
use mc1322x_sys::{
    CRM_BASE, INTBASE, INTENNUM_OFF, REF_OSC, interrupt_nums_INT_NUM_CRM, rtc_delay_ms, rtc_freq,
    rtc_init_osc,
};

use crate::util::WakerCell;

// ITC (interrupt controller) number of the CRM interrupt; `irq()` calls `rtc_isr` from its
// `INT_NUM_CRM` block.
const INT_NUM_CRM: u32 = interrupt_nums_INT_NUM_CRM;

/// Core clock frequency in Hz: `REF_OSC` (24 MHz), assuming the PLL is not enabled.
const CORE_CLOCK_HZ: u32 = REF_OSC;

/// Approximate number of core cycles per busy-wait iteration of [`spin`].
const CYCLES_PER_ITERATION: u32 = 2;

const WU_CNTL: *mut u32 = (CRM_BASE as usize + 0x04) as *mut u32;
const STATUS: *mut u32 = (CRM_BASE as usize + 0x18) as *mut u32;
const RTC_COUNT: *const u32 = (CRM_BASE as usize + 0x28) as *const u32;
const RTC_TIMEOUT: *mut u32 = (CRM_BASE as usize + 0x2c) as *mut u32;

// WU_CNTL/STATUS bits (RM Tables 5-7/5-13) used by the async `delay_ms`. The RTC time-out
// interrupt also works while awake (RM §5.9.12), so no sleep mode is involved.
const RTC_WU_EN: u32 = 1 << 1;
const RTC_WU_IEN: u32 = 1 << 17;
const RTC_WU_EVT: u32 = 1 << 3;

/// Waker for the in-flight async `delay_ms` call, if any.
static WAKER: WakerCell = WakerCell::new();

#[inline]
unsafe fn read_reg(reg: *mut u32) -> u32 {
    unsafe { reg.read_volatile() }
}

#[inline]
unsafe fn write_reg(reg: *mut u32, value: u32) {
    unsafe { reg.write_volatile(value) }
}

/// RTC-based delay.
///
/// Millisecond delays are measured with the RTC, running on the ~2 kHz ring oscillator
/// calibrated by `rtc_init_osc`. Microsecond and nanosecond delays busy-wait based on the 24 MHz
/// core clock; they are approximate and may over-delay.
///
/// The async `delay_ms` uses the RTC wake-up comparator (`RTC_TIMEOUT`), a single hardware
/// resource: don't run async millisecond delays from two [`Delay`] instances concurrently, and
/// don't combine them with an RTC wake source in [`crate::sleep`].
pub struct Delay;

impl Delay {
    /// Start and calibrate the RTC on the ring oscillator.
    ///
    /// The CRM interrupt is only enabled on the first async `delay_ms`; the blocking delays don't
    /// use interrupts.
    pub fn new() -> Self {
        unsafe {
            rtc_init_osc(0);
        }
        Delay
    }

    /// Enable the CRM interrupt in the ITC, so `irq()` dispatches to `rtc_isr`.
    ///
    /// Only needed by the async path ([`Delay::wait_rtc_ticks`]).
    fn ensure_crm_interrupt_enabled() {
        unsafe {
            core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_CRM);
        }
    }
}

impl Default for Delay {
    fn default() -> Self {
        Self::new()
    }
}

impl Delay {
    /// Wait `ticks` RTC ticks without busy-waiting.
    ///
    /// Each round disarms the comparator and clears a stale `RTC_WU_EVT` before reprogramming
    /// `RTC_TIMEOUT`, since the comparator is periodic rather than one-shot (RM §5.9.12). The
    /// check-then-arm sequence runs inside one critical section so an event between the check and
    /// enabling `RTC_WU_IEN` isn't missed.
    ///
    /// # Why this loops
    ///
    /// The comparator keeps a self-advancing periodic schedule independent of `RTC_WU_EN`, and
    /// a new `RTC_TIMEOUT` only takes effect after the next RTC clock (RM §5.9.12), so a
    /// re-armed wait can fire early on a boundary left over from the previous period.
    /// `RTC_COUNT` is the ground truth: after each wake, the wait is re-armed for whatever is
    /// left until the full duration has elapsed.
    async fn wait_rtc_ticks(&mut self, ticks: u32) {
        Self::ensure_crm_interrupt_enabled();
        let anchor = unsafe { RTC_COUNT.read_volatile() };
        let mut remaining = ticks.max(1);
        // Held across all re-arm iterations, so a sleep-aware executor never sleeps mid-wait.
        let mut inhibit = None;
        loop {
            unsafe {
                write_reg(WU_CNTL, read_reg(WU_CNTL) & !(RTC_WU_EN | RTC_WU_IEN));
                write_reg(STATUS, RTC_WU_EVT);
                write_reg(RTC_TIMEOUT, remaining);
            }
            core::future::poll_fn(|cx| {
                critical_section::with(|cs| {
                    if unsafe { read_reg(STATUS) } & RTC_WU_EVT != 0 {
                        return Poll::Ready(());
                    }
                    inhibit.get_or_insert_with(crate::sleep::SleepInhibitGuard::new);
                    WAKER.set(cs, cx.waker());
                    unsafe {
                        write_reg(WU_CNTL, read_reg(WU_CNTL) | RTC_WU_EN | RTC_WU_IEN);
                    }
                    Poll::Pending
                })
            })
            .await;

            let elapsed = unsafe { RTC_COUNT.read_volatile() }.wrapping_sub(anchor);
            if elapsed >= ticks {
                return;
            }
            remaining = ticks - elapsed;
        }
    }
}

/// Busy-wait for `iterations` loop iterations.
///
/// `#[inline(never)]` keeps the loop body, and thus [`CYCLES_PER_ITERATION`], the same at every
/// call site.
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
        // One core cycle at 24 MHz is ~41.7 ns, so one iteration is ~83 ns.
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

/// Async delays.
///
/// `delay_ns`/`delay_us` busy-wait exactly like the blocking versions: the ~500 µs RTC tick is
/// far too coarse for sub-millisecond delays. Only `delay_ms` waits on the RTC interrupt and
/// yields to the executor; it rounds down to whole RTC ticks (waiting at least one) and inhibits
/// a sleep-aware executor from sleeping while waiting (see [`crate::sleep::SleepInhibitGuard`]).
impl embedded_hal_async::delay::DelayNs for Delay {
    async fn delay_ns(&mut self, ns: u32) {
        DelayNs::delay_ns(self, ns);
    }

    async fn delay_us(&mut self, us: u32) {
        DelayNs::delay_us(self, us);
    }

    async fn delay_ms(&mut self, ms: u32) {
        // `rtc_freq` is the calibrated RTC rate in Hz; wait at least one tick.
        let ticks = ((ms as u64 * unsafe { rtc_freq as u32 } as u64) / 1000).max(1) as u32;
        self.wait_rtc_ticks(ticks).await;
    }
}

/// RTC wake-up time-out interrupt handler, overriding the weak `rtc_isr` symbol from
/// `libmc1322x`'s `isr.h`. `irq()` calls it from its `INT_NUM_CRM` block when `rtc_wu_evt()` is
/// set.
///
/// Masks `RTC_WU_IEN` and leaves the periodic comparator running; per RM §5.9.2 disabling the
/// interrupt enable also retracts the pending request. `STATUS.RTC_WU_EVT` is cleared by the next
/// [`Delay::wait_rtc_ticks`] call.
#[unsafe(no_mangle)]
extern "C" fn rtc_isr() {
    unsafe {
        write_reg(WU_CNTL, read_reg(WU_CNTL) & !RTC_WU_IEN);
    }
    WAKER.wake();
}
