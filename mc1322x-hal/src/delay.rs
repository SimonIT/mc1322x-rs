use core::cell::RefCell;
use core::hint::black_box;
use core::task::{Poll, Waker};
use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use mc1322x_sys::{CRM_BASE, INTBASE, REF_OSC, rtc_delay_ms, rtc_freq, rtc_init_osc};

// ITC (interrupt controller) offset/number for the CRM interrupt (`rtc_isr` is dispatched from
// within `irq()`'s `INT_NUM_CRM` block - see `isr.h`'s `INTENNUM_OFF` and `interrupt_nums`),
// following the same wiring as `crate::i2c`'s `INT_NUM_I2C`/`crate::spi`'s `INT_NUM_SPI`.
const INTENNUM_OFF: u32 = 0x8;
const INT_NUM_CRM: u32 = 3;

/// Core clock frequency in Hz. The MC1322x core runs at `REF_OSC`
/// (24 MHz) unless the PLL is enabled.
const CORE_CLOCK_HZ: u32 = REF_OSC;

/// Approximate number of core cycles spent per busy-wait iteration,
/// based on the compiled loop body.
const CYCLES_PER_ITERATION: u32 = 2;

const WU_CNTL: *mut u32 = (CRM_BASE as usize + 0x04) as *mut u32;
const STATUS: *mut u32 = (CRM_BASE as usize + 0x18) as *mut u32;
const RTC_COUNT: *const u32 = (CRM_BASE as usize + 0x28) as *const u32;
const RTC_TIMEOUT: *mut u32 = (CRM_BASE as usize + 0x2c) as *mut u32;

// WU_CNTL/STATUS bits (RM Tables 5-7/5-13) used by the async `delay_ms`. Per RM §5.9.12
// (RTC_TIMEOUT): "An interrupt request based on an RTC time-out can be used while awake" —
// this doesn't need [`crate::sleep::sleep`]'s HIB/DOZE at all, so the async wait here runs
// with the CPU (and any other cooperative task) fully live, unlike a whole-system sleep.
const RTC_WU_EN: u32 = 1 << 1;
const RTC_WU_IEN: u32 = 1 << 17;
const RTC_WU_EVT: u32 = 1 << 3;

/// Waker for the in-flight async `delay_ms` call, if any.
///
/// The RTC wake-up comparator (`WU_CNTL.RTC_WU_EN`/`RTC_TIMEOUT`/`STATUS.RTC_WU_EVT`) is a
/// single shared hardware resource: don't run two [`Delay`] instances' async `delay_ms`
/// concurrently, and don't mix this with [`crate::sleep::sleep`]'s RTC wake source — both
/// would fight over the same registers. Guarded by `critical_section`'s `Mutex`, backed by
/// this crate's own `critical_section::Impl` (see `crate::critical_section_impl`) — see
/// `crate::i2c`'s `WAKER` for why that's safe to rely on unconditionally.
static WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));

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
/// Milliseconds are measured against the RTC (started on the ~2 kHz ring
/// oscillator and calibrated by `rtc_init_osc`). Microsecond and nanosecond
/// delays use an approximate busy-wait derived from the core clock and may
/// therefore over-delay.
pub struct Delay;

impl Delay {
    /// Start and calibrate the RTC on the ring oscillator.
    ///
    /// This only sets up the RTC itself; it deliberately does *not* enable the CRM interrupt
    /// in the ITC. The blocking [`DelayNs::delay_ms`] impl below (`rtc_delay_ms`, a ROM
    /// busy-wait) never uses interrupts at all, so a caller that only ever uses the
    /// synchronous API should never have a live CRM IRQ vector enabled on its behalf - see
    /// [`Delay::ensure_crm_interrupt_enabled`], called lazily from the async path instead.
    pub fn new() -> Self {
        unsafe {
            rtc_init_osc(0);
        }
        Delay
    }

    /// Route the CRM interrupt through the ITC: `NIPEND` only reports `INT_NUM_CRM` as pending,
    /// and `irq()` only enters the block that calls `rtc_isr`, if this is enabled. Only the
    /// async [`Delay::wait_rtc_ticks`] path needs this, so it's called from there instead of
    /// unconditionally in [`Delay::new`].
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
    /// Async equivalent of [`rtc_delay_ms`], waiting `ticks` RTC ticks without busy-spinning.
    ///
    /// Disarms and clears any stale `RTC_WU_EN`/`RTC_WU_EVT` left over from a previous call
    /// before reprogramming `RTC_TIMEOUT`, since the comparator is periodic (RM §5.9.12: "As
    /// soon as the time-out period occurs, the next time-out point ... is calculated in
    /// hardware and saved") rather than one-shot — without this, a stale still-enabled
    /// comparator from an earlier wait could fire immediately on the new period, or this
    /// wait's own event could still be pending from before it was even armed. The
    /// check-then-arm sequence runs inside one [`critical_section::with`] call for the same
    /// reason as [`crate::i2c::I2c0::wait_byte_async`]: so an event landing between the check
    /// and enabling `RTC_WU_IEN` can't be missed.
    ///
    /// # Why this loops
    ///
    /// RM §5.9.12 (RTC_TIMEOUT) says: "the next time-out point (based on current RTC count)
    /// is calculated in hardware and saved," and "a new time-out value ... will be calculated
    /// and effective only after the next RTC clock" — the comparator runs a continuously
    /// self-advancing periodic schedule *independent of `RTC_WU_EN`*, rewriting `RTC_TIMEOUT`
    /// doesn't re-anchor it to "now". A rearm can therefore inherit a boundary left over from
    /// the previous period's own unattended continuation and fire early. `RTC_COUNT` (plain
    /// free-running, no such staleness) is used as ground truth: every wake is provisional
    /// until it's confirmed the full requested duration has actually elapsed, re-arming for
    /// whatever's left otherwise. Since each rearm only undershoots by the same kind of stale
    /// margin rather than never progressing, this converges in a handful of iterations at
    /// most.
    async fn wait_rtc_ticks(&mut self, ticks: u32) {
        Self::ensure_crm_interrupt_enabled();
        let anchor = unsafe { RTC_COUNT.read_volatile() };
        let mut remaining = ticks.max(1);
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
                    *WAKER.borrow(cs).borrow_mut() = Some(cx.waker().clone());
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

/// `embedded-hal-async`'s `DelayNs` reuses no associated items from `embedded-hal`'s, so there
/// is nothing shared with the impl above beyond the trait shape.
///
/// `delay_ns`/`delay_us` still busy-spin exactly like [`DelayNs::delay_ns`]/[`DelayNs::delay_us`]
/// above: the RTC's ~2 kHz ring-oscillator tick (~500 µs) is far too coarse to represent
/// sub-millisecond durations, and an interrupt round-trip would be slower than the delay
/// itself at that scale. Only `delay_ms` waits on the real RTC interrupt — see
/// [`Delay::wait_rtc_ticks`] — so it's the only one that actually yields to the executor.
impl embedded_hal_async::delay::DelayNs for Delay {
    async fn delay_ns(&mut self, ns: u32) {
        DelayNs::delay_ns(self, ns);
    }

    async fn delay_us(&mut self, us: u32) {
        DelayNs::delay_us(self, us);
    }

    async fn delay_ms(&mut self, ms: u32) {
        // rtc_freq is calibrated in Hz; ticks = ms * Hz / 1000, at least 1 so a sub-tick
        // request still waits a full tick rather than treating 0 ticks as already-elapsed.
        let ticks = ((ms as u64 * unsafe { rtc_freq as u32 } as u64) / 1000).max(1) as u32;
        self.wait_rtc_ticks(ticks).await;
    }
}

/// RTC wake-up-timeout interrupt handler, backing [`Delay::wait_rtc_ticks`].
///
/// Overrides the weak `rtc_isr` symbol declared in `libmc1322x`'s `isr.h`; the linked `irq()`
/// handler (`mc1322x-sys/libmc1322x/src/isr.c`) already dispatches to it whenever
/// `rtc_wu_evt()` is true within the `INT_NUM_CRM` block — unlike `spi_isr`, no submodule
/// patch was needed for this one.
///
/// This masks `RTC_WU_IEN` rather than `RTC_WU_EN`: the comparator is periodic (see
/// [`Delay::wait_rtc_ticks`]), so leaving it running and only masking the interrupt request
/// matches the same mask-not-disable pattern `i2c_isr`/`uart1_isr`/`spi_isr` use, and avoids
/// re-deriving a fresh "current time" base the next time it's armed. `STATUS.RTC_WU_EVT`
/// itself is left for [`Delay::wait_rtc_ticks`]'s next call to clear, exactly as
/// `I2c0::poll_byte_status` (run from task context) owns clearing `I2C_MIF` rather than the
/// ISR.
#[unsafe(no_mangle)]
extern "C" fn rtc_isr() {
    unsafe {
        write_reg(WU_CNTL, read_reg(WU_CNTL) & !RTC_WU_IEN);
    }
    critical_section::with(|cs| {
        if let Some(waker) = WAKER.borrow(cs).borrow_mut().take() {
            waker.wake();
        }
    });
}
