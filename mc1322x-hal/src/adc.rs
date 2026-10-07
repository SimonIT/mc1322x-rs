//! 12-bit analog-to-digital converter.
//!
//! The ADC free-runs over all enabled channels; its FIFO interrupt caches the latest sample of
//! each channel, which [`Adc::read`] and [`Adc::read_async`] consume.

use core::cell::Cell;
use core::task::Poll;

use critical_section::Mutex;
use mc1322x_sys::{
    ADC_BASE, INTBASE, INTENNUM_OFF, adc_init, adc_setup_chan, interrupt_nums_INT_NUM_ADC,
};

use crate::util::WakerCell;

/// Internal 1.2 V reference used to convert raw readings to millivolts.
const INTERNAL_REFERENCE_MV: u32 = 1200;

/// Default per-unit offset applied to the battery reading (`libmc1322x`'s `adc.h`).
const ADC_VBATT_TRIM: u32 = 183;

/// Number of ADC channels: GPIO0-7 plus the internal reference on channel 8.
pub const ADC_CHANNELS: u8 = 9;

// ADC registers only allow 16-bit access (RM §17.6).
const CONTROL: *mut u16 = (ADC_BASE as usize + 0x18) as *mut u16;
const FIFO_READ: *const u16 = (ADC_BASE as usize + 0x20) as *const u16;
const FIFO_CONTROL: *mut u16 = (ADC_BASE as usize + 0x22) as *mut u16;
const FIFO_STATUS: *const u16 = (ADC_BASE as usize + 0x24) as *const u16;
const IRQ: *mut u16 = (ADC_BASE as usize + 0x42) as *mut u16;

// RM §17.7.6 (ADC_CONTROL): despite the "_Mask" names these are *enable* bits (1 = enabled),
// the opposite of AES's `CONTROL1_MASK_IRQ`. `adc_init()` sets all four (`CONTROL = 0xF001`).
// All four sources share one interrupt line (RM §17.5.5), and the free-running sequencer 1
// would keep its cycle-complete source pending constantly, so `Adc::new` leaves only the FIFO
// one enabled.
const CONTROL_FIFO_IRQ_ENABLE: u16 = 1 << 15;
const CONTROL_SEQ2_IRQ_ENABLE: u16 = 1 << 14;
const CONTROL_SEQ1_IRQ_ENABLE: u16 = 1 << 13;
const CONTROL_COMPARE_IRQ_ENABLE: u16 = 1 << 12;

const FIFO_STATUS_EMPTY: u16 = 1 << 5;

// RM §17.7.10: `FIFO_CONTROL`'s Level field accepts 1-8. Use 1 so the ISR caches every sample
// as soon as it lands.
const FIFO_LEVEL_MIN: u16 = 1;

// RM §17.7.20 (ADC_IRQ): bit 15 = FIFO status, write-1-to-clear, independent of the
// Seq2/Seq1/Compare bits in the same register.
const IRQ_FIFO: u16 = 1 << 15;

// ITC (interrupt controller) number of the ADC interrupt. The `adc_isr` dispatch is an addition
// in this project's `libmc1322x` fork (`mc1322x-sys/libmc1322x/src/isr.c`), not upstream.
const INT_NUM_ADC: u32 = interrupt_nums_INT_NUM_ADC;

/// Per-channel cache of the most recent sample, filled only by [`adc_isr`].
///
/// The sequencer free-runs once [`Adc::new`] starts it, so only the ISR drains the FIFO and the
/// read methods just consume this cache; draining it from task context as well would race with
/// the ISR for the same entries.
static ADC_CACHE: Mutex<Cell<[Option<u16>; ADC_CHANNELS as usize]>> =
    Mutex::new(Cell::new([None; ADC_CHANNELS as usize]));

/// Per-channel waker for an in-flight [`Adc::read_async`] call, if any.
static ADC_WAKERS: [WakerCell; ADC_CHANNELS as usize] =
    [const { WakerCell::new() }; ADC_CHANNELS as usize];

/// Take and clear channel `channel`'s cached sample, if any.
fn take_cached(channel: u8) -> Option<u16> {
    critical_section::with(|cs| {
        let mut cache = ADC_CACHE.borrow(cs).get();
        let value = cache[channel as usize].take();
        ADC_CACHE.borrow(cs).set(cache);
        value
    })
}

/// MC1322x 12-bit ADC driver.
///
/// The ADC runs in automatic mode: every enabled channel plus the internal reference is converted
/// in a repeating sequence, and the FIFO interrupt stores the latest sample of each channel.
///
/// Channel 8 measures the internal 1.2 V reference and is always enabled; it is the denominator
/// for the millivolt conversions in [`Adc::voltage`] and [`Adc::battery_voltage`].
///
/// The handle is `Copy` and holds no state, so copies may be used from different tasks as long as
/// each channel is read from only one place at a time: concurrent reads of the same channel
/// compete for its single cached sample.
#[derive(Clone, Copy)]
pub struct Adc;

impl Adc {
    /// Power up the ADC and start the conversion sequence.
    ///
    /// Enables the ADC clock, starts sampling (with the internal reference on channel 8) and
    /// enables the ADC FIFO interrupt in the interrupt controller.
    pub fn new() -> Self {
        unsafe {
            adc_init();

            let control = (CONTROL as *const u16).read_volatile();
            CONTROL.write_volatile(
                (control
                    & !(CONTROL_SEQ2_IRQ_ENABLE
                        | CONTROL_SEQ1_IRQ_ENABLE
                        | CONTROL_COMPARE_IRQ_ENABLE))
                    | CONTROL_FIFO_IRQ_ENABLE,
            );
            FIFO_CONTROL.write_volatile(FIFO_LEVEL_MIN);

            // Enabled once, permanently: the sequencer free-runs, so the ISR must keep draining the
            // FIFO even when nobody is waiting for a reading.
            core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_ADC);
        }
        Adc
    }

    /// Add `channel` (0-7) to the conversion sequence and mux its GPIO pad to the ADC.
    ///
    /// The internal reference on channel 8 is always enabled.
    ///
    /// # Panics
    ///
    /// Panics if `channel` is greater than 7.
    pub fn enable_channel(&mut self, channel: u8) {
        assert!(channel < 8, "ADC channel {} out of range 0-7", channel);
        unsafe { adc_setup_chan(channel) }
    }

    /// Read one raw 12-bit sample (0-4095) from `channel`, blocking.
    ///
    /// Discards any previously cached sample and spins until a new one arrives, so the result is
    /// always fresh. Relies on the ADC interrupt, so it never returns if called with interrupts
    /// masked (e.g. inside a critical section) or for a channel that is not enabled.
    ///
    /// # Panics
    ///
    /// Panics if `channel` is greater than 8.
    pub fn read(&mut self, channel: u8) -> u16 {
        take_cached(channel);
        loop {
            if let Some(value) = take_cached(channel) {
                return value;
            }
            core::hint::spin_loop();
        }
    }

    /// Read one raw 12-bit sample (0-4095) from `channel`, asynchronously.
    ///
    /// Like [`Self::read`], discards any previously cached sample first, then waits for the ADC
    /// interrupt instead of spinning. Inhibits a sleep-aware executor from sleeping while waiting
    /// (see [`crate::sleep::SleepInhibitGuard`]).
    ///
    /// # Panics
    ///
    /// Panics if `channel` is greater than 8.
    pub async fn read_async(&mut self, channel: u8) -> u16 {
        assert!(
            channel < ADC_CHANNELS,
            "ADC channel {} out of range 0-{}",
            channel,
            ADC_CHANNELS - 1
        );
        take_cached(channel);
        let mut inhibit = None;
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                let mut cache = ADC_CACHE.borrow(cs).get();
                if let Some(value) = cache[channel as usize].take() {
                    ADC_CACHE.borrow(cs).set(cache);
                    return Poll::Ready(value);
                }
                inhibit.get_or_insert_with(crate::sleep::SleepInhibitGuard::new);
                ADC_WAKERS[channel as usize].set(cs, cx.waker());
                Poll::Pending
            })
        })
        .await
    }

    /// Read `channel` (0-8) in millivolts, relative to the internal 1.2 V reference on channel 8.
    ///
    /// Blocks like [`Self::read`].
    ///
    /// # Panics
    ///
    /// Panics if `channel` is greater than 8.
    pub fn voltage(&mut self, channel: u8) -> u32 {
        let reference = self.read(8) as u32;
        (self.read(channel) as u32) * INTERNAL_REFERENCE_MV / reference
    }

    /// Read the supply (battery) voltage in millivolts.
    ///
    /// Derived from the internal reference reading, plus a fixed per-unit trim. Blocks like
    /// [`Self::read`].
    pub fn battery_voltage(&mut self) -> u32 {
        let reference = self.read(8) as u32;
        (4095 * INTERNAL_REFERENCE_MV / reference) + ADC_VBATT_TRIM
    }
}

impl Default for Adc {
    fn default() -> Self {
        Self::new()
    }
}

/// ADC interrupt handler, overriding the weak `adc_isr` symbol from `libmc1322x`'s `isr.c`.
///
/// Acknowledges every set `ADC_IRQ` bit (write-1-to-clear), not just the FIFO one: `irq()`
/// re-enters the handler for as long as the interrupt stays pending, so an unacknowledged
/// Seq1/Seq2/Compare source (e.g. after a caller reconfigured `ADC_CONTROL`) would livelock.
///
/// If the FIFO bit was set, drains the FIFO (at most 8 entries), caches each sample by its tagged
/// channel number and wakes the waiters.
#[unsafe(no_mangle)]
extern "C" fn adc_isr() {
    unsafe {
        let irq = (IRQ as *const u16).read_volatile();
        IRQ.write_volatile(irq);
        if irq & IRQ_FIFO == 0 {
            return;
        }
        critical_section::with(|cs| {
            let mut cache = ADC_CACHE.borrow(cs).get();
            while FIFO_STATUS.read_volatile() & FIFO_STATUS_EMPTY == 0 {
                let sample = FIFO_READ.read_volatile();
                let channel = (sample >> 12) as usize;
                if channel < ADC_CHANNELS as usize {
                    cache[channel] = Some(sample & 0x0FFF);
                }
            }
            ADC_CACHE.borrow(cs).set(cache);
        });
    }
    for waker in &ADC_WAKERS {
        waker.wake();
    }
}
