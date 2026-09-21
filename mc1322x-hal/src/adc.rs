use core::cell::Cell;
use core::task::Poll;

use critical_section::Mutex;
use mc1322x_sys::{ADC_BASE, INTBASE, adc_init, adc_setup_chan};

use crate::util::WakerCell;

/// Internal 1.2 V reference used to convert raw readings to millivolts.
const INTERNAL_REFERENCE_MV: u32 = 1200;

/// Per-unit offset applied to the battery reading (see `adc.h`).
const ADC_VBATT_TRIM: u32 = 183;

/// Number of ADC channels: GPIO0-7 plus the internal reference on channel 8.
pub const ADC_CHANNELS: u8 = 9;

// ADC registers are all 16-bit wide with 16-bit access only (RM §17.6) - unlike every other
// MMIO block this crate talks to directly (AES/SPI/I2C's registers are 32-bit or byte-wide), so
// every raw access here must go through `*const/*mut u16`, never `u32`.
const CONTROL: *mut u16 = (ADC_BASE as usize + 0x18) as *mut u16;
const FIFO_READ: *const u16 = (ADC_BASE as usize + 0x20) as *const u16;
const FIFO_CONTROL: *mut u16 = (ADC_BASE as usize + 0x22) as *mut u16;
const FIFO_STATUS: *const u16 = (ADC_BASE as usize + 0x24) as *const u16;
const IRQ: *mut u16 = (ADC_BASE as usize + 0x42) as *mut u16;

// RM §17.7.6 (ADC_CONTROL): despite the "_Mask" naming, these are *enable* bits (1 = IRQ
// enabled, 0 = masked/disabled) - the opposite of what the name suggests, and the opposite of
// AES's `CONTROL1_MASK_IRQ`. `adc_init()` (`lib/adc.c`) unconditionally sets all four to 1 via
// `ADC->CONTROL = 0xF001` regardless of its own `ADC_USE_INTERRUPTS` compile-time flag (which
// only gates `FIFO_CONTROL`'s level, not this write) - harmless while nothing ever arms the ITC
// channel (as was true until now), but would mean the FIFO interrupt this driver actually wants
// shares one physical interrupt line with two sequencer-cycle-complete conditions and a
// comparator condition this driver never services, each logically OR'ed together (RM §17.5.5).
// Since Sequencer 1 free-runs continuously in this driver's automatic mode, its cycle-complete
// condition would otherwise become pending constantly - [`Adc::new`] explicitly clears the
// other three enables, keeping only the FIFO one.
const CONTROL_FIFO_IRQ_ENABLE: u16 = 1 << 15;
const CONTROL_SEQ2_IRQ_ENABLE: u16 = 1 << 14;
const CONTROL_SEQ1_IRQ_ENABLE: u16 = 1 << 13;
const CONTROL_COMPARE_IRQ_ENABLE: u16 = 1 << 12;

const FIFO_STATUS_EMPTY: u16 = 1 << 5;

// RM §17.7.10: `FIFO_CONTROL`'s Level field only accepts 1-8 (0 is reserved/invalid) - fire as
// soon as any sample lands, so the cache [`adc_isr`] maintains stays as fresh as possible.
const FIFO_LEVEL_MIN: u16 = 1;

// RM §17.7.20 (ADC_IRQ): bit 15 = FIFO status, write-1-to-clear, independent of the other three
// (Seq2/Seq1/Compare) bits also live in this register.
const IRQ_FIFO: u16 = 1 << 15;

// ITC (interrupt controller) offset/number for the ADC FIFO completion interrupt (see
// `isr.h`'s `INTENNUM_OFF` and `interrupt_nums`), following the same wiring as `crate::i2c`'s
// `INT_NUM_I2C`. `INT_NUM_ADC`/`adc_isr` did not exist in upstream `libmc1322x` - like SPI's,
// this dispatch is this project's own fork addition (see `mc1322x-sys/libmc1322x/src/isr.c`).
const INTENNUM_OFF: u32 = 0x8;
const INT_NUM_ADC: u32 = 9;

/// Per-channel cache of the most recent sample, maintained exclusively by [`adc_isr`].
///
/// Unlike AES/SPI/I2C's discrete, per-call operations, the ADC's sequencer free-runs
/// continuously once [`Adc::new`] brings it up, regardless of whether anything is currently
/// waiting on a reading - so both [`Adc::read`] and [`Adc::read_async`] are pure *consumers* of
/// this cache rather than touching the FIFO themselves; touching `FIFO_READ`/`FIFO_STATUS` from
/// more than one place (this cache's own drain loop in [`adc_isr`], *and* task context) would
/// race for the same hardware FIFO entries.
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

/// MC1322x 12-bit ADC.
///
/// The ADC runs in automatic mode: every enabled channel plus the internal
/// reference is converted in a repeating sequence and pushed to the FIFO,
/// tagged with its channel number in the upper four bits. Sampling is started
/// and calibrated by [`Adc::new`]; [`Adc::read`] then blocks until a fresh
/// sample tagged with the requested channel arrives.
///
/// Channel 8 measures the internal 1.2 V reference against the ADC reference
/// rail and is always enabled; it is the denominator for the millivolt
/// conversions in [`Adc::voltage`] and [`Adc::battery_voltage`].
///
/// `Copy` since this type carries no state of its own (the real state is [`ADC_CACHE`]/
/// [`ADC_WAKERS`], module-level statics) - safe to freely duplicate a handle obtained from
/// [`Adc::new`] across concurrent tasks that each read a *different* channel (e.g. one per
/// `embassy` task); reading the *same* channel from more than one place concurrently races for
/// that channel's single cache slot, the same class of caveat AES/SPI's docs already call out
/// for concurrent use of one peripheral handle.
#[derive(Clone, Copy)]
pub struct Adc;

impl Adc {
    /// Power up and calibrate the ADC.
    ///
    /// Enables the ADC clock and timings and starts the conversion sequence
    /// (channel 8 / internal reference included). Also brings up the FIFO completion
    /// interrupt that backs both [`Adc::read`] and [`Adc::read_async`] - see the comment on
    /// [`ADC_CACHE`].
    pub fn new() -> Self {
        unsafe {
            adc_init();

            let control = (CONTROL as *const u16).read_volatile();
            CONTROL.write_volatile(
                (control
                    & !(CONTROL_SEQ2_IRQ_ENABLE | CONTROL_SEQ1_IRQ_ENABLE | CONTROL_COMPARE_IRQ_ENABLE))
                    | CONTROL_FIFO_IRQ_ENABLE,
            );
            FIFO_CONTROL.write_volatile(FIFO_LEVEL_MIN);

            // Route the ADC FIFO interrupt to the core, once, permanently - see [`ADC_CACHE`]'s
            // doc comment for why this differs from AES/SPI's per-operation arm/disarm.
            core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_ADC);
        }
        Adc
    }

    /// Include `channel` (0-7) in the conversion sequence and mux its GPIO pad
    /// to the ADC. The internal reference on channel 8 is always enabled.
    pub fn enable_channel(&mut self, channel: u8) {
        assert!(channel < 8, "ADC channel {} out of range 0-7", channel);
        unsafe { adc_setup_chan(channel) }
    }

    /// Read one raw 12-bit sample (0-4095) from `channel`.
    ///
    /// Blocks until [`adc_isr`] caches a *fresh* sample tagged with `channel`: any
    /// already-cached value from before this call is discarded first, matching the old
    /// FIFO-flush-then-wait behavior this replaced.
    pub fn read(&mut self, channel: u8) -> u16 {
        take_cached(channel);
        loop {
            if let Some(value) = take_cached(channel) {
                return value;
            }
            core::hint::spin_loop();
        }
    }

    /// Async equivalent of [`Self::read`].
    ///
    /// Waits for [`adc_isr`] to wake this channel's waiter rather than polling in a loop. Like
    /// [`Self::read`], discards any already-cached value first so the result is a fresh sample.
    /// The check-then-arm sequence runs inside a single [`critical_section::with`] call so a
    /// sample landing between the check and arming the waker can't be missed, same as
    /// [`crate::i2c::I2c0::wait_byte_async`].
    ///
    /// Holds a [`crate::sleep::SleepInhibitGuard`] for as long as this wait is in flight - see
    /// that type's doc comment for why a sleep-aware executor must not sleep while this
    /// module's completion interrupt is what a task is waiting on.
    ///
    /// # Panics
    ///
    /// Panics if `channel` is out of the 0-8 range described by [`ADC_CHANNELS`].
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

    /// Read `channel` (0-8) in millivolts, referenced to the internal 1.2 V
    /// reference. Requires the reference on channel 8 to be enabled, which
    /// [`Adc::new`] does.
    pub fn voltage(&mut self, channel: u8) -> u32 {
        let reference = self.read(8) as u32;
        (self.read(channel) as u32) * INTERNAL_REFERENCE_MV / reference
    }

    /// Battery voltage in millivolts, derived from the full-scale ADC reading
    /// and the internal reference, with the default per-unit trim applied.
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

/// ADC FIFO completion interrupt handler.
///
/// Overrides the weak `adc_isr` symbol this crate adds to `libmc1322x`'s `isr.h`/`isr.c` (see
/// the comment above [`INT_NUM_ADC`]) - like `spi_isr`, this dispatch only exists in this
/// project's fork.
///
/// Acks *all* currently-set `ADC_IRQ` bits unconditionally (a read followed by writing back the
/// same value - RM §17.7.20's write-1-to-clear semantics mean this clears exactly the bits that
/// were set), not just the FIFO one this driver cares about: `irq()`'s dispatch loop re-enters
/// this handler for as long as `INT_NUM_ADC` stays pending, and since [`Adc::new`] otherwise
/// takes care to disable the other three sources ([`CONTROL_SEQ2_IRQ_ENABLE`] etc.), any that
/// did fire regardless (e.g. a caller reconfiguring `ADC_CONTROL` directly) would livelock
/// forever if left unacknowledged - see `crate::aes::asm_isr`'s doc comment for the
/// hardware-confirmed version of this same mistake, made and fixed on this same peripheral
/// family earlier in the same session.
///
/// Only drains the FIFO (bounded by the FIFO's own fixed 8-word depth, so this always
/// terminates) when the FIFO bit was actually set, caching each sample by its tagged channel
/// number and waking that channel's waiter, if any.
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
