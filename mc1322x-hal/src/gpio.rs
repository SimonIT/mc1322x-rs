use core::convert::Infallible;
use core::task::Poll;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};
use mc1322x_sys::{
    CRM_BASE, INTBASE, gpio_read, gpio_reg_clear, gpio_reg_set, gpio_reset, gpio_select_function,
    gpio_set, gpio_set_pad_dir,
};

use crate::util::WakerCell;

const PAD_DIR_INPUT: u8 = 0;
const PAD_DIR_OUTPUT: u8 = 1;

// GPIO block-0 pull registers (pins 0-31; GPIO26-29/KBI4-7 all fall in this block) - same
// addresses `crate::i2c` already uses for its own pull-up setup.
const GPIO_PAD_PU_EN0: *mut u32 = 0x8000_0010 as *mut u32;
const GPIO_PAD_PU_SEL0: *mut u32 = 0x8000_0030 as *mut u32;

// ITC (interrupt controller) offset/number for the CRM interrupt - same wiring as
// `crate::delay`'s `INT_NUM_CRM` (KBI4-7's edge events share `irq()`'s `INT_NUM_CRM` dispatch
// block with the RTC wake-up-timeout and calibration-done events; no submodule patch needed,
// `kbi4_isr`..`kbi7_isr` are all already weak slots `isr.c` dispatches to).
const INTENNUM_OFF: u32 = 0x8;
const INT_NUM_CRM: u32 = 3;

const WU_CNTL: *mut u32 = (CRM_BASE as usize + 0x04) as *mut u32;
const STATUS: *mut u32 = (CRM_BASE as usize + 0x18) as *mut u32;

// WU_CNTL/STATUS bit *bases* for the 4-bit EXT_WU_* fields (RM §5.9.2 Table 5-7, §5.9.6 Table
// 5-13; `mc1322x-sys/libmc1322x/lib/include/crm.h`'s `EXT_WU_EN`/`EDGE`/`POL`/`IEN`/(STATUS's)
// `EXT_WU_EVT` constants). Each field packs one bit per KBI pin at `BASE + (kbi_number - 4)`;
// [`KbiPin::index`] is already that `kbi_number - 4` (0..=3 for KBI4..=KBI7), so the bit for a
// given pin is just `1 << (BASE + index)`.
const EXT_WU_EN_BASE: u32 = 4;
const EXT_WU_EDGE_BASE: u32 = 8;
const EXT_WU_POL_BASE: u32 = 12;
const EXT_WU_IEN_BASE: u32 = 20;
const EXT_WU_EVT_BASE: u32 = 4; // in STATUS, not WU_CNTL

const fn ext_wu_en_bit(index: u8) -> u32 {
    1 << (EXT_WU_EN_BASE + index as u32)
}
const fn ext_wu_edge_bit(index: u8) -> u32 {
    1 << (EXT_WU_EDGE_BASE + index as u32)
}
const fn ext_wu_pol_bit(index: u8) -> u32 {
    1 << (EXT_WU_POL_BASE + index as u32)
}
const fn ext_wu_ien_bit(index: u8) -> u32 {
    1 << (EXT_WU_IEN_BASE + index as u32)
}
const fn ext_wu_evt_bit(index: u8) -> u32 {
    1 << (EXT_WU_EVT_BASE + index as u32)
}

#[inline]
unsafe fn read_reg(reg: *mut u32) -> u32 {
    unsafe { reg.read_volatile() }
}

#[inline]
unsafe fn write_reg(reg: *mut u32, value: u32) {
    unsafe { reg.write_volatile(value) }
}

/// One of the four GPIO pins with real edge/level-triggered wake-up-comparator hardware behind
/// it (GPIO26-29, aka KBI4-7) - see `mc1322x-sys/libmc1322x/lib/include/kbi.h`'s `k-4`-indexed
/// macros: unlike KBI0-3, whose enable/edge/polarity bits don't exist in hardware at all
/// (`crm.h`'s `EXT_WU_*` fields are 4 bits wide, indexed `k-4` for `k` in 4..=7 only), only
/// these four pins can ever raise an edge interrupt on this chip. This is deliberately a
/// closed enum rather than a fallible `TryFrom<Pin>` check: there is no invalid inhabitant to
/// reject in the first place.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum KbiPin {
    Kbi4,
    Kbi5,
    Kbi6,
    Kbi7,
}

impl KbiPin {
    /// The underlying GPIO pin (26-29).
    pub const fn gpio(self) -> Pin {
        Pin::new(26 + self.index())
    }

    /// `kbi_number - 4`, i.e. 0..=3 - the index used throughout `crm.h`'s `EXT_WU_*` bit-field
    /// macros.
    const fn index(self) -> u8 {
        self as u8
    }
}

/// Edge to wait for with [`KbiInput::wait_for_edge`].
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Edge {
    Rising,
    Falling,
}

/// Waker for an in-flight [`KbiInput::wait_for_edge`] call on each of the 4 KBI pins, if any.
static KBI_WAKERS: [WakerCell; 4] = [const { WakerCell::new() }; 4];

/// A [`KbiPin`] configured as an edge-triggered wake-up input.
///
/// The wake-up comparator (`WU_CNTL`'s `EXT_WU_*` fields) is real hardware shared with
/// [`crate::sleep::sleep`]'s KBI wake sources — don't use both for the same pin at once, same
/// class of caveat as [`crate::delay::Delay`]'s RTC wait vs. `sleep`'s RTC wake source.
pub struct KbiInput {
    pin: KbiPin,
}

impl KbiInput {
    /// Configure `pin` as a GPIO input. Per RM §5.9.2: "During run mode, a pad must be
    /// programmed as a GPIO input and a pulldown or pullup enabled (as appropriate) via the
    /// GPIO Module" for the wake-up comparator to see a well-defined idle level — the actual
    /// pull direction is chosen per call by [`Self::wait_for_edge`], matching whichever edge
    /// is being waited for.
    pub fn new(pin: KbiPin) -> Self {
        pin.gpio().into_input();
        KbiInput { pin }
    }

    /// Wait for one occurrence of `edge` on this pin.
    ///
    /// Configures the pull resistor, edge sense and polarity for `edge` (RM §5.9.2 notes
    /// `EXT_WU_POL`/`EXT_WU_EDGE` should only be changed while `EXT_WU_EN` is disabled, so this
    /// disables it first), clears any stale latched event, then waits for [`kbi4_isr`] (or
    /// `kbi5_isr`/`kbi6_isr`/`kbi7_isr`, whichever matches this pin) to wake this task rather
    /// than polling in a loop. The check-then-arm sequence runs inside a single
    /// [`critical_section::with`] call so an edge landing between the check and enabling the
    /// interrupt can't be missed, same as [`crate::delay::Delay::wait_rtc_ticks`] — this is a
    /// genuine one-shot wait, though, not a periodic comparator, so (unlike that one) no
    /// re-arm/ground-truth loop is needed.
    ///
    /// Per RM §5.9.2's note on `EXT_WU_EDGE`: "The pulse width of the signal must be at least 2
    /// clocks of whatever clock is running the edge detector" (the 24 MHz reference in run
    /// mode) - not a concern for anything slower than a few tens of ns, e.g. a button or another
    /// GPIO pin toggled by software.
    pub async fn wait_for_edge(&mut self, edge: Edge) {
        let index = self.pin.index();
        let pin_number = self.pin.gpio().pin_number();
        unsafe {
            // EXT_WU_POL/EDGE must not change while EXT_WU_EN is set (RM §5.9.2) - disable it
            // first. Leaving EXT_WU_IEN alone here is fine: if it was still set from a prior
            // wait somehow left armed, the disabled EXT_WU_EN below means it can't newly fire
            // from stale state either way.
            write_reg(WU_CNTL, read_reg(WU_CNTL) & !ext_wu_en_bit(index));

            match edge {
                // Idle-low, wake on the rising edge that follows an external driver pulling the
                // pin high: pull the pad down while idle (RM §5.9.2's own automatic-mode
                // pairing of POL=1/positive-edge with a pulldown, applied manually here since
                // this runs in run mode, not sleep, where the GPIO module - not `EXT_WU_POL` -
                // controls the pad).
                Edge::Rising => {
                    gpio_reg_clear(GPIO_PAD_PU_SEL0, pin_number);
                    write_reg(
                        WU_CNTL,
                        read_reg(WU_CNTL) | ext_wu_edge_bit(index) | ext_wu_pol_bit(index),
                    );
                }
                // Idle-high (pulled up), wake on the falling edge that follows an external
                // driver pulling the pin low.
                Edge::Falling => {
                    gpio_reg_set(GPIO_PAD_PU_SEL0, pin_number);
                    write_reg(
                        WU_CNTL,
                        (read_reg(WU_CNTL) | ext_wu_edge_bit(index)) & !ext_wu_pol_bit(index),
                    );
                }
            }
            gpio_reg_set(GPIO_PAD_PU_EN0, pin_number);

            // Clear any stale latched event from before this call (RM §5.9.6 / `kbi.h`: "you
            // have to clear these events by writing a one to them").
            write_reg(STATUS, ext_wu_evt_bit(index));

            core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_CRM);
        }

        let mut inhibit = None;
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                if unsafe { read_reg(STATUS) } & ext_wu_evt_bit(index) != 0 {
                    return Poll::Ready(());
                }
                inhibit.get_or_insert_with(crate::sleep::SleepInhibitGuard::new);
                KBI_WAKERS[index as usize].set(cs, cx.waker());
                unsafe {
                    write_reg(
                        WU_CNTL,
                        read_reg(WU_CNTL) | ext_wu_en_bit(index) | ext_wu_ien_bit(index),
                    );
                }
                Poll::Pending
            })
        })
        .await;
    }
}

/// A single MC1322x GPIO pin (GPIO_00 through GPIO_63).
///
/// The pin can be reconfigured between input and output at any time via
/// [`Pin::into_input`] and [`Pin::into_output`].
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Pin {
    num: u8,
}

impl Pin {
    /// Create a pin from its GPIO number (0-63).
    pub const fn new(num: u8) -> Self {
        Self { num }
    }

    /// The GPIO number of this pin.
    pub const fn pin_number(&self) -> u8 {
        self.num
    }

    /// Configure the pin as a push-pull output.
    ///
    /// The data register is set before the pad direction is switched to
    /// output, so no glitch is driven on the pin.
    pub fn into_output(self, state: bool) -> Self {
        unsafe {
            gpio_select_function(self.num, 0);
        }
        if state {
            unsafe { gpio_set(self.num) }
        } else {
            unsafe { gpio_reset(self.num) }
        }
        unsafe {
            gpio_set_pad_dir(self.num, PAD_DIR_OUTPUT);
        }
        self
    }

    /// Configure the pin as a high-impedance input.
    pub fn into_input(self) -> Self {
        unsafe {
            gpio_select_function(self.num, 0);
            gpio_set_pad_dir(self.num, PAD_DIR_INPUT);
        }
        self
    }
}

impl ErrorType for Pin {
    type Error = Infallible;
}

impl OutputPin for Pin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe { gpio_reset(self.num) }
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe { gpio_set(self.num) }
        Ok(())
    }
}

impl StatefulOutputPin for Pin {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { gpio_read(self.num) })
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!unsafe { gpio_read(self.num) })
    }
}

impl InputPin for Pin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { gpio_read(self.num) })
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!unsafe { gpio_read(self.num) })
    }
}

/// KBI4-7 external wake-up interrupt handlers, backing [`KbiInput::wait_for_edge`].
///
/// Overrides the weak `kbi{4,5,6,7}_isr` symbols declared in `libmc1322x`'s `isr.h`; the linked
/// `irq()` handler (`mc1322x-sys/libmc1322x/src/isr.c`) already dispatches to them, guarded by
/// `kbi_evnt(n)`, within the `INT_NUM_CRM` block — like `rtc_isr`, no submodule patch was
/// needed for these.
///
/// Masks this pin's own `EXT_WU_IEN` bit rather than `EXT_WU_EN`: per RM §5.9.2, "the
/// status/interrupt request will be cleared immediately upon servicing" once `EXT_WU_IEN` is
/// disabled — the same retracting behavior [`crate::delay::rtc_isr`]
/// already relies on for `RTC_WU_IEN` (word-for-word the same description in the RM), unlike
/// AES's `CONTROL1_MASK_IRQ`, which does *not* retract an already-latched interrupt (see
/// `crate::aes::asm_isr`'s doc comment for that livelock). `STATUS`'s
/// `EXT_WU_EVT` bit is left for [`KbiInput::wait_for_edge`]'s next call to clear, exactly as
/// `rtc_isr` leaves `RTC_WU_EVT` for [`crate::delay::Delay::wait_rtc_ticks`].
macro_rules! kbi_isr {
    ($name:ident, $index:expr) => {
        #[unsafe(no_mangle)]
        extern "C" fn $name() {
            unsafe {
                write_reg(WU_CNTL, read_reg(WU_CNTL) & !ext_wu_ien_bit($index));
            }
            KBI_WAKERS[$index as usize].wake();
        }
    };
}

kbi_isr!(kbi4_isr, 0u8);
kbi_isr!(kbi5_isr, 1u8);
kbi_isr!(kbi6_isr, 2u8);
kbi_isr!(kbi7_isr, 3u8);
