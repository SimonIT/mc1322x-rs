//! General-purpose I/O pins and edge-triggered KBI inputs.

use core::convert::Infallible;
use core::task::Poll;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};
use mc1322x_sys::{
    CRM_BASE, INTBASE, INTENNUM_OFF, gpio_read, gpio_reg_clear, gpio_reg_set, gpio_reset,
    gpio_select_function, gpio_set, gpio_set_pad_dir, interrupt_nums_INT_NUM_CRM,
};

use crate::util::WakerCell;

const PAD_DIR_INPUT: u8 = 0;
const PAD_DIR_OUTPUT: u8 = 1;

// GPIO block-0 pull registers (pins 0-31, which includes GPIO26-29/KBI4-7).
const GPIO_PAD_PU_EN0: *mut u32 = 0x8000_0010 as *mut u32;
const GPIO_PAD_PU_SEL0: *mut u32 = 0x8000_0030 as *mut u32;

// ITC (interrupt controller) number of the CRM interrupt. KBI4-7 edge events are dispatched by
// `irq()`'s `INT_NUM_CRM` block to the weak `kbi4_isr`..`kbi7_isr` symbols.
const INT_NUM_CRM: u32 = interrupt_nums_INT_NUM_CRM;

const WU_CNTL: *mut u32 = (CRM_BASE as usize + 0x04) as *mut u32;
const STATUS: *mut u32 = (CRM_BASE as usize + 0x18) as *mut u32;

// Bit offsets of the 4-bit `EXT_WU_*` fields in WU_CNTL/STATUS (RM Tables 5-7 and 5-13,
// `crm.h`). Each field has one bit per KBI pin, at `BASE + KbiPin::index()`.
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

/// One of the four pins with edge-detection hardware: GPIO26-29, also called KBI4-7.
///
/// Only these pins have wake-up comparator bits in `WU_CNTL`; KBI0-3 cannot raise edge
/// interrupts.
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

    /// `kbi_number - 4` (0..=3), the bit index within the `EXT_WU_*` fields.
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

/// A [`KbiPin`] configured as an edge-triggered input.
///
/// The edge detector is the CRM wake-up comparator, which [`crate::sleep`] also uses for its KBI
/// wake sources: don't use both for the same pin at once.
pub struct KbiInput {
    pin: KbiPin,
}

impl KbiInput {
    /// Configure `pin` as a GPIO input.
    ///
    /// The pull resistor is selected later by [`Self::wait_for_edge`], depending on the edge.
    pub fn new(pin: KbiPin) -> Self {
        pin.gpio().into_input();
        KbiInput { pin }
    }

    /// Wait for one occurrence of `edge` on this pin.
    ///
    /// Enables the internal pull-down (for [`Edge::Rising`]) or pull-up (for [`Edge::Falling`]),
    /// configures the edge detector, clears any stale event and waits for the KBI interrupt.
    /// Inhibits a sleep-aware executor from sleeping while waiting (see
    /// [`crate::sleep::SleepInhibitGuard`]).
    ///
    /// Pulses must be at least 2 clocks of the edge detector clock (24 MHz in run mode) wide
    /// (RM §5.9.2).
    pub async fn wait_for_edge(&mut self, edge: Edge) {
        let index = self.pin.index();
        let pin_number = self.pin.gpio().pin_number();
        unsafe {
            // EXT_WU_POL/EDGE must not change while EXT_WU_EN is set (RM §5.9.2).
            write_reg(WU_CNTL, read_reg(WU_CNTL) & !ext_wu_en_bit(index));

            match edge {
                // Idle low: pull down and detect the rising edge. In run mode the GPIO module, not
                // `EXT_WU_POL`, controls the pad's pull.
                Edge::Rising => {
                    gpio_reg_clear(GPIO_PAD_PU_SEL0, pin_number);
                    write_reg(
                        WU_CNTL,
                        read_reg(WU_CNTL) | ext_wu_edge_bit(index) | ext_wu_pol_bit(index),
                    );
                }
                // Idle high: pull up and detect the falling edge.
                Edge::Falling => {
                    gpio_reg_set(GPIO_PAD_PU_SEL0, pin_number);
                    write_reg(
                        WU_CNTL,
                        (read_reg(WU_CNTL) | ext_wu_edge_bit(index)) & !ext_wu_pol_bit(index),
                    );
                }
            }
            gpio_reg_set(GPIO_PAD_PU_EN0, pin_number);

            // Clear a stale latched event (write-1-to-clear, RM §5.9.6).
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

/// A single GPIO pin (GPIO_00 to GPIO_63).
///
/// Can be switched between input and output at any time with [`Pin::into_input`] and
/// [`Pin::into_output`].
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Pin {
    num: u8,
}

impl Pin {
    /// Create a pin from its GPIO number (0-63).
    ///
    /// Does not change the pin's configuration.
    pub const fn new(num: u8) -> Self {
        Self { num }
    }

    /// The GPIO number of this pin.
    pub const fn pin_number(&self) -> u8 {
        self.num
    }

    /// Configure the pin as a push-pull output driving `state` (`true` = high).
    ///
    /// The output level is set before the pad is switched to output, so the pin doesn't glitch.
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

    /// Configure the pin as an input.
    ///
    /// Pull resistor settings are left unchanged.
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

/// KBI4-7 interrupt handlers, overriding the weak `kbi{4,5,6,7}_isr` symbols from `libmc1322x`'s
/// `isr.h`. `irq()` calls them from its `INT_NUM_CRM` block when `kbi_evnt(n)` is set.
///
/// Each masks its pin's `EXT_WU_IEN` bit, which also retracts the pending request (RM §5.9.2),
/// and wakes the waiter. `STATUS.EXT_WU_EVT` is cleared by the next
/// [`KbiInput::wait_for_edge`] call.
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
