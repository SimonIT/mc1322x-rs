use core::convert::Infallible;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};
use mc1322x_sys::{gpio_read, gpio_reset, gpio_select_function, gpio_set, gpio_set_pad_dir};

const PAD_DIR_INPUT: u8 = 0;
const PAD_DIR_OUTPUT: u8 = 1;

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
