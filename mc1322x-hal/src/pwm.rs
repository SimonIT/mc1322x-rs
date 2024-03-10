use core::ffi::c_int;
use embedded_hal::pwm::{ErrorKind, ErrorType, SetDutyCycle};
use mc1322x_sys::{pwm_duty_ex, pwm_init_ex};

pub struct Pwm {
    timer_num: u8,
    rate: u32,
    initialized: bool,
}

impl Pwm {
    pub fn new(timer_num: u8, rate: u32) -> Self {
        Self {
            timer_num,
            rate,
            initialized: false,
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Error {}

impl embedded_hal::pwm::Error for Error {
    fn kind(&self) -> ErrorKind {
        match *self {}
    }
}

impl ErrorType for Pwm { type Error = Error; }

impl SetDutyCycle for Pwm {
    fn max_duty_cycle(&self) -> u16 {
        u16::MAX
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        if !self.initialized {
            unsafe {
                pwm_init_ex(self.timer_num as c_int, self.rate, duty as u32, 1);
            }
            self.initialized = true;
        } else {
            unsafe {
                pwm_duty_ex(self.timer_num as c_int, duty as u32);
            }
        }
        Ok(())
    }
}
