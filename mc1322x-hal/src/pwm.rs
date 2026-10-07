//! PWM output on the TMR timers, using `libmc1322x`'s `pwm_*` routines.

use core::ffi::c_int;
use embedded_hal::pwm::{ErrorKind, ErrorType, SetDutyCycle};
use mc1322x_sys::{pwm_duty_ex, pwm_init_ex};

/// PWM output on one TMR timer's output pin, implementing [`SetDutyCycle`].
///
/// The duty cycle has full `u16` resolution ([`SetDutyCycle::max_duty_cycle`] is `u16::MAX`);
/// `libmc1322x` scales it to the timer's actual period.
pub struct Pwm {
    timer_num: u8,
    rate: u32,
    initialized: bool,
}

impl Pwm {
    /// Create a PWM output on timer `timer_num` (0..=3) at `rate` Hz.
    ///
    /// Doesn't touch the hardware: the timer is configured and started by the first
    /// [`SetDutyCycle::set_duty_cycle`] call.
    pub fn new(timer_num: u8, rate: u32) -> Self {
        Self {
            timer_num,
            rate,
            initialized: false,
        }
    }
}

/// PWM error. Uninhabited: setting the duty cycle can't fail.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Error {}

impl embedded_hal::pwm::Error for Error {
    fn kind(&self) -> ErrorKind {
        match *self {}
    }
}

impl ErrorType for Pwm {
    type Error = Error;
}

impl SetDutyCycle for Pwm {
    fn max_duty_cycle(&self) -> u16 {
        u16::MAX
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        if !self.initialized {
            unsafe {
                // `pwm_init_ex` with duty 0 never programs COMP1/LOAD/CNTR (its internal
                // `pwm_duty_ex(_, 0)` returns early) but still enables the timer. With COMP1 and
                // CNTR both 0 the counter never advances, and every later `pwm_duty_ex` hangs
                // waiting for CNTR to leave COMP1's guard band. So initialize with 50% and then
                // apply 0. A smaller placeholder could still scale (`duty * period / 65536`) to 0
                // for a short period.
                let init_duty = if duty == 0 { 32768 } else { duty as u32 };
                pwm_init_ex(self.timer_num as c_int, self.rate, init_duty, 1);
                if duty == 0 {
                    pwm_duty_ex(self.timer_num as c_int, 0);
                }
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
