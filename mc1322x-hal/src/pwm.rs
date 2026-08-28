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
                // Initializing directly with duty=0 leaves COMP1/LOAD/CNTR all zero
                // (`pwm_init_ex`'s own internal `pwm_duty_ex(_, 0)` call takes an early-return
                // path that never programs them, then unconditionally enables the timer
                // anyway): with COMP1 and CNTR both 0, the compare self-matches every tick
                // and CNTR never actually advances. A *later* `set_duty_cycle` call's
                // `pwm_duty_ex` busy-waits for CNTR to move away from COMP1 by more than a
                // guard band before it's safe to retime, which never happens if CNTR is
                // pinned at 0 - that call hangs forever. Avoid the degenerate state by
                // initializing with a placeholder duty of 50% when the caller actually wants
                // 0, then immediately applying the real duty - safe once the counter isn't
                // pinned. 50% is deliberately generous rather than "just barely nonzero":
                // `pwm_duty_ex` scales duty by the timer's period (`duty * period / 65536`,
                // rounded), and a too-small placeholder can still round down to a scaled duty
                // of 0 for a small period, reproducing the same hang.
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
