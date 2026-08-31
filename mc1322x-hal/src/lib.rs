#![no_std]

extern crate embedded_hal;
extern crate mc1322x_sys;

pub mod adc;
pub mod aes;
mod critical_section_impl;
pub mod delay;
pub mod gpio;
pub mod i2c;
pub mod nvm;
mod power;
pub mod pwm;
pub mod rng;
pub mod rtc;
pub mod spi;
pub mod uart;
mod util;
