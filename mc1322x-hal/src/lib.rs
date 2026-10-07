//! Hardware abstraction layer for the NXP/Freescale MC1322x (MC13224V/MC13226V), an ARM7TDMI
//! microcontroller with an integrated IEEE 802.15.4 radio.
//!
//! The drivers implement the [`embedded-hal`](https://docs.rs/embedded-hal) 1.0 traits and, where
//! the peripheral has an interrupt, the [`embedded-hal-async`](https://docs.rs/embedded-hal-async)
//! traits as well. They build on the C startup code and boot-ROM bindings in `mc1322x-sys`.
//!
//! # Peripherals
//!
//! - [`adc`]: 12-bit ADC (channels 0-7 on GPIO0-7, plus the internal reference).
//! - [`aes`]: hardware AES-128 engine, exposed as an `embedded-cal` AES-CCM provider.
//! - [`delay`]: RTC-backed blocking and async delays.
//! - [`gpio`]: GPIO pins and edge-triggered KBI4-7 inputs.
//! - [`i2c`]: I2C master.
//! - [`nvm`]: serial flash, through the boot ROM's NVM routines.
//! - [`pwm`]: PWM output on the TMR timers.
//! - [`reset`]: software system reset.
//! - [`rng`]: hardware random number generator.
//! - [`rtc`]: free-running RTC tick counter.
//! - [`sleep`]: Hibernate/Doze low-power modes and wake-up sources.
//! - [`spi`]: SPI master.
//! - [`uart`]: UART1 and UART2.
//! - [`watchdog`]: COP watchdog.
//!
//! The crate also provides the `critical-section` implementation for the chip (it masks all
//! interrupts in the interrupt controller), so users don't need to supply their own.
//!
//! # Features
//!
//! - `board-redbee-econotag` (default): board-specific constants for the Redbee Econotag
//!   (crystal trim, serial flash interface, I2C clock divider). Exactly one `board-*` feature
//!   must be enabled.
//! - `fugit`: `now()` methods returning `fugit` instants on [`rtc::RtcRingOscillator`] and
//!   [`rtc::RtcCrystal`].
//! - `embedded-time`: implements `embedded_time::Clock` for [`rtc::RtcCrystal`].
//! - `task-watchdog`: implements `task_watchdog::HardwareWatchdog` for [`watchdog::Watchdog`].
//!
//! # Example
//!
//! ```ignore
//! #![no_std]
//! #![no_main]
//!
//! use embedded_hal::delay::DelayNs;
//! use embedded_hal::digital::OutputPin;
//! use mc1322x_hal::{delay::Delay, gpio::Pin};
//!
//! mc1322x_hal::entry!(app_main);
//!
//! #[panic_handler]
//! fn panic(_info: &core::panic::PanicInfo) -> ! {
//!     loop {}
//! }
//!
//! fn app_main() -> ! {
//!     // Green LED on the Redbee Econotag.
//!     let mut led = Pin::new(45).into_output(false);
//!     let mut delay = Delay::new();
//!     loop {
//!         led.set_high().unwrap();
//!         delay.delay_ms(500);
//!         led.set_low().unwrap();
//!         delay.delay_ms(500);
//!     }
//! }
//! ```

#![no_std]

extern crate embedded_hal;
extern crate mc1322x_sys;

pub mod adc;
pub mod aes;

/// Board-specific constants, selected by a `board-*` Cargo feature.
///
/// To add a board, create `src/board/my_board.rs` providing the same items as
/// `board/redbee_econotag.rs`, add a `board-my-board = []` feature to `Cargo.toml`, and add a
/// matching `#[cfg]`/`#[path]` declaration here.
#[cfg(feature = "board-redbee-econotag")]
#[path = "board/redbee_econotag.rs"]
mod board;

#[cfg(not(feature = "board-redbee-econotag"))]
compile_error!(
    "mc1322x-hal needs exactly one `board-*` feature enabled to select its board-specific \
     constants (see `crate::board`'s doc comment) - enable `board-redbee-econotag`, or port to \
     a new board by adding its own `board-*` feature and `src/board/*.rs` file."
);

mod critical_section_impl;
pub mod delay;
pub mod gpio;
pub mod i2c;
pub mod nvm;
mod power;
pub mod pwm;
pub mod reset;
pub mod rng;
pub mod rtc;
pub mod sleep;
pub mod spi;
pub mod uart;
mod util;
pub mod watchdog;

/// Define the program entry point.
///
/// The startup code (`start.S` in `libmc1322x`) jumps to an `extern "C" fn main() -> !`. This
/// macro generates that function and forwards to `$entry`, which must be a `fn() -> !`.
///
/// ```ignore
/// mc1322x_hal::entry!(arm_main);
///
/// fn arm_main() -> ! {
///     loop {}
/// }
/// ```
#[macro_export]
macro_rules! entry {
    ($entry:path) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn main() -> ! {
            let f: fn() -> ! = $entry;
            f()
        }
    };
}
