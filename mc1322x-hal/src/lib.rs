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

/// Defines the `extern "C" fn main() -> !` entry point that `libmc1322x`'s `start.S` calls
/// directly by that exact symbol name, forwarding to `$entry` (a plain `fn() -> !`).
///
/// `start.S` (compiled C, not Rust) has `main`'s C-ABI symbol name and calling convention
/// hardcoded - every firmware binary in this project needs *some* `#[unsafe(no_mangle)] pub
/// extern "C" fn main() -> !` to satisfy that, but the real application logic doesn't need to
/// be written as `extern "C"` itself. This just generates that one, tiny, otherwise-identical
/// trampoline, so it doesn't need to be hand-written (and kept syntactically correct - the
/// exact `#[unsafe(no_mangle)]` form is edition 2024-specific) in every firmware crate.
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
