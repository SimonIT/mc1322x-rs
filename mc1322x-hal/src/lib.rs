//! A `embedded-hal`-based HAL for the NXP/Freescale MC1322x chip family (MC13224V/MC13226V),
//! usable on any board built around either part - not tied to one specific board's design.
//!
//! Peripheral drivers here (GPIO, UART, SPI, I2C, ADC, AES, PWM, RTC, NVM flash, CRM
//! sleep/wake, the async delay) expose the chip's own registers and behavior; board-specific
//! choices a caller makes per use (which pins something is wired to, an I2C bus's clock
//! divider) are always parameters, never hardcoded into a driver - [`i2c::I2c0::
//! BOARD_CLOCK_DIVIDER`] is only offered as a documented starting point, not applied
//! automatically. Board-specific values that are instead a *fixed physical property* of one
//! board - the crystal trim `rng::ensure_maca_ready` needs, and which bus a board's serial
//! flash is wired to ([`nvm::NvmInterface`]) - live in their own file under [`board`], one of
//! which is selected at compile time by a `board-*` Cargo feature (see [`board`]'s own doc
//! comment), and are resolved down to their final form at compile time too (e.g.
//! `power::BOARD_XTAL_CNTL`, the packed register value) rather than assembled at runtime from
//! parts, since a value that's already fixed at compile time has nothing left to compute later -
//! the smallest and fastest form it can take in the compiled binary. This distinction - runtime
//! parameter for a per-use choice, compile-time-selected file for a fixed board property - is
//! the pattern to follow for any future board-specific need here.
//!
//! [`entry!`] generates the `extern "C" fn main` every firmware binary using this crate needs;
//! [`critical_section_impl`] (private) supplies this crate's `critical_section::Impl` so any
//! consumer gets a working one without separate wiring - see its own doc comment.

#![no_std]

extern crate embedded_hal;
extern crate mc1322x_sys;

pub mod adc;
pub mod aes;

/// Board-specific constants, one file per board under `src/board/`, exactly one of which is
/// compiled in - selected here by a `board-*` Cargo feature via `#[path]`, so the rest of this
/// crate can just write `crate::board::WHATEVER` without needing to know or care which board
/// that resolves to.
///
/// Porting to a new board: add `src/board/my_board.rs` (providing the same items
/// `board/redbee_econotag.rs` does), a `board-my-board = []` feature in `Cargo.toml`, and a
/// matching arm below. Building with the new board's features instead
/// (`--no-default-features --features board-my-board`) then picks it up automatically.
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
pub mod rng;
pub mod rtc;
pub mod sleep;
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
