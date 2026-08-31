//! Embassy platform support for the NXP MC1322x.
//!
//! The MC1322x is a single-core ARM7TDMI (ARMv4T) system-on-chip. Because it predates the ARMv7
//! atomics and its Thumb instruction set cannot mask interrupts from the CPSR directly, every
//! Embassy port for it needs a [`critical_section`] implementation that masks all interrupts by
//! clearing the ITC's `INTENABLE` register. That implementation lives in `mc1322x-hal` (as a
//! private module), not here — `critical_section::Impl` registration is link-wide and singular,
//! and `mc1322x-hal` is the crate every consumer, HAL or Embassy alike, already needs — so this
//! crate just depends on it for that side effect (see the `use mc1322x_hal as _;` below).
//!
//! What this crate itself provides is a 1 kHz `embassy-time` driver based on a free-running TMR0
//! compare interrupt ([`time_driver`]).
//!
//! Everything else — the executor, `Spawner`, the `#[task]` macro — comes from the real
//! [`embassy-executor`](https://docs.rs/embassy-executor) crate, using its `platform-spin` backend
//! (the MC1322x has no `WFI`/`WFE` instruction, so the thread executor busy-polls instead of
//! sleeping). `embassy-executor` requires atomics that this chip doesn't have either; this
//! workspace vendors a small patched copy that plugs the one spot that needed them into
//! `mc1322x-hal`'s `critical_section` implementation instead (see `vendor/README.md` for what and
//! why).
//!
//! # Usage
//!
//! ```ignore
//! use embassy_executor::{Executor, Spawner};
//! use embassy_time::{Duration, Timer};
//! use static_cell::StaticCell;
//!
//! #[embassy_executor::task]
//! async fn my_task(_spawner: Spawner) {
//!     loop {
//!         Timer::after(Duration::from_secs(1)).await;
//!     }
//! }
//!
//! static EXECUTOR: StaticCell<Executor> = StaticCell::new();
//!
//! fn main() -> ! {
//!     mc1322x_embassy::init();
//!     let executor = EXECUTOR.init(Executor::new());
//!     executor.run(|spawner| {
//!         spawner.spawn(my_task(spawner).unwrap());
//!     })
//! }
//! ```
//!
//! (This example can't be compiled as a doctest: it targets a `no_std`, no-`panic_handler`,
//! bare-metal ARM target that rustdoc's test harness can't build for.)

#![no_std]

// Depended on purely for its `critical_section::Impl` side effect (see the module docs above),
// not for any item this crate's own code calls.
use mc1322x_hal as _;

pub mod time_driver;

/// Bring up the chip-specific Embassy platform pieces.
///
/// Currently this just starts the [`time_driver`]; call it once, before running your executor.
/// Idempotent, and safe to call alongside [`mc1322x_hal::rng::ensure_maca_ready`] or
/// `Mc1322xRadio::init` (none of them touch each other's hardware).
pub fn init() {
    time_driver::init();
}
