//! Embassy platform support for the NXP MC1322x.
//!
//! This crate provides the chip-specific pieces an [`embassy-executor`] application needs:
//!
//! - a 1 kHz `embassy-time` driver on TMR0 ([`time_driver`], started by [`init`]),
//! - an optional sleep-aware executor that enters CRM `Doze` while idle (`sleepy_executor`),
//! - an optional `task-watchdog` integration on the COP hardware watchdog (`task_watchdog`).
//!
//! The `critical_section` implementation lives in `mc1322x-hal` (it masks all interrupts through
//! the ITC's `INTENABLE` register, since Thumb code cannot mask them in the CPSR); this crate
//! links it in by depending on `mc1322x-hal`. The ARMv4T core has no atomic instructions, so
//! `embassy-executor` is used with its `portable-atomic` feature, which falls back to that
//! critical section.
//!
//! With the plain `embassy_executor::Executor`, the application enables `embassy-executor`'s
//! `platform-spin` feature: the MC1322x has no `WFI`/`WFE`, so the executor busy-polls.
//!
//! [`embassy-executor`]: https://docs.rs/embassy-executor
//!
//! # Features
//!
//! - `defmt`: implement `defmt::Format` for this crate's public types and enable
//!   `embassy-time/defmt`.
//! - `sleepy-executor`: enable `sleepy_executor::SleepyExecutor`, which enters CRM `Doze`
//!   between polls instead of busy-polling. It registers its own executor `Pender`, so a binary
//!   using this feature must not enable any `embassy-executor` `platform-*` feature (such as
//!   `platform-spin`), or linking fails with a duplicate `__pender`.
//! - `task-watchdog`: enable `task_watchdog`, which feeds the COP watchdog only while every
//!   registered task keeps checking in (built on the
//!   [`task-watchdog`](https://docs.rs/task-watchdog) crate).
//!
//! # Example
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

#![no_std]
#![warn(missing_docs)]

// Linked in for its `critical_section::Impl`.
use mc1322x_hal as _;

#[cfg(feature = "sleepy-executor")]
pub mod sleepy_executor;
#[cfg(feature = "task-watchdog")]
pub mod task_watchdog;
pub mod time_driver;

/// Initialize the Embassy platform: start the [`time_driver`].
///
/// Call this before running the executor. Calling it again does nothing.
pub fn init() {
    time_driver::init();
}
