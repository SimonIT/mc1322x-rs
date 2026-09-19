//! Raw `bindgen`-generated FFI bindings to `libmc1322x` (vendored as a submodule, see
//! `build.rs`), the C register-level driver library for the NXP/Freescale MC1322x chip family
//! (MC13224V/MC13226V). Chip-generic: nothing here is specific to any one board built around
//! either part - see `mc1322x-hal` for a safe, `embedded-hal`-based Rust API built on top of
//! this.

#![no_std]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
