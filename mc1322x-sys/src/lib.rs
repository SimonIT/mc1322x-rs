//! Raw `bindgen` bindings to libmc1322x, the C register-level driver library for the NXP
//! MC1322x (MC13224V/MC13226V).
//!
//! The library is built from the `libmc1322x` submodule and linked in by the build script, which
//! needs `arm-none-eabi-gcc`, `arm-none-eabi-ar` and `make`. For a safe Rust API, use
//! `mc1322x-hal`.

#![no_std]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
