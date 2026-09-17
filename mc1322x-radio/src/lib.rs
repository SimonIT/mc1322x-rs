//! IEEE 802.15.4 radio driver for the MC1322x.
//!
//! This crate drives the MC1322x's MACA coprocessor through a small set of
//! frame-crate-agnostic async primitives on [`Mc1322xRadio`]. Two optional,
//! independent features build on top of those primitives to connect to a
//! higher-level 802.15.4 frame representation:
//!
//! - `dot15d4`: implements [`dot15d4::phy::radio::Radio`],
//!   connecting the `dot15d4` CSMA/MAC stack to real hardware. Adds
//!   [`layout::dot15d4`]'s `RadioFrame`/token glue types.
//! - `ieee802154`: adds [`Mc1322xRadio::send_frame`] and
//!   [`Mc1322xRadio::receive_frame`], which fold encoding/decoding an
//!   [`ieee802154::mac::Frame`] directly into the transmit/receive path, with
//!   no `Radio` trait and no CSMA/ACK handling in between — the lower-level
//!   [`layout::ieee802154::write_frame`]/[`layout::ieee802154::read_frame`]
//!   codec these build on stays available for manual buffer management.
//! - `radio-hal`: implements the [`rust-iot/radio-hal`](https://docs.rs/radio)
//!   crate's `Transmit`/`Receive`/`State`/`Channel`/`Busy` traits. See
//!   [`radio_hal`] for what's intentionally left out and why.
//!
//! Both features may be enabled together, independently, or not at all (core
//! primitives only). [`layout`] itself — the 128-byte buffer layout (PHR +
//! PSDU) and its plain byte-slice helpers — has no feature requirement and is
//! pure Rust, usable from host-side tests. [`Mc1322xRadio`] is built on the
//! `libmc1322x` packet pool API (`mc1322x-sys`) and is linkable only for
//! `arm` targets.
//!
//! # Example
//!
//! ```no_run
//! use dot15d4::phy::radio::Radio;
//! use mc1322x_radio::Mc1322xRadio;
//!
//! async fn demo() {
//!     let mut radio = Mc1322xRadio::init([0; 8]);
//!     radio.enable().await;
//! }
//! ```

#![no_std]
#![warn(missing_docs)]

pub mod layout;
mod radio;

pub use radio::Mc1322xRadio;
#[cfg(feature = "radio-hal")]
pub use radio::radio_hal;
