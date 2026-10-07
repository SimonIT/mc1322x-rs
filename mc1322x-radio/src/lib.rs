//! IEEE 802.15.4 radio driver for the MC1322x MACA coprocessor.
//!
//! [`Mc1322xRadio`] moves raw PHR+PSDU buffers (see [`layout`]) in and out of the radio through
//! the `libmc1322x` packet pool, using a small set of async primitives that are independent of
//! any 802.15.4 frame crate. The optional features below connect these primitives to higher-level
//! stacks.
//!
//! The FCS is generated and checked by the MACA in hardware; buffers handed to and returned by the
//! driver never contain it. Hardware CCA is not used: frames are transmitted immediately, and
//! contention has to be handled in software (e.g. by the `dot15d4` CSMA-CA layer).
//!
//! [`layout`] is plain Rust and can be used in host-side tests; [`Mc1322xRadio`] links against
//! `libmc1322x` and only builds for the MC1322x.
//!
//! # Features
//!
//! - `dot15d4`: implements [`dot15d4::phy::radio::Radio`] for [`Mc1322xRadio`] and adds the
//!   frame/token types in [`layout::dot15d4`].
//! - `ieee802154`: adds [`Mc1322xRadio::send_frame`] and [`Mc1322xRadio::receive_frame`], which
//!   encode/decode an [`ieee802154::mac::Frame`] as part of the transmit/receive call, and the
//!   lower-level buffer codec in [`layout::ieee802154`].
//! - `radio-hal`: implements the [`radio`](https://docs.rs/radio) crate's blocking traits, see
//!   [`radio_hal`].
//!
//! All features are off by default and can be combined freely.
//!
//! # Example
//!
//! ```no_run
//! use mc1322x_radio::{Mc1322xRadio, layout};
//!
//! async fn demo() {
//!     let mut radio = Mc1322xRadio::init([0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef]);
//!     radio.enable().await;
//!     radio.set_channel(26);
//!
//!     let mut buf = [0u8; 128];
//!     // Safety: `buf` outlives the receive operation.
//!     unsafe { radio.prepare_receive(&mut buf) }.await;
//!     if radio.receive().await {
//!         let psdu = layout::psdu_view(&buf);
//!         // ...
//!     }
//! }
//! ```

#![no_std]
#![warn(missing_docs)]

pub mod layout;
mod radio;

pub use radio::Mc1322xRadio;
#[cfg(feature = "radio-hal")]
pub use radio::radio_hal;
