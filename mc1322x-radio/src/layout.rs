//! The 128-byte frame buffer layout used by [`Mc1322xRadio`](crate::Mc1322xRadio), plus glue
//! types for the optional frame crates.
//!
//! This module is plain Rust and does not touch any C symbols, so it can be used on the host.
//!
//! # Buffer layout
//!
//! The `dot15d4` `Radio` API passes the driver an opaque 128-byte buffer without a separate length,
//! so the length is stored in the buffer itself, as on the nRF52840 where the PHR is part of the
//! radio's packet buffer:
//!
//! ```text
//!        0         1                                127
//!      +-----+------------------------------------------+
//!      | PHR |              PSDU (without FCS)          |
//!      +-----+------------------------------------------+
//! ```
//!
//! - `buffer[0]` is the PHR: the PSDU length **including** the 2-byte FCS. Valid values are
//!   [`MIN_PHR`]`..=`[`MAX_PHR`], i.e. 0..=125 bytes of PSDU without FCS.
//! - `buffer[1..]` is the PSDU without FCS. The MACA appends the FCS on TX and strips it on RX.
//!
//! Code that builds a frame must set the PHR with [`set_frame_length`] before handing the buffer
//! to the radio.

/// Minimum valid PHR value: a PSDU consisting of only the FCS.
pub const MIN_PHR: usize = 2;
/// Maximum valid PHR value (IEEE 802.15.4 PSDU including FCS).
pub const MAX_PHR: usize = 127;

/// Write the PHR (PSDU length including FCS) into `buffer[0]`.
///
/// `psdu_len` is the frame length without FCS and must not exceed 125.
///
/// # Panics
///
/// Panics if `buffer` is empty.
pub fn set_frame_length(buffer: &mut [u8], psdu_len: usize) {
    buffer[0] = (psdu_len + 2) as u8;
}

/// Read the PSDU length (without FCS) from `buffer[0]`.
///
/// # Panics
///
/// Panics if `buffer` is empty.
pub fn frame_length(buffer: &[u8]) -> usize {
    (buffer[0] as usize).saturating_sub(2)
}

/// Return the PSDU (without FCS) of a buffer, as given by its PHR.
pub fn psdu_region(buffer: &mut [u8; 128]) -> &mut [u8] {
    let len = frame_length(buffer).min(buffer.len() - 1);
    &mut buffer[1..1 + len]
}

/// Return the PSDU (without FCS) of a buffer, as given by its PHR.
pub fn psdu_view(buffer: &[u8; 128]) -> &[u8] {
    let len = frame_length(buffer).min(buffer.len() - 1);
    &buffer[1..1 + len]
}

#[cfg(feature = "dot15d4")]
pub mod dot15d4 {
    //! Types implementing the [`dot15d4::phy::radio`] frame and token traits on top of the
    //! [buffer layout](super#buffer-layout).

    use dot15d4::phy::radio::{RadioFrame, RadioFrameMut, RxToken, TxToken};

    use super::{MAX_PHR, MIN_PHR, frame_length};

    /// A frame buffer in the PHR+PSDU layout.
    ///
    /// [`RadioFrame::data`] returns the PSDU without FCS, starting with the frame control
    /// field.
    pub struct Mc1322xFrame<T: AsRef<[u8]>> {
        buffer: T,
    }

    impl<T: AsRef<[u8]>> RadioFrame<T> for Mc1322xFrame<T> {
        type Error = ();

        fn new_unchecked(buffer: T) -> Self {
            Self { buffer }
        }

        fn new_checked(buffer: T) -> Result<Self, Self::Error> {
            let b = buffer.as_ref();
            let phr = b[0] as usize;
            if (MIN_PHR..=MAX_PHR).contains(&phr) && b.len() >= 1 + (phr - 2) {
                Ok(Self { buffer })
            } else {
                Err(())
            }
        }

        fn data(&self) -> &[u8] {
            let b = self.buffer.as_ref();
            let len = frame_length(b).min(b.len().saturating_sub(1));
            &b[1..1 + len]
        }
    }

    impl<T: AsRef<[u8]> + AsMut<[u8]>> RadioFrameMut<T> for Mc1322xFrame<T> {
        fn data_mut(&mut self) -> &mut [u8] {
            let b = self.buffer.as_mut();
            let len = frame_length(b).min(b.len().saturating_sub(1));
            &mut b[1..1 + len]
        }
    }

    /// RX token for a buffer in the PHR+PSDU layout.
    ///
    /// The `dot15d4` CSMA layer receives through
    /// [`Radio::receive`](dot15d4::phy::radio::Radio::receive) and does not use this token; it
    /// exists because the trait requires it.
    pub struct Mc1322xRxToken<'a> {
        buffer: &'a mut [u8],
    }

    impl<'a> RxToken for Mc1322xRxToken<'a> {
        fn consume<F, R>(self, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            let len = frame_length(self.buffer).min(self.buffer.len().saturating_sub(1));
            f(&mut self.buffer[1..1 + len])
        }
    }

    impl<'a> From<&'a mut [u8]> for Mc1322xRxToken<'a> {
        fn from(value: &'a mut [u8]) -> Self {
            Self { buffer: value }
        }
    }

    /// TX token for a buffer in the PHR+PSDU layout.
    ///
    /// [`TxToken::consume`] writes the PHR for `len` (clamped to 125 bytes) and passes only the
    /// PSDU region to the closure.
    pub struct Mc1322xTxToken<'a> {
        buffer: &'a mut [u8],
    }

    impl<'a> TxToken for Mc1322xTxToken<'a> {
        fn consume<F, R>(self, len: usize, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            let len = len.min(MAX_PHR - 2);
            self.buffer[0] = (len + 2) as u8;
            f(&mut self.buffer[1..1 + len])
        }
    }

    impl<'a> From<&'a mut [u8]> for Mc1322xTxToken<'a> {
        fn from(value: &'a mut [u8]) -> Self {
            Self { buffer: value }
        }
    }
}

#[cfg(feature = "ieee802154")]
pub mod ieee802154 {
    //! Conversion between an [`ieee802154::mac::Frame`] and the
    //! [buffer layout](super#buffer-layout).
    //!
    //! Frames are encoded and decoded with
    //! [`FooterMode::None`], since the MACA handles the FCS.
    //! [`Mc1322xRadio::send_frame`](crate::Mc1322xRadio::send_frame) and
    //! [`Mc1322xRadio::receive_frame`](crate::Mc1322xRadio::receive_frame) use these functions
    //! internally.

    use byte::{TryRead, TryWrite};
    use ieee802154::mac::{FooterMode, Frame, FrameSerDesContext};

    use super::{MAX_PHR, psdu_view, set_frame_length};

    /// Encode `frame` into `buffer`'s PSDU region and set the PHR.
    ///
    /// Returns the number of PSDU bytes written (without FCS).
    ///
    /// # Errors
    ///
    /// Returns an error if `frame` cannot be serialized, e.g. because it is longer than 125
    /// bytes.
    pub fn write_frame(buffer: &mut [u8; 128], frame: Frame<'_>) -> byte::Result<usize> {
        let mut ctx = FrameSerDesContext::no_security(FooterMode::None);
        let capacity = (MAX_PHR - 2).min(buffer.len() - 1);
        let len = frame.try_write(&mut buffer[1..1 + capacity], &mut ctx)?;
        set_frame_length(buffer, len);
        Ok(len)
    }

    /// Decode the frame in `buffer`'s PSDU region.
    ///
    /// Returns the frame and the number of bytes consumed.
    ///
    /// # Errors
    ///
    /// Returns an error if the PSDU is not a valid IEEE 802.15.4 frame.
    pub fn read_frame(buffer: &[u8; 128]) -> byte::Result<(Frame<'_>, usize)> {
        Frame::try_read(psdu_view(buffer), FooterMode::None)
    }
}
