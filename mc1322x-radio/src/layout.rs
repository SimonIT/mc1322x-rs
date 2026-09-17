//! The 128-byte buffer layout shared between the radio driver and the layers
//! above it, plus the `RadioFrame`/token glue. This module is pure Rust and
//! does not touch any C symbols, so it can be exercised on the host.
//!
//! # Buffer layout
//!
//! The dot15d4 0.1.2 `Radio` API hands the driver an opaque 128-byte buffer
//! without an explicit length. This driver therefore uses the layout the API
//! evolved from (the nRF52840 802.15.4 radio, where the PHR is a hardware
//! register):
//!
//! ```text
//!        0         1                                127
//!      +-----+------------------------------------------+
//!      | PHR |              PSDU (without FCS)          |
//!      +-----+------------------------------------------+
//! ```
//!
//! - `buffer[0]` is the PHR: the PSDU length **including** the 2-byte FCS.
//!   Valid values are `2..=127`, i.e. a PSDU of 1..=125 bytes.
//! - `buffer[1..]` is the PSDU without FCS. The FCS is computed by the MACA
//!   coprocessor in hardware on TX and stripped on RX.
//!
//! The layer that builds a frame must write the PHR with [`set_frame_length`]
//! before handing the buffer to the radio; the ACK path does this
//! automatically through [`Mc1322xTxToken`].

/// Minimum valid PHR value (a PSDU of length 0 is meaningless).
pub const MIN_PHR: usize = 2;
/// Maximum valid PHR value (IEEE 802.15.4 PSDU including FCS).
pub const MAX_PHR: usize = 127;

/// Write the PHR (PSDU length including FCS) into `buffer[0]`.
///
/// `psdu_len` is the frame length without FCS.
pub fn set_frame_length(buffer: &mut [u8], psdu_len: usize) {
    buffer[0] = (psdu_len + 2) as u8;
}

/// Read the PSDU length (without FCS) from `buffer[0]`.
pub fn frame_length(buffer: &[u8]) -> usize {
    (buffer[0] as usize).saturating_sub(2)
}

/// Writable PSDU region of a prepared buffer, after the PHR at `buffer[0]`.
pub fn psdu_region(buffer: &mut [u8; 128]) -> &mut [u8] {
    let len = frame_length(buffer).min(buffer.len() - 1);
    &mut buffer[1..1 + len]
}

/// Read-only PSDU region of a prepared buffer.
pub fn psdu_view(buffer: &[u8; 128]) -> &[u8] {
    let len = frame_length(buffer).min(buffer.len() - 1);
    &buffer[1..1 + len]
}

/// Glue types implementing the [`dot15d4::phy::radio`] traits on top of the
/// buffer layout above.
#[cfg(feature = "dot15d4")]
pub mod dot15d4 {
    use dot15d4::phy::radio::{RadioFrame, RadioFrameMut, RxToken, TxToken};

    use super::{MAX_PHR, MIN_PHR, frame_length};

    /// Wraps a 128-byte buffer using the PHR layout above.
    ///
    /// [`RadioFrame::data`] yields the PSDU without FCS, so the MAC layer
    /// always parses a clean IEEE 802.15.4 MPDU (FCF at offset 0).
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

    /// RX token. The dot15d4 0.1.2 CSMA layer does not use it (RX goes
    /// through [`Radio::receive`](dot15d4::phy::radio::Radio::receive)), but
    /// the trait requires it.
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

    /// TX token. Records the frame length into the PHR and hands the writer
    /// only the PSDU region, so the radio driver needs a single source of
    /// truth for the length of the frame it is about to transmit.
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

/// Buffer glue for the [`ieee802154`] crate's MAC frame codec.
///
/// Unlike `dot15d4`, `ieee802154` has no `Radio` trait to implement: it is
/// purely a frame encoder/decoder. [`write_frame`] and [`read_frame`] convert
/// between an [`ieee802154::mac::Frame`] and the PHR+PSDU buffer layout
/// documented above, reusing the same [`set_frame_length`]/[`frame_length`]
/// bookkeeping as the `dot15d4` glue. The FCS is always handled in
/// [`FooterMode::None`](ieee802154::mac::FooterMode::None): the MACA hardware
/// computes/strips it, so it must not appear in the PSDU.
///
/// [`crate::Mc1322xRadio::send_frame`]/[`crate::Mc1322xRadio::receive_frame`]
/// call these internally to fold the codec into the transmit/receive path;
/// they're kept public here as the lower-level building block for callers
/// who want to manage the 128-byte buffer themselves.
#[cfg(feature = "ieee802154")]
pub mod ieee802154 {
    use byte::{TryRead, TryWrite};
    use ieee802154::mac::{FooterMode, Frame, FrameSerDesContext};

    use super::{MAX_PHR, psdu_view, set_frame_length};

    /// Encode `frame` into `buffer`'s PSDU region and set the PHR.
    ///
    /// Returns the number of PSDU bytes written (without FCS).
    pub fn write_frame(buffer: &mut [u8; 128], frame: Frame<'_>) -> byte::Result<usize> {
        let mut ctx = FrameSerDesContext::no_security(FooterMode::None);
        let capacity = (MAX_PHR - 2).min(buffer.len() - 1);
        let len = frame.try_write(&mut buffer[1..1 + capacity], &mut ctx)?;
        set_frame_length(buffer, len);
        Ok(len)
    }

    /// Decode the frame in `buffer`'s PSDU region (as set by [`write_frame`]
    /// or the radio driver's receive path).
    pub fn read_frame(buffer: &[u8; 128]) -> byte::Result<(Frame<'_>, usize)> {
        Frame::try_read(psdu_view(buffer), FooterMode::None)
    }
}
