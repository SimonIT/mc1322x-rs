//! Master-mode SPI, implementing the blocking `embedded-hal` and async `embedded-hal-async`
//! [`SpiBus`] traits.
//!
//! # Interrupts
//!
//! The async implementation waits on the SPI completion interrupt (`INT_NUM_SPI`), dispatched by
//! `libmc1322x`'s `irq()` to this module's `spi_isr`.

use core::convert::Infallible;
use core::task::Poll;
use embedded_hal::spi::{self, SpiBus};
use mc1322x_sys::{
    INTBASE, INTDISNUM_OFF, INTENNUM_OFF, REF_OSC, gpio_select_function, gpio_set_pad_dir,
    interrupt_nums_INT_NUM_SPI,
};

use crate::util::WakerCell;

const SPI_BASE: usize = 0x8000_2000;

#[allow(clippy::identity_op)]
const SPI_TX_DATA: usize = SPI_BASE + 0x00;
const SPI_RX_DATA: usize = SPI_BASE + 0x04;
const SPI_CLK_CTRL: usize = SPI_BASE + 0x08;
const SPI_SETUP: usize = SPI_BASE + 0x0C;
const SPI_STATUS: usize = SPI_BASE + 0x10;

const SPI_START: u32 = 1 << 7;
const SPI_SCK_COUNT_SHIFT: u32 = 8;

const SPI_SS_SETUP_SHIFT: u32 = 0;
const SPI_SDO_INACTIVE_ST_SHIFT: u32 = 4;
const SPI_SCK_POL: u32 = 1 << 8;
const SPI_SCK_PHASE: u32 = 1 << 9;
const SPI_SCK_FREQ_SHIFT: u32 = 12;

const SPI_INT: u32 = 1 << 0;

const DATA_LENGTH: u32 = 8;
// RM Table 15-6: "the number of SPI_SCK periods is equal to SPI_SCK_COUNT + 1", so this must
// be one less than DATA_LENGTH to generate exactly one SCK period per shifted data bit.
const SCK_COUNT: u32 = DATA_LENGTH - 1;

const SPI_ALT_FUNCTION: u8 = 1;

const SPI_SCK_PIN: u8 = 7;
const SPI_MOSI_PIN: u8 = 6;
const SPI_MISO_PIN: u8 = 5;

const PAD_DIR_INPUT: u8 = 0;
const PAD_DIR_OUTPUT: u8 = 1;

// ITC number of the SPI completion interrupt. SPI_INT has no local mask bit (RM §15.5.5), so
// unlike I2C/UART, `Spi::transfer_word_async` arms the ITC channel itself via `INTENNUM` and
// `spi_isr` disables it again via `INTDISNUM`. `irq()` dispatches to the weak `spi_isr` symbol
// overridden at the bottom of this file; that dispatch exists only in this project's
// `libmc1322x` fork (`src/isr.c`), not upstream.
const INT_NUM_SPI: u32 = interrupt_nums_INT_NUM_SPI;

/// Waker for the in-flight [`Spi::transfer_word_async`] call, if any.
///
/// Set by `transfer_word_async` before it returns `Pending`, woken by [`spi_isr`]. One slot is
/// enough: there is only one SPI peripheral and one transfer in flight at a time.
static WAKER: WakerCell = WakerCell::new();

/// SPI bus clock mode.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Mode {
    /// CPOL = 0, CPHA = 0
    Mode0,
    /// CPOL = 0, CPHA = 1
    Mode1,
    /// CPOL = 1, CPHA = 0
    Mode2,
    /// CPOL = 1, CPHA = 1
    Mode3,
}

/// Master-mode SPI bus on GPIO5 (MISO), GPIO6 (MOSI) and GPIO7 (SCK).
///
/// Chip select is not managed by this driver: GPIO4 (`SPI_SS`) is left in GPIO mode. Drive
/// chip select from a GPIO output around each transaction yourself, e.g. through an
/// `embedded-hal-bus` `SpiDevice`.
pub struct Spi;

impl Spi {
    /// Create a master SPI bus running at up to `frequency` Hz.
    ///
    /// Picks the highest SCK rate that doesn't exceed `frequency`: `REF_OSC / 2^(n + 1)` for
    /// `n` in 0..=7, i.e. 12 MHz down to 93.75 kHz with the 24 MHz reference oscillator. Below
    /// 93.75 kHz, the slowest rate is used.
    ///
    /// Configures GPIO5 (MISO), GPIO6 (MOSI) and GPIO7 (SCK) for SPI.
    pub fn new(frequency: u32, mode: Mode) -> Self {
        unsafe {
            gpio_select_function(SPI_SCK_PIN, SPI_ALT_FUNCTION);
            gpio_select_function(SPI_MOSI_PIN, SPI_ALT_FUNCTION);
            gpio_select_function(SPI_MISO_PIN, SPI_ALT_FUNCTION);

            gpio_set_pad_dir(SPI_SCK_PIN, PAD_DIR_OUTPUT);
            gpio_set_pad_dir(SPI_MOSI_PIN, PAD_DIR_OUTPUT);
            gpio_set_pad_dir(SPI_MISO_PIN, PAD_DIR_INPUT);
        }

        let sck_freq = spi_sck_freq_divisor(frequency) as u32;
        let mut setup = sck_freq << SPI_SCK_FREQ_SHIFT;
        setup |= match mode {
            Mode::Mode0 => 0,
            Mode::Mode1 => SPI_SCK_PHASE,
            Mode::Mode2 => SPI_SCK_POL,
            Mode::Mode3 => SPI_SCK_POL | SPI_SCK_PHASE,
        };
        setup |= 0b11 << SPI_SDO_INACTIVE_ST_SHIFT;
        // SS_SETUP = 0b10: SPI_SS_OUT held low (RM Table 15-10); the pad isn't muxed to SPI.
        setup |= 0b10 << SPI_SS_SETUP_SHIFT;

        unsafe {
            (SPI_SETUP as *mut u32).write_volatile(setup);
        }

        Spi
    }

    /// Start one 8-bit transfer; [`Self::poll_transfer_status`] reports its completion.
    fn start_transfer(&mut self, tx: u8) {
        unsafe {
            (SPI_TX_DATA as *mut u32).write_volatile((tx as u32) << 24);
            (SPI_CLK_CTRL as *mut u32)
                .write_volatile((SCK_COUNT << SPI_SCK_COUNT_SHIFT) | SPI_START | DATA_LENGTH);
        }
    }

    /// Check once whether the transfer started by [`Self::start_transfer`] has completed, and if
    /// so, clear `SPI_INT` and return the received byte.
    fn poll_transfer_status(&mut self) -> Poll<u8> {
        unsafe {
            if (SPI_STATUS as *const u32).read_volatile() & SPI_INT == 0 {
                return Poll::Pending;
            }
            let rx = (SPI_RX_DATA as *const u32).read_volatile() & 0xFF;
            (SPI_STATUS as *mut u32).write_volatile(SPI_INT);
            Poll::Ready(rx as u8)
        }
    }

    fn transfer_word(&mut self, tx: u8) -> u8 {
        self.start_transfer(tx);
        loop {
            if let Poll::Ready(rx) = self.poll_transfer_status() {
                return rx;
            }
        }
    }

    /// Async equivalent of [`Self::transfer_word`]: waits for [`spi_isr`] instead of polling.
    ///
    /// Holds a [`crate::sleep::SleepInhibitGuard`] while the transfer is in flight.
    async fn transfer_word_async(&mut self, tx: u8) -> u8 {
        self.start_transfer(tx);
        let mut inhibit = None;
        core::future::poll_fn(|cx| {
            // Arm the ITC channel *outside* the critical section: the critical section restores
            // the saved `INTENABLE` on exit, which would undo an `INTENNUM` write made inside
            // it. Re-arming on every poll is harmless.
            unsafe {
                core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_SPI);
            }
            // Check and register the waker atomically, so a completion in between isn't missed.
            critical_section::with(|cs| {
                if let Poll::Ready(rx) = self.poll_transfer_status() {
                    return Poll::Ready(rx);
                }
                inhibit.get_or_insert_with(crate::sleep::SleepInhibitGuard::new);
                WAKER.set(cs, cx.waker());
                Poll::Pending
            })
        })
        .await
    }
}

fn spi_sck_freq_divisor(frequency: u32) -> u8 {
    let mut freq = 0;
    while freq < 7 && (REF_OSC >> (freq + 1)) > frequency {
        freq += 1;
    }
    freq
}

impl spi::ErrorType for Spi {
    type Error = Infallible;
}

impl SpiBus<u8> for Spi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            *word = self.transfer_word(0xFF);
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &word in words {
            self.transfer_word(word);
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        for i in 0..read.len() {
            let w = if i < write.len() { write[i] } else { 0xFF };
            read[i] = self.transfer_word(w);
        }
        for &w in &write[read.len().min(write.len())..] {
            self.transfer_word(w);
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            *word = self.transfer_word(*word);
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Waits on the SPI completion interrupt between bytes instead of busy-polling.
impl embedded_hal_async::spi::SpiBus<u8> for Spi {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            *word = self.transfer_word_async(0xFF).await;
        }
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &word in words {
            self.transfer_word_async(word).await;
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        for i in 0..read.len() {
            let w = if i < write.len() { write[i] } else { 0xFF };
            read[i] = self.transfer_word_async(w).await;
        }
        for &w in &write[read.len().min(write.len())..] {
            self.transfer_word_async(w).await;
        }
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            *word = self.transfer_word_async(*word).await;
        }
        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// SPI completion interrupt handler.
///
/// Overrides the weak `spi_isr` symbol in this project's `libmc1322x` fork (see the comment
/// above `INT_NUM_SPI`).
///
/// Doesn't touch `SPI_STATUS`: [`Spi::poll_transfer_status`] clears `SPI_INT` and reads the
/// byte from task context, as on the blocking path. This only disables the ITC's SPI channel,
/// so `irq()` doesn't re-enter the handler forever, and wakes the waiting task.
///
/// `irq()` runs in ARM state and must call this Thumb code with interworking (`bx`).
#[unsafe(no_mangle)]
extern "C" fn spi_isr() {
    unsafe {
        core::ptr::write_volatile((INTBASE + INTDISNUM_OFF) as *mut u32, INT_NUM_SPI);
    }
    WAKER.wake();
}
