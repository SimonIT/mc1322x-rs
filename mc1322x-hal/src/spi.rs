use core::convert::Infallible;
use embedded_hal::spi::{self, SpiBus};
use mc1322x_sys::{REF_OSC, gpio_select_function, gpio_set_pad_dir};

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
const SCK_COUNT: u32 = 8;

const SPI_ALT_FUNCTION: u8 = 1;

const SPI_SCK_PIN: u8 = 7;
const SPI_MOSI_PIN: u8 = 6;
const SPI_MISO_PIN: u8 = 5;

const PAD_DIR_INPUT: u8 = 0;
const PAD_DIR_OUTPUT: u8 = 1;

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
/// The chip-select pin (GPIO4) is not managed by this driver: de-assert it by
/// holding a GPIO output low while a transfer is in progress (i.e. while the
/// [`Spi`] handle is borrowed for a [`SpiBus`] operation) and release it
/// afterwards. SPI_SS is held de-asserted at all times.
pub struct Spi;

impl Spi {
    /// Create a master SPI bus running at up to `frequency` Hz.
    ///
    /// The SCK divider is chosen as the highest clock rate that does not
    /// exceed `frequency`, assuming a 24 MHz peripheral reference clock
    /// (`REF_OSC`, the default `xtal_clkdiv` of 1). Actual rates are
    /// `REF_OSC / 2^(FREQ+1)` for `FREQ` in 0..=7, i.e. 12 MHz down to
    /// 93.75 kHz.
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
        setup |= 0b10 << SPI_SS_SETUP_SHIFT;

        unsafe {
            (SPI_SETUP as *mut u32).write_volatile(setup);
        }

        Spi
    }

    fn transfer_word(&mut self, tx: u8) -> u8 {
        unsafe {
            (SPI_TX_DATA as *mut u32).write_volatile((tx as u32) << 24);
            (SPI_CLK_CTRL as *mut u32)
                .write_volatile((SCK_COUNT << SPI_SCK_COUNT_SHIFT) | SPI_START | DATA_LENGTH);
            while (SPI_STATUS as *const u32).read_volatile() & SPI_INT == 0 {}
            let rx = (SPI_RX_DATA as *const u32).read_volatile() & 0xFF;
            (SPI_STATUS as *mut u32).write_volatile(SPI_INT);
            rx as u8
        }
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
