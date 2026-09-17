use core::convert::Infallible;
use core::task::Poll;
use embedded_hal::spi::{self, SpiBus};
use mc1322x_sys::{INTBASE, REF_OSC, gpio_select_function, gpio_set_pad_dir};

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

// ITC (interrupt controller) offset/number for the SPI completion interrupt (see `isr.h`'s
// `INTENNUM_OFF`/`INTDISNUM_OFF` and `interrupt_nums`), following the same wiring as
// `crate::i2c`'s `INT_NUM_I2C`. Per the MC1322x Reference Manual §15.5.5: "There is no local
// mask bit for the SPI_INT. The interrupt for the module is enabled/disabled via the
// Interrupt Controller (ITC)." So unlike I2C/UART, [`Spi::transfer_word_async`] toggles the
// ITC channel itself — writing `INT_NUM_SPI` to `INTENNUM` to arm it, and [`spi_isr`] writing
// it to `INTDISNUM` to quiesce it — the same technique `libmc1322x`'s own small-RX-buffer
// `uart1_isr`/`uart1_putc` use for UART1's TX interrupt. `irq()` (linked from `libmc1322x`)
// dispatches to the weak `spi_isr` symbol overridden at the bottom of this file; that
// dispatch is this crate's own addition (see `mc1322x-sys/libmc1322x/src/isr.c`), not present
// in upstream `libmc1322x`.
const INTENNUM_OFF: u32 = 0x8;
const INTDISNUM_OFF: u32 = 0xC;
const INT_NUM_SPI: u32 = 10;

/// Waker for the in-flight [`Spi::transfer_word_async`] call, if any.
///
/// Set (with the ITC's SPI channel armed) by `transfer_word_async` before it returns
/// `Pending`, and taken and woken by [`spi_isr`] the next time the module raises the
/// interrupt. A single instance suffices: unlike UART's independent RX/TX FIFOs, this crate's
/// SPI driver only ever has one transfer in flight.
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

    /// Kick off one 8-bit transfer; the result becomes available once
    /// [`Self::poll_transfer_status`] reports it.
    fn start_transfer(&mut self, tx: u8) {
        unsafe {
            (SPI_TX_DATA as *mut u32).write_volatile((tx as u32) << 24);
            (SPI_CLK_CTRL as *mut u32)
                .write_volatile((SCK_COUNT << SPI_SCK_COUNT_SHIFT) | SPI_START | DATA_LENGTH);
        }
    }

    /// Check once whether the transfer started by [`Self::start_transfer`] has completed.
    ///
    /// Shared by the blocking [`Self::transfer_word`] (spins on this) and the async
    /// [`Self::transfer_word_async`] (checked once up front, then again each time [`spi_isr`]
    /// wakes the task).
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

    /// Async equivalent of [`Self::transfer_word`].
    ///
    /// Arms the ITC's SPI channel (`INT_NUM_SPI`) and waits for [`spi_isr`] to wake this task,
    /// rather than polling. The check-then-arm sequence runs inside a single
    /// [`critical_section::with`] call so a completion landing between the check and enabling
    /// the interrupt can't be missed: interrupts stay masked for the whole sequence, so if
    /// `SPI_INT` is already set by the time `INT_NUM_SPI` is written to `INTENNUM`, the
    /// pending interrupt fires as soon as the critical section ends.
    async fn transfer_word_async(&mut self, tx: u8) -> u8 {
        self.start_transfer(tx);
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                if let Poll::Ready(rx) = self.poll_transfer_status() {
                    return Poll::Ready(rx);
                }
                WAKER.set(cs, cx.waker());
                unsafe {
                    core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_SPI);
                }
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

/// `embedded-hal-async`'s `SpiBus` reuses `embedded-hal`'s `ErrorType`, so [`spi::ErrorType`]
/// above already covers it; see [`Spi::transfer_word_async`] for why this waits on the
/// module's real completion interrupt rather than polling in a loop.
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
/// Overrides the weak `spi_isr` symbol this crate adds to `libmc1322x`'s `isr.h`/`isr.c` (see
/// the comment above `INT_NUM_SPI`) — unlike `i2c_isr`/`uart1_isr`/`uart2_isr`, this dispatch
/// is not present in upstream `libmc1322x`, so it only exists in this project's fork.
///
/// This deliberately does *not* touch `SPI_STATUS` itself: [`Spi::poll_transfer_status`] (run
/// from task context once woken) owns clearing `SPI_INT` and reading the received byte,
/// exactly as it does for the blocking path, so there's only one place that decides "are we
/// actually done". Instead this disables the ITC's SPI channel — which deasserts the
/// interrupt line so `irq()`'s dispatch loop can terminate rather than re-entering this
/// handler forever — and wakes whichever task armed the wait.
///
/// See [`crate::util::WakerCell::wake`] for why waking a waker left behind by a cancelled
/// (dropped) async transfer future is harmless.
///
/// # Caveats
///
/// Unverified on hardware, like `crate::i2c::i2c_isr`.
#[unsafe(no_mangle)]
extern "C" fn spi_isr() {
    unsafe {
        core::ptr::write_volatile((INTBASE + INTDISNUM_OFF) as *mut u32, INT_NUM_SPI);
    }
    WAKER.wake();
}
