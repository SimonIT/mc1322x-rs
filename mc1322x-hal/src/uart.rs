//! UART1/UART2 driver, implementing the blocking `embedded-io` and async `embedded-io-async`
//! `Read`/`Write` traits.
//!
//! # Interrupts
//!
//! [`Uart::new`] enables the UART's interrupt in the ITC (`INT_NUM_UART1`/`INT_NUM_UART2`),
//! dispatched by `libmc1322x`'s `irq()` to this module's `uart1_isr`/`uart2_isr`. Only the async
//! implementation unmasks the peripheral's RX/TX-ready interrupts.

use core::convert::Infallible;
use core::task::Poll;
use embedded_io::{ErrorType, Read, Write};
use mc1322x_sys::{
    INTBASE, INTENNUM_OFF, UART_struct, UART1_BASE, UART2_BASE, UCON, UDATA, URXCON, USTAT,
    UTXCON, gpio_select_function, gpio_set_pad_dir, interrupt_nums_INT_NUM_UART1,
    interrupt_nums_INT_NUM_UART2, uart_flowctl, uart_setbaud,
};

use crate::util::WakerCell;

bitflags::bitflags! {
    /// `UART_CON` bits (RM 11.5.1.4).
    #[derive(Clone, Copy, PartialEq, Eq)]
    struct Ucon: u32 {
        const TXE = 1 << 0;
        const RXE = 1 << 1;
        // `MTXR`/`MRXR` *mask* the TX/RX-ready interrupts: 1 = masked, the opposite polarity of
        // `TXE`/`RXE`. They reset to 0, so `Uart::new` sets them; otherwise the TX-ready
        // condition (true whenever the FIFO has room) would fire as soon as the ITC enable is set.
        const MTXR = 1 << 13;
        const MRXR = 1 << 14;
    }
}

bitflags::bitflags! {
    /// `UART_STAT` bits 6/7 (RM 11.5.1.5): level-sensitive "FIFO level reached the watermark in
    /// `URXCON`/`UTXCON`" flags. They clear themselves once the FIFO level no longer satisfies
    /// the watermark; software never clears them.
    #[derive(Clone, Copy, PartialEq, Eq)]
    struct Ustat: u32 {
        const RXRDY = 1 << 6;
        const TXRDY = 1 << 7;
    }
}

// `URXCON`/`UTXCON` are dual-purpose (see `libmc1322x`'s `uart.c`): a write sets the watermark
// for `USTAT_RXRDY`/`USTAT_TXRDY`, a read returns the FIFO's current level. A watermark of 1
// makes "ready" mean `rx_count() > 0` / `tx_free() > 0`. The async `flush` therefore wakes on
// any free TX slot rather than on empty, and simply re-checks and waits again.
const FIFO_WATERMARK: u32 = 1;

// ITC numbers of the UART interrupts. `irq()` dispatches them to the weak `uart1_isr`/`uart2_isr`
// symbols overridden at the bottom of this file.
const INT_NUM_UART1: u32 = interrupt_nums_INT_NUM_UART1;
const INT_NUM_UART2: u32 = interrupt_nums_INT_NUM_UART2;

const UART_FUNCTION: u8 = 1;

const U1TX_PIN: u8 = 14;
const U1RX_PIN: u8 = 15;
const U2TX_PIN: u8 = 18;
const U2RX_PIN: u8 = 19;

const PAD_DIR_INPUT: u8 = 0;
const PAD_DIR_OUTPUT: u8 = 1;

const TX_FIFO_DEPTH: u32 = 32;

/// Per-UART RX/TX wakers for the `embedded-io-async` implementation.
///
/// A pending read and a pending write can be armed at the same time, so each gets its own
/// slot.
struct UartWakers {
    rx: WakerCell,
    tx: WakerCell,
}

impl UartWakers {
    const fn new() -> Self {
        Self {
            rx: WakerCell::new(),
            tx: WakerCell::new(),
        }
    }
}

static UART1_WAKERS: UartWakers = UartWakers::new();
static UART2_WAKERS: UartWakers = UartWakers::new();

/// Which UART peripheral to use.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum UartId {
    /// UART1: TX on GPIO14, RX on GPIO15.
    Uart1,
    /// UART2: TX on GPIO18, RX on GPIO19.
    Uart2,
}

/// UART driver implementing [`embedded_io::Read`]/[`embedded_io::Write`] and
/// [`embedded_io_async::Read`]/[`embedded_io_async::Write`].
///
/// Uses the 32-byte hardware FIFOs, 8 data bits, no parity and one stop bit. The blocking
/// implementation busy-polls the FIFO levels; the async implementation waits for the
/// RX/TX-ready interrupt instead and holds a [`crate::sleep::SleepInhibitGuard`] while
/// waiting.
pub struct Uart {
    uart: *mut UART_struct,
    id: UartId,
}

#[inline]
unsafe fn read_reg_raw(uart: *mut UART_struct, offset: u32) -> u32 {
    unsafe { ((uart as usize + offset as usize) as *const u32).read_volatile() }
}

#[inline]
unsafe fn write_reg_raw(uart: *mut UART_struct, offset: u32, value: u32) {
    unsafe { ((uart as usize + offset as usize) as *mut u32).write_volatile(value) }
}

impl Uart {
    /// Configure and enable a UART at `baud` bit/s, and switch its TX/RX pins to the UART
    /// function.
    pub fn new(id: UartId, baud: u32) -> Self {
        let (uart, tx_pin, rx_pin, int_num) = match id {
            UartId::Uart1 => (
                UART1_BASE as *mut UART_struct,
                U1TX_PIN,
                U1RX_PIN,
                INT_NUM_UART1,
            ),
            UartId::Uart2 => (
                UART2_BASE as *mut UART_struct,
                U2TX_PIN,
                U2RX_PIN,
                INT_NUM_UART2,
            ),
        };

        let uart = Self { uart, id };

        // The UART must be enabled before its alternate function is selected on the pads,
        // otherwise the pads stay in GPIO mode (RM 11.5.1.2).
        //
        // MTXR/MRXR start masked; only `wait_rx_ready`/`wait_tx_ready` unmask them.
        unsafe {
            uart.write_reg(
                UCON,
                (Ucon::TXE | Ucon::RXE | Ucon::MTXR | Ucon::MRXR).bits(),
            );
            uart.write_reg(URXCON, FIFO_WATERMARK);
            uart.write_reg(UTXCON, FIFO_WATERMARK);
        }

        unsafe {
            gpio_select_function(tx_pin, UART_FUNCTION);
            gpio_select_function(rx_pin, UART_FUNCTION);
            gpio_set_pad_dir(tx_pin, PAD_DIR_OUTPUT);
            gpio_set_pad_dir(rx_pin, PAD_DIR_INPUT);
        }

        uart.set_baud(baud);

        // Enable the UART's interrupt in the ITC. Nothing fires until `wait_rx_ready`/
        // `wait_tx_ready` unmask `MRXR`/`MTXR`.
        unsafe {
            core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, int_num);
        }

        uart
    }

    /// This UART's waker pair, keyed by which physical peripheral it is.
    fn wakers(&self) -> &'static UartWakers {
        match self.id {
            UartId::Uart1 => &UART1_WAKERS,
            UartId::Uart2 => &UART2_WAKERS,
        }
    }

    /// Reprogram the baud rate divider. `uart_setbaud` disables TX/RX while doing so and
    /// re-enables them afterwards.
    fn set_baud(&self, baud: u32) {
        unsafe {
            uart_setbaud(self.uart, baud);
        }
    }

    /// Enable or disable hardware RTS/CTS flow control.
    ///
    /// Enabling muxes the RTS/CTS pins (UART1: GPIO17/16, UART2: GPIO21/20) onto the UART,
    /// overriding any previous GPIO configuration of those pins.
    pub fn set_flow_control(&mut self, on: bool) {
        unsafe {
            uart_flowctl(self.uart, on as u8);
        }
    }

    /// Number of free slots in the TX FIFO (0 = full, 32 = empty).
    fn tx_free(&self) -> u32 {
        unsafe { self.read_reg(UTXCON) & 0x3F }
    }

    /// Number of bytes waiting in the RX FIFO.
    fn rx_count(&self) -> u32 {
        unsafe { self.read_reg(URXCON) & 0x3F }
    }

    fn write_byte(&self, byte: u8) {
        unsafe {
            self.write_reg(UDATA, byte as u32);
        }
    }

    fn read_byte(&self) -> u8 {
        unsafe { self.read_reg(UDATA) as u8 }
    }

    /// Async wait until the RX FIFO holds at least one byte.
    ///
    /// Unmasks `UCON_MRXR` and waits for [`uart1_isr`]/[`uart2_isr`] to wake this task. The
    /// check and the unmask run in one critical section, so a byte arriving in between raises
    /// the interrupt as soon as the section ends instead of being missed.
    ///
    /// Holds a [`crate::sleep::SleepInhibitGuard`] while waiting.
    async fn wait_rx_ready(&mut self) {
        let mut inhibit = None;
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                if self.rx_count() > 0 {
                    return Poll::Ready(());
                }
                inhibit.get_or_insert_with(crate::sleep::SleepInhibitGuard::new);
                self.wakers().rx.set(cs, cx.waker());
                unsafe {
                    self.write_reg(UCON, self.read_reg(UCON) & !Ucon::MRXR.bits());
                }
                Poll::Pending
            })
        })
        .await
    }

    /// Async wait until the TX FIFO has a free slot. Same mechanism as [`Self::wait_rx_ready`],
    /// using `UCON_MTXR`.
    async fn wait_tx_ready(&mut self) {
        let mut inhibit = None;
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                if self.tx_free() > 0 {
                    return Poll::Ready(());
                }
                inhibit.get_or_insert_with(crate::sleep::SleepInhibitGuard::new);
                self.wakers().tx.set(cs, cx.waker());
                unsafe {
                    self.write_reg(UCON, self.read_reg(UCON) & !Ucon::MTXR.bits());
                }
                Poll::Pending
            })
        })
        .await
    }

    #[inline]
    unsafe fn read_reg(&self, offset: u32) -> u32 {
        unsafe { read_reg_raw(self.uart, offset) }
    }

    #[inline]
    unsafe fn write_reg(&self, offset: u32, value: u32) {
        unsafe { write_reg_raw(self.uart, offset, value) }
    }
}

impl ErrorType for Uart {
    type Error = Infallible;
}

impl Read for Uart {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        while self.rx_count() == 0 {}
        let mut n = 0;
        while n < buf.len() && self.rx_count() > 0 {
            buf[n] = self.read_byte();
            n += 1;
        }
        Ok(n)
    }
}

impl Write for Uart {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut n = 0;
        while n < buf.len() {
            while self.tx_free() == 0 {}
            self.write_byte(buf[n]);
            n += 1;
        }
        Ok(n)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        while self.tx_free() != TX_FIFO_DEPTH {}
        Ok(())
    }
}

/// Waits on the RX-ready interrupt instead of busy-polling.
impl embedded_io_async::Read for Uart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        if self.rx_count() == 0 {
            self.wait_rx_ready().await;
        }
        let mut n = 0;
        while n < buf.len() && self.rx_count() > 0 {
            buf[n] = self.read_byte();
            n += 1;
        }
        Ok(n)
    }
}

/// Waits on the TX-ready interrupt instead of busy-polling.
impl embedded_io_async::Write for Uart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut n = 0;
        while n < buf.len() {
            if self.tx_free() == 0 {
                self.wait_tx_ready().await;
            }
            self.write_byte(buf[n]);
            n += 1;
        }
        Ok(n)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        while self.tx_free() != TX_FIFO_DEPTH {
            self.wait_tx_ready().await;
        }
        Ok(())
    }
}

/// Shared body of [`uart1_isr`] and [`uart2_isr`].
///
/// The `USTAT` ready flags clear themselves, so this doesn't touch them. It re-masks
/// `UCON_MRXR`/`UCON_MTXR` for whichever condition is asserted, so `irq()` doesn't re-enter
/// the handler forever, and wakes the matching task.
fn uart_isr_common(uart: *mut UART_struct, wakers: &UartWakers) {
    let stat = Ustat::from_bits_truncate(unsafe { read_reg_raw(uart, USTAT) });
    let mut mask = Ucon::empty();
    if stat.contains(Ustat::RXRDY) {
        mask |= Ucon::MRXR;
    }
    if stat.contains(Ustat::TXRDY) {
        mask |= Ucon::MTXR;
    }
    if !mask.is_empty() {
        unsafe {
            write_reg_raw(uart, UCON, read_reg_raw(uart, UCON) | mask.bits());
        }
    }
    if stat.contains(Ustat::RXRDY) {
        wakers.rx.wake();
    }
    if stat.contains(Ustat::TXRDY) {
        wakers.tx.wake();
    }
}

/// UART1 RX/TX-ready interrupt handler.
///
/// Overrides the weak `uart1_isr` symbol declared in `libmc1322x`'s `isr.h`; `irq()` calls it
/// whenever `INT_NUM_UART1` is pending. Linked into every binary that depends on this crate,
/// whether or not it uses UART1.
///
/// `irq()` runs in ARM state and must call this Thumb code with interworking (`bx`).
#[unsafe(no_mangle)]
extern "C" fn uart1_isr() {
    uart_isr_common(UART1_BASE as *mut UART_struct, &UART1_WAKERS);
}

/// UART2 equivalent of [`uart1_isr`].
#[unsafe(no_mangle)]
extern "C" fn uart2_isr() {
    uart_isr_common(UART2_BASE as *mut UART_struct, &UART2_WAKERS);
}
