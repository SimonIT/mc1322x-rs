use core::convert::Infallible;
use embedded_io::{ErrorType, Read, Write};
use mc1322x_sys::{
    UART_struct, UCON, UDATA, URXCON, UTXCON, gpio_select_function, gpio_set_pad_dir, uart_flowctl,
    uart_setbaud,
};

const UART1_BASE: usize = 0x8000_5000;
const UART2_BASE: usize = 0x8000_B000;

const UCON_TXE: u32 = 1 << 0;
const UCON_RXE: u32 = 1 << 1;

const UART_FUNCTION: u8 = 1;

const U1TX_PIN: u8 = 14;
const U1RX_PIN: u8 = 15;
const U2TX_PIN: u8 = 18;
const U2RX_PIN: u8 = 19;

const PAD_DIR_INPUT: u8 = 0;
const PAD_DIR_OUTPUT: u8 = 1;

const TX_FIFO_DEPTH: u32 = 32;

/// Which UART peripheral to use.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum UartId {
    Uart1,
    Uart2,
}

/// Polling UART on the given peripheral, implementing
/// [`embedded_io::Read`] and [`embedded_io::Write`].
///
/// TX and RX use the 32-byte hardware FIFOs. The UART is configured for 8
/// data bits, no parity and one stop bit; the transmitter interrupt mask is
/// left at its reset value so nothing fires unless an ISR is registered.
pub struct Uart {
    uart: *mut UART_struct,
}

impl Uart {
    /// Configure and enable a UART at the requested baud rate.
    pub fn new(id: UartId, baud: u32) -> Self {
        let (uart, tx_pin, rx_pin) = match id {
            UartId::Uart1 => (UART1_BASE as *mut UART_struct, U1TX_PIN, U1RX_PIN),
            UartId::Uart2 => (UART2_BASE as *mut UART_struct, U2TX_PIN, U2RX_PIN),
        };

        let uart = Self { uart };

        // The UART must be enabled before its alternate function is selected
        // on the pads, otherwise the pads stay in GPIO mode (RM 11.5.1.2).
        unsafe {
            uart.write_reg(UCON, UCON_TXE | UCON_RXE);
        }

        unsafe {
            gpio_select_function(tx_pin, UART_FUNCTION);
            gpio_select_function(rx_pin, UART_FUNCTION);
            gpio_set_pad_dir(tx_pin, PAD_DIR_OUTPUT);
            gpio_set_pad_dir(rx_pin, PAD_DIR_INPUT);
        }

        uart.set_baud(baud);

        uart
    }

    /// Reprogram the baud rate divider (UART must be disabled while doing so).
    fn set_baud(&self, baud: u32) {
        unsafe {
            uart_setbaud(self.uart, baud);
        }
    }

    /// Enable or disable hardware RTS/CTS flow control.
    ///
    /// Muxes the RTS/CTS pins (UART1: GPIO17/16, UART2: GPIO21/20) onto the
    /// UART, so a previous GPIO configuration of those pins is overridden.
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

    #[inline]
    unsafe fn read_reg(&self, offset: u32) -> u32 {
        unsafe { ((self.uart as usize + offset as usize) as *const u32).read_volatile() }
    }

    #[inline]
    unsafe fn write_reg(&self, offset: u32, value: u32) {
        unsafe {
            ((self.uart as usize + offset as usize) as *mut u32).write_volatile(value);
        }
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
