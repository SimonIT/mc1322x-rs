use core::convert::Infallible;
use core::task::Poll;
use embedded_io::{ErrorType, Read, Write};
use mc1322x_sys::{
    INTBASE, UART_struct, UART1_BASE, UART2_BASE, UCON, UDATA, URXCON, USTAT, UTXCON,
    gpio_select_function, gpio_set_pad_dir, uart_flowctl, uart_setbaud,
};

use crate::util::WakerCell;

bitflags::bitflags! {
    /// `UART_CON` bits (RM 11.5.1.4).
    #[derive(Clone, Copy, PartialEq, Eq)]
    struct Ucon: u32 {
        const TXE = 1 << 0;
        const RXE = 1 << 1;
        // `MTXR`/`MRXR` *mask* the TX-ready/RX-ready interrupt sources: 1 = masked (off). This
        // is the opposite polarity of `TXE`/`RXE` above, so both must be set at init time to
        // keep the async path's interrupts quiescent until armed (see
        // [`Uart::wait_rx_ready`]/[`Uart::wait_tx_ready`]) — otherwise the reset-value-0 mask
        // bits leave the (already latched, see [`FIFO_WATERMARK`]) TX-ready condition
        // unmasked, and once [`Uart::new`] routes the interrupt through the ITC it fires
        // immediately and forever.
        const MTXR = 1 << 13;
        const MRXR = 1 << 14;
    }
}

bitflags::bitflags! {
    /// `UART_STAT` bits 6/7 (RM 11.5.1.5): level-triggered "FIFO has crossed its watermark"
    /// flags, set by hardware whenever `rx_count()`/`tx_free()` cross the level last written to
    /// `URXCON`/`UTXCON`. Unlike I2C's `I2C_MIF`, nothing here needs to be cleared by software:
    /// the condition self-clears as soon as the FIFO count no longer satisfies the watermark.
    #[derive(Clone, Copy, PartialEq, Eq)]
    struct Ustat: u32 {
        const RXRDY = 1 << 6;
        const TXRDY = 1 << 7;
    }
}

// `URXCON`/`UTXCON` are dual-purpose (see `libmc1322x`'s `uart.c`): a write latches the
// watermark used for `USTAT_RXRDY`/`USTAT_TXRDY`, while a read (as in `rx_count`/`tx_free`
// below) returns the FIFO's live byte count, independent of the watermark. Using the lowest
// possible watermark (1) makes "RX/TX ready" mean the same thing an interrupt-free polling
// loop already checks (`rx_count() > 0` / `tx_free() > 0`), at the cost of `flush` seeing
// spurious wakeups before the TX FIFO is fully drained (`USTAT_TXRDY` triggers on *any* free
// slot, not on empty) — harmless since `flush` re-checks and re-arms each time.
const FIFO_WATERMARK: u32 = 1;

// ITC (interrupt controller) offset/numbers for the UART completion interrupts (see
// `isr.h`'s `INTENNUM_OFF` and `interrupt_nums`), following the same wiring as
// `crate::i2c`'s `INT_NUM_I2C`. `irq()` (linked from `libmc1322x`) dispatches them to the
// weak `uart1_isr`/`uart2_isr` symbols overridden at the bottom of this file.
const INTENNUM_OFF: u32 = 0x8;
const INT_NUM_UART1: u32 = 1;
const INT_NUM_UART2: u32 = 2;

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
/// RX and TX are independent FIFOs, so a pending read and a pending write can be armed at
/// the same time; each gets its own slot. One instance per physical UART, since both can be
/// in use concurrently.
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
    Uart1,
    Uart2,
}

/// UART on the given peripheral, implementing both the blocking [`embedded_io::Read`]/
/// [`embedded_io::Write`] and, via [`embedded_io_async::Read`]/[`embedded_io_async::Write`],
/// an async equivalent.
///
/// TX and RX use the 32-byte hardware FIFOs. The UART is configured for 8
/// data bits, no parity and one stop bit. The blocking implementation polls the FIFO level
/// registers directly; the async implementation instead arms the RX/TX-ready interrupt
/// (`UCON_MRXR`/`UCON_MTXR`, routed through the ITC as `INT_NUM_UART1`/`INT_NUM_UART2`) and
/// waits to be woken by [`uart1_isr`]/[`uart2_isr`] — see [`Self::wait_rx_ready`]/
/// [`Self::wait_tx_ready`] for the arm/wake handshake.
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
    /// Configure and enable a UART at the requested baud rate.
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

        // The UART must be enabled before its alternate function is selected
        // on the pads, otherwise the pads stay in GPIO mode (RM 11.5.1.2).
        //
        // MTXR/MRXR are masked (disabled) here so the async path's interrupts stay
        // quiescent until `wait_rx_ready`/`wait_tx_ready` explicitly arm them; the
        // blocking `Read`/`Write` impls below never touch these bits.
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

        // Route the UART's interrupt to the core. This only affects the async path: the
        // peripheral-local masks (`UCON_MTXR`/`UCON_MRXR`) stay set until `wait_rx_ready`/
        // `wait_tx_ready` arm them, so the blocking API is unaffected.
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

    /// Async wait until the RX FIFO holds at least one byte.
    ///
    /// Arms `UCON_MRXR` and waits for [`uart1_isr`]/[`uart2_isr`] to wake this task, rather
    /// than polling. The check-then-arm sequence runs inside a single
    /// [`critical_section::with`] call so a byte landing between the check and enabling the
    /// interrupt can't be missed: interrupts stay masked for the whole sequence, so if
    /// `USTAT_RXRDY` is already set by the time `UCON_MRXR` is cleared, the pending interrupt
    /// fires as soon as the critical section ends.
    async fn wait_rx_ready(&mut self) {
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                if self.rx_count() > 0 {
                    return Poll::Ready(());
                }
                self.wakers().rx.set(cs, cx.waker());
                unsafe {
                    self.write_reg(UCON, self.read_reg(UCON) & !Ucon::MRXR.bits());
                }
                Poll::Pending
            })
        })
        .await
    }

    /// Async equivalent of [`Self::wait_rx_ready`] for the TX FIFO having a free slot.
    async fn wait_tx_ready(&mut self) {
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                if self.tx_free() > 0 {
                    return Poll::Ready(());
                }
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

/// `embedded-io-async`'s `Read` reuses `embedded-io`'s `ErrorType`, already implemented
/// above; see [`Uart::wait_rx_ready`] for why this waits on the real RX-ready interrupt
/// rather than polling in a loop like [`Read::read`] above.
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

/// `embedded-io-async`'s `Write` reuses `embedded-io`'s `ErrorType`, already implemented
/// above; see [`Uart::wait_tx_ready`] for why this waits on the real TX-ready interrupt
/// rather than polling in a loop like [`Write::write`]/[`Write::flush`] above.
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
/// This deliberately does *not* touch `USTAT`: unlike I2C's `I2C_MIF`, the ready flags
/// self-clear once the FIFO count no longer satisfies its watermark, so
/// [`Uart::wait_rx_ready`]/[`Uart::wait_tx_ready`] (run from task context once woken) already
/// observe the up-to-date state via `rx_count`/`tx_free`. Instead this re-masks whichever of
/// `UCON_MRXR`/`UCON_MTXR` is currently asserted — which deasserts the interrupt line so
/// `irq()`'s dispatch loop can terminate rather than re-entering this handler forever — and
/// wakes whichever task armed the wait.
///
/// See [`crate::util::WakerCell::wake`] for why waking a waker left behind by a cancelled
/// (dropped) async read/write future is harmless.
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
/// Overrides the weak `uart1_isr` symbol declared in `libmc1322x`'s `isr.h`; the linked
/// `irq()` handler (`mc1322x-sys/libmc1322x/src/isr.c`) dispatches here whenever
/// `INT_NUM_UART1` is pending. See [`uart_isr_common`] for the shared logic and
/// [`crate::i2c::i2c_isr`] for why this is unconditionally linked into any binary that
/// depends on `mc1322x-hal`, whether or not it ever constructs a UART1 [`Uart`].
///
/// # Caveats
///
/// Like `crate::i2c::i2c_isr`, unverified on hardware.
#[unsafe(no_mangle)]
extern "C" fn uart1_isr() {
    uart_isr_common(UART1_BASE as *mut UART_struct, &UART1_WAKERS);
}

/// UART2 equivalent of [`uart1_isr`].
#[unsafe(no_mangle)]
extern "C" fn uart2_isr() {
    uart_isr_common(UART2_BASE as *mut UART_struct, &UART2_WAKERS);
}
