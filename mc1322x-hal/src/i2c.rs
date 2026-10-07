//! I2C master (blocking and async).

use core::task::Poll;
use embedded_hal::i2c::{self, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress};
use mc1322x_sys::{
    I2C_BASE, I2C_CKEN, I2C_MAL, I2C_MBB, I2C_MCF, I2C_MEN, I2C_MIEN, I2C_MIF, I2C_MSTA, I2C_MTX,
    I2C_RSTA, I2C_RXAK, I2C_SCL, I2C_SDA, I2C_TXAK, INTBASE, INTENNUM_OFF, gpio_reg_set,
    gpio_select_function, interrupt_nums_INT_NUM_I2C,
};

use crate::util::{WakerCell, yield_now};

// I2C register map (byte-wide MMIO, see libmc1322x/lib/include/i2c.h)
const I2C_ADR: *mut u8 = I2C_BASE as usize as *mut u8;
const I2C_FDR: *mut u8 = (I2C_BASE as usize + 0x04) as *mut u8;
const I2C_CR: *mut u8 = (I2C_BASE as usize + 0x08) as *mut u8;
const I2C_SR: *mut u8 = (I2C_BASE as usize + 0x0C) as *mut u8;
const I2C_DR: *mut u8 = (I2C_BASE as usize + 0x10) as *mut u8;
const I2C_CKER: *mut u8 = (I2C_BASE as usize + 0x18) as *mut u8;

// GPIO block-0 registers (pins 0..31); SCL/SDA are GPIO12/GPIO13
const GPIO_PAD_PU_EN0: *mut u32 = 0x8000_0010 as *mut u32;
const GPIO_PAD_PU_SEL0: *mut u32 = 0x8000_0030 as *mut u32;

const I2C_ALT_FUNCTION: u8 = 1;

// ITC (interrupt controller) number of the I2C completion interrupt. `irq()` (linked from
// `libmc1322x`) dispatches it to the weak `i2c_isr` symbol overridden at the bottom of this file.
const INT_NUM_I2C: u32 = interrupt_nums_INT_NUM_I2C;

/// Waker for the in-flight [`I2c0::wait_byte_async`] call, if any; woken by [`i2c_isr`].
static WAKER: WakerCell = WakerCell::new();

/// I2C master error.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Error {
    /// The addressed slave did not acknowledge its address or a data byte.
    NoAcknowledge(NoAcknowledgeSource),
    /// The arbitration was lost (e.g. bus contention).
    ArbitrationLost,
}

impl i2c::Error for Error {
    fn kind(&self) -> i2c::ErrorKind {
        match *self {
            Error::NoAcknowledge(n) => i2c::ErrorKind::NoAcknowledge(n),
            Error::ArbitrationLost => i2c::ErrorKind::ArbitrationLoss,
        }
    }
}

/// I2C master on the I2C module (SCL = GPIO12, SDA = GPIO13).
///
/// Implements both the blocking `embedded-hal` and the `embedded-hal-async` [`I2c`] traits. The
/// blocking implementation polls the status register; the async one waits for the I2C
/// interrupt after each byte, and inhibits a sleep-aware executor from sleeping while waiting
/// (see [`crate::sleep::SleepInhibitGuard`]).
///
/// Only 7-bit addressing is supported.
pub struct I2c0;

impl I2c0 {
    /// Clock divider index for [`Self::new`] giving about 150 kHz SCL on the board selected by the
    /// `board-*` feature.
    ///
    /// The resulting SCL rate also depends on bus loading and pull-up strength, so this is a
    /// starting point rather than a guaranteed rate.
    pub const BOARD_CLOCK_DIVIDER: u8 = crate::board::I2C_CLOCK_DIVIDER;

    /// Create the I2C master.
    ///
    /// Enables the I2C module, muxes GPIO12/GPIO13 to SCL/SDA, enables their internal pull-ups and
    /// enables the I2C interrupt in the interrupt controller.
    ///
    /// `clock_divider` is the raw `I2C_FDR[5:0]` index selecting the SCL divider (RM Table 14-5);
    /// [`Self::BOARD_CLOCK_DIVIDER`] is a good default.
    pub fn new(clock_divider: u8) -> Self {
        unsafe {
            // gate the clock to the I2C module
            write_u8(I2C_CKER, I2C_CKEN as u8);
            // SCL frequency divider
            write_u8(I2C_FDR, clock_divider);
            // our own (unused, master-only) slave address
            write_u8(I2C_ADR, 0x01);
            // enable the module; auto-ack on, no interrupts
            write_u8(I2C_CR, I2C_MEN as u8);

            // GPIO12 (SCL) and GPIO13 (SDA) -> I2C function
            gpio_select_function(I2C_SCL as u8, I2C_ALT_FUNCTION);
            gpio_select_function(I2C_SDA as u8, I2C_ALT_FUNCTION);
            // internal pull-ups on both lines
            gpio_reg_set(GPIO_PAD_PU_EN0, I2C_SCL as u8);
            gpio_reg_set(GPIO_PAD_PU_EN0, I2C_SDA as u8);
            gpio_reg_set(GPIO_PAD_PU_SEL0, I2C_SCL as u8);
            gpio_reg_set(GPIO_PAD_PU_SEL0, I2C_SDA as u8);

            // Route the I2C interrupt to the core. `I2C_MIEN` stays off until `wait_byte_async`
            // arms it, so the blocking path is unaffected.
            core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_I2C);
        }
        I2c0
    }

    /// Clear pending interrupt and arbitration-lost flags.
    fn clear_status(&mut self) {
        unsafe {
            let sr = read_u8(I2C_SR);
            write_u8(I2C_SR, sr & !(I2C_MIF as u8 | I2C_MAL as u8));
        }
    }

    /// Generate a STOP condition and release the bus.
    ///
    /// A plain `MSTA` clear leaves `I2C_SR.MBB` (bus busy) stuck after an aborted transfer
    /// (no-acknowledge or arbitration loss), hanging the next transaction's bus-idle wait. This
    /// uses the recovery from `libmc1322x`'s `i2c_force_reset()` instead: toggle `MEN` off and on
    /// with `MSTA` set plus a dummy `I2CDR` read, then leave `I2CCR` at plain `MEN`.
    fn stop(&mut self) {
        // The recovery only works with a short delay between the steps. The length is empirical;
        // the RM doesn't document this sequence.
        fn settle() {
            for _ in 0..500u32 {
                core::hint::black_box(0);
            }
        }
        unsafe {
            write_u8(I2C_CR, I2C_MSTA as u8);
            settle();
            write_u8(I2C_CR, I2C_MEN as u8 | I2C_MSTA as u8);
            settle();
            let _ = read_u8(I2C_DR);
            settle();
            write_u8(I2C_CR, I2C_MEN as u8);
        }
    }

    /// Check once whether the current byte transfer has completed or failed.
    ///
    /// Shared by [`Self::wait_byte`] and [`Self::wait_byte_async`]; clears `I2C_MIF`/`I2C_MAL`.
    fn poll_byte_status(&mut self) -> Poll<Result<(), Error>> {
        unsafe {
            let sr = read_u8(I2C_SR);
            if sr & I2C_MAL as u8 != 0 {
                write_u8(I2C_SR, sr & !(I2C_MAL as u8));
                self.stop();
                return Poll::Ready(Err(Error::ArbitrationLost));
            }
            if sr & I2C_MIF as u8 != 0 && sr & I2C_MCF as u8 != 0 {
                write_u8(I2C_SR, sr & !(I2C_MIF as u8));
                return Poll::Ready(Ok(()));
            }
        }
        Poll::Pending
    }

    /// Wait until the module reports a completed byte transfer.
    fn wait_byte(&mut self) -> Result<(), Error> {
        loop {
            if let Poll::Ready(result) = self.poll_byte_status() {
                return result;
            }
        }
    }

    /// Async equivalent of [`Self::wait_byte`].
    ///
    /// Arms `I2C_MIEN` and waits for [`i2c_isr`]. The check-then-arm sequence runs inside one
    /// critical section, so a completion between the status check and arming the interrupt fires
    /// the interrupt as soon as the critical section ends instead of being missed.
    async fn wait_byte_async(&mut self) -> Result<(), Error> {
        let mut inhibit = None;
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                if let Poll::Ready(result) = self.poll_byte_status() {
                    return Poll::Ready(result);
                }
                inhibit.get_or_insert_with(crate::sleep::SleepInhibitGuard::new);
                WAKER.set(cs, cx.waker());
                unsafe {
                    write_u8(I2C_CR, read_u8(I2C_CR) | I2C_MIEN as u8);
                }
                Poll::Pending
            })
        })
        .await
    }

    /// Wait for the bus to be free, generate a START condition and transmit the
    /// slave address. `read` selects the R/W bit (1 = read).
    fn start(&mut self, address: u8, read: bool) -> Result<(), Error> {
        unsafe {
            while read_u8(I2C_SR) & I2C_MBB as u8 != 0 {}
            write_u8(I2C_CR, I2C_MEN as u8 | I2C_MSTA as u8 | I2C_MTX as u8);
            self.clear_status();
            write_u8(I2C_DR, (address << 1) | read as u8);
            self.wait_byte()?;
            if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                self.stop();
                return Err(Error::NoAcknowledge(NoAcknowledgeSource::Address));
            }
            Ok(())
        }
    }

    /// Generate a repeated START and transmit the slave address while keeping
    /// ownership of the bus.
    fn restart(&mut self, address: u8, read: bool) -> Result<(), Error> {
        unsafe {
            let cr = read_u8(I2C_CR);
            write_u8(I2C_CR, cr | I2C_MTX as u8 | I2C_RSTA as u8);
            self.clear_status();
            write_u8(I2C_DR, (address << 1) | read as u8);
            self.wait_byte()?;
            if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                self.stop();
                return Err(Error::NoAcknowledge(NoAcknowledgeSource::Address));
            }
            Ok(())
        }
    }

    /// Transmit `data`, checking for an ACK after every byte.
    fn transmit_data(&mut self, data: &[u8]) -> Result<(), Error> {
        for &byte in data {
            unsafe {
                write_u8(I2C_DR, byte);
            }
            self.wait_byte()?;
            unsafe {
                if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                    self.stop();
                    return Err(Error::NoAcknowledge(NoAcknowledgeSource::Data));
                }
            }
        }
        Ok(())
    }

    /// Receive `data` bytes. Call directly after an address/restart with the
    /// read bit set: the module is switched to receive mode and the address
    /// byte is discarded. When `nack_tail` is set this is the final read of the
    /// transaction, so the last byte is NACKed and the NACKed trailing byte is
    /// waited out before the STOP.
    fn receive_data(&mut self, data: &mut [u8], nack_tail: bool) -> Result<(), Error> {
        unsafe {
            // the address byte was just transmitted: switch to receive mode and
            // discard it
            let cr = read_u8(I2C_CR);
            write_u8(I2C_CR, cr & !(I2C_MTX as u8));
            read_u8(I2C_DR);
        }

        let len = data.len();
        for (index, byte) in data.iter_mut().enumerate() {
            self.wait_byte()?;
            unsafe {
                if nack_tail && index + 1 == len {
                    // last requested byte: disable auto-ack so the trailing byte
                    // is NACKed and the slave releases the bus
                    let cr = read_u8(I2C_CR);
                    write_u8(I2C_CR, cr | I2C_TXAK as u8);
                }
                *byte = read_u8(I2C_DR);
            }
        }

        if nack_tail && len > 0 {
            // wait for the NACKed trailing byte to complete before the STOP
            self.wait_byte()?;
        }
        Ok(())
    }

    /// Continue receiving after a previous read operation without a restart.
    fn continue_receive(&mut self, data: &mut [u8], nack_tail: bool) -> Result<(), Error> {
        let len = data.len();
        for (index, byte) in data.iter_mut().enumerate() {
            self.wait_byte()?;
            unsafe {
                if nack_tail && index + 1 == len {
                    let cr = read_u8(I2C_CR);
                    write_u8(I2C_CR, cr | I2C_TXAK as u8);
                }
                *byte = read_u8(I2C_DR);
            }
        }
        if nack_tail && len > 0 {
            self.wait_byte()?;
        }
        Ok(())
    }

    /// Async equivalent of [`Self::start`].
    async fn start_async(&mut self, address: u8, read: bool) -> Result<(), Error> {
        unsafe {
            while read_u8(I2C_SR) & I2C_MBB as u8 != 0 {
                yield_now().await;
            }
            write_u8(I2C_CR, I2C_MEN as u8 | I2C_MSTA as u8 | I2C_MTX as u8);
            self.clear_status();
            write_u8(I2C_DR, (address << 1) | read as u8);
        }
        self.wait_byte_async().await?;
        unsafe {
            if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                self.stop();
                return Err(Error::NoAcknowledge(NoAcknowledgeSource::Address));
            }
        }
        Ok(())
    }

    /// Async equivalent of [`Self::restart`].
    async fn restart_async(&mut self, address: u8, read: bool) -> Result<(), Error> {
        unsafe {
            let cr = read_u8(I2C_CR);
            write_u8(I2C_CR, cr | I2C_MTX as u8 | I2C_RSTA as u8);
            self.clear_status();
            write_u8(I2C_DR, (address << 1) | read as u8);
        }
        self.wait_byte_async().await?;
        unsafe {
            if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                self.stop();
                return Err(Error::NoAcknowledge(NoAcknowledgeSource::Address));
            }
        }
        Ok(())
    }

    /// Async equivalent of [`Self::transmit_data`].
    async fn transmit_data_async(&mut self, data: &[u8]) -> Result<(), Error> {
        for &byte in data {
            unsafe {
                write_u8(I2C_DR, byte);
            }
            self.wait_byte_async().await?;
            unsafe {
                if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                    self.stop();
                    return Err(Error::NoAcknowledge(NoAcknowledgeSource::Data));
                }
            }
        }
        Ok(())
    }

    /// Async equivalent of [`Self::receive_data`].
    async fn receive_data_async(&mut self, data: &mut [u8], nack_tail: bool) -> Result<(), Error> {
        unsafe {
            let cr = read_u8(I2C_CR);
            write_u8(I2C_CR, cr & !(I2C_MTX as u8));
            read_u8(I2C_DR);
        }

        let len = data.len();
        for (index, byte) in data.iter_mut().enumerate() {
            self.wait_byte_async().await?;
            unsafe {
                if nack_tail && index + 1 == len {
                    let cr = read_u8(I2C_CR);
                    write_u8(I2C_CR, cr | I2C_TXAK as u8);
                }
                *byte = read_u8(I2C_DR);
            }
        }

        if nack_tail && len > 0 {
            self.wait_byte_async().await?;
        }
        Ok(())
    }

    /// Async equivalent of [`Self::continue_receive`].
    async fn continue_receive_async(
        &mut self,
        data: &mut [u8],
        nack_tail: bool,
    ) -> Result<(), Error> {
        let len = data.len();
        for (index, byte) in data.iter_mut().enumerate() {
            self.wait_byte_async().await?;
            unsafe {
                if nack_tail && index + 1 == len {
                    let cr = read_u8(I2C_CR);
                    write_u8(I2C_CR, cr | I2C_TXAK as u8);
                }
                *byte = read_u8(I2C_DR);
            }
        }
        if nack_tail && len > 0 {
            self.wait_byte_async().await?;
        }
        Ok(())
    }
}

impl ErrorType for I2c0 {
    type Error = Error;
}

impl I2c<SevenBitAddress> for I2c0 {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }
        let n_ops = operations.len();
        let mut prev_is_read: Option<bool> = None;
        for (index, operation) in operations.iter_mut().enumerate() {
            let is_read = matches!(operation, Operation::Read(_));
            let is_last = index + 1 == n_ops;

            match prev_is_read {
                None => self.start(address, is_read)?,
                Some(prev) if prev != is_read => self.restart(address, is_read)?,
                Some(_) => {}
            }

            match operation {
                Operation::Read(buf) => {
                    if prev_is_read.is_none() || prev_is_read != Some(is_read) {
                        self.receive_data(buf, is_last)?;
                    } else {
                        self.continue_receive(buf, is_last)?;
                    }
                }
                Operation::Write(buf) => {
                    self.transmit_data(buf)?;
                }
            }
            prev_is_read = Some(is_read);
        }
        self.stop();
        Ok(())
    }
}

/// Waits for the I2C interrupt after each byte instead of polling. The initial wait for the bus
/// to become idle still polls (yielding to the executor in between), since bus-busy has no
/// interrupt.
impl embedded_hal_async::i2c::I2c<SevenBitAddress> for I2c0 {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }
        let n_ops = operations.len();
        let mut prev_is_read: Option<bool> = None;
        for (index, operation) in operations.iter_mut().enumerate() {
            let is_read = matches!(operation, Operation::Read(_));
            let is_last = index + 1 == n_ops;

            match prev_is_read {
                None => self.start_async(address, is_read).await?,
                Some(prev) if prev != is_read => self.restart_async(address, is_read).await?,
                Some(_) => {}
            }

            match operation {
                Operation::Read(buf) => {
                    if prev_is_read.is_none() || prev_is_read != Some(is_read) {
                        self.receive_data_async(buf, is_last).await?;
                    } else {
                        self.continue_receive_async(buf, is_last).await?;
                    }
                }
                Operation::Write(buf) => {
                    self.transmit_data_async(buf).await?;
                }
            }
            prev_is_read = Some(is_read);
        }
        self.stop();
        Ok(())
    }
}

#[inline]
unsafe fn read_u8(reg: *mut u8) -> u8 {
    unsafe { reg.read_volatile() }
}

#[inline]
unsafe fn write_u8(reg: *mut u8, value: u8) {
    unsafe { reg.write_volatile(value) }
}

/// I2C interrupt handler, overriding the weak `i2c_isr` symbol from `libmc1322x`'s `isr.h`.
///
/// Doesn't touch `I2C_SR`: [`I2c0::poll_byte_status`] clears the flags from task context, as on
/// the blocking path. The handler only disables `I2C_MIEN`, which deasserts the interrupt so
/// `irq()`'s dispatch loop can exit, and wakes the waiter.
///
/// This symbol is always linked in (the weak reference from `irq()` keeps it alive under
/// `--gc-sections`), even in binaries that never use [`I2c0`]. That is fine since this crate
/// always provides the `critical-section` implementation [`WAKER`] needs.
#[unsafe(no_mangle)]
extern "C" fn i2c_isr() {
    unsafe {
        write_u8(I2C_CR, read_u8(I2C_CR) & !(I2C_MIEN as u8));
    }
    WAKER.wake();
}
