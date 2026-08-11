use embedded_hal::i2c::{self, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress};
use mc1322x_sys::{
    gpio_reg_set, gpio_select_function, I2C_BASE, I2C_CKEN, I2C_MAL, I2C_MBB, I2C_MCF, I2C_MEN,
    I2C_MIF, I2C_MSTA, I2C_MTX, I2C_RSTA, I2C_RXAK, I2C_SCL, I2C_SDA, I2C_TXAK,
};

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
const I2C_CLOCK_DIVIDER: u8 = 0x20; // ~150 kHz on the Redbee Econotag

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

/// I2C master on the module 0 instance (SCL = GPIO12, SDA = GPIO13).
///
/// The module is driven synchronously by polling the status register, so no
/// interrupt handler is required.
pub struct I2c0;

impl I2c0 {
    /// Enable the I2C module, mux the SDA/SCL pads to their I2C function and
    /// activate the internal pull-ups, then return a master-ready instance.
    pub fn new() -> Self {
        unsafe {
            // gate the clock to the I2C module
            write_u8(I2C_CKER, I2C_CKEN as u8);
            // SCL frequency divider
            write_u8(I2C_FDR, I2C_CLOCK_DIVIDER);
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

    /// Generate a STOP condition.
    fn stop(&mut self) {
        unsafe {
            let cr = read_u8(I2C_CR);
            write_u8(I2C_CR, cr & !(I2C_MSTA as u8));
        }
    }

    /// Wait until the module reports a completed byte transfer.
    fn wait_byte(&mut self) -> Result<(), Error> {
        unsafe {
            loop {
                let sr = read_u8(I2C_SR);
                if sr & I2C_MAL as u8 != 0 {
                    write_u8(I2C_SR, sr & !(I2C_MAL as u8));
                    self.stop();
                    return Err(Error::ArbitrationLost);
                }
                if sr & I2C_MIF as u8 != 0 && sr & I2C_MCF as u8 != 0 {
                    write_u8(I2C_SR, sr & !(I2C_MIF as u8));
                    return Ok(());
                }
            }
        }
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
}

impl Default for I2c0 {
    fn default() -> Self {
        Self::new()
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

#[inline]
unsafe fn read_u8(reg: *mut u8) -> u8 {
    unsafe { reg.read_volatile() }
}

#[inline]
unsafe fn write_u8(reg: *mut u8, value: u8) {
    unsafe { reg.write_volatile(value) }
}
