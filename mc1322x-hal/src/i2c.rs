use embedded_hal::i2c::{self, SevenBitAddress, I2c, Operation};
use mc1322x_sys::{i2c_disable, i2c_enable, I2C_NON_BLOCKING, i2c_receiveinit, i2c_transferred, i2c_transmitinit};

pub struct I2c0 {
    enabled: bool,
}

impl I2c0 {
    pub fn enable(&mut self) {
        unsafe {
            i2c_enable()
        }
        self.enabled = true;
    }

    pub fn disable(&mut self) {
        unsafe {
            i2c_disable()
        }
        self.enabled = false;
    }

    pub fn transferred(&mut self) -> bool {
        let transferred = unsafe {
            i2c_transferred()
        };
        transferred == 1
    }

    pub fn transmit(&mut self, address: u8, data: &[u8]) {
        unsafe {
            i2c_transmitinit(address, data.len() as _, data.as_ptr() as _)
        }
    }

    pub fn receive(&mut self, address: u8, data: &mut [u8]) {
        unsafe {
            i2c_receiveinit(address, data.len() as _, data.as_mut_ptr() as _)
        };
    }
}

impl Drop for I2c0 {
    fn drop(&mut self) {
        if self.enabled {
            self.disable();
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Error {}

impl i2c::Error for Error {
    fn kind(&self) -> i2c::ErrorKind {
        match *self {}
    }
}

impl i2c::ErrorType for I2c0 {
    type Error = Error;
}

impl I2c<SevenBitAddress> for I2c0 {
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        operations.iter_mut().for_each(|op|
            {
                match op {
                    Operation::Read(buf) => {
                        self.receive(address, buf);
                    }
                    Operation::Write(buf) => {
                        self.transmit(address, buf);
                    }
                }
                if I2C_NON_BLOCKING == 1 {
                    while !self.transferred() {
                        // wait
                    }
                }
            }
        );
        Ok(())
    }
}
