use embedded_hal::i2c::{self, SevenBitAddress, I2c, Operation};
use mc1322x_sys::{i2c_disable, i2c_enable, i2c_receiveinit, i2c_transmitinit};

pub struct I2c0;

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
        unsafe {
            i2c_enable()
        }
        operations.iter_mut().for_each(|op|
            {
                match op {
                    Operation::Read(buf) => {
                        unsafe {
                            i2c_receiveinit(address, buf.len() as _, buf.as_mut_ptr() as _)
                        };
                    }
                    Operation::Write(buf) => {
                        unsafe {
                            i2c_transmitinit(address, buf.len() as _, buf.as_ptr() as _)
                        }
                    }
                }
            }
        );
        unsafe {
            i2c_disable()
        }
        Ok(())
    }
}
