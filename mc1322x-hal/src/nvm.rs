//! Serial flash (NVM) access through the boot ROM's `nvm_*` routines.
//!
//! [`Nvm`] implements `embedded-storage`'s [`NorFlash`]/[`ReadNorFlash`] for the SST-compatible
//! serial flash (32 sectors of 4096 bytes, 128 KiB) that MC1322x boards boot from.

use core::ffi::c_void;
use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash, check_erase, check_read,
    check_write,
};
use mc1322x_sys::{
    nvm_detect, nvm_erase, nvm_read, nvm_setsvar, nvm_write, nvmErr_t,
    nvmErr_t_gNvmErrAddressSpaceOverflow_c, nvmErr_t_gNvmErrNoError_c, nvmInterface_t,
    nvmInterface_t_gNvmExternalInterface_c, nvmInterface_t_gNvmInternalInterface_c, nvmType_t,
    nvmType_t_gNvmType_SST_c,
};

use crate::power::power_up_regulators;

/// Power up the regulators the ROM's `nvm_*` routines need, then call `nvm_setsvar(0)`.
///
/// `nvm_setsvar(0)` is required regardless of interface; the order (after the regulators are
/// ready, before `nvm_detect`) follows `libmc1322x`'s `nvm-read.c`.
fn prepare_flash_access() {
    power_up_regulators();
    unsafe {
        nvm_setsvar.expect("nvm_setsvar ROM entry point missing")(0);
    }
}

/// Number of erase sectors on the SST-compatible part this driver targets.
const SECTOR_COUNT: usize = 32;
/// Bytes per erase sector (see `libmc1322x`'s `nvm.h`: "SST flash has 32 sectors 4096 bytes
/// each").
const SECTOR_SIZE: usize = 4096;
const CAPACITY: usize = SECTOR_COUNT * SECTOR_SIZE;

/// Which bus the boot ROM's `nvm_*` routines drive.
///
/// The serial flash is wired either to the chip's internal interface or to external pins
/// (shared with [`crate::spi::Spi`] on GPIO4-7). This is a fixed property of the board, so
/// [`Nvm::new`]/[`Nvm::new_assume_sst`] use the interface selected by the `board-*` Cargo
/// feature instead of taking it as a parameter.
///
/// The wrong interface doesn't necessarily fail loudly: on an unconnected bus, `nvm_detect`,
/// `nvm_read` and `nvm_erase` can report success with fake data. Usually only `nvm_write`'s
/// internal verify step notices, returning [`Error::Rom`] with `gNvmErrVerifyError_c`. When
/// porting to a new board, confirm the choice with a write and read-back round trip. The Redbee
/// Econotag uses [`NvmInterface::Internal`].
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum NvmInterface {
    /// The internal NVM interface (`gNvmInternalInterface_c`).
    Internal,
    /// The external, GPIO-brought-out NVM interface (`gNvmExternalInterface_c`).
    External,
}

impl NvmInterface {
    fn as_raw(self) -> nvmInterface_t {
        match self {
            NvmInterface::Internal => nvmInterface_t_gNvmInternalInterface_c,
            NvmInterface::External => nvmInterface_t_gNvmExternalInterface_c,
        }
    }
}

/// NVM error.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Error {
    /// `nvm_detect` found no flash, or a type other than the SST-compatible part (32 sectors of
    /// 4096 bytes) this driver supports.
    UnsupportedFlash,
    /// An out-of-bounds or misaligned argument, caught before calling into the ROM.
    Bounds(NorFlashErrorKind),
    /// A ROM `nvm_*` routine returned an error.
    Rom(nvmErr_t),
}

impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match *self {
            Error::UnsupportedFlash => NorFlashErrorKind::Other,
            Error::Bounds(kind) => kind,
            Error::Rom(err) if err == nvmErr_t_gNvmErrAddressSpaceOverflow_c => {
                NorFlashErrorKind::OutOfBounds
            }
            Error::Rom(_) => NorFlashErrorKind::Other,
        }
    }
}

fn check_rom(err: nvmErr_t) -> Result<(), Error> {
    if err == nvmErr_t_gNvmErrNoError_c {
        Ok(())
    } else {
        Err(Error::Rom(err))
    }
}

/// Serial flash driven through the boot ROM's `nvm_*` routines.
///
/// Only the SST-compatible geometry from `libmc1322x`'s `nvm.h` (32 sectors of 4096 bytes,
/// 128 KiB) is supported. Reads and writes have byte granularity; erases work on whole
/// sectors.
pub struct Nvm {
    interface: nvmInterface_t,
    nvm_type: nvmType_t,
}

impl Nvm {
    /// Detect the flash chip on the board's [`NvmInterface`].
    ///
    /// Also powers up the 1.5 V/1.8 V regulators the ROM routines need.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Rom`] if `nvm_detect` fails, or [`Error::UnsupportedFlash`] if it
    /// detects a chip other than the SST-compatible part.
    pub fn new() -> Result<Self, Error> {
        prepare_flash_access();
        let interface = crate::board::NVM_INTERFACE.as_raw();
        let mut nvm_type: nvmType_t = 0;
        let err = unsafe {
            nvm_detect.expect("nvm_detect ROM entry point missing")(interface, &mut nvm_type)
        };
        check_rom(err)?;
        if nvm_type != nvmType_t_gNvmType_SST_c {
            return Err(Error::UnsupportedFlash);
        }
        Ok(Self {
            interface,
            nvm_type,
        })
    }

    /// Create an `Nvm` on the board's [`NvmInterface`] that assumes the SST-compatible part,
    /// without calling `nvm_detect`.
    ///
    /// For situations where `nvm_detect` hangs or faults, e.g. code loaded into RAM over JTAG
    /// that bypassed the normal ROM boot sequence.
    pub fn new_assume_sst() -> Self {
        prepare_flash_access();
        Self {
            interface: crate::board::NVM_INTERFACE.as_raw(),
            nvm_type: nvmType_t_gNvmType_SST_c,
        }
    }
}

impl ErrorType for Nvm {
    type Error = Error;
}

impl ReadNorFlash for Nvm {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        check_read(self, offset, bytes.len()).map_err(Error::Bounds)?;
        let err = unsafe {
            nvm_read.expect("nvm_read ROM entry point missing")(
                self.interface,
                self.nvm_type,
                bytes.as_mut_ptr() as *mut c_void,
                offset,
                bytes.len() as u32,
            )
        };
        check_rom(err)
    }

    fn capacity(&self) -> usize {
        CAPACITY
    }
}

impl NorFlash for Nvm {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = SECTOR_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        check_erase(self, from, to).map_err(Error::Bounds)?;
        let first_sector = from as usize / SECTOR_SIZE;
        let sector_count = (to - from) as usize / SECTOR_SIZE;
        let sector_bitfield =
            (0..sector_count).fold(0u32, |bits, i| bits | (1 << (first_sector + i)));
        let err = unsafe {
            nvm_erase.expect("nvm_erase ROM entry point missing")(
                self.interface,
                self.nvm_type,
                sector_bitfield,
            )
        };
        check_rom(err)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        check_write(self, offset, bytes.len()).map_err(Error::Bounds)?;
        let err = unsafe {
            nvm_write.expect("nvm_write ROM entry point missing")(
                self.interface,
                self.nvm_type,
                bytes.as_ptr() as *mut c_void,
                offset,
                bytes.len() as u32,
            )
        };
        check_rom(err)
    }
}
