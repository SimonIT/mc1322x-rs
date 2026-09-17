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

/// Power up the regulators the ROM's `nvm_*` routines need, and clear the ROM's internal
/// "secure variable" state they also require.
///
/// The `nvm_setsvar(0)` call is unconditionally required, regardless of interface, matching
/// the ROM's own `nvm-read.c` reference flow's call order (right after regulator ready,
/// before `nvm_detect`).
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
/// The MC1322x has no on-die flash: `nvm_detect`/`nvm_read`/`nvm_write`/`nvm_erase` are boot
/// ROM routines that bit-bang a serial flash chip, wired either to the chip's internal
/// interface or brought out to external pins (sharing GPIO4-7 with [`crate::spi::Spi`]),
/// depending on the board - a fixed property of the board's own PCB design, not a runtime
/// choice, which is why [`Nvm::new`]/[`Nvm::new_assume_sst`] use the one a `board-*` Cargo
/// feature selects (`crate::board::NVM_INTERFACE`) rather than taking it as a parameter.
///
/// Picking the wrong one doesn't necessarily fail loudly: against a floating/unconnected bus,
/// `nvm_detect`/`nvm_read`/`nvm_erase` can read back a consistent-but-fake pattern and report
/// success, since none of them compare against expected data. `nvm_write`'s internal verify
/// step is usually the first thing to actually notice, failing with [`Error::Rom`] wrapping
/// `gNvmErrVerifyError_c`. Confirm against real hardware behavior (a write+read-back round
/// trip, not just a read), not board documentation, when porting to a new board (see
/// `crate::board`'s doc comment) - on the Redbee Econotag, the only board this has been
/// hardware-verified against so far, `Internal` is correct despite `External` superficially
/// "working" for detect/read/erase.
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
    /// `nvm_detect` found no flash, or a type other than the SST-compatible 32 x 4096-byte
    /// sector part this driver's [`NorFlash::ERASE_SIZE`]/[`ReadNorFlash::capacity`] assume.
    UnsupportedFlash,
    /// An out-of-bounds or misaligned argument, caught before calling into the ROM.
    Bounds(NorFlashErrorKind),
    /// A ROM routine reported a specific failure.
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

/// External or internal serial flash, driven through the boot ROM's `nvm_*` routines.
///
/// Only the SST-compatible geometry documented in `libmc1322x`'s `nvm.h` (32 sectors of 4096
/// bytes each, 128 KiB total) is supported: [`Nvm::new`] calls `nvm_detect` and fails with
/// [`Error::UnsupportedFlash`] if a different (or no) chip is found, rather than guessing at
/// its geometry.
pub struct Nvm {
    interface: nvmInterface_t,
    nvm_type: nvmType_t,
}

impl Nvm {
    /// Detect the flash chip on the board's [`NvmInterface`] (`crate::board::NVM_INTERFACE`,
    /// selected at compile time - see that constant's doc comment).
    ///
    /// # Errors
    ///
    /// Returns [`Error::UnsupportedFlash`] if `nvm_detect` finds no chip, or one that isn't
    /// the SST-compatible part this type assumes.
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

    /// Construct an `Nvm` on the board's [`NvmInterface`] assuming the SST-compatible
    /// geometry, without calling the ROM's `nvm_detect` at all.
    ///
    /// Diagnostic escape hatch for boards/situations where `nvm_detect` itself hangs or faults
    /// (observed when this driver is exercised from code loaded directly into RAM over JTAG,
    /// bypassing the chip's normal boot sequence) - skips straight to the same SST type
    /// [`Self::new`] would use anyway if detection succeeded.
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
