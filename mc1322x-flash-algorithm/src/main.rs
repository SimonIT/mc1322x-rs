//! probe-rs flash algorithm for the MC1322x's serial flash, driving it through the boot ROM's
//! `nvm_*` routines.

#![no_std]
#![no_main]

use flash_algorithm::*;

mod rom_vectors;

// No-op stand-in for `rprintln!`: RTT needs atomics, which ARM7TDMI lacks.
macro_rules! debug {
    ($($arg:tt)*) => {{}};
}

// Flash chip types reported by `nvm_detect`.
#[repr(u32)]
#[derive(Copy, Clone, Debug)]
enum NvmType {
    NoNvm = 0,
    SST = 1,
    ST = 2,
    ATM = 3,
}

impl NvmType {
    /// Convert the code `nvm_detect` wrote, or `None` for an unknown one. (`nvm_detect` writes
    /// through a `*mut u32` so an unexpected value can't become an invalid enum.)
    fn from_raw(code: u32) -> Option<Self> {
        Some(match code {
            0 => NvmType::NoNvm,
            1 => NvmType::SST,
            2 => NvmType::ST,
            3 => NvmType::ATM,
            _ => return None,
        })
    }
}

// libmc1322x's `nvmInterface_t`. Only `Internal` is used.
#[repr(u32)]
#[derive(Copy, Clone, Debug)]
enum NvmInterface {
    Internal = 0,
    #[allow(dead_code)]
    External = 1,
}

// ROM `nvm_*` status codes (libmc1322x's `nvmErr_t`).
#[repr(u32)]
#[derive(Copy, Clone, Debug, PartialEq)]
enum NvmErr {
    NoError = 0,
    InvalidInterface = 1,
    InvalidNvmType = 2,
    InvalidPointer = 3,
    WriteProtect = 4,
    VerifyError = 5,
    AddressSpaceOverflow = 6,
    BlankCheckError = 7,
    RestrictedArea = 8,
    /// Not a ROM status code: any undocumented value, see [`NvmErr::from_raw`].
    Unknown = 9,
}

impl NvmErr {
    /// Convert a raw ROM return code; unknown values become [`NvmErr::Unknown`].
    fn from_raw(code: u32) -> Self {
        match code {
            0 => NvmErr::NoError,
            1 => NvmErr::InvalidInterface,
            2 => NvmErr::InvalidNvmType,
            3 => NvmErr::InvalidPointer,
            4 => NvmErr::WriteProtect,
            5 => NvmErr::VerifyError,
            6 => NvmErr::AddressSpaceOverflow,
            7 => NvmErr::BlankCheckError,
            8 => NvmErr::RestrictedArea,
            _ => NvmErr::Unknown,
        }
    }
}

// MC1322x voltage regulator registers (CRM)
const CRM_BASE: u32 = 0x80003000;
const CRM_SYS_CNTL: *mut u32 = (CRM_BASE + 0x00) as *mut u32;
const CRM_VREG_CNTL: *mut u32 = (CRM_BASE + 0x48) as *mut u32;
const CRM_STATUS: *const u32 = (CRM_BASE + 0x18) as *const u32;
// CRM_STATUS ready flags
const VREG_1P5V_RDY: u32 = 1 << 19;
const VREG_1P8V_RDY: u32 = 1 << 18;

// Boot ROM routines (Thumb). They return a raw `u32` rather than `NvmErr`, since an unexpected
// value would be an invalid enum; callers convert with `NvmErr::from_raw`.
type NvmDetectFn = unsafe extern "C" fn(NvmInterface, *mut u32) -> u32;
type NvmWriteFn = unsafe extern "C" fn(NvmInterface, NvmType, *const u8, u32, u32) -> u32;
type NvmEraseFn = unsafe extern "C" fn(NvmInterface, NvmType, u32) -> u32;
type NvmVerifyFn = unsafe extern "C" fn(NvmInterface, NvmType, *const u8, u32, u32) -> u32;
type NvmSetSvarFn = unsafe extern "C" fn(u32);

// ROM function addresses, with bit 0 set for Thumb.
const NVM_DETECT_ADDR: u32 = 0x00006cb9 | 1;
const NVM_WRITE_ADDR: u32 = 0x00006ec5 | 1;
const NVM_ERASE_ADDR: u32 = 0x00006e05 | 1;
const NVM_VERIFY_ADDR: u32 = 0x00006f85 | 1;
const NVM_SETSVAR_ADDR: u32 = 0x00007085 | 1;

/// Initializes the ROM's RAM variables (see `rom_vectors`); must run before any `nvm_*` call.
/// libmc1322x's `start.S` calls it first thing when `USE_ROM_VARS` is set. ARM code, so no `| 1`.
type RomDataInitFn = unsafe extern "C" fn();
const ROM_DATA_INIT_ADDR: u32 = 0x000108d0;

// Flash parameters for MC1322x
const FLASH_SECTOR_SIZE: u32 = 4096; // 4 KB sectors
const FLASH_NUM_SECTORS: u32 = 32; // 32 sectors = 128 KB total (MC13224V)

struct Algorithm {
    nvm_type: NvmType,
}

algorithm!(Algorithm, {
    device_name: "MC1322x",
    device_type: DeviceType::Onchip,
    flash_address: 0x00000000,
    flash_size: 0x00020000,     // 128 KB for MC13224V/MC13226V
    page_size: 0x100,           // 256 bytes programming page
    empty_value: 0xFF,
    program_time_out: 1000,
    erase_time_out: 2000,
    sectors: [{
        size: 0x1000,           // 4096 bytes per sector
        address: 0x0,
    }]
});

impl FlashAlgorithm for Algorithm {
    fn new(_address: u32, _clock: u32, _function: Function) -> Result<Self, ErrorCode> {
        debug!("MC1322x Flash Algorithm Init");

        // Give every banked exception mode its own stack, like libmc1322x's start.S does before
        // `rom_data_init`: probe-rs only sets the current mode's SP, and a ROM routine switching
        // modes would otherwise push through an uninitialized SP. The stacks sit above this
        // algorithm's RAM window (`link.x`: `0x400000..0x402000`). IRQ and FIQ stay masked
        // afterwards, so a stray interrupt can't vector into the trap table mid-`Init()`.
        unsafe {
            core::arch::asm!(
                "mrs r4, cpsr",
                "orr r5, r4, #0xc0", // keep current mode, mask IRQ+FIQ while we bank-hop
                "msr cpsr_c, r5",

                "bic r5, r4, #0x1f",
                "orr r5, r5, #0xc0",
                "orr r5, r5, #0x11", // FIQ_MODE
                "msr cpsr_c, r5",
                "ldr sp, ={fiq_sp}",

                "bic r5, r4, #0x1f",
                "orr r5, r5, #0xc0",
                "orr r5, r5, #0x12", // IRQ_MODE
                "msr cpsr_c, r5",
                "ldr sp, ={irq_sp}",

                "bic r5, r4, #0x1f",
                "orr r5, r5, #0xc0",
                "orr r5, r5, #0x13", // SVC_MODE
                "msr cpsr_c, r5",
                "ldr sp, ={svc_sp}",

                "bic r5, r4, #0x1f",
                "orr r5, r5, #0xc0",
                "orr r5, r5, #0x1b", // UND_MODE
                "msr cpsr_c, r5",
                "ldr sp, ={und_sp}",

                "bic r5, r4, #0x1f",
                "orr r5, r5, #0xc0",
                "orr r5, r5, #0x17", // ABT_MODE
                "msr cpsr_c, r5",
                "ldr sp, ={abt_sp}",

                // Back to the original mode, with IRQ+FIQ left masked.
                "orr r4, r4, #0xc0",
                "msr cpsr_c, r4",
                fiq_sp = const 0x403100u32,
                irq_sp = const 0x403200u32,
                svc_sp = const 0x403300u32,
                und_sp = const 0x403400u32,
                abt_sp = const 0x403500u32,
                out("r4") _,
                out("r5") _,
            );
        }

        // Initialize the ROM's RAM variables before any `nvm_*` call, like libmc1322x's start.S.
        let rom_data_init: RomDataInitFn = unsafe { core::mem::transmute(ROM_DATA_INIT_ADDR) };
        unsafe { rom_data_init() };

        // Initialize voltage regulators for NVM, mirroring libmc1322x's
        // default_vreg_init() plus the readiness waits from nvm-read.c.
        unsafe {
            // Set default system state.
            core::ptr::write_volatile(CRM_SYS_CNTL, 0x00000018);

            // Bypass the buck converter.
            core::ptr::write_volatile(CRM_VREG_CNTL, 0x00000f04);

            // Delay for the bypass to take effect.
            for _ in 0..0x161a8 {
                core::ptr::read_volatile(CRM_STATUS);
            }

            // Start the regulators.
            core::ptr::write_volatile(CRM_VREG_CNTL, 0x00000ff8);

            // Wait for the regulators to be ready.
            while (core::ptr::read_volatile(CRM_STATUS) & VREG_1P5V_RDY) == 0 {}
            while (core::ptr::read_volatile(CRM_STATUS) & VREG_1P8V_RDY) == 0 {}
        }

        debug!("Voltage regulators initialized");

        // Detect NVM type
        let mut nvm_type_raw: u32 = 0;
        let nvm_detect: NvmDetectFn = unsafe { core::mem::transmute(NVM_DETECT_ADDR) };

        let result =
            NvmErr::from_raw(unsafe { nvm_detect(NvmInterface::Internal, &mut nvm_type_raw) });

        debug!("NVM detect result: {:?}, type: {:?}", result, nvm_type_raw);

        if result != NvmErr::NoError {
            debug!("Failed to detect NVM");
            return Err(ErrorCode::new(0x7001).unwrap());
        }

        let Some(nvm_type) = NvmType::from_raw(nvm_type_raw) else {
            debug!("Unrecognized NVM type code from ROM");
            return Err(ErrorCode::new(0x7008).unwrap());
        };

        if matches!(nvm_type, NvmType::NoNvm) {
            debug!("No NVM detected");
            return Err(ErrorCode::new(0x7002).unwrap());
        }

        debug!("NVM initialized successfully, type: {:?}", nvm_type);

        // Set up the ROM NVM driver for normal operation (see libmc1322x nvm-read.c).
        let nvm_setsvar: NvmSetSvarFn = unsafe { core::mem::transmute(NVM_SETSVAR_ADDR) };
        unsafe { nvm_setsvar(0) };

        Ok(Self { nvm_type })
    }

    fn erase_all(&mut self) -> Result<(), ErrorCode> {
        debug!("Erase All");

        // Erase every sector except the reserved top one (bit 31), with the same 0x7fffffff mask
        // as libmc1322x's flasher.c.
        let nvm_erase: NvmEraseFn = unsafe { core::mem::transmute(NVM_ERASE_ADDR) };

        let result = NvmErr::from_raw(unsafe {
            nvm_erase(NvmInterface::Internal, self.nvm_type, 0x7FFFFFFF)
        });

        debug!("Erase all result: {:?}", result);

        if result != NvmErr::NoError {
            debug!("Erase all failed");
            return Err(ErrorCode::new(0x7003).unwrap());
        }

        Ok(())
    }

    fn erase_sector(&mut self, addr: u32) -> Result<(), ErrorCode> {
        debug!("Erase sector addr: 0x{:08x}", addr);

        // Calculate sector number from address
        let sector_num = addr / FLASH_SECTOR_SIZE;

        if sector_num >= FLASH_NUM_SECTORS {
            debug!("Invalid sector number: {}", sector_num);
            return Err(ErrorCode::new(0x7004).unwrap());
        }

        // Create bitmask for this sector
        let sector_bitmask = 1u32 << sector_num;

        debug!(
            "Erasing sector {}, bitmask: 0x{:08x}",
            sector_num, sector_bitmask
        );

        let nvm_erase: NvmEraseFn = unsafe { core::mem::transmute(NVM_ERASE_ADDR) };

        let result = NvmErr::from_raw(unsafe {
            nvm_erase(NvmInterface::Internal, self.nvm_type, sector_bitmask)
        });

        debug!("Erase sector result: {:?}", result);

        if result != NvmErr::NoError {
            debug!("Erase sector failed");
            return Err(ErrorCode::new(0x7005).unwrap());
        }

        Ok(())
    }

    fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), ErrorCode> {
        debug!("Program page addr: 0x{:08x}, size: {}", addr, data.len());

        if data.is_empty() {
            return Ok(());
        }

        // Ensure address is within flash range
        if addr >= 0x00020000 || (addr + data.len() as u32) > 0x00020000 {
            debug!("Address out of range");
            return Err(ErrorCode::new(0x7006).unwrap());
        }

        let nvm_write: NvmWriteFn = unsafe { core::mem::transmute(NVM_WRITE_ADDR) };

        // Nonzero means failure.
        let error_count = unsafe {
            nvm_write(
                NvmInterface::Internal,
                self.nvm_type,
                data.as_ptr(),
                addr,
                data.len() as u32,
            )
        };

        debug!("Write error count: {}", error_count);

        if error_count != 0 {
            debug!("Program page failed with {} errors", error_count);
            return Err(ErrorCode::new(0x7007).unwrap());
        }

        Ok(())
    }

    fn verify(&mut self, address: u32, size: u32, data: Option<&[u8]>) -> Result<(), u32> {
        debug!("Verify addr: 0x{:08x}, size: {}", address, size);

        // No data to compare against: nothing to verify.
        let Some(data) = data else {
            return Ok(());
        };

        if size == 0 {
            return Ok(());
        }

        if address >= 0x00020000 || (address + size) > 0x00020000 {
            debug!("Verify: address out of range");
            return Err(address);
        }

        // `nvm_verify` compares flash against `data` in the ROM.
        let nvm_verify: NvmVerifyFn = unsafe { core::mem::transmute(NVM_VERIFY_ADDR) };

        let result = NvmErr::from_raw(unsafe {
            nvm_verify(
                NvmInterface::Internal,
                self.nvm_type,
                data.as_ptr(),
                address,
                size,
            )
        });

        debug!("Verify result: {:?}", result);

        if result != NvmErr::NoError {
            debug!("Verify failed at address 0x{:08x}", address);
            // The ROM doesn't report where the mismatch is; report the region start.
            return Err(address);
        }

        Ok(())
    }
}

impl Drop for Algorithm {
    fn drop(&mut self) {
        debug!("MC1322x Flash Algorithm Uninit");
    }
}
