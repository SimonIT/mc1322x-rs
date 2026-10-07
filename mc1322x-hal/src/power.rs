//! Voltage regulator power-up and crystal trim, shared by peripherals whose bring-up needs them.

use mc1322x_sys::CRM_BASE;

/// `CRM_XTAL_CNTL` trim values for the 24 MHz reference crystal oscillator (see [`trim_xtal`]).
///
/// These compensate for the board's crystal and load capacitance, so each board defines its own
/// in its `src/board/` file.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub(crate) struct XtalTrim {
    pub ctune_4pf: u32,
    pub ctune: u32,
    pub ftune: u32,
    pub ibias: u32,
}

impl XtalTrim {
    /// Pack into the raw `CRM_XTAL_CNTL` value [`trim_xtal`] writes. `const` so
    /// [`BOARD_XTAL_CNTL`] is computed at compile time.
    const fn pack(self) -> u32 {
        (self.ctune_4pf << 25) | (self.ctune << 21) | (self.ftune << 16) | (self.ibias << 8) | 0x52
    }
}

/// The selected board's [`crate::board::XTAL_TRIM`], packed into the `CRM_XTAL_CNTL` value
/// [`trim_xtal`] writes.
pub(crate) const BOARD_XTAL_CNTL: u32 = crate::board::XTAL_TRIM.pack();

/// Trim the 24 MHz reference crystal oscillator to [`BOARD_XTAL_CNTL`].
///
/// Must run before `maca_init()`: an untrimmed crystal is an out-of-spec reference for the
/// MACA's PLL, and the first transmit then fails with MACA status 12 (`PLL_UNLOCK`).
/// `libmc1322x`'s radio tests and Contiki's `redbee-econotag` platform do the same early in
/// `main`.
pub(crate) fn trim_xtal() {
    unsafe {
        let xtal_cntl = (CRM_BASE + 0x40) as *mut u32;
        xtal_cntl.write_volatile(BOARD_XTAL_CNTL);
    }
}

/// Turn on the 1.5 V/1.8 V regulators that NVM flash access and the ASM crypto block need,
/// and wait until both report ready.
///
/// The boot ROM leaves them off to save power (AN3860 §4.4, step 5), so code that didn't come
/// through the ROM's normal boot flow (e.g. loaded into RAM over JTAG) has to turn them on
/// itself. Same register writes as `libmc1322x`'s `default_vreg_init()`
/// (`src/default_lowlevel.c`), followed by polling `VREG_1P5V_RDY`/`VREG_1P8V_RDY`.
pub(crate) fn power_up_regulators() {
    unsafe {
        let sys_cntl = CRM_BASE as *mut u32;
        let vreg_cntl = (CRM_BASE + 0x48) as *mut u32;
        let status = (CRM_BASE + 0x18) as *const u32;

        sys_cntl.write_volatile(0x0000_0018);
        vreg_cntl.write_volatile(0x0000_0f04); // bypass the buck
        for _ in 0..0x000_161a8u32 {
            core::hint::black_box(0); // wait for the bypass to take
        }
        vreg_cntl.write_volatile(0x0000_0ff8); // start the regulators

        while status.read_volatile() & (1 << 19) == 0 {} // VREG_1P5V_RDY
        while status.read_volatile() & (1 << 18) == 0 {} // VREG_1P8V_RDY
    }
}
