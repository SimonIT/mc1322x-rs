//! Voltage regulator power-up and crystal trim, shared by peripherals whose bring-up needs them.

use mc1322x_sys::CRM_BASE;

/// `CRM_XTAL_CNTL` trim values for the 24 MHz reference crystal oscillator (see [`trim_xtal`]).
///
/// These compensate for the specific crystal and its board-level load capacitance, so they are
/// a per-board calibration, not chip behavior - each board gets its own preset in its own file
/// under `src/board/` (see `crate::board`'s doc comment), one of which is selected at compile
/// time by a `board-*` Cargo feature.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub(crate) struct XtalTrim {
    pub ctune_4pf: u32,
    pub ctune: u32,
    pub ftune: u32,
    pub ibias: u32,
}

impl XtalTrim {
    /// Pack into the raw `CRM_XTAL_CNTL` value [`trim_xtal`] writes. A `const fn` (rather than
    /// a plain method) so [`BOARD_XTAL_CNTL`] below can fold this - the shifts, ORs and all -
    /// into a single compile-time constant: since the board's trim never changes at runtime,
    /// there's nothing left for `trim_xtal` to compute, only one constant to write.
    const fn pack(self) -> u32 {
        (self.ctune_4pf << 25) | (self.ctune << 21) | (self.ftune << 16) | (self.ibias << 8) | 0x52
    }
}

/// [`crate::board::XTAL_TRIM`] (the board selected at compile time - see `crate::board`'s doc
/// comment), pre-packed at compile time (see [`XtalTrim::pack`]) into the exact value
/// [`trim_xtal`] writes verbatim - not just *which* board's trim is selected, but the register
/// value itself, is fully resolved before this ever runs.
pub(crate) const BOARD_XTAL_CNTL: u32 = crate::board::XTAL_TRIM.pack();

/// Trim the 24 MHz reference crystal oscillator to [`BOARD_XTAL_CNTL`].
///
/// This chip-level operation (a single `CRM_XTAL_CNTL` register write) applies to any MC1322x
/// board; only the packed value itself is board-specific - see [`XtalTrim`] and the `board-*`
/// Cargo features that select it.
///
/// Every radio-using program in `libmc1322x`'s own `tests/` (`rftest-tx`, `rftest-rx`,
/// `autoack-tx`, `autoack-rx`, ...) and Contiki's own `redbee-econotag` platform
/// (`init_lowlevel()` in `contiki-mc1322x-main.c`) calls the equivalent of this unconditionally
/// as one of the very first steps of `main`, before `maca_init()` - this workspace's own boot
/// path never did. An untrimmed crystal is an out-of-spec reference clock for the MACA's PLL
/// frequency synthesizer; this is the prime suspect for a MACA status 12 (`PLL_UNLOCK`)
/// reliably seen on the very first real transmit (see the `maca_tx_pll_unlock_runaway` project
/// memory) - true for any board using the MACA, not just the one this crate has verified.
pub(crate) fn trim_xtal() {
    unsafe {
        let xtal_cntl = (CRM_BASE + 0x40) as *mut u32;
        xtal_cntl.write_volatile(BOARD_XTAL_CNTL);
    }
}

/// Turn on the 1.5V/1.8V regulators that several peripherals (NVM flash access, the ASM
/// crypto block) need running before they'll work.
///
/// The boot ROM leaves these off to save power (AN3860 section 4.4, step 5): code that
/// reached `main` by a path other than the ROM's own normal boot flow (which turns them on
/// itself before jumping to flash-resident code) - e.g. loaded directly into RAM over JTAG -
/// needs to do this itself first, or those peripherals hang or fail. Replicates
/// `default_vreg_init()` (`mc1322x-sys/libmc1322x/src/default_lowlevel.c`) exactly via raw
/// register writes rather than reconstructing it from named bitfields, then polls the real
/// `VREG_1P5V_RDY`/`VREG_1P8V_RDY` status bits instead of guessing at a delay.
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
