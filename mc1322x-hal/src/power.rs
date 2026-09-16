//! Voltage regulator power-up and crystal trim, shared by peripherals whose bring-up needs them.

use mc1322x_sys::CRM_BASE;

/// Trim the 24 MHz reference crystal oscillator to the board's calibrated frequency.
///
/// Replicates `trim_xtal()` (`mc1322x-sys/libmc1322x/src/default_lowlevel.h`'s
/// `pack_XTAL_CNTL(CTUNE_4PF, CTUNE, FTUNE, IBIAS)` macro) for the Redbee Econotag board (the
/// only board this workspace targets - see `board/redbee-econotag.h`'s `CTUNE_4PF`/`CTUNE`/
/// `FTUNE`, and `board/std_conf.h`'s `IBIAS` default, which Econotag doesn't override).
///
/// Every radio-using program in `libmc1322x`'s own `tests/` (`rftest-tx`, `rftest-rx`,
/// `autoack-tx`, `autoack-rx`, ...) and Contiki's own `redbee-econotag` platform
/// (`init_lowlevel()` in `contiki-mc1322x-main.c`) calls this unconditionally as one of the very
/// first steps of `main`, before `maca_init()` - this workspace's own boot path never did. An
/// untrimmed crystal is an out-of-spec reference clock for the MACA's PLL frequency synthesizer;
/// this is the prime suspect for a MACA status 12 (`PLL_UNLOCK`) reliably seen on the very first
/// real transmit (see the `maca_tx_pll_unlock_runaway` project memory).
pub(crate) fn trim_xtal() {
    const CTUNE_4PF: u32 = 1;
    const CTUNE: u32 = 11;
    const FTUNE: u32 = 7;
    const IBIAS: u32 = 0x1F;

    unsafe {
        let xtal_cntl = (CRM_BASE + 0x40) as *mut u32;
        xtal_cntl.write_volatile((CTUNE_4PF << 25) | (CTUNE << 21) | (FTUNE << 16) | (IBIAS << 8) | 0x52);
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
