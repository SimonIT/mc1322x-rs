//! Board-specific constants for the Redbee Econotag - the only board this crate has been
//! hardware-verified against so far. Selected at compile time by the `board-redbee-econotag`
//! Cargo feature (on by default) - see `src/lib.rs`'s `mod board` declaration.
//!
//! A new board gets its own sibling file here (e.g. `board/my_board.rs`) plus a matching
//! `board-my-board` feature and `#[path = "board/my_board.rs"]` arm in `lib.rs`, providing the
//! same set of items this file does.

use crate::nvm::NvmInterface;
use crate::power::XtalTrim;

/// 24 MHz reference crystal trim (see `power::XtalTrim`'s doc comment for what this
/// compensates for). Replicates `trim_xtal()` (`mc1322x-sys/libmc1322x/src/
/// default_lowlevel.h`'s `pack_XTAL_CNTL(CTUNE_4PF, CTUNE, FTUNE, IBIAS)` macro) using
/// `board/redbee-econotag.h`'s `CTUNE_4PF`/`CTUNE`/`FTUNE`, and `board/std_conf.h`'s `IBIAS`
/// default, which Econotag doesn't override.
pub(crate) const XTAL_TRIM: XtalTrim = XtalTrim {
    ctune_4pf: 1,
    ctune: 11,
    ftune: 7,
    ibias: 0x1F,
};

/// Which bus this board's serial flash is wired to (see `nvm::NvmInterface`'s doc comment for
/// why picking the wrong one matters and is easy to get wrong silently). Hardware-verified:
/// `External` (GPIO4-7) reads back a floating-bus pattern (0x00, consistent but not real data)
/// that made `erase`/`read` falsely report success, while `write`'s internal verify step
/// caught the mismatch (`gNvmErrVerifyError_c`); `Internal` reads back proper 0xFF post-erase
/// and verifies real writes correctly.
pub(crate) const NVM_INTERFACE: NvmInterface = NvmInterface::Internal;

/// `I2C_FDR[5:0]` divider index (RM Table 14-5) measured at ~150 kHz SCL on this board - see
/// `i2c::I2c0::BOARD_CLOCK_DIVIDER`'s doc comment for why the real-world frequency this
/// produces also depends on bus loading and pull-up strength, i.e. is board-specific in a way
/// that can't be computed from the chip alone.
pub(crate) const I2C_CLOCK_DIVIDER: u8 = 0x20;
