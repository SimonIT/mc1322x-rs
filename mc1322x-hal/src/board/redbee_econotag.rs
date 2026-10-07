//! Board-specific constants for the Redbee Econotag (feature `board-redbee-econotag`).

use crate::nvm::NvmInterface;
use crate::power::XtalTrim;

/// 24 MHz reference crystal trim.
///
/// Values from `libmc1322x`'s `board/redbee-econotag.h` (`CTUNE_4PF`, `CTUNE`, `FTUNE`) and the
/// `IBIAS` default from `board/std_conf.h`.
pub(crate) const XTAL_TRIM: XtalTrim = XtalTrim {
    ctune_4pf: 1,
    ctune: 11,
    ftune: 7,
    ibias: 0x1F,
};

/// Bus the board's serial flash is wired to.
///
/// The Econotag uses the internal interface. With `External` (GPIO4-7) the driver reads a
/// floating bus on this board: erase and read appear to succeed, only write's verify fails.
pub(crate) const NVM_INTERFACE: NvmInterface = NvmInterface::Internal;

/// `I2C_FDR[5:0]` divider index (RM Table 14-5) giving about 150 kHz SCL on this board.
///
/// The actual SCL rate also depends on bus loading and pull-up strength.
pub(crate) const I2C_CLOCK_DIVIDER: u8 = 0x20;
