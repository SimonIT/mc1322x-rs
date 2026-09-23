//! Software-initiated system reset (RM §5.9.19, `SW_RST`).
//!
//! The reset is complete - the same power-on sequence as a [`crate::watchdog`] time-out,
//! including the ROM boot sequence - so what runs afterwards is whatever the ROM boots next (the
//! NVM image, or UART/SPI boot), not whatever was loaded into RAM over JTAG.
//!
//! Anything still queued in a UART TX FIFO is lost, so wait for it to drain first if the last
//! message matters.
//!
//! The chip has no reset-cause register, so software can't tell a reset triggered here apart
//! from a watchdog time-out or a power cycle afterwards.

use mc1322x_sys::CRM_BASE;

const SW_RST: *mut u32 = (CRM_BASE as usize + 0x50) as *mut u32;

/// Value that must be written (as one 32-bit access) to `SW_RST` to trigger the reset.
const RESET_KEY: u32 = 0x8765_1234;

/// Reset the whole chip immediately.
pub fn software_reset() -> ! {
    unsafe { SW_RST.write_volatile(RESET_KEY) };
    // The reset takes effect asynchronously to the write; don't run anything past it meanwhile.
    loop {
        core::hint::spin_loop();
    }
}
