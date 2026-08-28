//! Platform critical section for the MC1322x.
//!
//! The MC1322x is a single-core ARM7TDMI (ARMv4T). It has no atomics, and its Thumb instruction
//! set has no `MRS`/`MSR`/`cpsid`/`cpsie` instructions, so interrupts can't be masked from the
//! CPSR directly.
//!
//! Instead we use the chip's interrupt controller (ITC): all interrupt enables are collected in
//! the 32-bit `INTENABLE` register. A critical section saves `INTENABLE`, clears it (masking all
//! interrupts), and restores it on release. This is exactly the scheme the reference C firmware
//! uses in its `disable_int`/`__int_disable` macros. Pending interrupts are latched by the ITC and
//! are delivered once the enables are restored.

use critical_section::{RawRestoreState, set_impl};
use mc1322x_sys::{INTBASE, INTENABLE_OFF};

struct CriticalSection;

set_impl!(CriticalSection);

unsafe impl critical_section::Impl for CriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        // Safety: `INTENABLE` is a valid MMIO register on this chip.
        let intenable = core::ptr::read_volatile((INTBASE + INTENABLE_OFF) as *const u32);
        // Safety: `INTENABLE` is a valid MMIO register on this chip.
        core::ptr::write_volatile((INTBASE + INTENABLE_OFF) as *mut u32, 0);
        intenable
    }

    unsafe fn release(restore_state: RawRestoreState) {
        // Safety: `INTENABLE` is a valid MMIO register on this chip.
        core::ptr::write_volatile((INTBASE + INTENABLE_OFF) as *mut u32, restore_state);
    }
}
