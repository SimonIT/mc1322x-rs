//! MC1322x ROM patch-vector table.
//!
//! The boot ROM's library routines (`nvm_detect` and friends) internally call through four
//! fixed RAM offsets from address `0x400000` - `0x20`, `0x60`, `0xa0`, `0xe0` - expecting either
//! a real patch or a harmless `bx lr` stub there (see libmc1322x's `src/start.S`, guarded by
//! `USE_ROM_VARS`). RAM offset `0x120`-`0x7ff` from the same base is reserved as the ROM's own
//! scratch storage. Without these, a ROM call jumps into whatever code happens to occupy that
//! RAM offset - on this crate, that used to be the middle of our own `EraseChip`/`EraseSector`/
//! `Init` functions, which are linked starting at `0x400000` with nothing reserved. Confirmed on
//! real hardware and independently cross-checked via OpenOCD: without this, resuming `Init()`
//! runs for the full timeout and crashes with an ARM "Undefined Instruction" exception deep in
//! RAM, well past this algorithm's own code.
//!
//! Runtime placement note: this target has no explicit `load_address` in
//! `MC1322x_Series.yaml`, so probe-rs prepends a fixed 4-byte "infinite loop" safety-net header
//! (`ARM_FLASH_BLOB_HEADER_LOOP_A32_LE` in probe-rs's `flash_algorithm.rs`, `[u32; 1]`) before
//! this blob when loading it into RAM. Every offset below is therefore 4 bytes less than the
//! ROM's actual expected absolute address (e.g. `0x1c` here lands at runtime `0x400020`). If
//! that header ever changes size, these offsets need to move with it.
//!
//! `link.x` places the `.rom_vectors` section first in `PrgCode`, before anything else, so these
//! offsets - relative to this section's own start - land exactly where the ROM expects them.
core::arch::global_asm!(
    r#"
.section .rom_vectors, "ax"

/* This chip's ARM7TDMI exception vector table lives at fixed, hardware-defined addresses
 * `0x400000`-`0x40001c` (runtime): Reset/Undef/SWI/PrefetchAbort/DataAbort/Reserved/IRQ/FIQ,
 * one word each - see libmc1322x's `src/start.S`, which installs a real vector table there for
 * normal application binaries. probe-rs's own prepended 4-byte safety-net header already traps
 * the Reset slot (runtime `0x400000`), but the other seven slots (runtime `0x400004`-`0x40001c`)
 * fell in this section's own unfilled padding before `_rptv_0` - meaning any exception at all
 * (a stray SWI, an undefined instruction from wandering execution, or even an ordinary IRQ,
 * since nothing here masks interrupts before/during `Init()`) would vector the core into
 * whatever garbage zero-fill happened to be sitting there, rather than a safe, recognizable
 * trap. Confirmed as the real mechanism behind repeated "core halts somewhere unexplained deep
 * in RAM" symptoms during JTAG flash-algorithm debugging: a halt landed exactly at runtime
 * `0x400008` - the SWI vector slot - with no other explanation. Each of the seven `b .`
 * self-branches below closes that gap with a safe infinite-loop trap (all sharing one label, so
 * any of the seven vectors firing parks the core at the same, easily recognizable address)
 * instead of leaving it open. This exactly fills the gap up to (not including) `_rptv_0` at
 * `.org 0x1c`, so it doesn't disturb the ROM patch-vector offsets below at all.
 */
.arm
_exception_trap:
    b _exception_trap /* Undef         (runtime 0x400004) */
    b _exception_trap /* SWI           (runtime 0x400008) */
    b _exception_trap /* PrefetchAbort (runtime 0x40000c) */
    b _exception_trap /* DataAbort     (runtime 0x400010) */
    b _exception_trap /* Reserved      (runtime 0x400014) */
    b _exception_trap /* IRQ           (runtime 0x400018) */
    b _exception_trap /* FIQ           (runtime 0x40001c) */

.org 0x1c
.thumb
_rptv_0:
    bx lr

.org 0x5c
_rptv_1:
    bx lr

.org 0x9c
_rptv_2:
    bx lr

.org 0xdc
_rptv_3:
    bx lr

/* Reserve the ROM's own scratch region (runtime 0x120-0x7ff) - leave it unfilled, nothing
 * of ours may start before this point. */
.org 0x7fb
.arm
    .word 0
"#
);
