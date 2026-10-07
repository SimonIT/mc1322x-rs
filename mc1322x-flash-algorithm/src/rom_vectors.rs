//! MC1322x ROM patch-vector table.
//!
//! The boot ROM's routines (`nvm_detect` and friends) call through four fixed RAM addresses,
//! `0x400020`, `0x400060`, `0x4000a0` and `0x4000e0`, expecting a patch or a `bx lr` stub there
//! (libmc1322x's `src/start.S`, `USE_ROM_VARS`), and use `0x400120..0x400800` as scratch RAM.
//! Without the stubs, ROM calls jump into this algorithm's own code, which starts at `0x400000`.
//!
//! `link.x` places `.rom_vectors` first in `PrgCode`. probe-rs prepends a 4-byte infinite-loop
//! header to the blob (no `load_address` in the target YAML), so every offset below is 4 less
//! than its runtime address (`0x1c` lands at `0x400020`). If that header changes size, these
//! offsets must move with it.
core::arch::global_asm!(
    r#"
.section .rom_vectors, "ax"

/* Exception vectors (runtime 0x400004-0x40001c; probe-rs's header covers Reset): trap any
 * exception in one self-branch instead of running into padding. Exactly fills the space up to
 * `_rptv_0`.
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

/* Reserve the ROM's scratch RAM (runtime 0x400120-0x4007ff): nothing of ours may start
 * before this point. */
.org 0x7fb
.arm
    .word 0
"#
);
