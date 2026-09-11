/* Linker script for MC1322x flash algorithm */
/* Uses PrgCode/PrgData sections as required by target-gen */

/* MC1322x RAM: 0x00400000 - 0x00418000 (96 KB) */
/* Reserve 8 KB for flash algorithm */
/*
 * ORIGIN is 0x00400004, not 0x00400000: probe-rs prepends its own generic 4-byte
 * ARMv4T header (an infinite-loop trap word, used as the completion-catch target) at
 * 0x00400000 before this ELF's own code, since this algorithm's YAML entry does not
 * set an explicit `load_address` to account for it otherwise. Without this 4-byte
 * offset, our own .rom_vectors table (which must land at ARM7TDMI's fixed,
 * hardware-mandated exception vector addresses - 0x400000, 0x400004, ..., 0x40001c -
 * once vectors are remapped to RAM) is itself shifted 4 bytes later than those fixed
 * addresses, so e.g. the real FIQ vector slot at runtime 0x40001c ends up containing
 * whatever real code follows the (now-misaligned) vector table instead of one of our
 * intended `b .` traps - a real exception firing there runs unintended code instead of
 * safely spinning. Confirmed via ground-truth OpenOCD JTAG trace comparison.
 */
MEMORY
{
  RAM : ORIGIN = 0x00400004, LENGTH = 8K
}

/* Provide stack pointer */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* The flash algorithm sections required by target-gen */
SECTIONS
{
  /* Code section - must be named PrgCode for target-gen */
  PrgCode :
  {
    /* MC1322x-specific ROM patch vectors and ROM-owned scratch region - see
     * rom_vectors.rs for why these must come first, byte-exact, before any of our
     * own code. */
    KEEP(*(.rom_vectors));
    KEEP(*(.entry));
    *(.text*);
    *(.rodata*);
  } > RAM

  /* Data section - must be named PrgData for target-gen */
  PrgData : ALIGN(4)
  {
    *(.data*);
    *(.bss*);
    *(COMMON);
  } > RAM

  /DISCARD/ :
  {
    *(.ARM.exidx*);
    *(.ARM.extab*);
  }
}
