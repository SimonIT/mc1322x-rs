# MC1322x Flash Algorithm for probe-rs

This is a flash programming algorithm for the Freescale/NXP MC1322x series microcontrollers. The
family has exactly two members, differing only in ROM contents (peripheral driver set vs.
ZigBee-Pro-profile optimization) - both 128 KB Flash / 96 KB RAM:
- MC13224V
- MC13226V

## Overview

The MC1322x series uses a unique approach where flash programming functions are built into ROM at fixed addresses. This flash algorithm is a thin wrapper that calls these ROM functions according to the CMSIS-Pack flash algorithm standard.

## Architecture

- **Core**: ARM7TDMI (ARMv4T)
- **Flash**: Internal NVM (Non-Volatile Memory)
  - 32 sectors of 4096 bytes each (128 KB total)
  - Supports SST, ST Microelectronics, and Atmel flash chips
- **RAM**: 96 KB starting at 0x00400000

## ROM Functions Used

The algorithm calls the following ROM functions (all in THUMB mode):

| Function | Address | Purpose |
|----------|---------|---------|
| `nvm_detect` | 0x00006cb9 | Detect flash type |
| `nvm_write` | 0x00006ec5 | Write to flash |
| `nvm_erase` | 0x00006e05 | Erase flash sectors |
| `nvm_verify` | 0x00006f85 | Verify flash contents against a supplied buffer |
| `nvm_setsvar` | 0x00007085 | Configure the ROM NVM driver for normal operation |

## Building

### Prerequisites

`cargo build --release` alone needs nothing beyond the toolchain: the nightly channel,
`armv4t-none-eabi` target, and `rust-src`/`llvm-tools-preview`/`rustfmt` components are all pinned
in `rust-toolchain.toml`, so `rustup` fetches them automatically the first time you build in this
directory (there's no prebuilt `rust-std` for this target, so `.cargo/config.toml` builds `core`
from source via `-Z build-std` instead).

`target-gen` is only needed for the separate step of extracting the built ELF into probe-rs's YAML
format (below) - install it if you're regenerating `mc1322x_flash.yaml` after a `main.rs` change:

```bash
cargo install target-gen
```

### Build the Algorithm

```bash
cd mc1322x-flash-algorithm

# Build the algorithm
cargo build --release

# Extract the flash algorithm to YAML
target-gen elf target/armv4t-none-eabi/release/mc1322x-flash-algorithm mc1322x_flash.yaml
```

This will generate `mc1322x_flash.yaml` containing the flash algorithm in probe-rs format.

### Integration with probe-rs

Copy the generated flash algorithm section from `mc1322x_flash.yaml` into the MC1322x target definition file:

```bash
# Copy the flash algorithm section to the target definition
cp mc1322x_flash.yaml ../probe-rs/targets/MC1322x_Series.yaml
```

Or manually copy the `flash_algorithms` section from the generated YAML into `probe-rs/targets/MC1322x_Series.yaml`.

## Testing

The `.cargo/config.toml` runner's `cargo run --release` path (`target-gen test template.yaml
target/definition.yaml`) needs a `template.yaml` test-parameter file that isn't checked in - use
`probe-rs`'s own `download`/`verify` commands directly against real hardware instead:

```bash
probe-rs download --chip MC13224V \
    --chip-description-path ../probe-rs/targets/MC1322x_Series.yaml \
    --protocol jtag --binary-format bin --base-address 0x0 --verify --non-interactive \
    <path-to-a-.bin-file>

# Independent re-check (a fresh JTAG session, re-runs Init() from scratch too):
probe-rs verify --chip MC13224V \
    --chip-description-path ../probe-rs/targets/MC1322x_Series.yaml \
    --protocol jtag --binary-format bin --base-address 0x0 \
    <path-to-the-same-.bin-file>
```

- `--base-address 0x0` targets the logical NVM/flash region and is what actually exercises this
  flash algorithm (a RAM-linked ELF like `blinky`, at `0x400000`, never touches it at all).
- **Never pass `--dry-run`**: it substitutes a stub probe that doesn't support the ARM7/JTAG path,
  and fails instantly with an unrelated-looking "the selected probe does not support the 'JTAG'
  interface" error.

**Status**: confirmed working end-to-end on real MC13224V hardware (Redbee Econotag) - a full
attach → `Init()` → erase → program cycle via `download`, followed by an independent `verify` in a
fresh JTAG session, both complete cleanly with no error.

## Technical Details

### Flash Properties

- **Sector size**: 4096 bytes (4 KB)
- **Number of sectors**: 32 (128 KB total)
- **Page size**: 256 bytes (programming granularity)
- **Erased byte value**: 0xFF
- **Erase method**: Sector bitmask (bit 0-31 for sectors 0-31)

### Voltage Regulator Initialization

Before accessing flash, the algorithm initializes the voltage regulators via the CRM (Clock and Reset Module), mirroring libmc1322x's `default_vreg_init()` plus the readiness waits from `nvm-read.c`:

1. `CRM_SYS_CNTL (0x80003000) = 0x00000018` - set default system state.
2. `CRM_VREG_CNTL (0x80003048) = 0x00000f04` - bypass the buck converter, then a short delay for it to take effect.
3. `CRM_VREG_CNTL (0x80003048) = 0x00000ff8` - start the regulators.
4. Poll `CRM_STATUS (0x80003018)` until both `VREG_1P5V_RDY` (bit 19) and `VREG_1P8V_RDY` (bit 18) are set.

### Privileged-Mode Stack Setup

`Init()` sets up a distinct stack pointer for every ARM7TDMI privileged mode (FIQ/IRQ/SVC/UND/ABT)
before calling the ROM's `rom_data_init`, mirroring what libmc1322x's `start.S` does on a normal
boot - `probe-rs`'s `call_function` only initializes the *current* mode's SP, and `rom_data_init`
internally bank-switches through the others. It also masks IRQ+FIQ for the rest of `Init()`
(deliberately not restored). Both together fixed a real SWI-vector-trap crash that used to happen
partway through `Init()` on real hardware; see `Algorithm::new`'s own doc comment in `main.rs` for
the full account.

### Flash Type Detection

The algorithm automatically detects the installed flash chip type:
- **SST** (Silicon Storage Technology)
- **ST** (STMicroelectronics)
- **ATM** (Atmel)

### Error Codes

The algorithm uses custom error codes in the range 0x7000-0x7008:

| Code | Description |
|------|-------------|
| 0x7001 | Failed to detect NVM |
| 0x7002 | No NVM detected |
| 0x7003 | Erase all failed |
| 0x7004 | Invalid sector number |
| 0x7005 | Erase sector failed |
| 0x7006 | Address out of range |
| 0x7007 | Program page failed |
| 0x7008 | Unrecognized NVM type code from ROM |

## Known Limitations

- **External Flash**: Only internal flash is supported. External flash would require additional configuration.
- **ARM7TDMI Target**: Requires nightly Rust with `build-std` feature for ARM7TDMI support.

## Comparison with libmc1322x

This flash algorithm provides JTAG-based flashing, compared to libmc1322x's UART bootloader approach:

| Feature | libmc1322x | This Algorithm |
|---------|------------|----------------|
| Interface | UART | JTAG |
| Speed | ~10s for 30KB | Faster via JTAG |
| Debug | No | Yes (with probe-rs) |
| Bootloader | Required | Not required |

## References

- [libmc1322x](https://github.com/malvira/libmc1322x) - Original UART-based flash tools
- [MC13224V Datasheet](https://www.nxp.com/docs/en/data-sheet/MC13224V.pdf)
- [probe-rs Flash Algorithm Template](https://github.com/probe-rs/flash-algorithm-template)
- [CMSIS-Pack Flash Algorithms](https://open-cmsis-pack.github.io/Open-CMSIS-Pack-Spec/main/html/algorithmFunc.html)

## License

Licensed under the BSD 3-Clause License ([LICENSE](../LICENSE)).

## Contributing

Contributions are welcome! Please ensure:
- Code is formatted with `cargo fmt`
- Algorithm is tested on real hardware
- Documentation is updated

## Troubleshooting

### Flash Operations Fail

1. **Check voltage regulators**: Ensure VREG initialization is working
2. **Verify ROM addresses**: These are specific to MC13224V ROM version
3. **Check JTAG connection**: Use `probe-rs info` to verify connection

Note: the `debug!` macro is a no-op — the ARM7TDMI has no atomic instructions, so RTT (which
`rprintln` relies on) isn't available here.

### NVM Detection Fails

The ROM detect function may need time for regulators to stabilize. The algorithm includes a delay, but you may need to adjust it for your hardware.

## Contact

For issues and questions:
- File an issue in the probe-rs repository
- Join the [probe-rs Matrix chat](https://matrix.to/#/#probe-rs:matrix.org)
