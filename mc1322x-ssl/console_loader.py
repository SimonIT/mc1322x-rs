#!/usr/bin/env python3
"""Full "Console Loader" (CL) flow, per NXP AN3860 section 3: update a MC1322x's internal
flash from a completely cold, blank chip - no JTAG/debug probe involved at any point.

Composes the two pieces the rest of this directory implements separately:
  1. `ssl_protocol.boot_handshake_load` - the ROM's own UART1 cold-boot bootstrap (AN3860
     section 2.6), used here to load the `ssl` RAM app itself.
  2. `flash_host.flash_image` - the SSL UART command protocol (AN3860 section 4), used to
     erase/write/commit the target firmware image once `ssl` reports itself `READY`.

Both phases run over the same serial connection, matching AN3860's own Console Loader
exactly (open COM port -> send ssl.bin during boot -> connect/communicate with the running
ssl -> load the desired flash image -> write the flash header).

Prerequisites (see the mc1322x-ssl README / AN3860 sections 2.2 and 2.6), neither of which
this script can do for you:
  - The target flash must already be erased/blank (via the board's erase-jumper procedure),
    or the ROM will just boot the existing image and never fall through to the UART1 boot
    source at all.
  - `UART1_RTS` must be physically driven low *before* the board is reset - `--rts-low` only
    asserts pyserial's RTS output; whether that actually reaches the chip's RTS pin depends
    on the serial adapter's wiring.

Usage:
    console_loader.py <serial-port> <path-to-ssl.bin> <unsecured|secured> <path-to-firmware.bin> [--rts-low|--rts-high]

Not yet verified end-to-end on real hardware (each phase has been exercised on its own - see
uart_load.py/flash_host.py's own histories - but not this exact composed cold-boot flow).
"""

import sys

import serial

from flash_host import flash_image
from ssl_protocol import ENG_SECURED, ENG_UNSECURED, boot_handshake_load, wait_for_ready


def main():
    if len(sys.argv) < 5:
        print(
            f"usage: {sys.argv[0]} <serial-port> <ssl.bin> <unsecured|secured> "
            "<firmware.bin> [--rts-low|--rts-high]"
        )
        sys.exit(1)

    port, ssl_path, security, firmware_path = sys.argv[1:5]
    rts_mode = sys.argv[5] if len(sys.argv) > 5 else None

    if security not in ("unsecured", "secured"):
        print("security must be 'unsecured' or 'secured'")
        sys.exit(1)
    secure_byte = ENG_UNSECURED if security == "unsecured" else ENG_SECURED

    with open(ssl_path, "rb") as f:
        ssl_image = f.read()
    print(f"Loaded ssl loader: {len(ssl_image)} bytes from {ssl_path}")

    with open(firmware_path, "rb") as f:
        firmware_image = f.read()
    print(f"Loaded target firmware: {len(firmware_image)} bytes from {firmware_path}")

    ser = serial.Serial(port, baudrate=115200, timeout=0.2)
    ser.rts = False  # pyserial default is line-idle; leave alone unless told otherwise
    if rts_mode == "--rts-low":
        ser.rts = True  # pyserial's .rts=True asserts the line (drives it low on most adapters)
        print("Asserted RTS (driving it low)")
    elif rts_mode == "--rts-high":
        ser.rts = False
        print("Deasserted RTS (driving it high)")

    print("Reset the board now (or it must already be freshly reset).")
    boot_handshake_load(ser, ssl_image)

    ser.timeout = 1
    wait_for_ready(ser)
    flash_image(ser, firmware_image, secure_byte)
    print("All done. Reset/power-cycle the board to boot the new image.")


if __name__ == "__main__":
    main()
