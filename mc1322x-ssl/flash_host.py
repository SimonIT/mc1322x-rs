#!/usr/bin/env python3
"""Host-side "Console Loader" for the ssl example, per NXP AN3860's documented protocol.

Talks to the ssl RAM app (already loaded and running on the target via JTAG - this script
does not do any UART bootstrap/baud-detection handshake, unlike AN3860's own host tool,
since it assumes ssl is already running) to erase, write, and commit a firmware image to
the MC1322x's internal flash.

For the full cold-boot flow (no debug probe at all, loading ssl itself over UART1 first),
see console_loader.py instead.

Usage:
    flash_host.py <serial-port> <unsecured|secured> <path-to-firmware.bin>
"""

import sys

import serial

from ssl_protocol import (
    CMD_COMMIT_REQUEST,
    CMD_ERASE_REQUEST,
    CMD_WRITE_REQUEST,
    ENG_SECURED,
    ENG_UNSECURED,
    HEADER_SIZE,
    build_frame,
    expect_confirm,
    wait_for_ready,
)

CHUNK_SIZE = 512


def main():
    if len(sys.argv) != 4:
        print(f"usage: {sys.argv[0]} <serial-port> <unsecured|secured> <firmware.bin>")
        sys.exit(1)

    port, security, path = sys.argv[1], sys.argv[2], sys.argv[3]
    if security not in ("unsecured", "secured"):
        print("security must be 'unsecured' or 'secured'")
        sys.exit(1)
    secure_byte = ENG_UNSECURED if security == "unsecured" else ENG_SECURED

    with open(path, "rb") as f:
        image = f.read()
    print(f"Loaded {len(image)} bytes from {path}")

    ser = serial.Serial(port, baudrate=115200, timeout=1)
    wait_for_ready(ser)
    flash_image(ser, image, secure_byte)
    print("All done. Reset/power-cycle the board to boot the new image.")


def flash_image(ser: serial.Serial, image: bytes, secure_byte: int):
    """Erase, write, and commit `image` to flash over an already-connected, already-`READY`
    SSL session. Shared with console_loader.py's end-to-end flow."""
    print("Erasing flash (whole chip except reserved last sector; can take a while)...")
    ser.write(build_frame(bytes([CMD_ERASE_REQUEST]) + (0xFFFFFFFF).to_bytes(4, "little")))
    expect_confirm(ser, "Erase", timeout_s=200.0)

    print(f"Writing {len(image)} bytes in {CHUNK_SIZE}-byte chunks (at flash offset {HEADER_SIZE})...")
    offset = 0
    while offset < len(image):
        chunk = image[offset : offset + CHUNK_SIZE]
        flash_addr = HEADER_SIZE + offset
        body = (
            bytes([CMD_WRITE_REQUEST])
            + flash_addr.to_bytes(4, "little")
            + len(chunk).to_bytes(2, "little")
            + chunk
        )
        ser.write(build_frame(body))
        expect_confirm(ser, f"Write @0x{flash_addr:06X} ({len(chunk)} bytes)", timeout_s=20.0)
        offset += len(chunk)

    security = "secured" if secure_byte == ENG_SECURED else "unsecured"
    print(f"Committing image (length={len(image)}, {security})...")
    ser.write(
        build_frame(
            bytes([CMD_COMMIT_REQUEST]) + len(image).to_bytes(4, "little") + bytes([secure_byte])
        )
    )
    expect_confirm(ser, "Commit", timeout_s=20.0)


if __name__ == "__main__":
    main()
