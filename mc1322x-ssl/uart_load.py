#!/usr/bin/env python3
"""Load a RAM binary via the MC1322x boot ROM's native UART1 bootstrap, matching
mc1322x-sys/libmc1322x/tools/mc1322x-load.c exactly: send '\\0' until "CONNECT", send a
4-byte little-endian length, then the binary itself.

Unlike our JTAG halt+load_image+jump technique, this goes through the chip's real reset
vector and the boot ROM's own full initialization before the loaded binary ever runs.

Usage:
    uart_load.py <serial-port> <binary.bin> [--rts-low]
"""

import sys

import serial

from ssl_protocol import boot_handshake_load


def main():
    if len(sys.argv) < 3:
        print(f"usage: {sys.argv[0]} <serial-port> <binary.bin> [--rts-low|--rts-high]")
        sys.exit(1)

    port, path = sys.argv[1], sys.argv[2]
    rts_mode = sys.argv[3] if len(sys.argv) > 3 else None

    with open(path, "rb") as f:
        image = f.read()
    print(f"Loaded {len(image)} bytes from {path}")

    ser = serial.Serial(port, baudrate=115200, timeout=0.2)
    ser.rts = False  # pyserial default is line-idle; leave alone unless told otherwise
    if rts_mode == "--rts-low":
        ser.rts = True  # pyserial's .rts=True asserts the line (drives it low on most adapters)
        print("Asserted RTS (driving it low)")
    elif rts_mode == "--rts-high":
        ser.rts = False
        print("Deasserted RTS (driving it high)")

    print("Reset the board now (or it must already be freshly reset).")
    boot_handshake_load(ser, image)
    print("Echoing serial output (Ctrl-C to stop):")

    ser.timeout = 1
    try:
        while True:
            data = ser.read(256)
            if data:
                sys.stdout.buffer.write(data)
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
