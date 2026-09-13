#!/usr/bin/env python3
"""Non-destructive smoke test for the ssl example: waits for READY, then issues a single
Read Request and prints what comes back. Doesn't erase or write anything."""

import sys

import serial

from ssl_protocol import CMD_READ_REQUEST, CMD_READ_RESPONSE, build_frame, read_frame, wait_for_ready


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
    address = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0
    length = int(sys.argv[3]) if len(sys.argv) > 3 else 16

    ser = serial.Serial(port, baudrate=115200, timeout=1)
    wait_for_ready(ser)

    print(f"Sending Read Request: address=0x{address:08X} length={length}")
    body = bytes([CMD_READ_REQUEST]) + address.to_bytes(4, "little") + length.to_bytes(2, "little")
    ser.write(build_frame(body))

    resp = read_frame(ser)
    if resp[0] != CMD_READ_RESPONSE:
        print(f"Unexpected response command 0x{resp[0]:02X}: {resp.hex()}")
        return
    status = resp[1]
    resp_len = int.from_bytes(resp[2:4], "little")
    data = resp[4 : 4 + resp_len]
    print(f"status=0x{status:02X} length={resp_len} data={data.hex()}")


if __name__ == "__main__":
    main()
