"""Shared SSL UART protocol + ROM UART1 boot-handshake helpers.

Factored out of flash_host.py/read_test.py/uart_load.py (which used to each carry their own
copy of the frame/CRC logic) so console_loader.py can compose the boot handshake and the SSL
command protocol into one end-to-end flow, matching AN3860's own "Console Loader" role.
"""

import time

import serial

SOF = 0x55

CMD_READ_REQUEST = 0x01
CMD_READ_RESPONSE = 0x02
CMD_WRITE_REQUEST = 0x03
CMD_COMMIT_REQUEST = 0x04
CMD_ERASE_REQUEST = 0x05
CMD_CONFIRM = 0xF0

STATUS_NAMES = {
    0x00: "gEngValidReq_c",
    0x01: "gEngInvalidReq_c",
    0x02: "gEngSuccessOp_c",
    0x03: "gEngWriteError_c",
    0x04: "gEngReadError_c",
    0x05: "gEngCRCError_c",
    0x06: "gEngCommError_c",
    0x07: "gEngExecError_c",
}

ENG_SECURED = 0xC3
ENG_UNSECURED = 0x3C

# AN3860 Table 1 ("Valid FLASH Boot Image"): the on-flash layout is an 8-byte header
# (4-byte Signature + 4-byte little-endian Length, written by Commit) followed by the
# executable itself - the image bytes must land at this offset, not 0, or Commit's header
# write clobbers the start of the loaded code.
HEADER_SIZE = 8


def build_frame(command_bytes: bytes) -> bytes:
    length = len(command_bytes)
    crc = sum(command_bytes) & 0xFF
    return bytes([SOF]) + length.to_bytes(2, "little") + command_bytes + bytes([crc])


def read_frame(ser: serial.Serial, timeout_s: float = 5.0) -> bytes:
    """Read one SOF-delimited frame, returning the command bytes (CRC already checked)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        b = ser.read(1)
        if not b or b[0] != SOF:
            continue
        length_bytes = ser.read(2)
        if len(length_bytes) != 2:
            continue
        length = int.from_bytes(length_bytes, "little")
        command = ser.read(length)
        if len(command) != length:
            continue
        crc = ser.read(1)
        if len(crc) != 1:
            continue
        expected = sum(command) & 0xFF
        if crc[0] != expected:
            print("  (bad CRC on received frame, ignoring)")
            continue
        return command
    raise TimeoutError("timed out waiting for a response frame")


def expect_confirm(ser: serial.Serial, what: str, timeout_s: float = 5.0):
    frame = read_frame(ser, timeout_s)
    if frame[0] != CMD_CONFIRM:
        raise RuntimeError(f"{what}: expected Confirm (0xF0), got command 0x{frame[0]:02X}")
    status = frame[1]
    name = STATUS_NAMES.get(status, f"0x{status:02X}")
    if status != 0x02:
        raise RuntimeError(f"{what}: failed with status {name}")
    print(f"  {what}: {name}")


def wait_for_ready(ser: serial.Serial, timeout_s: float = 10.0):
    """Wait for the SSL's own "READY" banner, printed once its UART/NVM init completes."""
    print("Waiting for READY banner from the target...")
    buf = b""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        buf += ser.read(64)
        if b"READY" in buf:
            print("Got READY.")
            return
    raise TimeoutError("never saw READY - is ssl running and connected to the right port?")


def boot_handshake_load(ser: serial.Serial, image: bytes, connect_timeout_s: float = 60.0):
    """Load `image` into RAM via the MC1322x boot ROM's native UART1 bootstrap (AN3860
    section 2.6): send '\\0' until "CONNECT", send a 4-byte little-endian length, then the
    binary itself - matching mc1322x-sys/libmc1322x/tools/mc1322x-load.c exactly.

    Unlike the JTAG halt+load_image+jump technique the rest of this project mostly uses, this
    goes through the chip's real reset vector and the boot ROM's own full initialization
    before the loaded binary ever runs - so it needs no debug probe at all, only a serial
    connection. The caller must have already reset the board with `UART1_RTS` driven low (see
    `console_loader.py`'s `--rts-low`) - this function only drives the UART side of the
    handshake, not the physical reset/RTS timing.
    """
    print("Waiting for CONNECT...")
    buf = b""
    deadline = time.time() + connect_timeout_s
    while time.time() < deadline:
        ser.write(b"\0")
        buf += ser.read(64)
        if b"CONNECT" in buf:
            print("Got CONNECT.")
            break
        print(".", end="", flush=True)
    else:
        print()
        raise TimeoutError("never got CONNECT - check RTS wiring/level, reset timing, or port")

    print(f"Sending length ({len(image)}) + binary...")
    ser.write(len(image).to_bytes(4, "little"))
    for byte in image:
        ser.write(bytes([byte]))
        time.sleep(50e-6)  # matches mc1322x-load's default first_delay (50us)
    print("Done sending.")
