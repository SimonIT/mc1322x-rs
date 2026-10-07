//! Minimal Second Stage Loader (SSL): writes a firmware image to flash over UART1.
//!
//! Implements the subset of the UART protocol from NXP AN3860 ("MC1322x Flash Loader Utility
//! (Second Stage Loader)") needed to erase, write, commit and read back an image: Read (0x01),
//! Write (0x03), Commit (0x04) and Erase (0x05) requests, answered with Confirm or Read Response
//! frames. It is loaded into RAM over JTAG rather than through the ROM UART1 bootstrap, so there
//! is no baud-rate detection: it prints `READY` and enters the command loop (AN3860 §4.4, from
//! step 6).
//!
//! # Building
//!
//! ```text
//! cargo +nightly build -p mc1322x-ssl --release
//! ```

#![no_std]
#![no_main]

const BAUD: u32 = 115_200;

/// Start of frame. A frame (AN3860 Table 2) is SOF, a 2-byte little-endian length, that many
/// command bytes, and a 1-byte sum-of-bytes checksum.
const SOF: u8 = 0x55;

mod cmd {
    pub const READ_REQUEST: u8 = 0x01;
    pub const READ_RESPONSE: u8 = 0x02;
    pub const WRITE_REQUEST: u8 = 0x03;
    pub const COMMIT_REQUEST: u8 = 0x04;
    pub const ERASE_REQUEST: u8 = 0x05;
    pub const CONFIRM: u8 = 0xF0;
}

mod status {
    pub const SUCCESS: u8 = 0x02;
    pub const WRITE_ERROR: u8 = 0x03;
    pub const READ_ERROR: u8 = 0x04;
    pub const CRC_ERROR: u8 = 0x05;
    pub const EXEC_ERROR: u8 = 0x07;
}

/// Commit's "Secure" field values (AN3860 §4.2.4).
const ENG_SECURED: u8 = 0xC3;
const ENG_UNSECURED: u8 = 0x3C;

mc1322x_hal::entry!(arm_main);

// Raw UART1 TX for diagnostics (panic handler, ROM error codes). Uses the UART as configured by
// `arm_main`'s `Uart::new`, without reinitializing it or needing the `Uart` handle.
fn debug_putc(byte: u8) {
    unsafe {
        let utxcon = (mc1322x_sys::UART1_BASE + mc1322x_sys::UTXCON) as *const u32;
        while utxcon.read_volatile() & 0x3F == 0 {}
        let udata = (mc1322x_sys::UART1_BASE + mc1322x_sys::UDATA) as *mut u32;
        udata.write_volatile(byte as u32);
    }
}

fn debug_puts(bytes: &[u8]) {
    for &b in bytes {
        debug_putc(b);
    }
}

fn debug_hex_u32(v: u32) {
    for shift in [24, 16, 8, 0] {
        let byte = (v >> shift) as u8;
        debug_puts(&[DIGITS[(byte >> 4) as usize], DIGITS[(byte & 0xf) as usize]]);
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    debug_puts(b"\r\nPANIC at ");
    if let Some(loc) = info.location() {
        debug_puts(b"file=");
        debug_puts(loc.file().as_bytes());
        debug_puts(b" line=0x");
        debug_hex_u32(loc.line());
    } else {
        debug_puts(b"<no location>");
    }
    debug_puts(b"\r\n");
    loop {}
}

/// Largest single write chunk this loader accepts (kept well under the 96 KB RAM budget).
const MAX_CHUNK: usize = 1024;
const DIGITS: &[u8; 16] = b"0123456789abcdef";

fn arm_main() -> ! {
    use embedded_io::Write;
    use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
    use mc1322x_hal::nvm::Nvm;
    use mc1322x_hal::uart::{Uart, UartId};

    let mut uart = Uart::new(UartId::Uart1, BAUD);
    let _ = uart.write_all(b"UART_OK\r\n");

    // Skips `nvm_detect`, which can hang when loaded over JTAG. The NVM interface comes from
    // mc1322x-hal's `board-*` feature.
    let mut nvm = Nvm::new_assume_sst();
    let _ = uart.write_all(b"NVM_READY\r\n");
    let mut probe = [0u8; 16];
    let _ = uart.write_all(b"READING\r\n");
    match nvm.read(0, &mut probe) {
        Ok(()) => {
            let _ = uart.write_all(b"READ_OK: ");
            for b in probe {
                let hex = [DIGITS[(b >> 4) as usize], DIGITS[(b & 0xf) as usize]];
                let _ = uart.write_all(&hex);
            }
            let _ = uart.write_all(b"\r\n");
        }
        Err(_) => {
            let _ = uart.write_all(b"READ_FAILED\r\n");
        }
    }

    let _ = uart.write_all(b"READY");

    let mut cmd_buf = [0u8; MAX_CHUNK + 8];

    loop {
        let Some(len) = read_frame(&mut uart, &mut cmd_buf) else {
            continue;
        };
        let frame = &cmd_buf[..len];
        let id = frame[0];

        match id {
            cmd::ERASE_REQUEST if len == 5 => {
                let address = u32::from_le_bytes([frame[1], frame[2], frame[3], frame[4]]);
                let ok = if address == 0xFFFF_FFFF {
                    // Whole chip, excluding the reserved last 4 KB sector (AN3860 §4.2.5).
                    nvm.erase(0, 31 * 4096).is_ok()
                } else {
                    let sector_start = address - (address % 4096);
                    nvm.erase(sector_start, sector_start + 4096).is_ok()
                };
                send_confirm(
                    &mut uart,
                    if ok {
                        status::SUCCESS
                    } else {
                        status::EXEC_ERROR
                    },
                );
            }
            cmd::WRITE_REQUEST if len >= 7 => {
                let address = u32::from_le_bytes([frame[1], frame[2], frame[3], frame[4]]);
                let data_len = u16::from_le_bytes([frame[5], frame[6]]) as usize;
                let ok = if len != 7 + data_len {
                    false
                } else {
                    match nvm.write(address, &frame[7..7 + data_len]) {
                        Ok(()) => true,
                        Err(e) => {
                            debug_puts(b"WRITE_ERR code=0x");
                            let code = match e {
                                mc1322x_hal::nvm::Error::Rom(c) => c as u32,
                                mc1322x_hal::nvm::Error::Bounds(_) => 0xB0000000,
                                mc1322x_hal::nvm::Error::UnsupportedFlash => 0xF0000000,
                            };
                            debug_hex_u32(code);
                            debug_puts(b"\r\n");
                            false
                        }
                    }
                };
                send_confirm(
                    &mut uart,
                    if ok {
                        status::SUCCESS
                    } else {
                        status::WRITE_ERROR
                    },
                );
            }
            cmd::COMMIT_REQUEST if len == 6 => {
                let image_len = u32::from_le_bytes([frame[1], frame[2], frame[3], frame[4]]);
                let secure = frame[5];
                let signature: [u8; 4] = match secure {
                    ENG_SECURED => *b"SECU",
                    ENG_UNSECURED => *b"OKOK",
                    _ => {
                        send_confirm(&mut uart, status::EXEC_ERROR);
                        continue;
                    }
                };
                let mut header = [0u8; 8];
                header[..4].copy_from_slice(&signature);
                header[4..].copy_from_slice(&image_len.to_le_bytes());
                let ok = nvm.write(0, &header).is_ok();
                send_confirm(
                    &mut uart,
                    if ok {
                        status::SUCCESS
                    } else {
                        status::WRITE_ERROR
                    },
                );
            }
            cmd::READ_REQUEST if len == 7 => {
                let address = u32::from_le_bytes([frame[1], frame[2], frame[3], frame[4]]);
                let data_len = (u16::from_le_bytes([frame[5], frame[6]]) as usize).min(MAX_CHUNK);
                let mut data = [0u8; MAX_CHUNK];
                match nvm.read(address, &mut data[..data_len]) {
                    Ok(()) => send_read_response(&mut uart, status::SUCCESS, &data[..data_len]),
                    Err(_) => send_read_response(&mut uart, status::READ_ERROR, &[]),
                }
            }
            _ => send_confirm(&mut uart, status::EXEC_ERROR),
        }
    }
}

/// Read one frame's command bytes into `buf` and return their length.
///
/// Returns `None` on a UART error, a zero or oversized length, or a checksum mismatch; only the
/// last one is answered (with a `CRC_ERROR` Confirm).
fn read_frame(uart: &mut mc1322x_hal::uart::Uart, buf: &mut [u8]) -> Option<usize> {
    use embedded_io::Read;

    // Scan for SOF, ignoring anything else on the wire.
    loop {
        let mut b = [0u8; 1];
        if uart.read(&mut b).ok()? != 1 {
            continue;
        }
        if b[0] == SOF {
            break;
        }
    }

    let mut len_bytes = [0u8; 2];
    read_exact(uart, &mut len_bytes)?;
    let len = u16::from_le_bytes(len_bytes) as usize;
    if len == 0 || len > buf.len() {
        return None;
    }

    read_exact(uart, &mut buf[..len])?;

    let mut crc_byte = [0u8; 1];
    read_exact(uart, &mut crc_byte)?;
    let expected: u8 = buf[..len].iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
    if crc_byte[0] != expected {
        send_confirm(uart, status::CRC_ERROR);
        return None;
    }

    Some(len)
}

fn read_exact(uart: &mut mc1322x_hal::uart::Uart, buf: &mut [u8]) -> Option<()> {
    use embedded_io::Read;
    let mut filled = 0;
    while filled < buf.len() {
        let n = uart.read(&mut buf[filled..]).ok()?;
        filled += n;
    }
    Some(())
}

fn send_confirm(uart: &mut mc1322x_hal::uart::Uart, status: u8) {
    send_frame(uart, &[cmd::CONFIRM, status]);
}

fn send_read_response(uart: &mut mc1322x_hal::uart::Uart, status: u8, data: &[u8]) {
    let mut header = [0u8; 4];
    header[0] = cmd::READ_RESPONSE;
    header[1] = status;
    header[2..4].copy_from_slice(&(data.len() as u16).to_le_bytes());

    use embedded_io::Write;
    let total_len = (header.len() + data.len()) as u16;
    let _ = uart.write_all(&[SOF]);
    let _ = uart.write_all(&total_len.to_le_bytes());
    let _ = uart.write_all(&header);
    let _ = uart.write_all(data);
    let crc = header
        .iter()
        .chain(data.iter())
        .fold(0u8, |acc, &b| acc.wrapping_add(b));
    let _ = uart.write_all(&[crc]);
}

fn send_frame(uart: &mut mc1322x_hal::uart::Uart, command: &[u8]) {
    use embedded_io::Write;
    let len = command.len() as u16;
    let _ = uart.write_all(&[SOF]);
    let _ = uart.write_all(&len.to_le_bytes());
    let _ = uart.write_all(command);
    let crc = command.iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
    let _ = uart.write_all(&[crc]);
}
