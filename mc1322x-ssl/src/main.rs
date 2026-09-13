//! Minimal Second Stage Loader (SSL): writes a firmware image to internal flash over UART1.
//!
//! Implements the subset of NXP AN3860's ("MC1322x Flash Loader Utility (Second Stage
//! Loader)") documented UART protocol needed to erase, write, and commit a flash image:
//! Erase Request (0x05), Write Request (0x03), Commit Request (0x04), Read Request (0x01),
//! and the Confirm/Read Response replies. Loaded into RAM via JTAG (halt + load_image + set
//! PC + resume) rather than the ROM UART1 bootstrap AN3860 describes, so there's no baud-rate
//! detection handshake - just the "READY" banner and command loop AN3860 section 4.4
//! describes from step 6 onward.
//!
//! # Building
//!
//! ```text
//! cargo build -p ssl --target thumbv4t-none-eabi
//! ```

#![no_std]
#![no_main]

const BAUD: u32 = 115_200;

/// SSL UART command format (AN3860 Table 2): SOF, then a 2-byte little-endian length, then
/// that many command bytes, then a 1-byte sum-of-bytes checksum.
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

/// Commit's "Secure" field values (AN3860 4.2.4).
const ENG_SECURED: u8 = 0xC3;
const ENG_UNSECURED: u8 = 0x3C;

mc1322x_hal::entry!(arm_main);

// Raw UART1 TX, bypassing `Uart::new` - the peripheral/pins are already configured from
// `arm_main`'s own `Uart::new` call, so this avoids reinitializing them (GPIO func-select,
// baud divider) mid-panic while a write may be in flight. Also used outside the panic handler
// (e.g. to report a ROM error code on a failed write) since it's a convenient,
// allocation-free way to get a diagnostic byte onto the wire.
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
    use mc1322x_hal::nvm::{Nvm, NvmInterface};
    use mc1322x_hal::uart::{Uart, UartId};

    let mut uart = Uart::new(UartId::Uart1, BAUD);
    let _ = uart.write_all(b"UART_OK\r\n");

    // Regulator power-up and ROM secure-variable clear (both required before any `nvm_*` ROM
    // call) live in `Nvm::new_assume_sst`/`Nvm::new` - see `mc1322x_hal::nvm` for why.
    //
    // On these particular Econotag boards the serial flash is wired to the *Internal* NVM
    // interface, not External (GPIO4-7): External read back a floating-bus pattern (0x00,
    // consistent but not real data) that made `erase`/`read` falsely report success while
    // `write`'s internal verify step caught the mismatch (`gNvmErrVerifyError_c`). Internal
    // reads back proper 0xFF post-erase and verifies real writes correctly.
    let mut nvm = Nvm::new_assume_sst(NvmInterface::Internal);
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
                    // Whole chip, excluding the reserved last 4 KB sector (AN3860 4.2.5).
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

/// Read one SOF-delimited frame into `buf`, returning the command length on success. Returns
/// `None` (and discards nothing but the bad checksum) on a CRC mismatch, matching AN3860's
/// "invalid command -> Confirm with an error status" behavior.
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
