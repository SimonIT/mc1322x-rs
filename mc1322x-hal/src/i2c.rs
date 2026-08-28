use core::cell::RefCell;
use core::task::{Poll, Waker};
use critical_section::Mutex;
use embedded_hal::i2c::{self, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress};
use mc1322x_sys::{
    I2C_BASE, I2C_CKEN, I2C_MAL, I2C_MBB, I2C_MCF, I2C_MEN, I2C_MIEN, I2C_MIF, I2C_MSTA, I2C_MTX,
    I2C_RSTA, I2C_RXAK, I2C_SCL, I2C_SDA, I2C_TXAK, INTBASE, gpio_reg_set, gpio_select_function,
};

use crate::util::yield_now;

// I2C register map (byte-wide MMIO, see libmc1322x/lib/include/i2c.h)
const I2C_ADR: *mut u8 = I2C_BASE as usize as *mut u8;
const I2C_FDR: *mut u8 = (I2C_BASE as usize + 0x04) as *mut u8;
const I2C_CR: *mut u8 = (I2C_BASE as usize + 0x08) as *mut u8;
const I2C_SR: *mut u8 = (I2C_BASE as usize + 0x0C) as *mut u8;
const I2C_DR: *mut u8 = (I2C_BASE as usize + 0x10) as *mut u8;
const I2C_CKER: *mut u8 = (I2C_BASE as usize + 0x18) as *mut u8;

// GPIO block-0 registers (pins 0..31); SCL/SDA are GPIO12/GPIO13
const GPIO_PAD_PU_EN0: *mut u32 = 0x8000_0010 as *mut u32;
const GPIO_PAD_PU_SEL0: *mut u32 = 0x8000_0030 as *mut u32;

const I2C_ALT_FUNCTION: u8 = 1;
const I2C_CLOCK_DIVIDER: u8 = 0x20; // ~150 kHz on the Redbee Econotag

// ITC (interrupt controller) offset/number for the I2C completion interrupt (see
// `isr.h`'s `INTENNUM_OFF` and `interrupt_nums`). `irq()` (linked from `libmc1322x`)
// dispatches it to the weak `i2c_isr` symbol overridden at the bottom of this file.
const INTENNUM_OFF: u32 = 0x8;
const INT_NUM_I2C: u32 = 4;

/// Waker for the in-flight [`I2c0::wait_byte_async`] call, if any.
///
/// Set (with `I2C_MIEN` armed) by `wait_byte_async` before it returns `Pending`, and taken
/// and woken by [`i2c_isr`] the next time the module raises the interrupt. Guarded by
/// `critical_section`'s `Mutex`, backed by this crate's own `critical_section::Impl` (see
/// `crate::critical_section_impl`), so every consumer of `mc1322x-hal` — not just this
/// module — gets a working provider without extra wiring.
static WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));

/// I2C master error.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Error {
    /// The addressed slave did not acknowledge its address or a data byte.
    NoAcknowledge(NoAcknowledgeSource),
    /// The arbitration was lost (e.g. bus contention).
    ArbitrationLost,
}

impl i2c::Error for Error {
    fn kind(&self) -> i2c::ErrorKind {
        match *self {
            Error::NoAcknowledge(n) => i2c::ErrorKind::NoAcknowledge(n),
            Error::ArbitrationLost => i2c::ErrorKind::ArbitrationLoss,
        }
    }
}

/// I2C master on the module 0 instance (SCL = GPIO12, SDA = GPIO13).
///
/// The blocking `embedded-hal` [`I2c`] implementation drives the module by polling the
/// status register. The `embedded-hal-async` implementation instead arms the module's
/// completion interrupt (`I2C_MIEN`, routed through the ITC as `INT_NUM_I2C`) and waits to be
/// woken by [`i2c_isr`] — see [`Self::wait_byte_async`] for the arm/wake handshake.
pub struct I2c0;

impl I2c0 {
    /// Enable the I2C module, mux the SDA/SCL pads to their I2C function and
    /// activate the internal pull-ups, then return a master-ready instance.
    pub fn new() -> Self {
        unsafe {
            // gate the clock to the I2C module
            write_u8(I2C_CKER, I2C_CKEN as u8);
            // SCL frequency divider
            write_u8(I2C_FDR, I2C_CLOCK_DIVIDER);
            // our own (unused, master-only) slave address
            write_u8(I2C_ADR, 0x01);
            // enable the module; auto-ack on, no interrupts
            write_u8(I2C_CR, I2C_MEN as u8);

            // GPIO12 (SCL) and GPIO13 (SDA) -> I2C function
            gpio_select_function(I2C_SCL as u8, I2C_ALT_FUNCTION);
            gpio_select_function(I2C_SDA as u8, I2C_ALT_FUNCTION);
            // internal pull-ups on both lines
            gpio_reg_set(GPIO_PAD_PU_EN0, I2C_SCL as u8);
            gpio_reg_set(GPIO_PAD_PU_EN0, I2C_SDA as u8);
            gpio_reg_set(GPIO_PAD_PU_SEL0, I2C_SCL as u8);
            gpio_reg_set(GPIO_PAD_PU_SEL0, I2C_SDA as u8);

            // Route the I2C completion interrupt to the core. This only affects the async
            // path: the peripheral-local enable (`I2C_MIEN`) stays off until
            // `wait_byte_async` arms it, so the blocking API is unaffected.
            core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_I2C);
        }
        I2c0
    }

    /// Clear pending interrupt and arbitration-lost flags.
    fn clear_status(&mut self) {
        unsafe {
            let sr = read_u8(I2C_SR);
            write_u8(I2C_SR, sr & !(I2C_MIF as u8 | I2C_MAL as u8));
        }
    }

    /// Generate a STOP condition.
    fn stop(&mut self) {
        unsafe {
            let cr = read_u8(I2C_CR);
            write_u8(I2C_CR, cr & !(I2C_MSTA as u8));
        }
    }

    /// Check once whether the module has completed (or failed) the current byte transfer.
    ///
    /// Shared by the blocking [`Self::wait_byte`] (spins on this) and the async
    /// [`Self::wait_byte_async`] (checked once up front, then again each time [`i2c_isr`]
    /// wakes the task).
    fn poll_byte_status(&mut self) -> Poll<Result<(), Error>> {
        unsafe {
            let sr = read_u8(I2C_SR);
            if sr & I2C_MAL as u8 != 0 {
                write_u8(I2C_SR, sr & !(I2C_MAL as u8));
                self.stop();
                return Poll::Ready(Err(Error::ArbitrationLost));
            }
            if sr & I2C_MIF as u8 != 0 && sr & I2C_MCF as u8 != 0 {
                write_u8(I2C_SR, sr & !(I2C_MIF as u8));
                return Poll::Ready(Ok(()));
            }
        }
        Poll::Pending
    }

    /// Wait until the module reports a completed byte transfer.
    fn wait_byte(&mut self) -> Result<(), Error> {
        loop {
            if let Poll::Ready(result) = self.poll_byte_status() {
                return result;
            }
        }
    }

    /// Async equivalent of [`Self::wait_byte`].
    ///
    /// Arms the completion interrupt (`I2C_MIEN`) and waits for [`i2c_isr`] to wake this
    /// task, rather than polling in a loop. The check-then-arm sequence runs inside a single
    /// [`critical_section::with`] call so a completion landing between the status check and
    /// enabling the interrupt can't be missed: interrupts stay masked for the whole
    /// sequence, so if the hardware flag is already set by the time `I2C_MIEN` is written,
    /// the pending interrupt fires as soon as the critical section ends.
    async fn wait_byte_async(&mut self) -> Result<(), Error> {
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                if let Poll::Ready(result) = self.poll_byte_status() {
                    return Poll::Ready(result);
                }
                *WAKER.borrow(cs).borrow_mut() = Some(cx.waker().clone());
                unsafe {
                    write_u8(I2C_CR, read_u8(I2C_CR) | I2C_MIEN as u8);
                }
                Poll::Pending
            })
        })
        .await
    }

    /// Wait for the bus to be free, generate a START condition and transmit the
    /// slave address. `read` selects the R/W bit (1 = read).
    fn start(&mut self, address: u8, read: bool) -> Result<(), Error> {
        unsafe {
            while read_u8(I2C_SR) & I2C_MBB as u8 != 0 {}
            write_u8(I2C_CR, I2C_MEN as u8 | I2C_MSTA as u8 | I2C_MTX as u8);
            self.clear_status();
            write_u8(I2C_DR, (address << 1) | read as u8);
            self.wait_byte()?;
            if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                self.stop();
                return Err(Error::NoAcknowledge(NoAcknowledgeSource::Address));
            }
            Ok(())
        }
    }

    /// Generate a repeated START and transmit the slave address while keeping
    /// ownership of the bus.
    fn restart(&mut self, address: u8, read: bool) -> Result<(), Error> {
        unsafe {
            let cr = read_u8(I2C_CR);
            write_u8(I2C_CR, cr | I2C_MTX as u8 | I2C_RSTA as u8);
            self.clear_status();
            write_u8(I2C_DR, (address << 1) | read as u8);
            self.wait_byte()?;
            if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                self.stop();
                return Err(Error::NoAcknowledge(NoAcknowledgeSource::Address));
            }
            Ok(())
        }
    }

    /// Transmit `data`, checking for an ACK after every byte.
    fn transmit_data(&mut self, data: &[u8]) -> Result<(), Error> {
        for &byte in data {
            unsafe {
                write_u8(I2C_DR, byte);
            }
            self.wait_byte()?;
            unsafe {
                if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                    self.stop();
                    return Err(Error::NoAcknowledge(NoAcknowledgeSource::Data));
                }
            }
        }
        Ok(())
    }

    /// Receive `data` bytes. Call directly after an address/restart with the
    /// read bit set: the module is switched to receive mode and the address
    /// byte is discarded. When `nack_tail` is set this is the final read of the
    /// transaction, so the last byte is NACKed and the NACKed trailing byte is
    /// waited out before the STOP.
    fn receive_data(&mut self, data: &mut [u8], nack_tail: bool) -> Result<(), Error> {
        unsafe {
            // the address byte was just transmitted: switch to receive mode and
            // discard it
            let cr = read_u8(I2C_CR);
            write_u8(I2C_CR, cr & !(I2C_MTX as u8));
            read_u8(I2C_DR);
        }

        let len = data.len();
        for (index, byte) in data.iter_mut().enumerate() {
            self.wait_byte()?;
            unsafe {
                if nack_tail && index + 1 == len {
                    // last requested byte: disable auto-ack so the trailing byte
                    // is NACKed and the slave releases the bus
                    let cr = read_u8(I2C_CR);
                    write_u8(I2C_CR, cr | I2C_TXAK as u8);
                }
                *byte = read_u8(I2C_DR);
            }
        }

        if nack_tail && len > 0 {
            // wait for the NACKed trailing byte to complete before the STOP
            self.wait_byte()?;
        }
        Ok(())
    }

    /// Continue receiving after a previous read operation without a restart.
    fn continue_receive(&mut self, data: &mut [u8], nack_tail: bool) -> Result<(), Error> {
        let len = data.len();
        for (index, byte) in data.iter_mut().enumerate() {
            self.wait_byte()?;
            unsafe {
                if nack_tail && index + 1 == len {
                    let cr = read_u8(I2C_CR);
                    write_u8(I2C_CR, cr | I2C_TXAK as u8);
                }
                *byte = read_u8(I2C_DR);
            }
        }
        if nack_tail && len > 0 {
            self.wait_byte()?;
        }
        Ok(())
    }

    /// Async equivalent of [`Self::start`].
    async fn start_async(&mut self, address: u8, read: bool) -> Result<(), Error> {
        unsafe {
            while read_u8(I2C_SR) & I2C_MBB as u8 != 0 {
                yield_now().await;
            }
            write_u8(I2C_CR, I2C_MEN as u8 | I2C_MSTA as u8 | I2C_MTX as u8);
            self.clear_status();
            write_u8(I2C_DR, (address << 1) | read as u8);
        }
        self.wait_byte_async().await?;
        unsafe {
            if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                self.stop();
                return Err(Error::NoAcknowledge(NoAcknowledgeSource::Address));
            }
        }
        Ok(())
    }

    /// Async equivalent of [`Self::restart`].
    async fn restart_async(&mut self, address: u8, read: bool) -> Result<(), Error> {
        unsafe {
            let cr = read_u8(I2C_CR);
            write_u8(I2C_CR, cr | I2C_MTX as u8 | I2C_RSTA as u8);
            self.clear_status();
            write_u8(I2C_DR, (address << 1) | read as u8);
        }
        self.wait_byte_async().await?;
        unsafe {
            if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                self.stop();
                return Err(Error::NoAcknowledge(NoAcknowledgeSource::Address));
            }
        }
        Ok(())
    }

    /// Async equivalent of [`Self::transmit_data`].
    async fn transmit_data_async(&mut self, data: &[u8]) -> Result<(), Error> {
        for &byte in data {
            unsafe {
                write_u8(I2C_DR, byte);
            }
            self.wait_byte_async().await?;
            unsafe {
                if read_u8(I2C_SR) & I2C_RXAK as u8 != 0 {
                    self.stop();
                    return Err(Error::NoAcknowledge(NoAcknowledgeSource::Data));
                }
            }
        }
        Ok(())
    }

    /// Async equivalent of [`Self::receive_data`].
    async fn receive_data_async(&mut self, data: &mut [u8], nack_tail: bool) -> Result<(), Error> {
        unsafe {
            let cr = read_u8(I2C_CR);
            write_u8(I2C_CR, cr & !(I2C_MTX as u8));
            read_u8(I2C_DR);
        }

        let len = data.len();
        for (index, byte) in data.iter_mut().enumerate() {
            self.wait_byte_async().await?;
            unsafe {
                if nack_tail && index + 1 == len {
                    let cr = read_u8(I2C_CR);
                    write_u8(I2C_CR, cr | I2C_TXAK as u8);
                }
                *byte = read_u8(I2C_DR);
            }
        }

        if nack_tail && len > 0 {
            self.wait_byte_async().await?;
        }
        Ok(())
    }

    /// Async equivalent of [`Self::continue_receive`].
    async fn continue_receive_async(
        &mut self,
        data: &mut [u8],
        nack_tail: bool,
    ) -> Result<(), Error> {
        let len = data.len();
        for (index, byte) in data.iter_mut().enumerate() {
            self.wait_byte_async().await?;
            unsafe {
                if nack_tail && index + 1 == len {
                    let cr = read_u8(I2C_CR);
                    write_u8(I2C_CR, cr | I2C_TXAK as u8);
                }
                *byte = read_u8(I2C_DR);
            }
        }
        if nack_tail && len > 0 {
            self.wait_byte_async().await?;
        }
        Ok(())
    }
}

impl Default for I2c0 {
    fn default() -> Self {
        Self::new()
    }
}

impl ErrorType for I2c0 {
    type Error = Error;
}

impl I2c<SevenBitAddress> for I2c0 {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }
        let n_ops = operations.len();
        let mut prev_is_read: Option<bool> = None;
        for (index, operation) in operations.iter_mut().enumerate() {
            let is_read = matches!(operation, Operation::Read(_));
            let is_last = index + 1 == n_ops;

            match prev_is_read {
                None => self.start(address, is_read)?,
                Some(prev) if prev != is_read => self.restart(address, is_read)?,
                Some(_) => {}
            }

            match operation {
                Operation::Read(buf) => {
                    if prev_is_read.is_none() || prev_is_read != Some(is_read) {
                        self.receive_data(buf, is_last)?;
                    } else {
                        self.continue_receive(buf, is_last)?;
                    }
                }
                Operation::Write(buf) => {
                    self.transmit_data(buf)?;
                }
            }
            prev_is_read = Some(is_read);
        }
        self.stop();
        Ok(())
    }
}

/// `embedded-hal-async`'s `I2c` reuses `embedded-hal`'s `ErrorType`/`Operation`/
/// `SevenBitAddress`, so [`ErrorType`] above already covers it; only `transaction` itself
/// needs an async implementation.
///
/// Waits on the module's real completion interrupt rather than polling in a loop; see
/// [`Self::wait_byte_async`] and [`i2c_isr`]. The one exception is the initial
/// bus-not-busy wait in [`Self::start_async`], which still yields in a loop
/// ([`yield_now`]): `I2C_MBB` reflects other masters' bus activity, which has no interrupt
/// of its own on this peripheral.
impl embedded_hal_async::i2c::I2c<SevenBitAddress> for I2c0 {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }
        let n_ops = operations.len();
        let mut prev_is_read: Option<bool> = None;
        for (index, operation) in operations.iter_mut().enumerate() {
            let is_read = matches!(operation, Operation::Read(_));
            let is_last = index + 1 == n_ops;

            match prev_is_read {
                None => self.start_async(address, is_read).await?,
                Some(prev) if prev != is_read => self.restart_async(address, is_read).await?,
                Some(_) => {}
            }

            match operation {
                Operation::Read(buf) => {
                    if prev_is_read.is_none() || prev_is_read != Some(is_read) {
                        self.receive_data_async(buf, is_last).await?;
                    } else {
                        self.continue_receive_async(buf, is_last).await?;
                    }
                }
                Operation::Write(buf) => {
                    self.transmit_data_async(buf).await?;
                }
            }
            prev_is_read = Some(is_read);
        }
        self.stop();
        Ok(())
    }
}

#[inline]
unsafe fn read_u8(reg: *mut u8) -> u8 {
    unsafe { reg.read_volatile() }
}

#[inline]
unsafe fn write_u8(reg: *mut u8, value: u8) {
    unsafe { reg.write_volatile(value) }
}

/// I2C completion interrupt handler.
///
/// Overrides the weak `i2c_isr` symbol declared in `libmc1322x`'s `isr.h`; the linked
/// `irq()` handler (`mc1322x-sys/libmc1322x/src/isr.c`) dispatches here whenever
/// `INT_NUM_I2C` is pending, i.e. whenever `I2C_MIEN` and the module's `I2C_MIF`/`I2C_MAL`
/// flags are both set.
///
/// This deliberately does *not* touch `I2C_SR` itself: [`I2c0::poll_byte_status`] (run from
/// task context once woken) owns clearing those flags, exactly as it does for the blocking
/// path, so there's only one place that decides "are we actually done". Instead this just
/// disables `I2C_MIEN` — which deasserts the interrupt line so `irq()`'s dispatch loop can
/// terminate rather than re-entering this handler forever — and wakes whichever task armed
/// the wait.
///
/// A [`Waker`] left behind by a cancelled (dropped) async I2C future is woken here like any
/// other; that's a harmless no-op on a well-behaved executor, not a use-after-free, since
/// [`Waker::wake`] on a waker whose task no longer exists is required by the `core` contract
/// to do nothing.
///
/// This handler is *unconditionally* linked into any binary that depends on `mc1322x-hal`,
/// whether or not it ever constructs an [`I2c0`]: `libmc1322x`'s `irq()` (statically linked
/// via `mc1322x-sys`'s `src.a`, itself linked into every binary regardless of which
/// peripherals it uses) holds a weak reference to the `i2c_isr` symbol, and once this crate's
/// strong definition satisfies that reference, the linker cannot discard it even under
/// `--gc-sections`. [`WAKER`] can nonetheless safely use `critical_section::with` rather than
/// a hand-rolled mask, because `mc1322x-hal` provides its own `critical_section::Impl` (see
/// `crate::critical_section_impl`) — every binary that reaches this function already has one
/// linked in, unconditionally, for the same reason.
///
/// # Caveats
///
/// Like `mc1322x-embassy`'s `tmr0_isr`, the ROM's `irq()` dispatcher must use interworking
/// (`bx`) to call this from ARM state into this crate's Thumb code; unverified on hardware.
#[unsafe(no_mangle)]
extern "C" fn i2c_isr() {
    unsafe {
        write_u8(I2C_CR, read_u8(I2C_CR) & !(I2C_MIEN as u8));
    }
    critical_section::with(|cs| {
        if let Some(waker) = WAKER.borrow(cs).borrow_mut().take() {
            waker.wake();
        }
    });
}
