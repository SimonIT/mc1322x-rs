//! Pseudo-random numbers from the MACA (radio) block's `MACA_RANDOM` LFSR.
//!
//! Not suitable for cryptography: [`Rng`] doesn't implement `CryptoRng`.

use core::convert::Infallible;
use core::sync::atomic::Ordering;

use mc1322x_sys::{MACA_BASE, maca_init};
use portable_atomic::AtomicBool;
use rand_core::TryRng;

use crate::power::{power_up_regulators, trim_xtal};

const MACA_RANDOM: *mut u32 = (MACA_BASE as usize + 0x08) as *mut u32;

static MACA_READY: AtomicBool = AtomicBool::new(false);

/// Bring up the MACA block, if this function hasn't already done so.
///
/// On the first call this trims the reference crystal for the board, powers up the 1.5 V/1.8 V
/// regulators and runs `mc1322x_sys::maca_init()` (a full MACA reset and PHY bring-up); later
/// calls do nothing. `mc1322x-radio`'s `Mc1322xRadio::init` goes through this function too, so
/// it's safe to use both [`Rng`] and the radio. Code that calls `maca_init`/`reset_maca`
/// directly bypasses this guard.
pub fn ensure_maca_ready() {
    let already_ready = MACA_READY.swap(true, Ordering::AcqRel);
    if !already_ready {
        trim_xtal();
        power_up_regulators();
        unsafe { maca_init() };
    }
}

/// Hardware RNG backed by the MACA block's `MACA_RANDOM` register (RM §9.7.2).
///
/// `MACA_RANDOM` is a free-running 32-bit LFSR: every read returns its current state and every
/// write reseeds it (see [`Rng::seed`]). Being deterministic rather than a physical entropy
/// source, it only implements [`TryRng`] (and, via the blanket impl, [`rand_core::Rng`]), not
/// `CryptoRng`.
///
/// The MACA must be running for the LFSR to advance: call [`ensure_maca_ready`] first, or
/// initialize the radio through `mc1322x-radio`.
pub struct Rng;

impl Rng {
    /// Create a new `Rng`. Doesn't touch the hardware.
    pub const fn new() -> Self {
        Rng
    }

    /// Reseed the LFSR.
    ///
    /// A `seed(x)` immediately followed by exactly one read returns a value that depends only
    /// on `x`. The LFSR keeps running, though, so once anything else happens between the write
    /// and the read, the result also depends on timing. [`Self::seed_and_read`] gives the
    /// deterministic pattern without relying on what the compiler or the call site puts in
    /// between.
    pub fn seed(&mut self, seed: u32) {
        unsafe { MACA_RANDOM.write_volatile(seed) }
    }

    /// Reseed the LFSR and return the value read immediately afterwards.
    ///
    /// The result depends only on `seed`, regardless of earlier seeds or reads.
    pub fn seed_and_read(&mut self, seed: u32) -> u32 {
        self.seed(seed);
        self.read()
    }

    fn read(&self) -> u32 {
        unsafe { MACA_RANDOM.read_volatile() }
    }
}

impl Default for Rng {
    fn default() -> Self {
        Self::new()
    }
}

impl TryRng for Rng {
    type Error = Infallible;

    fn try_next_u32(&mut self) -> Result<u32, Self::Error> {
        Ok(self.read())
    }

    fn try_next_u64(&mut self) -> Result<u64, Self::Error> {
        let hi = self.read() as u64;
        let lo = self.read() as u64;
        Ok((hi << 32) | lo)
    }

    fn try_fill_bytes(&mut self, dst: &mut [u8]) -> Result<(), Self::Error> {
        for chunk in dst.chunks_mut(4) {
            let bytes = self.read().to_ne_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }
        Ok(())
    }
}
