use core::cell::Cell;
use core::convert::Infallible;
use critical_section::Mutex;
use mc1322x_sys::{MACA_BASE, maca_init};
use rand_core::TryRng;

const MACA_RANDOM: *mut u32 = (MACA_BASE as usize + 0x08) as *mut u32;

// The ARM7TDMI core in the MC1322x has no atomic instructions, so the
// one-shot init guard below is a plain `Cell` behind a `critical_section`
// lock rather than an `AtomicBool`.
static MACA_READY: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

/// Bring up the MACA block for [`Rng`] use, if this function hasn't already
/// done so.
///
/// Calls `mc1322x_sys::maca_init()` (a full MACA reset and PHY bring-up) on
/// the first call and is a no-op afterwards. `mc1322x-radio`'s
/// `Mc1322xRadio::init` calls through this same function, so it is safe to
/// call this whether or not the radio is also in use: whichever of the two
/// runs first performs the real init, and the other just confirms MACA is
/// already up. This only covers coordination through this function, though —
/// code that calls `maca_init` / `reset_maca` directly, bypassing this guard,
/// can still race with it.
pub fn ensure_maca_ready() {
    let already_ready = critical_section::with(|cs| {
        let ready = MACA_READY.borrow(cs);
        let was_ready = ready.get();
        ready.set(true);
        was_ready
    });
    if !already_ready {
        unsafe { maca_init() };
    }
}

/// Hardware RNG backed by the MACA (radio) block's `MACA_RANDOM` register.
///
/// Per the reference manual this is a 32-bit LFSR clocked at the bus rate,
/// running an internal 32-bit primitive polynomial: every read of
/// `MACA_RANDOM` fetches the current 32-bit state and every write reseeds it
/// (see [`Rng::seed`]). Being a deterministic LFSR, not a physical entropy
/// source, it is not fit for cryptographic use — this type only implements
/// the infallible [`TryRng`] (and, via its blanket impl, [`rand_core::Rng`]),
/// not `CryptoRng`.
///
/// The MACA block must already be clocked and running for `MACA_RANDOM` to
/// free-run: bring it up first via [`ensure_maca_ready`] if nothing else
/// initializes MACA, or via `mc1322x-radio` if the radio is in use.
/// Constructing this type does not touch any hardware state itself.
pub struct Rng;

impl Rng {
    /// Wrap the `MACA_RANDOM` register. Does not touch any hardware state.
    pub const fn new() -> Self {
        Rng
    }

    /// Reseed the LFSR.
    ///
    /// Two `Rng`s seeded with the same value produce the same stream: seed
    /// from a value that actually varies (e.g. a factory-programmed ID or a
    /// timer) if you need runs to differ across resets.
    pub fn seed(&mut self, seed: u32) {
        unsafe { MACA_RANDOM.write_volatile(seed) }
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
