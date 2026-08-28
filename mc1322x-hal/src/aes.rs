//! Driver for the MC1322x's ASM (AES Security Module) hardware AES-128 engine,
//! wired up as an `embedded-cal` [`AeadProvider`] implementing AES-CCM-16-64-128
//! (COSE algorithm 10: 128-bit key, 13-byte nonce, 8-byte tag).
//!
//! Register layout and field offsets are taken from
//! `mc1322x-sys/libmc1322x/lib/include/asm.h` and the bring-up sequence from
//! `mc1322x-sys/libmc1322x/tests/asm.c` (there is no compiled `asm.c` driver in
//! libmc1322x to bind against, and `mc1322x_sys::ASM`/`ASM_struct` are unusable:
//! `ASM` is generated as an `extern "C" static` but the C header only ever
//! declares it as a file-local `static` pointer literal, so no linkable
//! `ASM` symbol actually exists in `libmc1322x.a` or `src.a`). This talks to
//! the peripheral directly via volatile MMIO instead, the same way
//! [`crate::rng`] does for `MACA_RANDOM`.
//!
//! The mapping between the 16-byte AES block representation used here and the `KEY`/`DATA`/
//! `CTR` word registers (word 0 = least-significant 32 bits, each word big-endian) is verified
//! against the FIPS-197 Appendix B / C.1 known-answer test in `examples/aes-selftest`.

use embedded_cal::{AadGenerator, AeadAlgorithm, AeadProvider, DecryptionFailed, build_b0};

use crate::power::power_up_regulators;

const BASE: usize = 0x8000_8000;

const KEY0: *mut u32 = BASE as *mut u32;
const DATA0: *mut u32 = (BASE + 0x10) as *mut u32;
const CTR0: *mut u32 = (BASE + 0x20) as *mut u32;
const CTR0_RESULT: *mut u32 = (BASE + 0x30) as *mut u32;
const CBC0_RESULT: *mut u32 = (BASE + 0x40) as *mut u32;
const CONTROL0: *mut u32 = (BASE + 0x50) as *mut u32;
const CONTROL1: *mut u32 = (BASE + 0x54) as *mut u32;
const STATUS: *mut u32 = (BASE + 0x58) as *mut u32;

const CONTROL0_START: u32 = 1 << 24;
const CONTROL0_CLEAR: u32 = 1 << 25;
const CONTROL0_CLEAR_IRQ: u32 = 1 << 31;
const CONTROL1_ON: u32 = 1 << 0;
const CONTROL1_NORMAL_MODE: u32 = 1 << 1;
const CONTROL1_CBC: u32 = 1 << 24;
const CONTROL1_CTR: u32 = 1 << 25;
const CONTROL1_SELF_TEST: u32 = 1 << 26;
const CONTROL1_MASK_IRQ: u32 = 1 << 31;
const STATUS_DONE: u32 = 1 << 24;
const STATUS_TEST_PASS: u32 = 1 << 25;

/// `CONTROL1` bits that stay set across every operation once the module is
/// brought up: powered on, running in `NORMAL_MODE` (as opposed to the boot
/// mode that decrypts from an internal secret key), interrupt masked because
/// this driver polls `STATUS.DONE` instead of servicing the IRQ.
const CONTROL1_IDLE: u32 = CONTROL1_ON | CONTROL1_NORMAL_MODE | CONTROL1_MASK_IRQ;

unsafe fn write_words(base: *mut u32, words: [u32; 4]) {
    for (i, w) in words.into_iter().enumerate() {
        unsafe { base.add(i).write_volatile(w) };
    }
}

unsafe fn read_words(base: *mut u32) -> [u32; 4] {
    core::array::from_fn(|i| unsafe { base.add(i).read_volatile() })
}

fn block_to_words(block: [u8; 16]) -> [u32; 4] {
    core::array::from_fn(|i| u32::from_be_bytes(block[4 * i..4 * i + 4].try_into().unwrap()))
}

fn words_to_block(words: [u32; 4]) -> [u8; 16] {
    let mut out = [0u8; 16];
    for (i, w) in words.into_iter().enumerate() {
        out[4 * i..4 * i + 4].copy_from_slice(&w.to_be_bytes());
    }
    out
}

/// Hardware AES-128 / AES-CCM engine backed by the MC1322x's ASM peripheral.
///
/// The engine is a single shared hardware block: constructing an [`Aes`]
/// brings it up (self-test, then switch to `NORMAL_MODE`) if this hasn't
/// already happened, but does not reserve exclusive access to it — don't run
/// two [`Aes`] instances concurrently from different contexts (e.g. one in an
/// ISR) without your own locking, same as any other `&mut`-based MMIO driver.
pub struct Aes {
    _private: (),
}

impl Aes {
    /// Bring up the ASM block (self-test + `NORMAL_MODE`) and wrap it.
    ///
    /// # Panics
    ///
    /// Panics if the hardware self-test does not report `TEST_PASS`.
    pub fn new() -> Self {
        power_up_regulators();
        unsafe {
            CONTROL1.write_volatile(CONTROL1_ON | CONTROL1_SELF_TEST);
            CONTROL0.write_volatile(CONTROL0_START);
            // `tests/asm.c` busy-waits ~3330 periph. clocks here rather than
            // polling STATUS.DONE (self-test doesn't appear to raise DONE);
            // this loop count is carried over from there rather than derived.
            // `black_box` (not `spin_loop`) is required: `spin_loop` alone has no
            // observable side effect on this target, so the loop was being eliminated
            // entirely by the optimizer, reading STATUS immediately instead of after a
            // real delay.
            for _ in 0..3330u32 {
                core::hint::black_box(0);
            }
            let pass = STATUS.read_volatile() & STATUS_TEST_PASS != 0;
            CONTROL1.write_volatile(CONTROL1_IDLE);
            assert!(pass, "ASM self-test failed");
        }
        Aes { _private: () }
    }

    fn load_key(&mut self, key: [u8; 16]) {
        unsafe { write_words(KEY0, block_to_words(key)) };
    }

    /// Start the currently-configured operation and block until it completes.
    ///
    /// `DONE` is cleared here (`CONTROL0.CLEAR_IRQ`) right after being observed, rather than
    /// left set: per RM Table 10-1, writing `CLEAR_IRQ` is the *only* documented way to clear
    /// it, and a fresh `START` does not do so implicitly. Without this, every call after the
    /// first would see `DONE` still set from the previous operation and return immediately,
    /// racing ahead of the 13/26-clock operation it just started instead of actually waiting
    /// for it.
    fn start_and_wait(&mut self) {
        unsafe {
            CONTROL0.write_volatile(CONTROL0_START);
            while STATUS.read_volatile() & STATUS_DONE == 0 {
                core::hint::spin_loop();
            }
            CONTROL0.write_volatile(CONTROL0_CLEAR_IRQ);
        }
    }

    /// Raw AES-128 ECB single-block encryption, `AES(key, block)`.
    ///
    /// Implemented as CTR mode with an all-zero data block: the module
    /// computes `DATA XOR AES(key, CTR)`, so `DATA = 0` yields the raw
    /// AES-encrypted counter block. Exposed mainly so this can be checked
    /// against a known test vector.
    pub fn ecb_encrypt_block(&mut self, key: [u8; 16], block: [u8; 16]) -> [u8; 16] {
        self.load_key(key);
        self.ctr_block(block, [0u8; 16])
    }

    /// One AES-CTR block: returns `data XOR AES(key, counter)`, i.e.
    /// ciphertext when `data` is plaintext and vice versa (CTR mode is its
    /// own inverse). Assumes the key has already been loaded.
    fn ctr_block(&mut self, counter: [u8; 16], data: [u8; 16]) -> [u8; 16] {
        unsafe {
            CONTROL1.write_volatile(CONTROL1_IDLE | CONTROL1_CTR);
            write_words(DATA0, block_to_words(data));
            write_words(CTR0, block_to_words(counter));
        }
        self.start_and_wait();
        words_to_block(unsafe { read_words(CTR0_RESULT) })
    }

    /// Feed one 16-byte block through the CBC-MAC accumulator. `first` must
    /// be `true` exactly once per MAC computation, to reset the accumulator
    /// (`CONTROL0.CLEAR`) before the first block.
    fn cbc_mac_block(&mut self, block: [u8; 16], first: bool) {
        unsafe {
            CONTROL1.write_volatile(CONTROL1_IDLE | CONTROL1_CBC);
            write_words(DATA0, block_to_words(block));
            if first {
                CONTROL0.write_volatile(CONTROL0_CLEAR);
            }
        }
        self.start_and_wait();
    }

    fn cbc_mac_result(&self) -> [u8; 16] {
        words_to_block(unsafe { read_words(CBC0_RESULT) })
    }
}

impl Default for Aes {
    fn default() -> Self {
        Self::new()
    }
}

/// Feed `bytes` into an in-progress CBC-MAC computation, buffering a partial
/// trailing block across calls. Used to stream the length-prefixed AAD
/// (`build_b0`'s "B1..Bu" per RFC 3610 §2.2) without needing it to fit in
/// one buffer.
fn feed_padded(asm: &mut Aes, buf: &mut [u8; 16], filled: &mut usize, mut bytes: &[u8]) {
    while !bytes.is_empty() {
        let take = (16 - *filled).min(bytes.len());
        buf[*filled..*filled + take].copy_from_slice(&bytes[..take]);
        *filled += take;
        bytes = &bytes[take..];
        if *filled == 16 {
            asm.cbc_mac_block(*buf, false);
            *buf = [0u8; 16];
            *filled = 0;
        }
    }
}

/// `A_i` counter block per RFC 3610 §2.3, specialized to `L = 2` (13-byte
/// nonce), which is what AES-CCM-16-64-128/256 (COSE algorithms 10/11) use.
fn build_a(nonce: &[u8; 13], counter: u16) -> [u8; 16] {
    let mut a = [0u8; 16];
    a[0] = 0x01; // Flags: L - 1, no Adata/M bits set for A_i (only B0 sets those)
    a[1..14].copy_from_slice(nonce);
    a[14..16].copy_from_slice(&counter.to_be_bytes());
    a
}

/// AES-CCM-16-64-128 — COSE algorithm 10: AES-128, 13-byte nonce, 8-byte tag.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AesCcm16_64_128;

impl AeadAlgorithm for AesCcm16_64_128 {
    fn key_length(&self) -> usize {
        16
    }

    fn tag_length(&self) -> usize {
        8
    }

    fn nonce_length(&self) -> usize {
        13
    }

    fn from_cose_number(number: impl Into<i128>) -> Option<Self> {
        (number.into() == 10).then_some(AesCcm16_64_128)
    }
}

impl Aes {
    /// CBC-MAC over `B0 || (length-prefixed AAD, zero-padded) || (message,
    /// zero-padded)`, per RFC 3610 §2.2. Returns the raw 128-bit MAC value —
    /// callers still need to mask it with `S_0` and truncate to the tag
    /// length.
    fn ccm_mac(&mut self, nonce: &[u8; 13], aad: &impl AadGenerator, message: &[u8]) -> [u8; 16] {
        let a_len: usize = aad.items().map(<[u8]>::len).sum();
        debug_assert!(a_len < 0xff00, "AAD too long for the 2-byte length prefix");

        let b0 = build_b0(nonce, message.len(), a_len, 8);
        self.cbc_mac_block(b0, true);

        if a_len > 0 {
            let mut buf = [0u8; 16];
            let mut filled = 0;
            feed_padded(self, &mut buf, &mut filled, &(a_len as u16).to_be_bytes());
            for chunk in aad.items() {
                feed_padded(self, &mut buf, &mut filled, chunk);
            }
            if filled > 0 {
                self.cbc_mac_block(buf, false);
            }
        }

        for chunk in message.chunks(16) {
            let mut block = [0u8; 16];
            block[..chunk.len()].copy_from_slice(chunk);
            self.cbc_mac_block(block, false);
        }

        self.cbc_mac_result()
    }

    /// CTR-encrypt (or, symmetrically, decrypt) `message` in place using
    /// counter blocks `A_1, A_2, ...`.
    fn ccm_ctr_crypt(&mut self, nonce: &[u8; 13], message: &mut [u8]) {
        for (i, chunk) in message.chunks_mut(16).enumerate() {
            let a = build_a(nonce, (i + 1) as u16);
            let mut block = [0u8; 16];
            block[..chunk.len()].copy_from_slice(chunk);
            let out = self.ctr_block(a, block);
            chunk.copy_from_slice(&out[..chunk.len()]);
        }
    }
}

impl AeadProvider for Aes {
    type Algorithm = AesCcm16_64_128;
    type Key = [u8; 16];
    type Tag = [u8; 8];

    fn load_from_keydata(&mut self, _alg: Self::Algorithm, key: &[u8]) -> Self::Key {
        key.try_into()
            .expect("AES-CCM-16-64-128 uses a 16-byte key")
    }

    fn encrypt_in_place(
        &mut self,
        key: &Self::Key,
        nonce: &[u8],
        message: &mut [u8],
        aad: impl AadGenerator,
    ) -> Self::Tag {
        let nonce: [u8; 13] = nonce
            .try_into()
            .expect("AES-CCM-16-64-128 uses a 13-byte nonce");

        self.load_key(*key);
        let mac = self.ccm_mac(&nonce, &aad, message);
        let s0 = self.ctr_block(build_a(&nonce, 0), [0u8; 16]);
        self.ccm_ctr_crypt(&nonce, message);

        let mut tag = [0u8; 8];
        for i in 0..8 {
            tag[i] = mac[i] ^ s0[i];
        }
        tag
    }

    fn decrypt_in_place(
        &mut self,
        key: &Self::Key,
        nonce: &[u8],
        message: &mut [u8],
        tag: &[u8],
        aad: impl AadGenerator,
    ) -> Result<(), DecryptionFailed> {
        let nonce: [u8; 13] = nonce
            .try_into()
            .expect("AES-CCM-16-64-128 uses a 13-byte nonce");
        assert_eq!(tag.len(), 8, "AES-CCM-16-64-128 uses an 8-byte tag");

        self.load_key(*key);
        self.ccm_ctr_crypt(&nonce, message);
        let mac = self.ccm_mac(&nonce, &aad, message);
        let s0 = self.ctr_block(build_a(&nonce, 0), [0u8; 16]);

        let mut expected = [0u8; 8];
        for i in 0..8 {
            expected[i] = mac[i] ^ s0[i];
        }

        // Not constant-time: fine for a sketch, worth revisiting if this
        // ever verifies tags on attacker-controlled input over a timing
        // side channel that matters (e.g. a network-facing MAC check).
        if expected != *tag {
            message.fill(0);
            return Err(DecryptionFailed);
        }
        Ok(())
    }
}
