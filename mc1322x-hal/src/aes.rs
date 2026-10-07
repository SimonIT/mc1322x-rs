//! Hardware AES-128 using the ASM (AES Security Module).
//!
//! [`Aes`] implements the `embedded-cal` [`AeadProvider`] trait for AES-CCM-16-64-128 (COSE
//! algorithm 10: 128-bit key, 13-byte nonce, 8-byte tag), and offers async equivalents as
//! inherent methods.

use core::task::Poll;

use embedded_cal::util::aesccm::build_b0;
use embedded_cal::{AadGenerator, AeadAlgorithm, AeadProvider, DecryptionFailed};
use mc1322x_sys::{INTBASE, INTENNUM_OFF, interrupt_nums_INT_NUM_ASM};

use crate::power::power_up_regulators;
use crate::util::WakerCell;

// Register layout from `libmc1322x`'s `lib/include/asm.h`, bring-up sequence from its
// `tests/asm.c`. The generated `mc1322x_sys::ASM` static can't be used: the C header only
// defines it as a file-local pointer, so there is no linkable symbol. A 16-byte block is
// written to the `KEY`/`DATA`/`CTR` word registers as four big-endian words, bytes 0-3 first.

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

/// `CONTROL1` resting state for the blocking operations: powered on, `NORMAL_MODE` (rather than
/// the boot mode that decrypts with an internal secret key), IRQ masked since the blocking path
/// polls `STATUS.DONE`. Also restored after each async operation, see [`Aes::poll_done_status`].
const CONTROL1_IDLE: u32 = CONTROL1_ON | CONTROL1_NORMAL_MODE | CONTROL1_MASK_IRQ;

/// Same as [`CONTROL1_IDLE`] but with the IRQ unmasked, for the async operations.
const CONTROL1_IDLE_ASYNC: u32 = CONTROL1_IDLE & !CONTROL1_MASK_IRQ;

// ITC (interrupt controller) number of the ASM interrupt. The ITC channel is enabled once in
// `Aes::new`; `CONTROL1_MASK_IRQ` gates whether an operation raises the interrupt.
const INT_NUM_ASM: u32 = interrupt_nums_INT_NUM_ASM;

/// Waker for the in-flight async operation, if any; woken by [`asm_isr`].
static WAKER: WakerCell = WakerCell::new();

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

/// Hardware AES-128 / AES-CCM engine.
///
/// The ASM block is a single shared resource and an [`Aes`] doesn't reserve it exclusively:
/// don't use two instances concurrently (e.g. one from an interrupt handler) without your own
/// locking.
pub struct Aes {
    _private: (),
}

impl Aes {
    /// Power up the ASM block, run its self-test and switch it to normal mode.
    ///
    /// Also enables the ASM interrupt in the interrupt controller, for the async methods.
    ///
    /// # Panics
    ///
    /// Panics if the hardware self-test fails.
    pub fn new() -> Self {
        power_up_regulators();
        unsafe {
            CONTROL1.write_volatile(CONTROL1_ON | CONTROL1_SELF_TEST);
            CONTROL0.write_volatile(CONTROL0_START);
            // Self-test doesn't raise DONE, so wait the fixed ~3330 cycles `tests/asm.c` uses.
            // `black_box` keeps the optimizer from removing the loop.
            for _ in 0..3330u32 {
                core::hint::black_box(0);
            }
            let pass = STATUS.read_volatile() & STATUS_TEST_PASS != 0;
            CONTROL1.write_volatile(CONTROL1_IDLE);
            assert!(pass, "ASM self-test failed");

            // Route the ASM interrupt to the core. `CONTROL1_MASK_IRQ` stays set until an async
            // operation clears it, so the blocking path is unaffected.
            core::ptr::write_volatile((INTBASE + INTENNUM_OFF) as *mut u32, INT_NUM_ASM);
        }
        Aes { _private: () }
    }

    fn load_key(&mut self, key: [u8; 16]) {
        unsafe { write_words(KEY0, block_to_words(key)) };
    }

    /// Check once whether the started operation has completed.
    ///
    /// On completion, clears `DONE` with `CONTROL0.CLEAR_IRQ` (the only way to clear it, RM
    /// Table 10-1; a new `START` doesn't) and re-masks the IRQ.
    fn poll_done_status(&mut self) -> Poll<()> {
        unsafe {
            if STATUS.read_volatile() & STATUS_DONE == 0 {
                return Poll::Pending;
            }
            CONTROL0.write_volatile(CONTROL0_CLEAR_IRQ);
            CONTROL1.write_volatile(CONTROL1.read_volatile() | CONTROL1_MASK_IRQ);
        }
        Poll::Ready(())
    }

    /// Start the currently-configured operation and block until it completes.
    fn start_and_wait(&mut self) {
        unsafe {
            CONTROL0.write_volatile(CONTROL0_START);
        }
        loop {
            if let Poll::Ready(()) = self.poll_done_status() {
                return;
            }
            core::hint::spin_loop();
        }
    }

    /// Async equivalent of [`Self::start_and_wait`].
    ///
    /// `CONTROL1` must have been set up with [`CONTROL1_IDLE_ASYNC`] (IRQ unmasked). The
    /// check-then-arm sequence runs inside one critical section so a completion can't be missed.
    ///
    /// `STATUS.DONE` is only checked on the first poll: [`asm_isr`] has to clear `DONE` itself (see
    /// there), so after arming, being polled again is the completion signal.
    async fn start_and_wait_async(&mut self) {
        unsafe {
            CONTROL0.write_volatile(CONTROL0_START);
        }
        let mut armed = false;
        let mut inhibit = None;
        core::future::poll_fn(|cx| {
            critical_section::with(|cs| {
                if armed {
                    return Poll::Ready(());
                }
                if let Poll::Ready(()) = self.poll_done_status() {
                    return Poll::Ready(());
                }
                inhibit.get_or_insert_with(crate::sleep::SleepInhibitGuard::new);
                WAKER.set(cs, cx.waker());
                armed = true;
                Poll::Pending
            })
        })
        .await
    }

    /// Encrypt a single block with raw AES-128 (ECB), returning `AES(key, block)`.
    pub fn ecb_encrypt_block(&mut self, key: [u8; 16], block: [u8; 16]) -> [u8; 16] {
        // CTR mode with all-zero data yields `0 XOR AES(key, counter)`.
        self.load_key(key);
        self.ctr_block(block, [0u8; 16])
    }

    /// Async version of [`Self::ecb_encrypt_block`].
    ///
    /// Waits for the ASM interrupt and inhibits a sleep-aware executor from sleeping meanwhile (see
    /// [`crate::sleep::SleepInhibitGuard`]).
    pub async fn ecb_encrypt_block_async(&mut self, key: [u8; 16], block: [u8; 16]) -> [u8; 16] {
        self.load_key(key);
        self.ctr_block_async(block, [0u8; 16]).await
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

    /// Async equivalent of [`Self::ctr_block`].
    async fn ctr_block_async(&mut self, counter: [u8; 16], data: [u8; 16]) -> [u8; 16] {
        unsafe {
            CONTROL1.write_volatile(CONTROL1_IDLE_ASYNC | CONTROL1_CTR);
            write_words(DATA0, block_to_words(data));
            write_words(CTR0, block_to_words(counter));
        }
        self.start_and_wait_async().await;
        words_to_block(unsafe { read_words(CTR0_RESULT) })
    }

    /// Async equivalent of [`Self::cbc_mac_block`].
    async fn cbc_mac_block_async(&mut self, block: [u8; 16], first: bool) {
        unsafe {
            CONTROL1.write_volatile(CONTROL1_IDLE_ASYNC | CONTROL1_CBC);
            write_words(DATA0, block_to_words(block));
            if first {
                CONTROL0.write_volatile(CONTROL0_CLEAR);
            }
        }
        self.start_and_wait_async().await;
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

/// Async equivalent of [`feed_padded`].
async fn feed_padded_async(
    asm: &mut Aes,
    buf: &mut [u8; 16],
    filled: &mut usize,
    mut bytes: &[u8],
) {
    while !bytes.is_empty() {
        let take = (16 - *filled).min(bytes.len());
        buf[*filled..*filled + take].copy_from_slice(&bytes[..take]);
        *filled += take;
        bytes = &bytes[take..];
        if *filled == 16 {
            asm.cbc_mac_block_async(*buf, false).await;
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

    /// Async equivalent of [`Self::ccm_mac`].
    async fn ccm_mac_async(
        &mut self,
        nonce: &[u8; 13],
        aad: &impl AadGenerator,
        message: &[u8],
    ) -> [u8; 16] {
        let a_len: usize = aad.items().map(<[u8]>::len).sum();
        debug_assert!(a_len < 0xff00, "AAD too long for the 2-byte length prefix");

        let b0 = build_b0(nonce, message.len(), a_len, 8);
        self.cbc_mac_block_async(b0, true).await;

        if a_len > 0 {
            let mut buf = [0u8; 16];
            let mut filled = 0;
            feed_padded_async(self, &mut buf, &mut filled, &(a_len as u16).to_be_bytes()).await;
            for chunk in aad.items() {
                feed_padded_async(self, &mut buf, &mut filled, chunk).await;
            }
            if filled > 0 {
                self.cbc_mac_block_async(buf, false).await;
            }
        }

        for chunk in message.chunks(16) {
            let mut block = [0u8; 16];
            block[..chunk.len()].copy_from_slice(chunk);
            self.cbc_mac_block_async(block, false).await;
        }

        self.cbc_mac_result()
    }

    /// Async equivalent of [`Self::ccm_ctr_crypt`].
    async fn ccm_ctr_crypt_async(&mut self, nonce: &[u8; 13], message: &mut [u8]) {
        for (i, chunk) in message.chunks_mut(16).enumerate() {
            let a = build_a(nonce, (i + 1) as u16);
            let mut block = [0u8; 16];
            block[..chunk.len()].copy_from_slice(chunk);
            let out = self.ctr_block_async(a, block).await;
            chunk.copy_from_slice(&out[..chunk.len()]);
        }
    }

    /// Async version of [`AeadProvider::encrypt_in_place`]: encrypt `message` in place and return
    /// the 8-byte tag.
    ///
    /// Waits for the ASM interrupt and inhibits a sleep-aware executor from sleeping meanwhile (see
    /// [`crate::sleep::SleepInhibitGuard`]).
    ///
    /// # Panics
    ///
    /// Panics if `nonce` is not 13 bytes long.
    pub async fn encrypt_in_place_async(
        &mut self,
        key: &[u8; 16],
        nonce: &[u8],
        message: &mut [u8],
        aad: impl AadGenerator,
    ) -> [u8; 8] {
        let nonce: [u8; 13] = nonce
            .try_into()
            .expect("AES-CCM-16-64-128 uses a 13-byte nonce");

        self.load_key(*key);
        let mac = self.ccm_mac_async(&nonce, &aad, message).await;
        let s0 = self.ctr_block_async(build_a(&nonce, 0), [0u8; 16]).await;
        self.ccm_ctr_crypt_async(&nonce, message).await;

        let mut tag = [0u8; 8];
        for i in 0..8 {
            tag[i] = mac[i] ^ s0[i];
        }
        tag
    }

    /// Async version of [`AeadProvider::decrypt_in_place`]: decrypt `message` in place and verify
    /// `tag`.
    ///
    /// Waits for the ASM interrupt and inhibits a sleep-aware executor from sleeping meanwhile (see
    /// [`crate::sleep::SleepInhibitGuard`]).
    ///
    /// # Errors
    ///
    /// Returns [`DecryptionFailed`] and zeroes `message` if the tag doesn't match. The
    /// comparison is not constant-time.
    ///
    /// # Panics
    ///
    /// Panics if `nonce` is not 13 bytes or `tag` is not 8 bytes long.
    pub async fn decrypt_in_place_async(
        &mut self,
        key: &[u8; 16],
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
        self.ccm_ctr_crypt_async(&nonce, message).await;
        let mac = self.ccm_mac_async(&nonce, &aad, message).await;
        let s0 = self.ctr_block_async(build_a(&nonce, 0), [0u8; 16]).await;

        let mut expected = [0u8; 8];
        for i in 0..8 {
            expected[i] = mac[i] ^ s0[i];
        }

        // Not constant-time.
        if expected != *tag {
            message.fill(0);
            return Err(DecryptionFailed);
        }
        Ok(())
    }
}

/// ASM interrupt handler, overriding the weak `asm_isr` symbol from `libmc1322x`'s `isr.h`.
///
/// Clears the completion itself with `CONTROL0.CLEAR_IRQ`, like `tests/asm.c`'s handler:
/// `CONTROL1_MASK_IRQ` doesn't retract an already-latched interrupt, so only masking it would
/// leave `irq()` re-entering this handler forever. See [`Aes::start_and_wait_async`] for how
/// the task detects completion afterwards.
///
/// Always linked in, even in binaries that never use [`Aes`] (see [`crate::i2c`]'s `i2c_isr`).
#[unsafe(no_mangle)]
extern "C" fn asm_isr() {
    unsafe {
        CONTROL0.write_volatile(CONTROL0_CLEAR_IRQ);
    }
    WAKER.wake();
}

/// Blocking AES-CCM-16-64-128.
///
/// The methods panic if the key is not 16 bytes, the nonce not 13 bytes or the tag not 8 bytes
/// long. Tag verification is not constant-time.
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

        // Not constant-time: may leak tag information through timing.
        if expected != *tag {
            message.fill(0);
            return Err(DecryptionFailed);
        }
        Ok(())
    }
}
