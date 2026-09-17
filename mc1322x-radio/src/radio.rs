//! IEEE 802.15.4 radio driver for the MC1322x MACA coprocessor.
//!
//! The driver drives the packet pool API of `libmc1322x` through a small set
//! of core async primitives (see the inherent methods on [`Mc1322xRadio`]
//! below). These are frame-crate-agnostic: they move raw PHR+PSDU buffers
//! (see [`crate::layout`]) in and out of the hardware and know nothing about
//! `dot15d4` or `ieee802154`.
//!
//! With the `dot15d4` feature, [`Mc1322xRadio`] additionally
//! implements [`dot15d4::phy::radio::Radio`] as a thin wrapper around those
//! primitives, which is the hardware glue that makes the dot15d4 CSMA layer
//! (and its example application layer) run on a real 2.4 GHz radio. The
//! `ieee802154` feature instead adds [`Mc1322xRadio::send_frame`] and
//! [`Mc1322xRadio::receive_frame`], which fold an
//! [`ieee802154::mac::Frame`]'s encoding/decoding directly into the transmit
//! and receive path (no `Radio` trait, no CSMA/ACK handling — just a typed
//! frame in, a typed frame out).
//!
//! # Architecture
//!
//! The MACA coprocessor runs a receive sequence at all times (re-armed by
//! `libmc1322x` from its interrupt handler). Received frames are pushed onto
//! an internal packet queue by the C library; the driver's RX callback
//! (`maca_rx_callback`) simply bumps a counter and wakes the executor. The
//! [`Mc1322xRadio::receive`] future then pops one packet and copies the PSDU
//! into the caller-provided 128-byte buffer using the layout documented in
//! [`crate::layout`].
//!
//! Transmissions go through the same packet pool:
//! [`Mc1322xRadio::prepare_transmit`] claims a free packet, copies the PSDU
//! out of the caller's buffer (reading the length from the PHR byte), and
//! queues it. The MACA sequencer performs the transmission and raises the
//! action-complete interrupt; the driver's TX callback (`maca_tx_callback`)
//! records the status code and wakes the executor, after which
//! [`Mc1322xRadio::transmit`] reports success.
//!
//! # CCA
//!
//! `libmc1322x` hardcodes the MACA control word to `maca_ctrl_mode_no_cca`
//! (see `post_tx` in `lib/maca.c`), so the radio always transmits immediately,
//! without a hardware channel-clear assessment. Contention is instead handled
//! by the software CSMA-CA backoff of the dot15d4 layer together with ACK
//! detection: a failed access or a missing ACK makes
//! [`Mc1322xRadio::transmit`] return `false` and the MAC layer retries.
//!
//! # Startup
//!
//! [`Mc1322xRadio::init`] initializes the MACA coprocessor and must be called
//! before the radio is used; it goes through
//! [`mc1322x_hal::rng::ensure_maca_ready`], so it is idempotent and safe to
//! call alongside HAL-side MACA users. The IEEE 802.15.4 extended address
//! used for address filtering must be supplied there.

use core::{
    cell::Cell,
    future::poll_fn,
    pin::Pin,
    ptr::NonNull,
    sync::atomic::Ordering,
    task::{Context, Poll, Waker},
};

use critical_section::Mutex;
use mc1322x_sys::packet;
use portable_atomic::AtomicU64;

use crate::layout::MAX_PHR;

/// The radio driver. A zero-sized type; all state lives in statics.
pub struct Mc1322xRadio;

/// Wraps [`Mc1322xRadio::receive`]'s returned future to make cancellation-by-drop safe on its
/// own, instead of relying on the caller to observe the documented "call
/// `cancel_current_operation` and poll to completion before dropping" contract.
///
/// That contract turns out to be impossible for callers to actually satisfy in general: a
/// `select()` between this future and a timeout (exactly how `dot15d4`'s CSMA layer races the
/// ACK-wait receive against its timeout) always drops the *losing* future as part of tearing
/// down the `select()` itself, before any of the winning branch's own code (including a
/// drop-guard that calls `cancel_current_operation`) gets a chance to run - Rust drops the
/// actively-polled inner future of a suspended `async fn` before any of its enclosing locals.
/// So by the time `cancel_current_operation` (called from `dot15d4`'s own `OnDrop` guard) runs,
/// this future is already gone, and `rx_buffer`/`cancelled` would otherwise be left set from
/// the abandoned operation until whatever `prepare_receive` call happens to come next. This is
/// a defensive fix for that real gap - found while bisecting a *separate*, since-fixed hardware
/// crash (an ARMv4T-incompatible interworking veneer for 64-bit division; see
/// `vendor/libgcc-thumbv4t/README.md`) that turned out not to be caused by this gap after all,
/// but the gap itself is real and worth closing regardless.
struct ReceiveFuture<F> {
    inner: F,
}

impl<F: Future<Output = bool> + Unpin> Future for ReceiveFuture<F> {
    type Output = bool;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<bool> {
        let this = self.get_mut();
        Pin::new(&mut this.inner).poll(cx)
    }
}

impl<F> Drop for ReceiveFuture<F> {
    fn drop(&mut self) {
        // Idempotent: a future that already resolved normally already cleared all of this
        // itself, so this is a no-op in that case. Only matters when dropped mid-flight.
        critical_section::with(|cs| {
            let state = STATE.borrow(cs);
            state.rx_buffer.set(None);
            state.waker.set(None);
            state.cancelled.set(false);
        });
    }
}

/// Wrapper around the receive buffer pointer to mark it `Send + Sync`.
///
/// The pointer is only ever dereferenced from the single executor while the
/// CSMA radio guard is held, so this is sound.
struct RxBuffer(NonNull<[u8; 128]>);

impl Copy for RxBuffer {}

impl Clone for RxBuffer {
    fn clone(&self) -> Self {
        *self
    }
}

// Safety: see the doc comment on `RxBuffer`.
unsafe impl Send for RxBuffer {}
// Safety: see the doc comment on `RxBuffer`.
unsafe impl Sync for RxBuffer {}

struct State {
    /// Buffer handed to us by `prepare_receive`. `None` while idle.
    rx_buffer: Cell<Option<RxBuffer>>,
    /// Number of received frames waiting in the C packet queue.
    rx_pending: Cell<u32>,
    /// Status code of the last completed transmission, from the MACA.
    tx_status: Cell<Option<u32>>,
    /// Cached MACA channel index (0..=15, i.e. IEEE channel 11..=26).
    tx_channel: Cell<Option<u8>>,
    /// Set when an operation was cancelled; the pending operation must abort.
    cancelled: Cell<bool>,
    /// Waker for whichever radio future is currently pending.
    waker: Cell<Option<Waker>>,
    /// Set while a `radio-hal` `start_transmit` is in flight; backs
    /// [`radio_hal`]'s `Busy` implementation.
    #[cfg(feature = "radio-hal")]
    tx_in_flight: Cell<bool>,
}

impl State {
    const fn new() -> Self {
        Self {
            rx_buffer: Cell::new(None),
            rx_pending: Cell::new(0),
            tx_status: Cell::new(None),
            tx_channel: Cell::new(None),
            cancelled: Cell::new(false),
            waker: Cell::new(None),
            #[cfg(feature = "radio-hal")]
            tx_in_flight: Cell::new(false),
        }
    }
}

static STATE: Mutex<State> = Mutex::new(State::new());

/// The IEEE 802.15.4 extended (64-bit) address configured via [`Mc1322xRadio::init`].
///
/// This target has no native atomics at all, so `portable-atomic`'s `critical-section` fallback
/// backs this (see `mc1322x-hal::rng`'s `MACA_READY` for the same pattern) - functionally the
/// same plain-Cell-behind-a-lock a bundled address field would need, just without a
/// `critical_section::with` closure at each access site.
static HW_ADDRESS: AtomicU64 = AtomicU64::new(0);

impl Mc1322xRadio {
    /// Number of entries in the MACA `PSMVAL`/`PAVAL`/`AIMVAL` power tables
    /// (see `PSMVAL` in `libmc1322x/lib/maca.c`), i.e. the valid range for
    /// [`Mc1322xRadio::set_output_power`].
    const POWER_LEVELS: u8 = 19;

    /// Initialize the MACA coprocessor and return the radio driver.
    ///
    /// `address` is the IEEE 802.15.4 extended address used by the layers
    /// above for address filtering (see [`Radio::ieee802154_address`]).
    ///
    /// Safe to call more than once, and safe to call alongside
    /// [`mc1322x_hal::rng::ensure_maca_ready`]: both funnel through the same
    /// guard, so whichever runs first performs the actual `maca_init` and the
    /// other is a no-op.
    pub fn init(address: [u8; 8]) -> Self {
        HW_ADDRESS.store(u64::from_ne_bytes(address), Ordering::Relaxed);
        mc1322x_hal::rng::ensure_maca_ready();
        Self
    }

    /// Set the current MACA RF channel (IEEE 802.15.4 channel number, 11..=26).
    ///
    /// The MACA has a single channel register shared between transmit and receive, so
    /// whichever side last programmed it determines both directions - [`Mc1322xRadio::
    /// prepare_transmit`] (and the `dot15d4`/`radio-hal` glue built on it) already does this as
    /// part of every transmit. This method exists for callers who only ever *receive* (e.g. the
    /// `ieee802154` feature's [`Mc1322xRadio::receive_frame`] used on its own): without it,
    /// there is no way to move off whatever channel [`Mc1322xRadio::init`] leaves the hardware
    /// on (`libmc1322x`'s `maca_init` defaults to index 0, i.e. channel 11), regardless of what
    /// channel the sender actually transmits on.
    ///
    /// # Panics
    ///
    /// Panics if `channel` is outside `11..=26`.
    pub fn set_channel(&mut self, channel: u8) {
        assert!(
            (11..=26).contains(&channel),
            "channel {} out of range 11..=26",
            channel
        );
        critical_section::with(|cs| set_channel_index(STATE.borrow(cs), channel - 11));
    }

    /// Set the RF output power. Indexes the MACA `PSMVAL`/`PAVAL`/`AIMVAL`
    /// tables (0 = lowest, `POWER_LEVELS - 1` = highest).
    ///
    /// # Panics
    ///
    /// Panics if `power >= POWER_LEVELS`. `libmc1322x`'s `set_power` indexes
    /// its power tables with no bounds check of its own, so an out-of-range
    /// value here would read out of bounds in C.
    pub fn set_output_power(&mut self, power: u8) {
        assert!(
            power < Self::POWER_LEVELS,
            "power level {} out of range 0..{}",
            power,
            Self::POWER_LEVELS
        );
        // Safety: single-threaded hardware register write; `power` was just
        // checked against the table size.
        unsafe {
            mc1322x_sys::set_power(power);
        }
    }

    /// Request the radio to idle to a low-power sleep mode.
    pub fn disable(&mut self) -> impl Future<Output = ()> {
        async {
            // Safety: disables the radio hardware; only called while no
            // operation is in flight.
            unsafe {
                mc1322x_sys::maca_off();
            }
            critical_section::with(|cs| {
                let state = STATE.borrow(cs);
                state.tx_channel.set(None);
                state.cancelled.set(false);
                // Whatever buffer was prepared for a still-pending receive is
                // no longer guaranteed valid once the radio state changes;
                // don't leave a dangling pointer for a later `receive()` call.
                state.rx_buffer.set(None);
            });
        }
    }

    /// Request the radio to wake from sleep.
    pub fn enable(&mut self) -> impl Future<Output = ()> {
        async {
            // Safety: enables the radio hardware.
            unsafe {
                mc1322x_sys::maca_on();
            }
            critical_section::with(|cs| {
                let state = STATE.borrow(cs);
                // `maca_on` resets the MACA, dropping the programmed channel.
                state.tx_channel.set(None);
                state.cancelled.set(false);
                state.rx_buffer.set(None);
            });
        }
    }

    /// Request the radio to go in receive mode and try to receive a frame
    /// into the supplied buffer.
    ///
    /// # Safety
    /// The supplied buffer must remain writable until either successful
    /// reception, or the radio state changed.
    pub unsafe fn prepare_receive(&mut self, bytes: &mut [u8; 128]) -> impl Future<Output = ()> {
        let ptr = NonNull::from(&mut *bytes);
        async move {
            critical_section::with(|cs| {
                let state = STATE.borrow(cs);
                state.rx_buffer.set(Some(RxBuffer(ptr)));
                // A new receive operation starts fresh: clear any cancel that
                // was registered by a dropped future (e.g. the CSMA ACK wait).
                state.cancelled.set(false);
            });
        }
    }

    /// Request the radio to go in receive mode and try to receive a frame.
    ///
    /// Safe to drop before it resolves without calling
    /// [`Mc1322xRadio::cancel_current_operation`] first (see [`ReceiveFuture`]'s doc comment):
    /// the returned future cleans up its own driver state on drop regardless of how it's
    /// polled.
    pub fn receive(&mut self) -> impl Future<Output = bool> {
        ReceiveFuture {
            inner: Self::receive_inner(),
        }
    }

    fn receive_inner() -> impl Future<Output = bool> + Unpin {
        poll_fn(move |cx| {
            critical_section::with(|cs| {
                let state = STATE.borrow(cs);
                // A cancelled operation resolves immediately instead of
                // waiting, and clears the flag so the next operation starts
                // cleanly.
                if state.cancelled.get() {
                    state.cancelled.set(false);
                    state.waker.set(None);
                    // The buffer that was prepared for this (now cancelled)
                    // operation may be dropped by the caller as soon as this
                    // future resolves; don't leave a dangling pointer behind
                    // for a future `receive()` call to dereference.
                    state.rx_buffer.set(None);
                    return Poll::Ready(false);
                }
                let Some(packet) = pop_receive_packet(state) else {
                    state.waker.set(Some(cx.waker().clone()));
                    return Poll::Pending;
                };
                state.waker.set(None);

                let Some(mut rx_buffer) = state.rx_buffer.get() else {
                    // No buffer prepared; drop the frame.
                    unsafe {
                        mc1322x_sys::free_packet(packet);
                    }
                    return Poll::Ready(false);
                };
                // The buffer is only valid for this one reception (see
                // `prepare_receive`'s safety contract); clear it so a stray
                // future `receive()` call without a preceding
                // `prepare_receive()` can't write through a dangling pointer.
                state.rx_buffer.set(None);
                let dst: &mut [u8; 128] = unsafe { rx_buffer.0.as_mut() };
                // Safety: `packet` was just popped from the RX queue by
                // `pop_receive_packet` and not yet freed.
                let (len, _lqi) = unsafe { copy_received_psdu(packet, &mut dst[1..]) };
                dst[0] = (len + 2) as u8;
                Poll::Ready(true)
            })
        })
    }

    /// Request the radio to go in transmit mode and try to send a frame on
    /// `channel` (the IEEE 802.15.4 channel number, 11..=26).
    ///
    /// The mutability of `bytes` is not to modify the buffer, but to hand
    /// over exclusive ownership while the transmission is in flight.
    ///
    /// Note: hardware CCA is not supported by `libmc1322x` (see the module
    /// docs); the caller's own CCA policy is intentionally not consulted
    /// here.
    ///
    /// # Safety
    /// The supplied buffer must remain valid until either successful
    /// transmission, or the radio state changed.
    pub unsafe fn prepare_transmit(
        &mut self,
        channel: u8,
        bytes: &mut [u8],
    ) -> impl Future<Output = ()> {
        let channel_index = channel - 11;
        poll_fn(move |cx| {
            critical_section::with(|cs| {
                let state = STATE.borrow(cs);
                // A new transmit operation starts fresh: clear any cancel that
                // was registered by a dropped future (e.g. the CSMA ACK wait).
                state.cancelled.set(false);

                unsafe {
                    let n = core::ptr::read_volatile(&raw const PREPARE_TRANSMIT_POLLS);
                    core::ptr::write_volatile(&raw mut PREPARE_TRANSMIT_POLLS, n + 1);
                }
                let Some(packet) = claim_free_packet() else {
                    // All packets are busy (e.g. received frames are queued);
                    // wait until the C library frees one.
                    unsafe {
                        let n = core::ptr::read_volatile(&raw const PREPARE_TRANSMIT_NO_PACKET);
                        core::ptr::write_volatile(&raw mut PREPARE_TRANSMIT_NO_PACKET, n + 1);
                    }
                    state.waker.set(Some(cx.waker().clone()));
                    return Poll::Pending;
                };
                unsafe {
                    core::ptr::write_volatile(&raw mut PREPARE_TRANSMIT_GOT_PACKET, 1);
                }

                set_channel_index(state, channel_index);

                // The caller stores the frame length in the PHR byte. Also
                // clamp to `bytes.len()`: callers only need to guarantee that
                // `bytes` stays valid, not that it's exactly 128 bytes long,
                // so a PHR claiming more than `bytes` actually holds must not
                // read past its end (and an empty `bytes` must not panic on
                // the `bytes[0]` read).
                let phr = bytes.first().copied().unwrap_or(0) as usize;
                let len = phr
                    .saturating_sub(2)
                    .min(MAX_PHR - 2)
                    .min(bytes.len().saturating_sub(1));
                // Safety: `packet` was just claimed by `claim_free_packet` and
                // not yet queued.
                unsafe { queue_transmit(packet, &bytes[1..1 + len]) };
                Poll::Ready(())
            })
        })
    }

    /// When working with futures, it is not always guaranteed that a future
    /// will complete. This method is a notification to the radio that it can
    /// prepare for cancelation of whichever operation is currently pending.
    pub fn cancel_current_operation(&mut self) {
        critical_section::with(|cs| {
            let state = STATE.borrow(cs);
            state.cancelled.set(true);
            state.tx_status.set(None);
            if let Some(waker) = state.waker.take() {
                waker.wake();
            }
        });
    }

    /// Request the radio to transmit the queued frame.
    ///
    /// Returns whether the transmission was successful.
    pub fn transmit(&mut self) -> impl Future<Output = bool> {
        poll_fn(move |cx| {
            critical_section::with(|cs| {
                let state = STATE.borrow(cs);
                unsafe {
                    let n = core::ptr::read_volatile(&raw const TRANSMIT_POLLS);
                    core::ptr::write_volatile(&raw mut TRANSMIT_POLLS, n + 1);
                }
                if let Some(status) = state.tx_status.take() {
                    state.waker.set(None);
                    unsafe {
                        core::ptr::write_volatile(&raw mut TRANSMIT_STATUS_SEEN, status + 1);
                    }
                    Poll::Ready(status == mc1322x_sys::SUCCESS)
                } else if state.cancelled.get() {
                    // A cancelled operation resolves immediately instead of
                    // waiting, and clears the flag so the next operation
                    // starts cleanly.
                    state.cancelled.set(false);
                    unsafe {
                        core::ptr::write_volatile(&raw mut TRANSMIT_CANCELLED_SEEN, 1);
                    }
                    Poll::Ready(false)
                } else {
                    state.waker.set(Some(cx.waker().clone()));
                    Poll::Pending
                }
            })
        })
    }

    /// TEMPORARY diagnostic (mc1322x-rs session), currently unused by anything (kept for
    /// reference/reuse only - wiring it in needs a `dot15d4` cached-registry-source edit that's
    /// no longer applied, see below): combines [`Mc1322xRadio::prepare_transmit`] and
    /// [`Mc1322xRadio::transmit`] into one `poll_fn` with a single `.await` point at the call
    /// site, instead of two consecutive ones - an A/B test for a hardware-only stall that
    /// reproduces right at that exact two-await boundary in `dot15d4`'s own `futures::transmit`
    /// wrapper (see the `dot15d4_transmit_await_gap` project memory). Behaviorally identical to
    /// calling both in sequence; the only difference is there is no `async fn` state-machine
    /// transition between them anymore.
    ///
    /// **Result, hardware-verified, reproduced twice**: did not fix the underlying issue - made
    /// it manifest *earlier and more severely* instead (`PRODUCER_LOOP_COUNT` stuck at `0`
    /// instead of `1`, and `IRQ_MIN_SP_SEEN` captured as literally `0` - an alarming, clearly
    /// corrupted stack pointer - during a real MACA interrupt). This rules out the two-await
    /// continuation mechanism itself as the root cause: whatever is actually wrong is deeper,
    /// and this change just shifted memory layout enough to make it manifest differently, same
    /// as every other layout-perturbing change tried across this investigation (see
    /// `dot15d4_stack_watermark_findings`). Wiring this back in requires re-adding the matching
    /// `prepare_and_transmit` default trait method to the cached `dot15d4` crate's
    /// `phy/radio/mod.rs` and the call-site swap in `phy/radio/futures.rs` - not currently
    /// applied, since it didn't help.
    ///
    /// # Safety
    /// Same contract as [`Mc1322xRadio::prepare_transmit`].
    pub unsafe fn prepare_and_transmit(
        &mut self,
        channel: u8,
        bytes: &mut [u8],
    ) -> impl Future<Output = bool> {
        let channel_index = channel - 11;
        let queued = Cell::new(false);
        poll_fn(move |cx| {
            critical_section::with(|cs| {
                let state = STATE.borrow(cs);
                if !queued.get() {
                    state.cancelled.set(false);

                    let Some(packet) = claim_free_packet() else {
                        state.waker.set(Some(cx.waker().clone()));
                        return Poll::Pending;
                    };

                    set_channel_index(state, channel_index);

                    let phr = bytes.first().copied().unwrap_or(0) as usize;
                    let len = phr
                        .saturating_sub(2)
                        .min(MAX_PHR - 2)
                        .min(bytes.len().saturating_sub(1));
                    // Safety: `packet` was just claimed by `claim_free_packet` and
                    // not yet queued.
                    unsafe { queue_transmit(packet, &bytes[1..1 + len]) };
                    queued.set(true);
                    // Fall through to check for completion in this same poll, rather than
                    // returning Pending here and waiting for a separate outer `.await` to
                    // re-poll - there is no separate outer await anymore.
                }

                if let Some(status) = state.tx_status.take() {
                    state.waker.set(None);
                    Poll::Ready(status == mc1322x_sys::SUCCESS)
                } else if state.cancelled.get() {
                    state.cancelled.set(false);
                    Poll::Ready(false)
                } else {
                    state.waker.set(Some(cx.waker().clone()));
                    Poll::Pending
                }
            })
        })
    }

    /// Returns the IEEE 802.15.4 8-octet MAC address of the radio device.
    pub fn ieee802154_address(&self) -> [u8; 8] {
        HW_ADDRESS.load(Ordering::Relaxed).to_ne_bytes()
    }
}

/// TEMPORARY bisection (see the `dot15d4_loopback_repro`/`dot15d4_addressing_panic` project
/// memory): lowest IRQ-mode stack pointer ever observed at the top of `maca_rx_callback`/
/// `maca_tx_callback` - both run nested inside `libmc1322x`'s `irq()` -> `maca_isr()` on the
/// dedicated 256-byte IRQ-mode stack (`IRQ_STACK_SIZE` in `mc1322x-sys/libmc1322x/mc1322x.lds`).
/// `0` until the first callback runs. Read back over JTAG; compare against that stack's own
/// `__stack_start__`/`__irq_stack_top__` linker symbols to see how close to overflow it got.
#[unsafe(no_mangle)]
pub static mut IRQ_MIN_SP_SEEN: u32 = u32::MAX;

/// TEMPORARY bisection: number of times `prepare_transmit`'s `poll_fn` has been polled at all.
#[unsafe(no_mangle)]
pub static mut PREPARE_TRANSMIT_POLLS: u32 = 0;
/// TEMPORARY bisection: number of those polls that found `claim_free_packet()` returning `None`
/// (the C packet pool exhausted) and returned `Poll::Pending`.
#[unsafe(no_mangle)]
pub static mut PREPARE_TRANSMIT_NO_PACKET: u32 = 0;
/// TEMPORARY bisection: set to `1` once `prepare_transmit` has successfully claimed a packet and
/// queued it (i.e. resolved `Poll::Ready`) at least once.
#[unsafe(no_mangle)]
pub static mut PREPARE_TRANSMIT_GOT_PACKET: u32 = 0;
/// TEMPORARY bisection: number of times `transmit`'s `poll_fn` has been polled at all.
#[unsafe(no_mangle)]
pub static mut TRANSMIT_POLLS: u32 = 0;
/// TEMPORARY bisection: `1 + ` the last real MACA status code `transmit()` ever saw via
/// `tx_status` (so `0` unambiguously means "never saw one").
#[unsafe(no_mangle)]
pub static mut TRANSMIT_STATUS_SEEN: u32 = 0;
/// TEMPORARY bisection: set to `1` if `transmit()` ever resolved via the `cancelled` branch
/// instead of a real status.
#[unsafe(no_mangle)]
pub static mut TRANSMIT_CANCELLED_SEEN: u32 = 0;

/// Reads the current stack pointer and lowers [`IRQ_MIN_SP_SEEN`] if it's a new minimum.
#[inline(always)]
fn record_irq_sp() {
    let sp: u32;
    unsafe {
        core::arch::asm!("mov {0}, sp", out(reg) sp, options(nomem, nostack, preserves_flags));
        let min = core::ptr::read_volatile(&raw const IRQ_MIN_SP_SEEN);
        if sp < min {
            core::ptr::write_volatile(&raw mut IRQ_MIN_SP_SEEN, sp);
        }
    }
}

/// RX callback, overriding the weak C symbol in `libmc1322x`.
///
/// Runs in interrupt context before the C library pushes the packet onto its
/// RX queue. Only wakes the executor; the packet is collected by
/// [`Radio::receive`].
#[unsafe(no_mangle)]
extern "C" fn maca_rx_callback(_packet: *mut packet) {
    record_irq_sp();
    critical_section::with(|cs| {
        let state = STATE.borrow(cs);
        state.rx_pending.set(state.rx_pending.get() + 1);
        if let Some(waker) = state.waker.take() {
            waker.wake();
        }
    });
}

/// TX callback, overriding the weak C symbol in `libmc1322x`.
///
/// Runs in interrupt context with the action-complete status already written
/// into the packet by `libmc1322x`. Records the status and wakes the executor.
#[unsafe(no_mangle)]
extern "C" fn maca_tx_callback(packet: *mut packet) {
    record_irq_sp();
    critical_section::with(|cs| {
        let state = STATE.borrow(cs);
        // Safety: the C library always passes a valid packet here.
        state
            .tx_status
            .set(Some(unsafe { (*packet).status } as u32));
        if let Some(waker) = state.waker.take() {
            waker.wake();
        }
    });
}

/// Programs the MACA channel register (0..=15, i.e. IEEE channel 11..=26)
/// if it differs from what's already cached in `state`. Shared by
/// [`Mc1322xRadio::prepare_transmit`] and, with the `radio-hal` feature,
/// [`radio_hal::Channel::set_channel`](radio_hal::Channel).
fn set_channel_index(state: &State, index: u8) {
    if state.tx_channel.get() != Some(index) {
        // Safety: programs the radio channel register.
        unsafe {
            mc1322x_sys::set_channel(index);
        }
        state.tx_channel.set(Some(index));
    }
}

/// Claims a free TX packet from the C pool, if one is available. Shared by
/// [`Mc1322xRadio::prepare_transmit`] and, with the `radio-hal` feature,
/// [`radio_hal::Transmit::start_transmit`](radio_hal::Transmit).
fn claim_free_packet() -> Option<*mut packet> {
    // Safety: claims a packet from the C free pool.
    let packet = unsafe { mc1322x_sys::get_free_packet() };
    if packet.is_null() { None } else { Some(packet) }
}

/// Copies `psdu` into a claimed TX packet and queues it for transmission on
/// whatever channel is currently programmed.
///
/// # Safety
/// `packet` must have come from [`claim_free_packet`] and not yet be queued.
unsafe fn queue_transmit(packet: *mut packet, psdu: &[u8]) {
    let len = psdu.len().min(MAX_PHR - 2);
    // Safety: copy the PSDU (without FCS; the MACA appends it) and queue
    // the packet for transmission.
    unsafe {
        core::ptr::copy_nonoverlapping(psdu.as_ptr(), (*packet).data.as_mut_ptr(), len);
        (*packet).length = len as u8;
        (*packet).offset = 0;
        mc1322x_sys::tx_packet(packet);
    }
}

/// Pops a packet from the C RX queue if one is ready, decrementing
/// `rx_pending`. Shared by [`Mc1322xRadio::receive`] and, with the
/// `radio-hal` feature,
/// [`radio_hal::Receive::get_received`](radio_hal::Receive).
fn pop_receive_packet(state: &State) -> Option<*mut packet> {
    if state.rx_pending.get() == 0 {
        return None;
    }
    // Safety: pops a packet from the C RX queue. The callback increments
    // `rx_pending` before the packet is queued, so a non-zero counter
    // guarantees one is available.
    let packet = unsafe { mc1322x_sys::rx_packet() };
    if packet.is_null() {
        return None;
    }
    state.rx_pending.set(state.rx_pending.get() - 1);
    Some(packet)
}

/// Copies the PSDU (without FCS) of a popped RX packet into `dst`, frees
/// the packet, and returns the number of bytes copied and the MACA's LQI
/// for it.
///
/// # Safety
/// `packet` must have come from [`pop_receive_packet`] and not yet be freed.
unsafe fn copy_received_psdu(packet: *mut packet, dst: &mut [u8]) -> (usize, u8) {
    // Safety: the packet's length field excludes the 2-byte FCS.
    let len = unsafe { (*packet).length as usize }
        .min(MAX_PHR - 2)
        .min(dst.len());
    let lqi = unsafe { (*packet).lqi };
    // Safety: copy the PSDU (FCS already validated and stripped by
    // hardware) and release the packet back to the free pool.
    unsafe {
        core::ptr::copy_nonoverlapping((*packet).data.as_ptr().add(1), dst.as_mut_ptr(), len);
        mc1322x_sys::free_packet(packet);
    }
    (len, lqi)
}

/// Implements `dot15d4`'s [`Radio`] trait as a thin wrapper around the core
/// primitives above, translating `dot15d4`'s config/frame types to and from
/// the driver's plain buffers.
#[cfg(feature = "dot15d4")]
mod dot15d4_radio {
    use dot15d4::phy::{
        config::{RxConfig, TxConfig},
        radio::Radio,
    };

    use super::Mc1322xRadio;
    use crate::layout::dot15d4::{Mc1322xFrame, Mc1322xRxToken, Mc1322xTxToken};

    impl Radio for Mc1322xRadio {
        type RadioFrame<T: AsRef<[u8]>> = Mc1322xFrame<T>;
        type RxToken<'a> = Mc1322xRxToken<'a>;
        type TxToken<'b> = Mc1322xTxToken<'b>;

        fn disable(&mut self) -> impl Future<Output = ()> {
            Mc1322xRadio::disable(self)
        }

        fn enable(&mut self) -> impl Future<Output = ()> {
            Mc1322xRadio::enable(self)
        }

        unsafe fn prepare_receive(
            &mut self,
            _cfg: &RxConfig,
            bytes: &mut [u8; 128],
        ) -> impl Future<Output = ()> {
            // Safety: the trait's safety contract on `bytes` matches
            // `Mc1322xRadio::prepare_receive`'s.
            unsafe { Mc1322xRadio::prepare_receive(self, bytes) }
        }

        fn receive(&mut self) -> impl Future<Output = bool> {
            Mc1322xRadio::receive(self)
        }

        unsafe fn prepare_transmit(
            &mut self,
            cfg: &TxConfig,
            bytes: &mut [u8],
        ) -> impl Future<Output = ()> {
            // Note: hardware CCA is not supported by libmc1322x (see the
            // module docs), so `cfg.cca` is intentionally ignored.
            let _cca = cfg.cca;
            let channel = u8::from(cfg.channel);
            // Safety: the trait's safety contract on `bytes` matches
            // `Mc1322xRadio::prepare_transmit`'s.
            unsafe { Mc1322xRadio::prepare_transmit(self, channel, bytes) }
        }

        fn cancel_current_opperation(&mut self) {
            Mc1322xRadio::cancel_current_operation(self)
        }

        fn transmit(&mut self) -> impl Future<Output = bool> {
            Mc1322xRadio::transmit(self)
        }

        fn ieee802154_address(&self) -> [u8; 8] {
            Mc1322xRadio::ieee802154_address(self)
        }
    }
}

/// Folds the `ieee802154` crate's MAC frame codec directly into the
/// transmit/receive path, the way the `dw1000` driver bakes
/// `ieee802154::mac::Frame` into its own `send`/`receive` API rather than
/// leaving codec and hardware as two separate steps the caller has to wire
/// up themselves.
#[cfg(feature = "ieee802154")]
mod ieee802154_frame {
    use ieee802154::mac::Frame;

    use super::Mc1322xRadio;
    use crate::layout::ieee802154::{read_frame, write_frame};

    impl Mc1322xRadio {
        /// Encode `frame` and transmit it on `channel` (the IEEE 802.15.4
        /// channel number, 11..=26).
        ///
        /// Returns whether the transmission succeeded, or the [`byte::Error`]
        /// if `frame` failed to serialize (e.g. it doesn't fit the 128-byte
        /// buffer) — in which case the hardware is never touched.
        ///
        /// # Safety
        /// Same contract as [`Mc1322xRadio::prepare_transmit`]: the returned
        /// future must be polled to completion, or
        /// [`Mc1322xRadio::cancel_current_operation`] called and the future
        /// then polled to completion, before being dropped.
        pub unsafe fn send_frame(
            &mut self,
            channel: u8,
            frame: Frame<'_>,
        ) -> impl Future<Output = Result<bool, byte::Error>> {
            async move {
                let mut buffer = [0u8; 128];
                write_frame(&mut buffer, frame)?;
                // Safety: forwarded from this method's own contract above.
                unsafe { self.prepare_transmit(channel, &mut buffer) }.await;
                Ok(self.transmit().await)
            }
        }

        /// Try to receive a frame into `buffer`, decoding it into a
        /// [`Frame`] that borrows from it.
        ///
        /// Returns `Ok(None)` if the operation was cancelled or superseded
        /// before a frame arrived, or the [`byte::Error`] if a frame arrived
        /// but failed to parse as a valid IEEE 802.15.4 PSDU.
        ///
        /// # Safety
        /// Same contract as [`Mc1322xRadio::prepare_receive`]: the returned
        /// future must be polled to completion, or
        /// [`Mc1322xRadio::cancel_current_operation`] called and the future
        /// then polled to completion, before being dropped.
        pub unsafe fn receive_frame<'b>(
            &mut self,
            buffer: &'b mut [u8; 128],
        ) -> impl Future<Output = Result<Option<Frame<'b>>, byte::Error>> {
            async move {
                // Safety: forwarded from this method's own contract above.
                unsafe { self.prepare_receive(buffer) }.await;
                if !self.receive().await {
                    return Ok(None);
                }
                let (frame, _len) = read_frame(buffer)?;
                Ok(Some(frame))
            }
        }
    }
}

/// Implements the [`radio-hal`](https://docs.rs/radio) crate's `Transmit`,
/// `Receive`, `State`, `Channel` and `Busy` traits for [`Mc1322xRadio`].
///
/// `radio-hal`'s API is polling-based (`start_*`/`check_*`) rather than
/// `async`, so these impls talk to the packet pool directly instead of going
/// through the futures above, mirroring their logic under synchronous,
/// single-poll semantics.
///
/// [`radio_hal::Power`] and [`radio_hal::Rssi`] are implemented against the
/// MC1322x Reference Manual (RM), since `libmc1322x` itself documents
/// neither a dBm mapping for its power tables nor a LQI/RSSI conversion:
///
/// - [`radio_hal::Power`]: RM Table 3-4 ("MC1322x PA Level vs. Output
///   Power") gives typical dBm values for 18 of the 19 entries in
///   `libmc1322x`'s `PSMVAL`/`PAVAL`/`AIMVAL` power tables (see
///   [`Mc1322xRadio::set_output_power`]); see [`POWER_TABLE_DBM_TENTHS`] for
///   the table and the caveat on the 19th, undocumented level.
/// - [`radio_hal::Rssi`]: RM §6 ("LQI Software Function Calls") gives
///   `Input Power (dBm) = (LQI / 3) - 100` as the conversion from the
///   MACA's LQI (`libmc1322x`'s `get_lqi`) to dBm; see [`lqi_to_rssi_dbm`].
///   [`radio_hal::Rssi::poll_rssi`] reflects the last packet the MACA
///   computed LQI for rather than a continuous energy-detect scan —
///   `libmc1322x` exposes no separate "poll now" primitive.
///
/// Not implemented, and why:
///
/// - [`radio_hal::Interrupts`] and [`radio_hal::Registers`]: this driver
///   hides interrupts and registers behind the packet pool API and has
///   nothing meaningful to expose at that level (see the module docs on
///   [`crate::radio`]).
#[cfg(feature = "radio-hal")]
pub mod radio_hal {
    use core::future::Future;
    use core::pin::pin;
    use core::sync::atomic::Ordering;
    use core::task::{Context, Poll, Waker};

    use portable_atomic::AtomicBool;
    use radio_hal::{
        Busy, Channel, Power, RadioState, Receive, ReceiveInfo, Rssi, State, Transmit,
    };

    use super::{Mc1322xRadio, STATE};

    /// Errors returned by the `radio-hal` trait implementations.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Error {
        /// No free TX packet was available, or no received packet is queued.
        Busy,
        /// Transmission completed with a non-success MACA status code.
        Transmit(u32),
        /// Requested channel is outside the IEEE 802.15.4 11..=26 range.
        InvalidChannel,
    }

    /// [`radio_hal::State`] state for [`Mc1322xRadio`]: on (continuously
    /// receiving, per the [`crate::radio`] module docs) or off.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Mc1322xRadioState {
        /// MACA enabled; the coprocessor is running its receive sequence.
        Idle,
        /// MACA disabled (low-power sleep).
        Sleep,
    }

    impl RadioState for Mc1322xRadioState {
        fn idle() -> Self {
            Self::Idle
        }

        fn sleep() -> Self {
            Self::Sleep
        }
    }

    /// Tracks the state last requested through [`State::set_state`] (`true` =
    /// [`Mc1322xRadioState::Idle`]); there is no hardware register to read it back from. This
    /// target has no native atomics at all, so `portable-atomic`'s `critical-section` fallback
    /// backs this - see `mc1322x-hal::rng`'s `MACA_READY` for the same pattern.
    static RADIO_STATE: AtomicBool = AtomicBool::new(true);

    /// Per-received-packet info: the MACA's link quality indicator.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct Info {
        /// Raw link quality indicator reported by the MACA for this packet.
        pub lqi: u8,
    }

    impl ReceiveInfo for Info {
        fn rssi(&self) -> i16 {
            lqi_to_rssi_dbm(self.lqi)
        }
    }

    /// Converts the MACA's LQI (`libmc1322x`'s `get_lqi`, 0x00..=0xFF) to an
    /// estimated received signal strength in dBm, per the MC1322x Reference
    /// Manual §6 ("LQI Software Function Calls"): `Input Power (dBm) =
    /// (LQI / 3) - 100`. 0x00 is documented as ~-100 dBm and 0xFF as ~-15
    /// dBm; this is a hardware-computed estimate, not a calibrated
    /// measurement.
    fn lqi_to_rssi_dbm(lqi: u8) -> i16 {
        i16::from(lqi) / 3 - 100
    }

    /// Typical output power in tenths of a dBm for MACA power levels
    /// 0..=17 (see [`Mc1322xRadio::set_output_power`]), from the MC1322x
    /// Reference Manual Table 3-4 ("MC1322x PA Level vs. Output Power").
    ///
    /// `libmc1322x`'s `PSMVAL`/`PAVAL`/`AIMVAL` tables have a 19th entry
    /// (index 18, `Mc1322xRadio::POWER_LEVELS - 1`) with register values
    /// distinct from index 17's, but Table 3-4 documents no dBm value for
    /// it — it's unreachable through [`Power::set_power`].
    const POWER_TABLE_DBM_TENTHS: [i16; 18] = [
        -300, -280, -270, -260, -240, -210, -190, -170, -160, -150, -110, -100, -45, -30, -15, -10,
        17, 30,
    ];

    impl Power for Mc1322xRadio {
        type Error = Error;

        /// Sets the output power to whichever of [`POWER_TABLE_DBM_TENTHS`]'s
        /// documented levels is closest to `power`.
        fn set_power(&mut self, power: i8) -> Result<(), Self::Error> {
            let target = i16::from(power) * 10;
            let index = POWER_TABLE_DBM_TENTHS
                .iter()
                .enumerate()
                .min_by_key(|&(_, &dbm_tenths)| (dbm_tenths - target).unsigned_abs())
                .map(|(index, _)| index as u8)
                .expect("POWER_TABLE_DBM_TENTHS is non-empty");
            Mc1322xRadio::set_output_power(self, index);
            Ok(())
        }
    }

    impl Rssi for Mc1322xRadio {
        type Error = Error;

        /// Returns the last LQI the MACA computed, converted to dBm via
        /// [`lqi_to_rssi_dbm`]. Reflects the last packet received, not a
        /// continuous energy-detect scan — `libmc1322x` exposes no separate
        /// "poll now" RSSI/ED primitive for this to call instead.
        fn poll_rssi(&mut self) -> Result<i16, Self::Error> {
            // Safety: `get_lqi` is a ROM entry point populated at boot,
            // takes no arguments and has no side effects beyond reading
            // hardware state already latched by the last reception.
            let lqi = unsafe { mc1322x_sys::get_lqi.expect("get_lqi ROM entry point missing")() };
            Ok(lqi_to_rssi_dbm(lqi))
        }
    }

    /// Poll a future exactly once and return its result.
    fn poll_now<F: Future>(fut: F) -> Poll<F::Output> {
        let waker = Waker::noop();
        let mut cx = Context::from_waker(waker);
        let mut fut = pin!(fut);
        fut.as_mut().poll(&mut cx)
    }

    /// Poll a future that is known to always resolve on its first poll (as
    /// [`Mc1322xRadio::enable`]/[`Mc1322xRadio::disable`] do: their bodies
    /// have no `.await` point).
    fn ready<F: Future>(fut: F) -> F::Output {
        match poll_now(fut) {
            Poll::Ready(output) => output,
            Poll::Pending => unreachable!("enable/disable resolve on their first poll"),
        }
    }

    impl State for Mc1322xRadio {
        type State = Mc1322xRadioState;
        type Error = Error;

        fn set_state(&mut self, state: Self::State) -> Result<(), Self::Error> {
            match state {
                Mc1322xRadioState::Idle => ready(Mc1322xRadio::enable(self)),
                Mc1322xRadioState::Sleep => ready(Mc1322xRadio::disable(self)),
            }
            RADIO_STATE.store(state == Mc1322xRadioState::Idle, Ordering::Relaxed);
            Ok(())
        }

        fn get_state(&mut self) -> Result<Self::State, Self::Error> {
            Ok(if RADIO_STATE.load(Ordering::Relaxed) {
                Mc1322xRadioState::Idle
            } else {
                Mc1322xRadioState::Sleep
            })
        }
    }

    impl Channel for Mc1322xRadio {
        /// IEEE 802.15.4 channel number, 11..=26.
        type Channel = u8;
        type Error = Error;

        fn set_channel(&mut self, channel: &Self::Channel) -> Result<(), Self::Error> {
            if !(11..=26).contains(channel) {
                return Err(Error::InvalidChannel);
            }
            let index = channel - 11;
            critical_section::with(|cs| super::set_channel_index(STATE.borrow(cs), index));
            Ok(())
        }
    }

    impl Busy for Mc1322xRadio {
        type Error = Error;

        fn is_busy(&mut self) -> Result<bool, Self::Error> {
            Ok(critical_section::with(|cs| {
                STATE.borrow(cs).tx_in_flight.get()
            }))
        }
    }

    impl Transmit for Mc1322xRadio {
        type Error = Error;

        fn start_transmit(&mut self, data: &[u8]) -> Result<(), Self::Error> {
            critical_section::with(|cs| {
                let state = STATE.borrow(cs);
                state.cancelled.set(false);

                // radio-hal has no PHR/length-prefix convention of its own:
                // `data` is the full PSDU, transmitted on whatever channel
                // `Channel::set_channel` last programmed.
                let Some(packet) = super::claim_free_packet() else {
                    return Err(Error::Busy);
                };
                // Safety: `packet` was just claimed and not yet queued.
                unsafe { super::queue_transmit(packet, data) };
                state.tx_in_flight.set(true);
                Ok(())
            })
        }

        fn check_transmit(&mut self) -> Result<bool, Self::Error> {
            critical_section::with(|cs| {
                let state = STATE.borrow(cs);
                match state.tx_status.take() {
                    Some(status) => {
                        state.tx_in_flight.set(false);
                        if status == mc1322x_sys::SUCCESS {
                            Ok(true)
                        } else {
                            Err(Error::Transmit(status))
                        }
                    }
                    None => Ok(false),
                }
            })
        }
    }

    impl Receive for Mc1322xRadio {
        type Error = Error;
        type Info = Info;

        fn start_receive(&mut self) -> Result<(), Self::Error> {
            // The MACA receives continuously once enabled (see the
            // `crate::radio` module docs); there is nothing to arm here
            // beyond clearing a stale cancellation.
            critical_section::with(|cs| STATE.borrow(cs).cancelled.set(false));
            Ok(())
        }

        fn check_receive(&mut self, _restart: bool) -> Result<bool, Self::Error> {
            // `restart` needs no handling: corrupted frames (bad CRC or
            // address filter mismatch) never reach the C library's RX queue
            // in the first place (see `checksum_failed_irq`/
            // `filter_failed_irq` in `libmc1322x`'s `maca_isr`), and the
            // hardware always re-arms itself for the next reception.
            Ok(critical_section::with(|cs| {
                STATE.borrow(cs).rx_pending.get() > 0
            }))
        }

        fn get_received(&mut self, buff: &mut [u8]) -> Result<(usize, Self::Info), Self::Error> {
            critical_section::with(|cs| {
                let state = STATE.borrow(cs);
                let Some(packet) = super::pop_receive_packet(state) else {
                    return Err(Error::Busy);
                };
                // Safety: `packet` was just popped from the RX queue and not
                // yet freed.
                let (len, lqi) = unsafe { super::copy_received_psdu(packet, buff) };
                Ok((len, Info { lqi }))
            })
        }
    }

    impl radio_hal::Radio for Mc1322xRadio {}
}
