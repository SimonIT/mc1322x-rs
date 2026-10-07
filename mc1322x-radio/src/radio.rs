//! MACA driver built on the `libmc1322x` packet pool.
//!
//! The MACA runs a receive sequence whenever it is not transmitting (`libmc1322x` re-arms it from
//! its interrupt handler). Received frames are queued by the C library; `maca_rx_callback` only
//! counts them and wakes the pending future, which then pops a packet and copies the PSDU into the
//! caller's buffer. Transmissions claim a free packet, copy the PSDU into it and queue it;
//! `maca_tx_callback` records the MACA status code on action-complete.
//!
//! `libmc1322x` always posts TX with `maca_ctrl_mode_no_cca` (`post_tx` in `lib/maca.c`), so there
//! is no hardware CCA.

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

/// IEEE 802.15.4 radio driver for the MACA coprocessor.
///
/// Buffers use the PHR+PSDU layout from [`crate::layout`]. The MACA appends and checks the FCS in
/// hardware and drops frames with a bad FCS. Frames are sent without hardware CCA; a failed
/// transmission makes [`transmit`](Self::transmit) return `false`.
///
/// This is a zero-sized handle; the driver state lives in statics shared with the MACA interrupt
/// callbacks.
pub struct Mc1322xRadio;

/// Future returned by [`Mc1322xRadio::receive`]; clears the receive state when dropped.
///
/// Callers can't always call `cancel_current_operation` before the future is dropped: when a
/// `select()` (e.g. `dot15d4`'s ACK wait against a timeout) is torn down, the losing future is
/// dropped before the caller's own drop guard runs. Without this, the stale `rx_buffer` pointer
/// and `cancelled` flag would survive until the next `prepare_receive`.
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
/// The pointer is only dereferenced by the receive future, inside a critical section, and only
/// within the validity window promised by `prepare_receive`'s safety contract.
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
    /// Cached MACA channel index (0..=15, i.e. IEEE channel 11..=26), used for TX and RX.
    /// `None` when unknown (after `enable`/`disable`).
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
/// ARMv4T has no atomic instructions; `portable-atomic` implements this with a critical section.
static HW_ADDRESS: AtomicU64 = AtomicU64::new(0);

impl Mc1322xRadio {
    /// Number of entries in the `PSMVAL`/`PAVAL`/`AIMVAL` power tables in `libmc1322x`'s
    /// `lib/maca.c`, i.e. the number of levels accepted by [`Mc1322xRadio::set_output_power`].
    const POWER_LEVELS: u8 = 19;

    /// Initialize the MACA coprocessor and return the radio driver.
    ///
    /// `address` is the IEEE 802.15.4 extended address reported by
    /// [`ieee802154_address`](Self::ieee802154_address) to the MAC layer; it is not programmed
    /// into the hardware.
    ///
    /// The MACA is initialized through [`mc1322x_hal::rng::ensure_maca_ready`], so calling this
    /// more than once, or after the HAL's RNG has already initialized the MACA, is fine. After
    /// initialization the radio is on channel 11.
    pub fn init(address: [u8; 8]) -> Self {
        HW_ADDRESS.store(u64::from_ne_bytes(address), Ordering::Relaxed);
        mc1322x_hal::rng::ensure_maca_ready();
        Self
    }

    /// Set the RF channel (IEEE 802.15.4 channel number, 11..=26).
    ///
    /// The MACA has a single channel register for both transmit and receive.
    /// [`prepare_transmit`](Self::prepare_transmit) sets the channel itself, so this is mainly
    /// needed to select the receive channel. [`enable`](Self::enable) resets the channel,
    /// so call this afterwards.
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

    /// Set the RF output power level.
    ///
    /// `power` indexes `libmc1322x`'s power tables, from 0 (lowest) to 18 (highest). RM Table 3-4
    /// lists the typical output power in dBm for levels 0..=17.
    ///
    /// # Panics
    ///
    /// Panics if `power` is greater than 18.
    pub fn set_output_power(&mut self, power: u8) {
        assert!(
            power < Self::POWER_LEVELS,
            "power level {} out of range 0..{}",
            power,
            Self::POWER_LEVELS
        );
        // Safety: single-threaded hardware register write; `power` was just checked against the
        // table size (the C `set_power` does no bounds check of its own).
        unsafe {
            mc1322x_sys::set_power(power);
        }
    }

    /// Turn the radio off.
    ///
    /// Any prepared receive buffer is released. The returned future resolves on its first poll.
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

    /// Turn the radio on and start receiving.
    ///
    /// This resets the MACA, including the channel, and releases any prepared receive buffer.
    /// The returned future resolves on its first poll.
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

    /// Set the buffer that the next [`receive`](Self::receive) writes the received frame into.
    ///
    /// The frame is stored in the [`crate::layout`] format. The radio receives continuously while
    /// enabled, so this does not start the receiver; the returned future resolves on its first
    /// poll.
    ///
    /// # Safety
    ///
    /// `bytes` must remain valid and writable until [`receive`](Self::receive) resolves or is
    /// dropped, or the radio is enabled or disabled.
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

    /// Wait for a frame and copy it into the buffer set by
    /// [`prepare_receive`](Self::prepare_receive).
    ///
    /// Returns `true` if a frame was received, or `false` if the operation was cancelled with
    /// [`cancel_current_operation`](Self::cancel_current_operation) or no buffer was prepared
    /// (the frame is then discarded).
    ///
    /// The returned future may be dropped before it resolves; it releases the prepared buffer.
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
                // The buffer is only valid for this one reception; clear it so a later
                // `receive()` without `prepare_receive()` can't write through a dangling pointer.
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

    /// Queue the frame in `bytes` for transmission on `channel` (IEEE 802.15.4 channel number,
    /// 11..=26).
    ///
    /// `bytes` uses the [`crate::layout`] format; the PSDU length is taken from the PHR and
    /// clamped to the buffer. The returned future waits until a free packet is available, copies
    /// the frame into it and queues it; the MACA starts sending right away, without CCA. Use
    /// [`transmit`](Self::transmit) to wait for the result.
    ///
    /// `bytes` is `&mut` only to hand over exclusive access; it is not modified.
    ///
    /// # Safety
    ///
    /// `bytes` must remain valid until the returned future resolves or is dropped.
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

                // The length comes from the PHR byte. `bytes` need not be 128 bytes long, so
                // clamp to its length, and treat an empty slice as an empty frame.
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

    /// Cancel the pending [`receive`](Self::receive) or [`transmit`](Self::transmit).
    ///
    /// The pending future resolves with `false` on its next poll. A frame that is already being
    /// sent is not stopped.
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

    /// Wait for the frame queued by [`prepare_transmit`](Self::prepare_transmit) to be sent.
    ///
    /// Returns `true` if the MACA reports success, or `false` on any other MACA status (e.g. no
    /// ACK received) or if the operation was cancelled.
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

    /// Queue a frame and wait for it to be sent.
    ///
    /// Equivalent to [`prepare_transmit`](Self::prepare_transmit) followed by
    /// [`transmit`](Self::transmit), as a single future.
    ///
    /// # Safety
    ///
    /// `bytes` must remain valid until the returned future resolves or is dropped.
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
                    // Fall through and check for completion in this same poll.
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

    /// Return the IEEE 802.15.4 extended address passed to [`init`](Self::init).
    pub fn ieee802154_address(&self) -> [u8; 8] {
        HW_ADDRESS.load(Ordering::Relaxed).to_ne_bytes()
    }
}

/// Debug value for inspection with a debugger: lowest IRQ-mode stack pointer seen on entry to
/// `maca_rx_callback`/`maca_tx_callback`, or `u32::MAX` before the first callback.
///
/// Both callbacks run inside `libmc1322x`'s `maca_isr` on the 256-byte IRQ-mode stack
/// (`IRQ_STACK_SIZE` in `mc1322x.lds`); compare against `__irq_stack_top__` to see the headroom.
#[unsafe(no_mangle)]
pub static mut IRQ_MIN_SP_SEEN: u32 = u32::MAX;

/// Debug counter: number of polls of `prepare_transmit`'s future.
#[unsafe(no_mangle)]
pub static mut PREPARE_TRANSMIT_POLLS: u32 = 0;
/// Debug counter: number of `prepare_transmit` polls that found no free packet.
#[unsafe(no_mangle)]
pub static mut PREPARE_TRANSMIT_NO_PACKET: u32 = 0;
/// Debug flag: `1` once `prepare_transmit` has queued a packet.
#[unsafe(no_mangle)]
pub static mut PREPARE_TRANSMIT_GOT_PACKET: u32 = 0;
/// Debug counter: number of polls of `transmit`'s future.
#[unsafe(no_mangle)]
pub static mut TRANSMIT_POLLS: u32 = 0;
/// Debug value: last MACA status code seen by `transmit`, plus one (`0` = none seen yet).
#[unsafe(no_mangle)]
pub static mut TRANSMIT_STATUS_SEEN: u32 = 0;
/// Debug flag: `1` once `transmit` has resolved because of a cancellation.
#[unsafe(no_mangle)]
pub static mut TRANSMIT_CANCELLED_SEEN: u32 = 0;

/// Update [`IRQ_MIN_SP_SEEN`] with the current stack pointer.
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
/// Runs in interrupt context just before the C library pushes the packet onto its RX queue.
/// Only counts the packet and wakes the pending future; the packet is collected by
/// [`Mc1322xRadio::receive`] or `radio_hal::Receive::get_received`.
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
/// Runs in interrupt context on action-complete, with the MACA status already written into the
/// packet by `libmc1322x`. Records the status and wakes the pending future.
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

/// Program the MACA channel index (0..=15, i.e. IEEE channel 11..=26) if it differs from the
/// one cached in `state`.
fn set_channel_index(state: &State, index: u8) {
    if state.tx_channel.get() != Some(index) {
        // Safety: programs the radio channel register.
        unsafe {
            mc1322x_sys::set_channel(index);
        }
        state.tx_channel.set(Some(index));
    }
}

/// Claim a free packet from the C pool, if one is available.
fn claim_free_packet() -> Option<*mut packet> {
    // Safety: claims a packet from the C free pool.
    let packet = unsafe { mc1322x_sys::get_free_packet() };
    if packet.is_null() { None } else { Some(packet) }
}

/// Copy `psdu` (clamped to 125 bytes) into a claimed packet and queue it for transmission on
/// the currently programmed channel.
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

/// Pop a packet from the C RX queue if one is ready, decrementing `rx_pending`.
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

/// Copy the PSDU (without FCS) of a popped RX packet into `dst` and free the packet.
///
/// Returns the number of bytes copied and the packet's LQI.
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

/// `dot15d4::phy::radio::Radio` implementation, forwarding to the inherent methods.
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
            // libmc1322x never uses hardware CCA, so `cfg.cca` is ignored.
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

/// Typed `ieee802154::mac::Frame` transmit/receive methods.
#[cfg(feature = "ieee802154")]
mod ieee802154_frame {
    use ieee802154::mac::Frame;

    use super::Mc1322xRadio;
    use crate::layout::ieee802154::{read_frame, write_frame};

    impl Mc1322xRadio {
        /// Encode `frame` and transmit it on `channel` (IEEE 802.15.4 channel number, 11..=26).
        ///
        /// Returns `Ok(true)` if the transmission succeeded and `Ok(false)` if it failed, as for
        /// [`Mc1322xRadio::transmit`].
        ///
        /// # Errors
        ///
        /// Returns the [`byte::Error`] if `frame` cannot be serialized (e.g. it is longer than
        /// 125 bytes). Nothing is transmitted in that case.
        ///
        /// # Safety
        ///
        /// The returned future must be polled to completion, or
        /// [`Mc1322xRadio::cancel_current_operation`] called and the future then polled to
        /// completion, before it is dropped.
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

        /// Receive a frame into `buffer` and decode it into a [`Frame`] borrowing from it.
        ///
        /// Returns `Ok(None)` if the operation was cancelled before a frame arrived.
        ///
        /// # Errors
        ///
        /// Returns the [`byte::Error`] if the received PSDU is not a valid IEEE 802.15.4 frame.
        ///
        /// # Safety
        ///
        /// Same contract as [`Mc1322xRadio::prepare_receive`].
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

#[cfg(feature = "radio-hal")]
pub mod radio_hal {
    //! Implementations of the [`radio`](https://docs.rs/radio) crate's (`radio-hal`) traits for
    //! [`Mc1322xRadio`].
    //!
    //! Implemented: [`Transmit`], [`Receive`], [`State`], [`Channel`], [`Busy`], [`Power`],
    //! [`Rssi`] and [`radio_hal::Radio`](::radio_hal::Radio). These traits are non-blocking
    //! (`start_*`/`check_*`), so they access the packet pool directly instead of going through
    //! the async methods.
    //!
    //! - [`Transmit::start_transmit`] takes the PSDU without PHR or FCS and sends it on the
    //!   channel last set with [`Channel::set_channel`] (or by an async transmit).
    //! - [`State::set_state`] with [`Mc1322xRadioState::Idle`] resets the channel; set the
    //!   channel afterwards.
    //! - [`Power::set_power`] selects the level from RM Table 3-4 ("MC1322x PA Level vs. Output
    //!   Power") closest to the requested dBm value. Level 18, which the table does not list, is
    //!   only reachable through [`Mc1322xRadio::set_output_power`].
    //! - [`Rssi::poll_rssi`] and [`ReceiveInfo::rssi`] convert the MACA's LQI with
    //!   `Input Power (dBm) = (LQI / 3) - 100` (RM chapter 6, before Table 6-2), giving -100 to
    //!   -15 dBm. `poll_rssi` reports the last received packet, not a live energy-detect reading.
    //!
    //! `Interrupts` and `Registers` are not implemented; the driver hides both behind the packet
    //! pool API.

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

    /// Radio state for the [`State`] trait.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Mc1322xRadioState {
        /// Radio on and receiving.
        Idle,
        /// Radio off.
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

    /// State last set through [`State::set_state`] (`true` = [`Mc1322xRadioState::Idle`]); it
    /// can't be read back from the hardware.
    static RADIO_STATE: AtomicBool = AtomicBool::new(true);

    /// Information about a received packet.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct Info {
        /// Link quality indicator reported by the MACA (0x00 = about -100 dBm, 0xFF = about
        /// -15 dBm).
        pub lqi: u8,
    }

    impl ReceiveInfo for Info {
        fn rssi(&self) -> i16 {
            lqi_to_rssi_dbm(self.lqi)
        }
    }

    /// Convert an LQI value to an estimated input power in dBm (RM chapter 6:
    /// `(LQI / 3) - 100`).
    fn lqi_to_rssi_dbm(lqi: u8) -> i16 {
        i16::from(lqi) / 3 - 100
    }

    /// Typical output power in tenths of a dBm for power levels 0..=17, from RM Table 3-4.
    ///
    /// `libmc1322x`'s tables have a 19th level (index 18) that Table 3-4 does not list, so
    /// [`Power::set_power`] never selects it.
    const POWER_TABLE_DBM_TENTHS: [i16; 18] = [
        -300, -280, -270, -260, -240, -210, -190, -170, -160, -150, -110, -100, -45, -30, -15, -10,
        17, 30,
    ];

    impl Power for Mc1322xRadio {
        type Error = Error;

        /// Set the output power to the documented level closest to `power` dBm (-30 to +3 dBm).
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

        /// Return the input power in dBm estimated from the LQI of the last received packet.
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

    /// Poll a future that always resolves on its first poll, such as
    /// [`Mc1322xRadio::enable`]/[`Mc1322xRadio::disable`].
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
            // The MACA receives continuously once enabled; only clear a stale cancellation.
            critical_section::with(|cs| STATE.borrow(cs).cancelled.set(false));
            Ok(())
        }

        fn check_receive(&mut self, _restart: bool) -> Result<bool, Self::Error> {
            // `restart` needs no handling: frames failing the CRC or address filter never reach
            // the RX queue (see `checksum_failed_irq`/`filter_failed_irq` in `maca_isr`), and
            // the MACA re-arms itself after every reception.
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
