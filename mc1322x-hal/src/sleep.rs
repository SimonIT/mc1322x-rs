//! CRM low-power (Hibernate/Doze) control.
//!
//! Hibernate and Doze both power down the whole chip except the sleep timer, differing only in
//! which clock keeps running it (RM §5.2.3, §5.3):
//!
//! - [`SleepMode::Hibernate`] keeps the ~2 kHz ring oscillator (or the 32 kHz crystal, if
//!   [`crate::rtc::RtcCrystal`] started it) running: lowest current (~1 µA), but an imprecise
//!   wake delay on the ring oscillator.
//! - [`SleepMode::Doze`] keeps the reference oscillator ÷128 (~187.5 kHz) running: higher
//!   current (~23 µA), but an accurate wake delay without a crystal.
//!
//! # Caveats
//!
//! After the *first* sleep/wake cycle following boot, UART transmits garbled bytes for a while.
//! Waiting doesn't clear it and neither does re-initializing the UART; further sleep/wake
//! cycles (Doze or Hibernate) do. If a peripheral's output must be correct right after the
//! first [`sleep`], run a couple of throwaway sleep/wake cycles first.

use mc1322x_sys::CRM_BASE;
use portable_atomic::{AtomicU32, Ordering};

const WU_CNTL: *mut u32 = (CRM_BASE as usize + 0x04) as *mut u32;
const SLEEP_CNTL: *mut u32 = (CRM_BASE as usize + 0x08) as *mut u32;
const STATUS: *mut u32 = (CRM_BASE as usize + 0x18) as *mut u32;
const WU_COUNT: *mut u32 = (CRM_BASE as usize + 0x20) as *mut u32;
const WU_TIMEOUT: *mut u32 = (CRM_BASE as usize + 0x24) as *mut u32;
const RTC_TIMEOUT: *mut u32 = (CRM_BASE as usize + 0x2c) as *mut u32;

bitflags::bitflags! {
    /// `SLEEP_CNTL` plain flag bits (RM Table 5-8). `RAM_RET` is a 2-bit sub-field rather than
    /// a single flag - see [`RAM_RET_SHIFT`].
    #[derive(Clone, Copy, PartialEq, Eq)]
    struct SleepCntl: u32 {
        const HIB = 1 << 0;
        const DOZE = 1 << 1;
        const MCU_RET = 1 << 6;
        const DIG_PAD_EN = 1 << 7;
    }
}
const RAM_RET_SHIFT: u32 = 4;

bitflags::bitflags! {
    /// `WU_CNTL` plain flag bits (RM Table 5-7). The 4-bit `EXT_WU_*` sub-fields aren't
    /// flags: bit (shift + n) controls KBI(4 + n) - see [`EXT_WU_EN_SHIFT`] etc.
    ///
    /// An RTC wake source sets both `RTC_WU_EN` and `RTC_WU_IEN`.
    #[derive(Clone, Copy, PartialEq, Eq)]
    struct WuCntl: u32 {
        const TIMER_WU_EN = 1 << 0;
        const RTC_WU_EN = 1 << 1;
        const RTC_WU_IEN = 1 << 17;
    }
}
const EXT_WU_EN_SHIFT: u32 = 4;
const EXT_WU_EDGE_SHIFT: u32 = 8;
const EXT_WU_POL_SHIFT: u32 = 12;

bitflags::bitflags! {
    /// `STATUS` plain flag bits (RM Table 5-13). All rw1c except `SLEEP_SYNC`, which hardware
    /// also sets (software only ever clears it). `EXT_WU_EVT` is a 4-bit sub-field rather than
    /// a single flag - see [`EXT_WU_EVT_SHIFT`].
    #[derive(Clone, Copy, PartialEq, Eq)]
    struct Status: u32 {
        const SLEEP_SYNC = 1 << 0;
        const HIB_WU_EVT = 1 << 1;
        const DOZE_WU_EVT = 1 << 2;
        const RTC_WU_EVT = 1 << 3;
        const CAL_DONE = 1 << 9;
        const COP_EVT = 1 << 10;
    }
}
const EXT_WU_EVT_SHIFT: u32 = 4;

/// Which low-power mode to enter. See the module docs for the Hibernate/Doze trade-off.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum SleepMode {
    /// Hibernate: sleep timer on the ring oscillator (or 32 kHz crystal).
    Hibernate,
    /// Doze: sleep timer on the reference oscillator ÷128.
    Doze,
}

/// How much of RAM stays powered during sleep (RM Table 5-8, `RAM_RET[1:0]`).
///
/// More retained RAM costs more sleep current.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum RamRetention {
    /// 8 KB (page 0 only) — reset default.
    Kb8 = 0b00,
    /// 32 KB (pages 0 & 1).
    Kb32 = 0b01,
    /// 64 KB (pages 0, 1 & 2).
    Kb64 = 0b10,
    /// 96 KB (all pages).
    Kb96 = 0b11,
}

/// State retained across sleep, beyond whatever [`RamRetention`] is chosen.
///
/// Without `mcu` set, wake is a cold restart from the bottom of RAM (RM §5.3.1): the CPU does
/// not resume where it left off, so software must have saved anything it needs into retained
/// RAM before sleeping. `gpio_pads` is only honored if `mcu` is also set.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Retention {
    /// Retain CPU, modem and analog control state (`SLEEP_CNTL.MCU_RET`).
    pub mcu: bool,
    /// Retain GPIO pad state (`SLEEP_CNTL.DIG_PAD_EN`); ignored unless `mcu` is set.
    pub gpio_pads: bool,
    /// How much RAM stays powered.
    pub ram: RamRetention,
}

/// One KBI4-7 pin configured as an external wake source (RM §5.2.3.8).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct KbiWake {
    /// `true` = edge sensitive, `false` = level sensitive (`WU_CNTL.EXT_WU_EDGE`).
    pub edge: bool,
    /// `true` = wake on high level / positive edge, `false` = low level / negative edge
    /// (`WU_CNTL.EXT_WU_POL`).
    pub active_high: bool,
}

/// Which sources can wake the chip from sleep, and their timeouts.
///
/// Per RM Table 5-7's note on `EXT_WU_EN`: if no KBI pin is armed, a timer source must be, or
/// the chip has no way to ever wake up. [`sleep`] enforces this.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Default)]
pub struct WakeSources {
    /// Wake-up timer timeout, in ticks of the active sleep-mode clock (`WU_TIMEOUT`).
    pub timer: Option<u32>,
    /// RTC periodic timeout, in RTC ticks (`RTC_TIMEOUT`) — shares the counter
    /// [`crate::rtc::RtcRingOscillator`]/[`crate::rtc::RtcCrystal`] read.
    pub rtc: Option<u32>,
    /// KBI4-7, indexed 0..=3 for KBI4..=7.
    pub kbi: [Option<KbiWake>; 4],
}

/// Why [`sleep`] returned.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum WakeReason {
    /// The wake-up timer fired (`HIB_WU_EVT`/`DOZE_WU_EVT`).
    Timer,
    /// The RTC's periodic timeout fired (`RTC_WU_EVT`).
    Rtc,
    /// A KBI pin transitioned (`EXT_WU_EVT`); the GPIO number, 4..=7.
    Kbi(u8),
    /// The ring oscillator calibration cycle completed (`CAL_DONE`) — see
    /// [`crate::rtc::RtcRingOscillator::recalibrate`].
    CalDone,
    /// The COP (watchdog) timed out with `COP_OUT` set to interrupt rather than reset
    /// (`COP_EVT`).
    Cop,
    /// Woke, but `STATUS` reported none of the above by the time it was read.
    Unknown,
}

#[inline]
unsafe fn read_reg(reg: *mut u32) -> u32 {
    unsafe { reg.read_volatile() }
}

#[inline]
unsafe fn write_reg(reg: *mut u32, value: u32) {
    unsafe { reg.write_volatile(value) }
}

static SLEEP_INHIBIT_COUNT: AtomicU32 = AtomicU32::new(0);

/// RAII guard that keeps a sleep-aware executor (`mc1322x_embassy::SleepyExecutor`) from
/// entering CRM sleep while it's held.
///
/// Every async peripheral wait in this crate (`uart`, `spi`, `i2c`, `delay`, `aes`, `adc`,
/// `gpio`'s `KbiInput`) holds one while it's pending. Those peripherals lose their clock during
/// [`sleep`] and could never raise the interrupt the wait depends on, so a sleep-aware executor
/// must check [`SleepInhibitGuard::count`] and not sleep while it's nonzero. Only this crate's
/// drivers can create one.
pub struct SleepInhibitGuard {
    _private: (),
}

impl SleepInhibitGuard {
    pub(crate) fn new() -> Self {
        SLEEP_INHIBIT_COUNT.fetch_add(1, Ordering::AcqRel);
        SleepInhibitGuard { _private: () }
    }

    /// Number of guards currently held. Nonzero means at least one async peripheral wait in this
    /// crate is in flight.
    pub fn count() -> u32 {
        SLEEP_INHIBIT_COUNT.load(Ordering::Acquire)
    }
}

impl Drop for SleepInhibitGuard {
    fn drop(&mut self) {
        SLEEP_INHIBIT_COUNT.fetch_sub(1, Ordering::AcqRel);
    }
}

/// Read the wake-up timer (`WU_COUNT`), in ticks of the sleep-mode clock the last [`sleep`]
/// ran on.
///
/// The timer restarts from zero on every entry into sleep and keeps counting after wake. Read
/// right after [`sleep`] returns, it gives the time since the chip started going to sleep,
/// including the wake-up sequence (reference oscillator start-up, regulator warm-up) that
/// runs past the programmed [`WakeSources::timer`] timeout.
pub fn timer_count() -> u32 {
    unsafe { read_reg(WU_COUNT) }
}

/// Enter `mode` until one of `sources` wakes the chip, then return why.
///
/// # Panics
///
/// Panics if `sources` has no wake source configured (RM Table 5-7), since the chip could
/// then never wake.
pub fn sleep(mode: SleepMode, sources: WakeSources, retention: Retention) -> WakeReason {
    sleep_with(mode, sources, retention, || {})
}

/// Like [`sleep`], but calls `before_power_down` at the last point before the clocks stop.
///
/// `before_power_down` runs once the sleep request has been accepted (`SLEEP_SYNC` set), right
/// before the write that lets the CRM power the chip down: the CPU clock, the peripheral
/// clocks and the TMR counters stop a few instructions after it returns, and the wake-up timer
/// ([`timer_count`]) starts from zero shortly after that. Use it for anything that must be
/// measured right up to the start of sleep, e.g. a timer's position, so time can be accounted
/// for across the sleep without a gap. Keep it short: the sleep request is already pending.
///
/// # Panics
///
/// Same as [`sleep`].
pub fn sleep_with(
    mode: SleepMode,
    sources: WakeSources,
    retention: Retention,
    before_power_down: impl FnOnce(),
) -> WakeReason {
    assert!(
        sources.timer.is_some() || sources.rtc.is_some() || sources.kbi.iter().any(Option::is_some),
        "no wake source configured — the chip would sleep forever"
    );

    let mut wu_cntl = 0u32;
    if sources.timer.is_some() {
        wu_cntl |= WuCntl::TIMER_WU_EN.bits();
    }
    if sources.rtc.is_some() {
        wu_cntl |= (WuCntl::RTC_WU_EN | WuCntl::RTC_WU_IEN).bits();
    }
    for (n, kbi) in sources.kbi.iter().enumerate() {
        if let Some(kbi) = kbi {
            wu_cntl |= 1 << (EXT_WU_EN_SHIFT + n as u32);
            if kbi.edge {
                wu_cntl |= 1 << (EXT_WU_EDGE_SHIFT + n as u32);
            }
            if kbi.active_high {
                wu_cntl |= 1 << (EXT_WU_POL_SHIFT + n as u32);
            }
        }
    }

    let mut sleep_cntl = (retention.ram as u32) << RAM_RET_SHIFT;
    if retention.mcu {
        sleep_cntl |= SleepCntl::MCU_RET.bits();
        if retention.gpio_pads {
            sleep_cntl |= SleepCntl::DIG_PAD_EN.bits();
        }
    }
    sleep_cntl |= match mode {
        SleepMode::Hibernate => SleepCntl::HIB.bits(),
        SleepMode::Doze => SleepCntl::DOZE.bits(),
    };

    unsafe {
        if let Some(timeout) = sources.timer {
            write_reg(WU_TIMEOUT, timeout);
        }
        if let Some(timeout) = sources.rtc {
            write_reg(RTC_TIMEOUT, timeout);
        }
        write_reg(WU_CNTL, wu_cntl);

        // Entering (RM §5.3.1): writing HIB/DOZE starts the power-down sequence, hardware sets
        // SLEEP_SYNC once its clock domain has synchronized (up to 2 sleep-clock cycles), and
        // clearing SLEEP_SYNC lets power drop. With `retention.mcu`, execution resumes here on
        // wake.
        write_reg(SLEEP_CNTL, sleep_cntl);
        while !Status::from_bits_truncate(read_reg(STATUS)).contains(Status::SLEEP_SYNC) {
            core::hint::spin_loop();
        }
        before_power_down();
        write_reg(STATUS, Status::SLEEP_SYNC.bits());

        // Exiting (RM §5.3.2): hardware reasserts SLEEP_SYNC on wake, and software must clear
        // it again to fully exit low-power mode.
        while !Status::from_bits_truncate(read_reg(STATUS)).contains(Status::SLEEP_SYNC) {
            core::hint::spin_loop();
        }
        // Write back the raw value (`from_bits_truncate` would drop the `EXT_WU_EVT`
        // sub-field) to clear SLEEP_SYNC and every *_EVT bit that fired in one rw1c write.
        let raw_status = read_reg(STATUS);
        write_reg(STATUS, raw_status);
        let status = Status::from_bits_truncate(raw_status);

        // RTC_WU_EVT goes first: an RTC wake also sets HIB_WU_EVT even with TIMER_WU_EN
        // clear, despite RM Table 5-13 saying it's "only set if enabled by TIMER_WU_EN".
        if status.contains(Status::RTC_WU_EVT) {
            WakeReason::Rtc
        } else if status.intersects(Status::HIB_WU_EVT | Status::DOZE_WU_EVT) {
            WakeReason::Timer
        } else if (raw_status >> EXT_WU_EVT_SHIFT) & 0xF != 0 {
            WakeReason::Kbi(4 + ((raw_status >> EXT_WU_EVT_SHIFT) & 0xF).trailing_zeros() as u8)
        } else if status.contains(Status::CAL_DONE) {
            WakeReason::CalDone
        } else if status.contains(Status::COP_EVT) {
            WakeReason::Cop
        } else {
            WakeReason::Unknown
        }
    }
}
