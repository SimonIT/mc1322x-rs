//! CRM low-power (Hibernate/Doze) control.
//!
//! Hibernate and Doze both power down the whole chip except the sleep timer, differing only
//! in which clock stays alive to run it (RM §5.2.3, §5.3):
//!
//! - [`SleepMode::Hibernate`] keeps the ~2 kHz ring oscillator (or the 32.768 kHz crystal, if
//!   [`crate::rtc::RtcCrystal`] started it) running — lowest current (~1 µA) but an imprecise
//!   wake delay off the ring oscillator.
//! - [`SleepMode::Doze`] keeps the reference oscillator ÷128 (~187.5 kHz) running — higher
//!   current (~23 µA) but an accurate wake delay without needing a crystal.
//!
//! There is no standard Rust trait for this (no HAL crate defines one — sleep/wake models are
//! too MCU-specific to generalize), so [`sleep`] is a bespoke, from-scratch API.
//!
//! # Hardware caveat: peripheral clocking is unreliable right after the first sleep/wake cycle
//!
//! Hardware-verified (`examples/sleep-selftest`): UART transmits garbled bytes for a while
//! after the *first* `sleep()`/wake cycle following boot. Ruled out as causes: a leftover
//! TX-in-flight race, a peripheral clock settling delay (up to ~830ms tested), and the UART
//! needing re-initialization. What does clear it, reproducibly, is completing further
//! sleep/wake cycles (Doze or Hibernate, not a fixed count) — not achievable by passively
//! waiting, however long. This points to a genuine MC1322x CRM/clock-generation quirk (likely
//! an edge-triggered PLL/divider resync state machine, not one that settles with elapsed time)
//! rather than a bug in this module. If you rely on a peripheral whose timing derives from the
//! same clock right after the first post-boot `sleep()` call, verify it independently (e.g. a
//! JTAG-readable static, as the example does) rather than trusting its output directly, or run
//! a couple of harmless throwaway sleep/wake cycles first.

use mc1322x_sys::CRM_BASE;

const WU_CNTL: *mut u32 = (CRM_BASE as usize + 0x04) as *mut u32;
const SLEEP_CNTL: *mut u32 = (CRM_BASE as usize + 0x08) as *mut u32;
const STATUS: *mut u32 = (CRM_BASE as usize + 0x18) as *mut u32;
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
    /// `WU_CNTL` plain flag bits (RM Table 5-7). The four `EXT_WU_*` sub-fields are all
    /// indexed the same way instead of being single flags: sub-bit n (0..=3) is KBI(4+n), so
    /// overall bit (shift+n) controls/reports KBI(4+n) - see [`EXT_WU_EN_SHIFT`] etc.
    ///
    /// TEMPORARY: testing whether `RTC_WU_IEN` (bit 17, distinct from `RTC_WU_EN`'s bit 1) is
    /// needed alongside `RTC_WU_EN` for the RTC wake comparator to actually assert
    /// `SLEEP_SYNC` on wake - `sleep()`'s RTC wake source currently hangs forever without it.
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
    Hibernate,
    Doze,
}

/// How much of RAM stays powered during sleep (RM Table 5-8, `RAM_RET[1:0]`).
///
/// More retained RAM costs more sleep current; less retained RAM means more of your data
/// needs to live outside RAM (e.g. in NVM) or be reconstructed on wake.
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

/// Enter `mode` until one of `sources` wakes the chip, then return why.
///
/// # Panics
///
/// Panics if `sources` has no wake source configured at all — per RM Table 5-7, with no KBI
/// pin armed a timer source (`timer` or `rtc`) must be, or the chip would sleep forever with
/// no way to wake.
pub fn sleep(mode: SleepMode, sources: WakeSources, retention: Retention) -> WakeReason {
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

        // Entering: RM §5.3.1. Writing HIB/DOZE starts the power-down sequence; hardware sets
        // SLEEP_SYNC once its clock domain has synchronized (up to 2 sleep-clock cycles), and
        // clearing SLEEP_SYNC is what actually lets power drop — execution pauses somewhere
        // around here as the CRM gates the CPU's own clock, resuming (with `retention.mcu`)
        // exactly where it left off once a wake source fires.
        write_reg(SLEEP_CNTL, sleep_cntl);
        while !Status::from_bits_truncate(read_reg(STATUS)).contains(Status::SLEEP_SYNC) {
            core::hint::spin_loop();
        }
        write_reg(STATUS, Status::SLEEP_SYNC.bits());

        // Exiting: RM §5.3.2, the same handshake in reverse — hardware reasserts SLEEP_SYNC as
        // part of waking, and software must clear it again to fully exit low-power mode.
        while !Status::from_bits_truncate(read_reg(STATUS)).contains(Status::SLEEP_SYNC) {
            core::hint::spin_loop();
        }
        // Kept as the raw register value (not `Status::from_bits_truncate`, which would drop
        // the non-flag `EXT_WU_EVT` sub-field): clears SLEEP_SYNC and every *_EVT bit that
        // fired in one write - rw1c bits that were 1 clear, bits that were 0 (including the
        // read-only VREG_*_RDY bits) are unaffected.
        let raw_status = read_reg(STATUS);
        write_reg(STATUS, raw_status);
        let status = Status::from_bits_truncate(raw_status);

        // RTC_WU_EVT is checked before HIB_WU_EVT/DOZE_WU_EVT: hardware-verified (a
        // 1000-ring-oscillator-tick RTC wake with `TIMER_WU_EN` never set still reported
        // HIB_WU_EVT set, misclassifying a precisely-on-time RTC wake as `Timer` when checked
        // in the other order) that a wake-up-timer-class status bit can be set alongside a
        // genuine RTC wake, contradicting the RM's Table 5-13 description of
        // HIB_WU_EVT/DOZE_WU_EVT as "only set if enabled by TIMER_WU_EN" - either a
        // documentation inaccuracy or an interaction not covered by it. RTC_WU_EN/RTC_WU_IEN
        // being the ones this call actually armed makes `Rtc` the correct answer whenever
        // RTC_WU_EVT is set, regardless of what else also is.
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
