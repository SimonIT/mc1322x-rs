//! Task watchdog: several tasks each check in with a [`WatchdogRunner`], and [`watchdog_run`]
//! feeds the COP hardware watchdog ([`mc1322x_hal::watchdog`]) only while every registered task
//! has checked in recently enough. If any task goes quiet for longer than its limit, the runner
//! stops feeding and the COP resets the chip within [`WatchdogConfig::hardware_timeout`].
//!
//! Built on [`task-watchdog`](https://docs.rs/task-watchdog)'s chip-independent traits
//! ([`HardwareWatchdog`], which `mc1322x-hal` implements for its COP driver behind its own
//! `task-watchdog` feature, enabled by this one; and [`Clock`], implemented here by
//! [`EmbassyClock`]). `task-watchdog`'s own Embassy runners are separate per-chip modules pinned
//! to older `embassy-time`/`embassy-sync` releases than this workspace uses, and its task table
//! has no notion of retries or setup phases, so [`WatchdogRunner`] keeps its own task table
//! instead. Per task ([`TaskConfig`]):
//!
//! - a **timeout**: the longest the task may go between [`WatchdogRunner::feed`]s;
//! - optional **retries**: how many of those timeouts in a row the task may miss before it
//!   counts as starved - i.e. it's starved after `(retries + 1) * timeout` without a feed. This
//!   is the same as a longer timeout, but keeps "expected every X, tolerate N misses" explicit.
//!   (Unlike `embassy-task-watchdog`'s retries, it doesn't depend on the check interval.)
//! - an optional **setup phase**: until its first feed, the task is held to a separate
//!   [`TaskConfig::setup_timeout`] instead - or not monitored at all, with
//!   [`TaskConfig::unbounded_setup`] - so slow start-up code before a task's main loop doesn't
//!   need a timeout as long as its worst-case start-up.
//!
//! [`WatchdogRunner`]'s methods are plain (non-`async`) functions: the shared state sits behind
//! a [`critical_section::Mutex`] and every method holds it only briefly, so there's nothing to
//! await - and it can be fed from non-async code too.
//!
//! # Caveats
//!
//! - [`WatchdogConfig::hardware_timeout`] must be at most
//!   [`Timeout::MAX`](mc1322x_hal::watchdog::Timeout::MAX) (~11.18 s), or [`watchdog_run`]
//!   panics; it's rounded up to the next COP step (~87 ms).
//!   [`WatchdogConfig::check_interval`] must be comfortably shorter than it.
//! - A task is found starved at the first check after its limit runs out, so the reset comes
//!   up to `check_interval + hardware_timeout` after that.
//! - The COP doesn't count while the chip sleeps, so `SleepyExecutor`'s `Doze` doesn't cause
//!   spurious resets; see [`mc1322x_hal::watchdog`].
//!
//! # Usage
//!
//! ```ignore
//! use embassy_time::{Duration, Timer};
//! use mc1322x_embassy::task_watchdog::{
//!     watchdog_run, Id, TaskConfig, WatchdogConfig, WatchdogRunner,
//! };
//! use static_cell::StaticCell;
//!
//! #[derive(Clone, Copy, PartialEq, Eq, Debug)]
//! enum TaskId { Main, Radio }
//! impl Id for TaskId {}
//!
//! static WATCHDOG: StaticCell<WatchdogRunner<TaskId, 2>> = StaticCell::new();
//!
//! #[embassy_executor::task]
//! async fn watchdog_task(watchdog: &'static WatchdogRunner<TaskId, 2>) -> ! {
//!     watchdog_run(watchdog).await
//! }
//!
//! #[embassy_executor::task]
//! async fn main_task(watchdog: &'static WatchdogRunner<TaskId, 2>) -> ! {
//!     loop {
//!         watchdog.feed(&TaskId::Main);
//!         Timer::after(Duration::from_millis(500)).await;
//!     }
//! }
//!
//! // In the executor's `run` closure:
//! let watchdog = WATCHDOG.init(WatchdogRunner::new(
//!     mc1322x_hal::watchdog::Watchdog::new(),
//!     WatchdogConfig {
//!         hardware_timeout: Duration::from_millis(5000),
//!         check_interval: Duration::from_millis(1000),
//!     },
//! ));
//! watchdog.register_task(&TaskId::Main, Duration::from_millis(2000)).unwrap();
//! // Up to 8 s to come up, then a feed at least every second, tolerating 2 misses in a row.
//! watchdog
//!     .register_task(
//!         &TaskId::Radio,
//!         TaskConfig::new(Duration::from_secs(1))
//!             .retries(2)
//!             .setup_timeout(Duration::from_secs(8)),
//!     )
//!     .unwrap();
//! spawner.spawn(main_task(watchdog).unwrap());
//! spawner.spawn(watchdog_task(watchdog).unwrap());
//! ```

use core::cell::RefCell;

use critical_section::Mutex;
use embassy_time::{Duration, Instant, Timer};
use mc1322x_hal::watchdog::Watchdog as Cop;
pub use task_watchdog::{Clock, HardwareWatchdog, Id, ResetReason};

/// `task-watchdog`'s configuration, on [`EmbassyClock`].
pub type WatchdogConfig = task_watchdog::WatchdogConfig<EmbassyClock>;

/// [`Clock`] backed by `embassy-time` (this crate's [`crate::time_driver`]).
// `Copy` so `WatchdogConfig` (which derives `Copy` bounded on its clock type) is too.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EmbassyClock;

impl Clock for EmbassyClock {
    type Instant = Instant;
    type Duration = Duration;

    fn now(&self) -> Instant {
        Instant::now()
    }

    fn elapsed_since(&self, instant: Instant) -> Duration {
        Instant::now() - instant
    }

    fn has_elapsed(&self, instant: Instant, duration: &Duration) -> bool {
        Instant::now() - instant >= *duration
    }

    fn duration_from_millis(&self, millis: u64) -> Duration {
        Duration::from_millis(millis)
    }
}

/// How a task's start-up, before its first [`WatchdogRunner::feed`], is monitored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Setup {
    /// Same as afterwards.
    None,
    /// With this limit instead.
    Bounded(Duration),
    /// Not at all.
    Unbounded,
}

/// How a task is monitored; see the module docs. A plain [`Duration`] converts into a
/// `TaskConfig` with just that timeout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TaskConfig {
    timeout: Duration,
    retries: u8,
    setup: Setup,
}

impl TaskConfig {
    /// Create a config requiring a feed at least every `timeout`, with no retries and no
    /// separate setup phase.
    pub const fn new(timeout: Duration) -> Self {
        Self {
            timeout,
            retries: 0,
            setup: Setup::None,
        }
    }

    /// Tolerate `retries` missed timeouts in a row: the task only counts as starved after
    /// `(retries + 1) * timeout` without a feed.
    pub const fn retries(mut self, retries: u8) -> Self {
        self.retries = retries;
        self
    }

    /// Allow up to `setup_timeout` until the task's first feed, counted from registration or
    /// from [`watchdog_run`] starting, whichever is later.
    pub const fn setup_timeout(mut self, setup_timeout: Duration) -> Self {
        self.setup = Setup::Bounded(setup_timeout);
        self
    }

    /// Don't monitor the task until its first feed.
    pub const fn unbounded_setup(mut self) -> Self {
        self.setup = Setup::Unbounded;
        self
    }
}

impl From<Duration> for TaskConfig {
    fn from(timeout: Duration) -> Self {
        Self::new(timeout)
    }
}

/// [`WatchdogRunner::register_task`] failed: all `N` task slots are taken.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NoSlotsAvailable;

impl core::fmt::Display for NoSlotsAvailable {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("all task watchdog slots are taken")
    }
}

impl core::error::Error for NoSlotsAvailable {}

struct Task<I> {
    id: I,
    last_feed: Instant,
    /// `(retries + 1) * timeout`, precomputed.
    limit: Duration,
    /// Still before the first feed, if the task has a setup phase.
    setup: Setup,
}

impl<I> Task<I> {
    fn new(id: I, config: TaskConfig) -> Self {
        Self {
            id,
            last_feed: Instant::now(),
            limit: config
                .timeout
                .checked_mul(config.retries as u32 + 1)
                .unwrap_or(Duration::MAX),
            setup: config.setup,
        }
    }

    fn feed(&mut self) {
        self.last_feed = Instant::now();
        self.setup = Setup::None;
    }

    fn is_starved(&self, now: Instant) -> bool {
        let limit = match self.setup {
            Setup::None => self.limit,
            Setup::Bounded(setup_timeout) => setup_timeout,
            Setup::Unbounded => return false,
        };
        now - self.last_feed >= limit
    }
}

struct State<I, const N: usize> {
    cop: Cop,
    config: WatchdogConfig,
    tasks: [Option<Task<I>>; N],
}

impl<I: Id, const N: usize> State<I, N> {
    fn task(&mut self, id: &I) -> Option<&mut Task<I>> {
        self.tasks.iter_mut().flatten().find(|task| task.id == *id)
    }
}

/// Task table shared between the tasks feeding it and [`watchdog_run`].
///
/// `N` is the number of task slots. Put it in a `static` (e.g. a `StaticCell`) so every task can
/// hold a `&'static` reference to it.
pub struct WatchdogRunner<I: Id, const N: usize> {
    state: Mutex<RefCell<State<I, N>>>,
}

impl<I: Id, const N: usize> WatchdogRunner<I, N> {
    /// Create a runner for the COP `cop` with `config`.
    ///
    /// The COP isn't started and no task is monitored until [`watchdog_run`] runs.
    pub fn new(cop: Cop, config: WatchdogConfig) -> Self {
        Self {
            state: Mutex::new(RefCell::new(State {
                cop,
                config,
                tasks: [const { None }; N],
            })),
        }
    }

    fn with<R>(&self, f: impl FnOnce(&mut State<I, N>) -> R) -> R {
        critical_section::with(|cs| f(&mut self.state.borrow_ref_mut(cs)))
    }

    /// Start monitoring `id`, counting from now.
    ///
    /// `config` is a [`TaskConfig`] or a plain [`Duration`] timeout. Tasks can be registered at
    /// any time. Registering an `id` again replaces its configuration and restarts its count,
    /// setup phase included.
    ///
    /// # Errors
    ///
    /// [`NoSlotsAvailable`] if `id` isn't registered yet and all `N` slots are taken.
    pub fn register_task(&self, id: &I, config: impl Into<TaskConfig>) -> Result<(), NoSlotsAvailable> {
        let task = Task::new(*id, config.into());
        self.with(|state| {
            if let Some(existing) = state.task(id) {
                *existing = task;
                return Ok(());
            }
            let slot = state.tasks.iter_mut().find(|slot| slot.is_none());
            *slot.ok_or(NoSlotsAvailable)? = Some(task);
            Ok(())
        })
    }

    /// Stop monitoring `id`, e.g. before it deliberately blocks for longer than its limit.
    pub fn deregister_task(&self, id: &I) {
        self.with(|state| {
            if let Some(slot) = state
                .tasks
                .iter_mut()
                .find(|slot| slot.as_ref().is_some_and(|task| task.id == *id))
            {
                *slot = None;
            }
        });
    }

    /// Check in for `id`.
    ///
    /// The first feed also ends the task's setup phase, if it has one. Does nothing for an `id`
    /// that isn't registered.
    pub fn feed(&self, id: &I) {
        self.with(|state| {
            if let Some(task) = state.task(id) {
                task.feed();
            }
        });
    }

    /// Reset the chip now, via [`mc1322x_hal::reset::software_reset`].
    pub fn trigger_reset(&self) -> ! {
        mc1322x_hal::reset::software_reset()
    }

    /// Always `None`: the MC1322x has no reset-cause register.
    pub fn reset_reason(&self) -> Option<ResetReason> {
        None
    }

    /// Feed the COP unless a task has starved; returns whether one has.
    ///
    /// [`watchdog_run`] calls this every [`WatchdogConfig::check_interval`]; call it yourself
    /// only if you don't use that.
    pub fn check_tasks(&self) -> bool {
        self.with(|state| {
            let now = Instant::now();
            let starved = state.tasks.iter().flatten().any(|task| task.is_starved(now));
            if !starved {
                HardwareWatchdog::<EmbassyClock>::feed(&mut state.cop);
            }
            starved
        })
    }

    /// Restart every task's count and start the COP; returns the check interval.
    fn start(&self) -> Duration {
        self.with(|state| {
            let now = Instant::now();
            // Only the count: a task still in its setup phase stays in it.
            for task in state.tasks.iter_mut().flatten() {
                task.last_feed = now;
            }
            let config = state.config;
            HardwareWatchdog::<EmbassyClock>::start(&mut state.cop, config.hardware_timeout);
            config.check_interval
        })
    }
}

/// Start the COP and keep feeding it for as long as every registered task keeps checking in.
///
/// Run this from its own `#[embassy_executor::task]`.
///
/// # Panics
///
/// Panics if [`WatchdogConfig::hardware_timeout`] is longer than
/// [`Timeout::MAX`](mc1322x_hal::watchdog::Timeout::MAX), or if the COP has been locked.
pub async fn watchdog_run<I: Id, const N: usize>(runner: &WatchdogRunner<I, N>) -> ! {
    let check_interval = runner.start();
    let mut next_check = Instant::now() + check_interval;
    loop {
        runner.check_tasks();
        Timer::at(next_check).await;
        next_check += check_interval;
    }
}
