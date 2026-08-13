//! Atomic Core0-to-Core1 command transport.

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::time::{Duration, Instant};
use esp_println::println;
use esp_rtos::CurrentThreadHandle;

const COMMAND_NONE: u32 = 0;
const COMMAND_SHIFT: u32 = 1;
const COMMAND_RESET: u32 = 2;

const STATE_IDLE: u32 = 0;
const STATE_BUSY: u32 = 1;
const STATE_DONE: u32 = 2;
const STATE_ERROR: u32 = 3;

struct Mailbox {
    state: AtomicU32,
    command: AtomicU32,
    next_sequence: AtomicU32,
    active_sequence: AtomicU32,
    completed_sequence: AtomicU32,
    bit_count: AtomicU32,
    tck_period_us: AtomicU32,
    tms_ptr: AtomicUsize,
    tdi_ptr: AtomicUsize,
    tdo_ptr: AtomicUsize,
    abort: AtomicBool,
}

impl Mailbox {
    const fn new() -> Self {
        Self {
            state: AtomicU32::new(STATE_IDLE),
            command: AtomicU32::new(COMMAND_NONE),
            next_sequence: AtomicU32::new(0),
            active_sequence: AtomicU32::new(0),
            completed_sequence: AtomicU32::new(0),
            bit_count: AtomicU32::new(0),
            tck_period_us: AtomicU32::new(super::MIN_TCK_PERIOD_US),
            tms_ptr: AtomicUsize::new(0),
            tdi_ptr: AtomicUsize::new(0),
            tdo_ptr: AtomicUsize::new(0),
            abort: AtomicBool::new(false),
        }
    }
}

static MAILBOX: Mailbox = Mailbox::new();
static COMPLETION: Signal<CriticalSectionRawMutex, u32> = Signal::new();

/// Zero means idle. A non-zero value identifies the command that owns all
/// buffers currently visible to Core1.
static ACTIVE_SEQUENCE: AtomicU32 = AtomicU32::new(0);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PublishError {
    Busy,
    WorkerCommandOccupied,
}

pub(super) struct ActiveCommand {
    sequence: u32,
    in_flight: bool,
}

impl ActiveCommand {
    pub(super) fn is_complete(&self) -> bool {
        MAILBOX.completed_sequence.load(Ordering::Acquire) == self.sequence
    }

    fn succeeded(&self) -> bool {
        MAILBOX.state.load(Ordering::Acquire) == STATE_DONE
    }

    pub(super) async fn wait_for_completion(&self) {
        while !self.is_complete() {
            let completed_sequence = COMPLETION.wait().await;
            if completed_sequence == self.sequence {
                break;
            }
        }
    }

    pub(super) fn finish(mut self) -> bool {
        debug_assert!(self.is_complete());
        let succeeded = self.succeeded();
        self.release();
        self.in_flight = false;
        succeeded
    }

    fn release(&self) {
        clear_abort();
        let released =
            ACTIVE_SEQUENCE.compare_exchange(self.sequence, 0, Ordering::AcqRel, Ordering::Acquire);
        debug_assert!(released.is_ok());
    }
}

impl Drop for ActiveCommand {
    fn drop(&mut self) {
        if !self.in_flight {
            return;
        }

        request_abort();
        let deadline = Instant::now() + Duration::from_secs(2);
        while !self.is_complete() {
            if Instant::now() >= deadline {
                println!(
                    "CRITICAL: Core1 stuck while cancelling sequence {}",
                    self.sequence
                );
                esp_hal::system::software_reset();
            }
            CurrentThreadHandle::get().delay(Duration::ZERO);
        }
        self.release();
    }
}

pub(super) enum WorkerCommand {
    Reset {
        sequence: u32,
        tck_period_us: u32,
    },
    Shift {
        sequence: u32,
        tck_period_us: u32,
        bit_count: usize,
        tms_ptr: *const u8,
        tdi_ptr: *const u8,
        tdo_ptr: *mut u8,
    },
    Invalid {
        sequence: u32,
    },
}

pub(super) fn publish_reset(tck_period_us: u32) -> Result<ActiveCommand, PublishError> {
    publish(COMMAND_RESET, tck_period_us, 0, 0, 0, 0)
}

pub(super) fn publish_shift(
    tck_period_us: u32,
    bit_count: usize,
    tms_ptr: *const u8,
    tdi_ptr: *const u8,
    tdo_ptr: *mut u8,
) -> Result<ActiveCommand, PublishError> {
    publish(
        COMMAND_SHIFT,
        tck_period_us,
        bit_count,
        tms_ptr as usize,
        tdi_ptr as usize,
        tdo_ptr as usize,
    )
}

fn publish(
    command: u32,
    tck_period_us: u32,
    bit_count: usize,
    tms_ptr: usize,
    tdi_ptr: usize,
    tdo_ptr: usize,
) -> Result<ActiveCommand, PublishError> {
    let sequence = next_sequence();
    if ACTIVE_SEQUENCE
        .compare_exchange(0, sequence, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return Err(PublishError::Busy);
    }

    MAILBOX.state.store(STATE_IDLE, Ordering::Relaxed);
    MAILBOX.abort.store(false, Ordering::Relaxed);
    MAILBOX
        .tck_period_us
        .store(tck_period_us, Ordering::Relaxed);
    MAILBOX.bit_count.store(bit_count as u32, Ordering::Relaxed);
    MAILBOX.tms_ptr.store(tms_ptr, Ordering::Relaxed);
    MAILBOX.tdi_ptr.store(tdi_ptr, Ordering::Relaxed);
    MAILBOX.tdo_ptr.store(tdo_ptr, Ordering::Relaxed);
    MAILBOX.active_sequence.store(sequence, Ordering::Relaxed);

    if MAILBOX
        .command
        .compare_exchange(COMMAND_NONE, command, Ordering::Release, Ordering::Relaxed)
        .is_err()
    {
        ACTIVE_SEQUENCE.store(0, Ordering::Release);
        return Err(PublishError::WorkerCommandOccupied);
    }

    COMPLETION.reset();
    Ok(ActiveCommand {
        sequence,
        in_flight: true,
    })
}

fn next_sequence() -> u32 {
    loop {
        let previous = MAILBOX.next_sequence.fetch_add(1, Ordering::Relaxed);
        let sequence = previous.wrapping_add(1);
        if sequence != 0 {
            return sequence;
        }
    }
}

pub(super) fn request_abort() {
    MAILBOX.abort.store(true, Ordering::Release);
}

pub(super) fn clear_abort() {
    MAILBOX.abort.store(false, Ordering::Release);
}

pub(super) fn abort_requested() -> bool {
    MAILBOX.abort.load(Ordering::Acquire)
}

pub(super) fn take_worker_command() -> Option<WorkerCommand> {
    let command = MAILBOX.command.swap(COMMAND_NONE, Ordering::AcqRel);
    if command == COMMAND_NONE {
        return None;
    }

    let sequence = MAILBOX.active_sequence.load(Ordering::Acquire);
    MAILBOX.state.store(STATE_BUSY, Ordering::Relaxed);

    Some(match command {
        COMMAND_RESET => WorkerCommand::Reset {
            sequence,
            tck_period_us: MAILBOX.tck_period_us.load(Ordering::Acquire),
        },
        COMMAND_SHIFT => WorkerCommand::Shift {
            sequence,
            tck_period_us: MAILBOX.tck_period_us.load(Ordering::Acquire),
            bit_count: MAILBOX.bit_count.load(Ordering::Acquire) as usize,
            tms_ptr: MAILBOX.tms_ptr.load(Ordering::Acquire) as *const u8,
            tdi_ptr: MAILBOX.tdi_ptr.load(Ordering::Acquire) as *const u8,
            tdo_ptr: MAILBOX.tdo_ptr.load(Ordering::Acquire) as *mut u8,
        },
        _ => WorkerCommand::Invalid { sequence },
    })
}

pub(super) fn complete(sequence: u32, succeeded: bool) {
    MAILBOX.state.store(
        if succeeded { STATE_DONE } else { STATE_ERROR },
        Ordering::Relaxed,
    );
    MAILBOX
        .completed_sequence
        .store(sequence, Ordering::Release);
    COMPLETION.signal(sequence);
}
