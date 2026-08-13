//! JTAG hardware ownership and synchronous Core1 execution service.

mod ipc;
mod worker;

use esp_hal::{
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    peripherals::{CPU_CTRL, GPIO18, GPIO19, GPIO23, GPIO34, SW_INTERRUPT},
    system::Stack,
    time::{Duration, Instant},
};
use esp_println::println;
use static_cell::ConstStaticCell;

use crate::{
    logging::xvc_log,
    runtime::{self, Clock},
};

pub(crate) const MIN_TCK_PERIOD_US: u32 = 1;
pub(crate) const MAX_TCK_PERIOD_US: u32 = 1_000;

const RESET_TIMEOUT_MS: i64 = 2_000;
const HARD_TIMEOUT_GRACE_MS: i64 = 2_000;

pub(crate) const fn bytes_for_bits(bit_count: usize) -> usize {
    bit_count.div_ceil(8)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum JtagError {
    Busy,
    WorkerFailed,
    InvalidShiftBuffers,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ShiftExecution {
    BufferedCommand,
    StreamChunk,
}

impl ShiftExecution {
    const fn timeout_multiplier(self) -> u64 {
        match self {
            Self::BufferedCommand => 2,
            Self::StreamChunk => 3,
        }
    }
}

pub(crate) struct JtagShift<'a> {
    execution: ShiftExecution,
    bit_count: usize,
    tms: &'a [u8],
    tdi: &'a [u8],
    tdo: &'a mut [u8],
}

impl<'a> JtagShift<'a> {
    pub(crate) fn new(
        execution: ShiftExecution,
        bit_count: usize,
        tms: &'a [u8],
        tdi: &'a [u8],
        tdo: &'a mut [u8],
    ) -> Result<Self, JtagError> {
        let byte_count = bytes_for_bits(bit_count);
        if bit_count > u32::MAX as usize
            || tms.len() != byte_count
            || tdi.len() != byte_count
            || tdo.len() != byte_count
        {
            return Err(JtagError::InvalidShiftBuffers);
        }

        Ok(Self {
            execution,
            bit_count,
            tms,
            tdi,
            tdo,
        })
    }
}

struct JtagPins {
    _tck: Output<'static>,
    _tms: Output<'static>,
    _tdi: Output<'static>,
    _tdo: Input<'static>,
}

pub(crate) struct JtagService {
    _pins: JtagPins,
    tck_period_us: u32,
}

pub(crate) struct JtagHardware {
    pub(crate) tck: GPIO18<'static>,
    pub(crate) tms: GPIO23<'static>,
    pub(crate) tdi: GPIO19<'static>,
    pub(crate) tdo: GPIO34<'static>,
    pub(crate) cpu_control: CPU_CTRL<'static>,
    pub(crate) software_interrupt: SW_INTERRUPT<'static>,
}

impl JtagService {
    pub(crate) fn start(hardware: JtagHardware) -> Self {
        let JtagHardware {
            tck,
            tms,
            tdi,
            tdo,
            cpu_control,
            software_interrupt,
        } = hardware;
        let output_config = OutputConfig::default();
        let input_config = InputConfig::default().with_pull(Pull::None);
        let tck = Output::new(tck, Level::Low, output_config);
        let tms = Output::new(tms, Level::Low, output_config);
        let tdi = Output::new(tdi, Level::Low, output_config);
        let tdo = Input::new(tdo, input_config);

        configure_tdo_input();
        #[cfg(feature = "tdo-diagnostic")]
        diagnostic::run(&tdo);

        static CORE1_STACK: ConstStaticCell<Stack<16_384>> = ConstStaticCell::new(Stack::new());
        let core1_stack = CORE1_STACK.take();
        let interrupts = SoftwareInterruptControl::new(software_interrupt);
        esp_rtos::start_second_core(
            cpu_control,
            interrupts.software_interrupt0,
            interrupts.software_interrupt1,
            core1_stack,
            || worker::run(),
        );

        println!("[Core0] XVC Server (Atomic IPC)");
        Self {
            _pins: JtagPins {
                _tck: tck,
                _tms: tms,
                _tdi: tdi,
                _tdo: tdo,
            },
            tck_period_us: MIN_TCK_PERIOD_US,
        }
    }

    pub(crate) fn restore_min_period(&mut self) {
        self.tck_period_us = MIN_TCK_PERIOD_US;
    }

    pub(crate) fn set_period_ns(&mut self, requested_ns: u32) -> u32 {
        self.tck_period_us = requested_ns
            .saturating_add(999)
            .checked_div(1_000)
            .unwrap_or(0)
            .clamp(MIN_TCK_PERIOD_US, MAX_TCK_PERIOD_US);
        self.period_ns()
    }

    pub(crate) fn period_ns(&self) -> u32 {
        self.tck_period_us.saturating_mul(1_000)
    }

    pub(crate) fn reset<F>(&mut self, clock: &Clock, mut poll: F) -> Result<(), JtagError>
    where
        F: FnMut() -> bool,
    {
        let command = ipc::publish_reset(self.tck_period_us).map_err(map_publish_error)?;
        let soft_deadline = clock.now_ms().saturating_add(RESET_TIMEOUT_MS);
        let hard_deadline = soft_deadline.saturating_add(HARD_TIMEOUT_GRACE_MS);
        let mut soft_timeout = false;

        while !command.is_complete() {
            if !poll() {
                ipc::request_abort();
            }

            let now = clock.now_ms();
            if now > soft_deadline && !soft_timeout {
                xvc_log!("Reset timeout, aborting");
                ipc::request_abort();
                soft_timeout = true;
            }
            if now > hard_deadline {
                println!("CRITICAL: Core1 stuck (RESET)");
                runtime::reset();
            }
            runtime::yield_now();
        }

        if command.succeeded() {
            Ok(())
        } else {
            Err(JtagError::WorkerFailed)
        }
    }

    pub(crate) fn shift<F>(&mut self, shift: JtagShift<'_>, mut poll: F) -> Result<(), JtagError>
    where
        F: FnMut() -> bool,
    {
        let command = ipc::publish_shift(
            self.tck_period_us,
            shift.bit_count,
            shift.tms.as_ptr(),
            shift.tdi.as_ptr(),
            shift.tdo.as_mut_ptr(),
        )
        .map_err(map_publish_error)?;

        let expected_ms = (shift.bit_count as u64)
            .saturating_mul(self.tck_period_us as u64)
            .div_ceil(1_000);
        let soft_deadline = Instant::now()
            + Duration::from_millis(
                expected_ms
                    .saturating_mul(shift.execution.timeout_multiplier())
                    .saturating_add(200),
            );
        let hard_deadline = soft_deadline + Duration::from_millis(HARD_TIMEOUT_GRACE_MS as u64);
        let mut soft_timeout = false;

        while !command.is_complete() {
            if !poll() {
                ipc::request_abort();
            }

            let now = Instant::now();
            if now > soft_deadline && !soft_timeout {
                xvc_log!("Core1 timeout ({} bits)", shift.bit_count);
                ipc::request_abort();
                soft_timeout = true;
            }
            if now > hard_deadline {
                println!("CRITICAL: Core1 stuck (SHIFT)");
                runtime::reset();
            }
            runtime::yield_now();
        }

        if !command.succeeded() {
            return Err(JtagError::WorkerFailed);
        }

        if !shift.bit_count.is_multiple_of(8) {
            let final_byte = shift.tdo.len() - 1;
            shift.tdo[final_byte] &= (1 << (shift.bit_count % 8)) - 1;
        }
        Ok(())
    }

    pub(crate) fn abort_and_wait(&self, clock: &Clock) {
        let Some(sequence) = ipc::active_sequence() else {
            return;
        };

        xvc_log!("Waiting for Core1 (seq={})...", sequence);
        ipc::request_abort();
        let deadline = clock.now_ms().saturating_add(HARD_TIMEOUT_GRACE_MS);
        while !ipc::sequence_completed(sequence) {
            if clock.now_ms() > deadline {
                println!("CRITICAL: Core1 stuck while aborting");
                runtime::reset();
            }
            runtime::yield_now();
        }
        ipc::clear_abort();
    }
}

fn map_publish_error(error: ipc::PublishError) -> JtagError {
    match error {
        ipc::PublishError::Busy => JtagError::Busy,
        ipc::PublishError::WorkerCommandOccupied => {
            println!("FATAL: Core1 command was not consumed");
            runtime::reset()
        }
    }
}

fn configure_tdo_input() {
    const IO_MUX_GPIO34: *mut u32 = 0x3FF4_9030 as *mut u32;
    const INPUT_ENABLE: u32 = 1 << 9;

    unsafe {
        let current = core::ptr::read_volatile(IO_MUX_GPIO34);
        if current & INPUT_ENABLE == 0 {
            core::ptr::write_volatile(IO_MUX_GPIO34, current | INPUT_ENABLE);
        }
    }
}

#[cfg(feature = "tdo-diagnostic")]
mod diagnostic {
    use esp_hal::{gpio::Input, rom::ets_delay_us};
    use esp_println::println;

    use super::worker::gpio;

    pub(super) fn run(tdo: &Input<'_>) {
        const GPIO1_IN: *const u32 = 0x3FF4_4040 as *const u32;
        const GPIO1_ENABLE: *const u32 = 0x3FF4_4020 as *const u32;

        println!("=== TDO Pin Diagnostic ===");
        unsafe {
            println!(
                "GPIO_IN1_REG     = 0x{:08x}",
                core::ptr::read_volatile(GPIO1_IN)
            );
            println!(
                "GPIO_ENABLE1_REG = 0x{:08x}",
                core::ptr::read_volatile(GPIO1_ENABLE)
            );
        }

        let mut high_count = 0_u32;
        for sample in 0..10 {
            let hal_level = tdo.level();
            let register_level = unsafe { gpio::read_tdo() };
            let raw_input = unsafe { core::ptr::read_volatile(GPIO1_IN) };
            println!(
                "TDO[{}]: HAL={:?}, REG={}, RAW_IN1=0x{:08x}, bit2={}",
                sample,
                hal_level,
                if register_level { "HIGH" } else { "LOW" },
                raw_input,
                (raw_input >> 2) & 1
            );
            high_count += u32::from(register_level);
            ets_delay_us(10_000);
        }

        println!("TDO: {}/10 readings were HIGH", high_count);
        if high_count == 0 {
            println!("WARNING: TDO is still LOW - check RTC_IO config!");
        } else {
            println!("SUCCESS: GPIO34 input is working");
        }
        println!("=========================");
    }
}
