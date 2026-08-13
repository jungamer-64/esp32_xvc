//! Core1 JTAG bit-banging worker.

use esp_println::println;

use crate::runtime::{cooperative_delay_us, delay_ms, yield_now};

use super::{bytes_for_bits, ipc};

pub(super) fn run() -> ! {
    println!("[Core1] Ready");

    let mut idle_spins = 0_u32;
    loop {
        let Some(command) = ipc::take_worker_command() else {
            if idle_spins < 200 {
                yield_now();
                idle_spins += 1;
            } else {
                delay_ms(1);
            }
            continue;
        };
        idle_spins = 0;

        match command {
            ipc::WorkerCommand::Reset {
                sequence,
                tck_period_us,
            } => ipc::complete(sequence, reset_tap(tck_period_us)),
            ipc::WorkerCommand::Shift {
                sequence,
                tck_period_us,
                bit_count,
                tms_ptr,
                tdi_ptr,
                tdo_ptr,
            } => {
                let byte_count = bytes_for_bits(bit_count);

                // Core0 keeps these validated buffers borrowed until this
                // sequence is completed, so Core1 has exclusive access for the
                // duration of `shift_bits`.
                let succeeded = unsafe {
                    let tms = core::slice::from_raw_parts(tms_ptr, byte_count);
                    let tdi = core::slice::from_raw_parts(tdi_ptr, byte_count);
                    let tdo = core::slice::from_raw_parts_mut(tdo_ptr, byte_count);
                    shift_bits(tck_period_us, tms, tdi, tdo, bit_count)
                };
                ipc::complete(sequence, succeeded);
            }
            ipc::WorkerCommand::Invalid { sequence } => {
                println!("Core1: invalid command");
                ipc::complete(sequence, false);
            }
        }
    }
}

fn reset_tap(tck_period_us: u32) -> bool {
    let high_us = tck_period_us.div_ceil(2).max(1);
    let low_us = tck_period_us / 2;

    cooperative_delay_us(50);

    unsafe {
        gpio::set_high(gpio::TMS_MASK);
        gpio::set_low(gpio::TDI_MASK);
    }

    for cycle in 0..8 {
        if ipc::abort_requested() {
            return false;
        }

        unsafe { gpio::set_high(gpio::TCK_MASK) };
        cooperative_delay_us(high_us);
        unsafe { gpio::set_low(gpio::TCK_MASK) };
        cooperative_delay_us(low_us);

        if tck_period_us > 100 && cycle % 2 == 1 {
            yield_now();
        }
    }

    unsafe {
        gpio::set_low(gpio::TMS_MASK);
        gpio::set_high(gpio::TCK_MASK);
    }
    cooperative_delay_us(high_us);
    unsafe { gpio::set_low(gpio::TCK_MASK) };
    cooperative_delay_us(low_us);
    true
}

fn shift_bits(
    tck_period_us: u32,
    tms_bits: &[u8],
    tdi_bits: &[u8],
    tdo_out: &mut [u8],
    bit_count: usize,
) -> bool {
    let high_us = tck_period_us.div_ceil(2).max(1);
    let low_us = tck_period_us / 2;
    let sample_us = (high_us / 2).max(1);
    let rest_high_us = high_us.saturating_sub(sample_us);
    let approximate_bit_us = high_us + low_us + 4;
    let abort_check_bits = (1_000 / approximate_bit_us).clamp(1, 4_096) as usize;
    let yield_bits = (50_000 / approximate_bit_us).clamp(1, 32_768) as usize;
    let byte_count = bytes_for_bits(bit_count);

    let mut abort_counter = abort_check_bits;
    let mut yield_counter = yield_bits;
    let mut watchdog_counter = 0_u8;

    for byte_index in 0..byte_count {
        let mut tms_byte = tms_bits[byte_index];
        let mut tdi_byte = tdi_bits[byte_index];
        let mut tdo_byte = 0_u8;
        let bits_in_byte = if byte_index == byte_count - 1 {
            let remainder = bit_count & 7;
            if remainder == 0 { 8 } else { remainder }
        } else {
            8
        };

        for bit_index in 0..bits_in_byte {
            abort_counter -= 1;
            if abort_counter == 0 {
                abort_counter = abort_check_bits;
                if ipc::abort_requested() {
                    return false;
                }
            }

            yield_counter -= 1;
            if yield_counter == 0 {
                yield_counter = yield_bits;
                watchdog_counter = watchdog_counter.wrapping_add(1);
                if watchdog_counter & 7 == 0 {
                    delay_ms(1);
                } else {
                    yield_now();
                }
            }

            let tms_high = tms_byte & 1 != 0;
            let tdi_high = tdi_byte & 1 != 0;
            unsafe {
                let set_mask = (if tms_high { gpio::TMS_MASK } else { 0 })
                    | (if tdi_high { gpio::TDI_MASK } else { 0 });
                let clear_mask = (if tms_high { 0 } else { gpio::TMS_MASK })
                    | (if tdi_high { 0 } else { gpio::TDI_MASK });
                gpio::set_pins(set_mask, clear_mask);
                gpio::set_high(gpio::TCK_MASK);
            }

            cooperative_delay_us(sample_us);
            if unsafe { gpio::read_tdo() } {
                tdo_byte |= 1 << bit_index;
            }
            cooperative_delay_us(rest_high_us);

            unsafe { gpio::set_low(gpio::TCK_MASK) };
            cooperative_delay_us(low_us);
            tms_byte >>= 1;
            tdi_byte >>= 1;
        }

        tdo_out[byte_index] = tdo_byte;
    }

    true
}

pub(super) mod gpio {
    use core::{
        ptr::{read_volatile, write_volatile},
        sync::atomic::{Ordering, compiler_fence},
    };

    const GPIO_OUT_W1TS: *mut u32 = 0x3FF4_4008 as *mut u32;
    const GPIO_OUT_W1TC: *mut u32 = 0x3FF4_400C as *mut u32;
    const GPIO1_IN: *const u32 = 0x3FF4_4040 as *const u32;

    pub(super) const TCK_MASK: u32 = 1 << 18;
    pub(super) const TDI_MASK: u32 = 1 << 19;
    pub(super) const TMS_MASK: u32 = 1 << 23;
    const TDO_BIT: u32 = 34 - 32;

    #[inline(always)]
    pub(super) unsafe fn set_pins(set_mask: u32, clear_mask: u32) {
        unsafe {
            if clear_mask != 0 {
                write_volatile(GPIO_OUT_W1TC, clear_mask);
            }
            if set_mask != 0 {
                write_volatile(GPIO_OUT_W1TS, set_mask);
            }
            core::arch::asm!("memw", options(nostack, preserves_flags));
        }
        compiler_fence(Ordering::Release);
    }

    #[inline(always)]
    pub(super) unsafe fn set_high(mask: u32) {
        unsafe {
            write_volatile(GPIO_OUT_W1TS, mask);
            core::arch::asm!("memw", options(nostack, preserves_flags));
        }
    }

    #[inline(always)]
    pub(super) unsafe fn set_low(mask: u32) {
        unsafe {
            write_volatile(GPIO_OUT_W1TC, mask);
            core::arch::asm!("memw", options(nostack, preserves_flags));
        }
    }

    #[inline(always)]
    pub(in crate::jtag) unsafe fn read_tdo() -> bool {
        unsafe {
            core::arch::asm!("memw", options(nostack, preserves_flags));
            read_volatile(GPIO1_IN) & (1 << TDO_BIT) != 0
        }
    }
}
