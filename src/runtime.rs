//! ESP32 runtime services shared by the firmware subsystems.

use core::mem::MaybeUninit;

use esp_hal::{
    rom::ets_delay_us,
    time::{Duration, Instant},
};
use esp_println::println;
use esp_rtos::CurrentThreadHandle;
use static_cell::StaticCell;

const HEAP_SIZE: usize = 96 * 1024;

#[repr(align(8))]
struct AlignedHeapMemory {
    _bytes: [u8; HEAP_SIZE],
}

pub(crate) struct Clock {
    started_at: Instant,
}

impl Clock {
    pub(crate) fn start() -> Self {
        Self {
            started_at: Instant::now(),
        }
    }

    pub(crate) fn now_ms(&self) -> i64 {
        self.started_at.elapsed().as_millis() as i64
    }
}

pub(crate) fn init_heap() {
    static HEAP: StaticCell<MaybeUninit<AlignedHeapMemory>> = StaticCell::new();

    let heap = HEAP.init_with(MaybeUninit::uninit);
    let heap_ptr = heap.as_mut_ptr().cast::<u8>();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            heap_ptr,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

#[inline]
pub(crate) fn delay_ms(milliseconds: u64) {
    CurrentThreadHandle::get().delay(Duration::from_millis(milliseconds));
}

#[inline]
pub(crate) fn yield_now() {
    CurrentThreadHandle::get().delay(Duration::ZERO);
}

#[inline]
pub(crate) fn cooperative_delay_us(mut microseconds: u32) {
    if microseconds >= 2_000 {
        while microseconds > 0 {
            let chunk = microseconds.min(250);
            ets_delay_us(chunk);
            microseconds -= chunk;
            yield_now();
        }
    } else if microseconds > 0 {
        ets_delay_us(microseconds);
    }
}

pub(crate) fn reset() -> ! {
    unsafe { software_reset() };
    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("Panic: {:?}", info);
    reset()
}

// This original-ESP32 ROM routine performs a whole-device software reset.
unsafe extern "C" {
    fn software_reset();
}
