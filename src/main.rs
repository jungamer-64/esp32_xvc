//! ESP32 XVC server firmware entrypoint.

#![feature(asm_experimental_arch)]
#![no_std]
#![no_main]

extern crate alloc;

mod app;
mod config;
mod jtag;
mod logging;
mod network;
mod xvc;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    let hal_config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    app::run(spawner, esp_hal::init(hal_config)).await
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    esp_println::println!("panic: {info}");
    esp_hal::system::software_reset()
}
