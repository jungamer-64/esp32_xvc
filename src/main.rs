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
mod runtime;
mod xvc;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let hal_config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    app::run(esp_hal::init(hal_config))
}
