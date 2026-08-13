//! Firmware composition root.

use embassy_executor::Spawner;
use esp_hal::{
    interrupt::software::SoftwareInterruptControl, peripherals::Peripherals, ram,
    timer::timg::TimerGroup,
};
use esp_println::println;

use crate::{
    config,
    jtag::{JtagHardware, JtagService},
    xvc::XvcServer,
};

pub(crate) async fn run(spawner: Spawner, peripherals: Peripherals) -> ! {
    println!("ESP32 XVC Server v1.2 (Stable)");
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1_024);
    esp_alloc::heap_allocator!(size: 32 * 1_024);

    let timer_group = TimerGroup::new(peripherals.TIMG0);
    let software_interrupts = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timer_group.timer0, software_interrupts.software_interrupt0);

    let jtag = JtagService::start(JtagHardware {
        tck: peripherals.GPIO18,
        tms: peripherals.GPIO23,
        tdi: peripherals.GPIO19,
        tdo: peripherals.GPIO34,
        cpu_control: peripherals.CPU_CTRL,
        software_interrupt: software_interrupts.software_interrupt1,
    });
    println!("JTAG: TCK=18, TMS=23, TDI=19, TDO=34");

    let stack = crate::network::start(spawner, peripherals.WIFI, config::NETWORK).await;
    let server = XvcServer::new();
    server.run(stack, jtag).await
}
