//! Firmware composition root.

use esp_hal::{peripherals::Peripherals, timer::timg::TimerGroup};
use esp_println::println;

use crate::{
    config,
    jtag::{JtagHardware, JtagService},
    network::Network,
    runtime::{self, Clock},
    xvc::XvcServer,
};

pub(crate) fn run(peripherals: Peripherals) -> ! {
    println!("ESP32 XVC Server v1.2 (Stable)");
    runtime::init_heap();
    let clock = Clock::start();

    let timer_group = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group.timer0);

    let mut jtag = JtagService::start(JtagHardware {
        tck: peripherals.GPIO18,
        tms: peripherals.GPIO23,
        tdi: peripherals.GPIO19,
        tdo: peripherals.GPIO34,
        cpu_control: peripherals.CPU_CTRL,
        software_interrupt: peripherals.SW_INTERRUPT,
    });
    println!("JTAG: TCK=18, TMS=23, TDI=19, TDO=34");

    let radio = esp_radio::init().expect("esp-radio failed");
    let mut network = Network::connect(&radio, peripherals.WIFI, config::NETWORK, &clock);
    let server = XvcServer::new();
    server.run(config::NETWORK, &mut network, &mut jtag, &clock)
}
