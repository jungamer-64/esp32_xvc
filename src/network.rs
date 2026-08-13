//! Wi-Fi ownership and Embassy network stack lifecycle.

use alloc::string::String;

use embassy_executor::Spawner;
use embassy_net::{Config, Ipv4Address, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use esp_hal::{peripherals::WIFI, rng::Rng};
use esp_println::println;
use esp_radio::wifi::{
    Config as WifiConfig, ControllerConfig, Interface, WifiController, sta::StationConfig,
};
use static_cell::StaticCell;

use crate::config::NetworkConfig;

const RECONNECT_ATTEMPTS: usize = 10;
const RECONNECT_DELAY: Duration = Duration::from_secs(5);

pub(crate) async fn start(
    spawner: Spawner,
    wifi: WIFI<'static>,
    config: NetworkConfig,
) -> Stack<'static> {
    let credentials = config.credentials();
    let station_config = StationConfig::default()
        .with_ssid(credentials.ssid())
        .with_password(String::from(credentials.password()));
    let controller_config =
        ControllerConfig::default().with_initial_config(WifiConfig::Station(station_config));
    let (mut controller, interfaces) =
        esp_radio::wifi::new(wifi, controller_config).expect("Wi-Fi initialization failed");

    connect_until_ready(&mut controller).await;
    let station = interfaces.station;
    println!(
        "Wi-Fi connected, station MAC={:02x?}",
        station.mac_address()
    );

    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let network_config = Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(
            Ipv4Address::from_octets(config.address().octets()),
            config.subnet_prefix(),
        ),
        gateway: Some(Ipv4Address::from_octets(config.gateway().octets())),
        dns_servers: Default::default(),
    });
    let rng = Rng::new();
    let seed = (u64::from(rng.random()) << 32) | u64::from(rng.random());
    let (stack, runner) = embassy_net::new(
        station,
        network_config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.spawn(network_runner(runner).expect("network runner task unavailable"));
    spawner.spawn(wifi_reconnector(controller).expect("Wi-Fi task unavailable"));
    stack.wait_config_up().await;
    println!(
        "Network ready: {}.{}.{}.{}/{}",
        config.address().octets()[0],
        config.address().octets()[1],
        config.address().octets()[2],
        config.address().octets()[3],
        config.subnet_prefix()
    );
    stack
}

async fn connect_until_ready(controller: &mut WifiController<'static>) {
    loop {
        match controller.connect_async().await {
            Ok(_) => return,
            Err(error) => {
                println!("Wi-Fi connection failed: {error:?}");
                Timer::after(RECONNECT_DELAY).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn wifi_reconnector(mut controller: WifiController<'static>) -> ! {
    loop {
        let disconnected = controller.wait_for_disconnect_async().await;
        println!("Wi-Fi disconnected: {disconnected:?}");

        for attempt in 1..=RECONNECT_ATTEMPTS {
            match controller.connect_async().await {
                Ok(_) => {
                    println!("Wi-Fi reconnected");
                    break;
                }
                Err(error) if attempt < RECONNECT_ATTEMPTS => {
                    println!("Wi-Fi reconnect {attempt}/{RECONNECT_ATTEMPTS} failed: {error:?}");
                    Timer::after(RECONNECT_DELAY).await;
                }
                Err(error) => {
                    println!("Wi-Fi reconnect exhausted: {error:?}");
                    esp_hal::system::software_reset();
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn network_runner(mut runner: Runner<'static, Interface<'static>>) -> ! {
    runner.run().await
}
