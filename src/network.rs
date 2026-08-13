//! Wi-Fi station and single-socket TCP transport.

use alloc::string::String;

use esp_hal::peripherals::WIFI;
use esp_println::println;
use esp_radio::{
    Controller as RadioController,
    wifi::{ClientConfig, Config as WifiConfig, ModeConfig, WifiController, WifiDevice},
};
use smoltcp::{
    iface::{Config as InterfaceConfig, Interface, SocketHandle, SocketSet, SocketStorage},
    socket::tcp::{Socket as TcpSocket, SocketBuffer, State as TcpState},
    time::Instant as SmolInstant,
    wire::{EthernetAddress, IpCidr, Ipv4Address, Ipv4Cidr},
};
use static_cell::ConstStaticCell;

use crate::{
    config::NetworkConfig,
    logging::xvc_log,
    runtime::{self, Clock},
};

const SOCKET_RX_BUFFER_SIZE: usize = 4_096;
const SOCKET_TX_BUFFER_SIZE: usize = 4_096;
const LINK_CHECK_INTERVAL_MS: i64 = 10_000;
const RECONNECT_COOLDOWN_MS: i64 = 5_000;
const MAX_RECONNECT_RETRIES: u32 = 10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ConnectionState {
    Closed,
    Listening,
    Established,
    Transitioning,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LinkEvent {
    Stable,
    SessionInvalidated,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum NetworkError {
    Disconnected,
    ListenFailed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WifiLinkState {
    Connected,
    Disconnected,
    Connecting {
        retry_count: u32,
        last_attempt_ms: i64,
    },
}

pub(crate) struct Network<'driver> {
    controller: WifiController<'driver>,
    device: WifiDevice<'driver>,
    interface: Interface,
    sockets: SocketSet<'static>,
    tcp_handle: SocketHandle,
    link_state: WifiLinkState,
    last_link_check_ms: i64,
}

impl<'driver> Network<'driver> {
    pub(crate) fn connect(
        radio: &'driver RadioController<'driver>,
        wifi: WIFI<'driver>,
        config: NetworkConfig,
        clock: &Clock,
    ) -> Self {
        let (mut controller, interfaces) =
            esp_radio::wifi::new(radio, wifi, WifiConfig::default()).expect("WiFi init failed");
        let credentials = config.credentials();
        let client_config = ClientConfig::default()
            .with_ssid(String::from(credentials.ssid()))
            .with_password(String::from(credentials.password()));

        controller
            .set_config(&ModeConfig::Client(client_config))
            .expect("WiFi config failed");
        controller.start().expect("WiFi start failed");
        println!("Connecting to {}...", credentials.ssid());
        loop {
            match controller.connect() {
                Ok(()) => {
                    println!("WiFi OK!");
                    break;
                }
                Err(error) => {
                    println!("Retry: {:?}", error);
                    runtime::delay_ms(2_000);
                }
            }
        }
        runtime::delay_ms(1_000);

        let mut device = interfaces.sta;
        let mac = esp_radio::wifi::sta_mac();
        println!(
            "MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
        );

        let interface_config = InterfaceConfig::new(EthernetAddress(mac).into());
        let mut interface = Interface::new(
            interface_config,
            &mut device,
            SmolInstant::from_millis(clock.now_ms()),
        );
        let address = config.address().octets();
        interface.update_ip_addrs(|addresses| {
            addresses
                .push(IpCidr::Ipv4(Ipv4Cidr::new(
                    Ipv4Address::new(address[0], address[1], address[2], address[3]),
                    config.subnet_prefix(),
                )))
                .expect("single static IP must fit");
        });
        let gateway = config.gateway().octets();
        interface
            .routes_mut()
            .add_default_ipv4_route(Ipv4Address::new(
                gateway[0], gateway[1], gateway[2], gateway[3],
            ))
            .expect("default route must fit");

        static SOCKET_STORAGE: ConstStaticCell<[SocketStorage<'static>; 1]> =
            ConstStaticCell::new([SocketStorage::EMPTY; 1]);
        static RX_BUFFER: ConstStaticCell<[u8; SOCKET_RX_BUFFER_SIZE]> =
            ConstStaticCell::new([0; SOCKET_RX_BUFFER_SIZE]);
        static TX_BUFFER: ConstStaticCell<[u8; SOCKET_TX_BUFFER_SIZE]> =
            ConstStaticCell::new([0; SOCKET_TX_BUFFER_SIZE]);

        let socket_storage = SOCKET_STORAGE.take();
        let mut sockets = SocketSet::new(&mut socket_storage[..]);
        let rx_buffer = SocketBuffer::new(&mut RX_BUFFER.take()[..]);
        let tx_buffer = SocketBuffer::new(&mut TX_BUFFER.take()[..]);
        let tcp_handle = sockets.add(TcpSocket::new(rx_buffer, tx_buffer));

        Self {
            controller,
            device,
            interface,
            sockets,
            tcp_handle,
            link_state: WifiLinkState::Connected,
            last_link_check_ms: clock.now_ms(),
        }
    }

    pub(crate) fn maintain_link(&mut self, clock: &Clock) -> LinkEvent {
        let now_ms = clock.now_ms();
        let mut event = LinkEvent::Stable;

        if now_ms.saturating_sub(self.last_link_check_ms) >= LINK_CHECK_INTERVAL_MS {
            self.last_link_check_ms = now_ms;
            let connected = self.controller.is_connected().unwrap_or(false);
            match self.link_state {
                WifiLinkState::Connected if !connected => {
                    xvc_log!("Wi-Fi disconnected, attempting recovery...");
                    self.link_state = WifiLinkState::Disconnected;
                    event = LinkEvent::SessionInvalidated;
                }
                WifiLinkState::Disconnected if connected => {
                    self.link_state = WifiLinkState::Connected;
                    xvc_log!("Wi-Fi link recovered externally");
                }
                WifiLinkState::Connecting { retry_count, .. } if connected => {
                    self.link_state = WifiLinkState::Connected;
                    xvc_log!("Wi-Fi connected after {} retries", retry_count);
                }
                _ => {}
            }
        }

        match self.link_state {
            WifiLinkState::Connected => {}
            WifiLinkState::Disconnected => {
                xvc_log!("Wi-Fi: reconnect attempt 1");
                self.link_state = match self.controller.connect() {
                    Ok(()) => {
                        xvc_log!("Wi-Fi reconnected!");
                        self.poll(clock);
                        WifiLinkState::Connected
                    }
                    Err(_) => WifiLinkState::Connecting {
                        retry_count: 1,
                        last_attempt_ms: now_ms,
                    },
                };
            }
            WifiLinkState::Connecting {
                retry_count,
                last_attempt_ms,
            } if now_ms.saturating_sub(last_attempt_ms) >= RECONNECT_COOLDOWN_MS => {
                if retry_count >= MAX_RECONNECT_RETRIES {
                    println!(
                        "Wi-Fi: {} retries failed, resetting device",
                        MAX_RECONNECT_RETRIES
                    );
                    runtime::reset();
                }

                let next_retry = retry_count + 1;
                xvc_log!("Wi-Fi: reconnect attempt {}", next_retry);
                self.link_state = match self.controller.connect() {
                    Ok(()) => {
                        xvc_log!("Wi-Fi reconnected!");
                        self.poll(clock);
                        WifiLinkState::Connected
                    }
                    Err(_) => WifiLinkState::Connecting {
                        retry_count: next_retry,
                        last_attempt_ms: now_ms,
                    },
                };
            }
            WifiLinkState::Connecting { .. } => {}
        }

        event
    }

    pub(crate) fn poll(&mut self, clock: &Clock) {
        self.interface.poll(
            SmolInstant::from_millis(clock.now_ms()),
            &mut self.device,
            &mut self.sockets,
        );
    }

    pub(crate) fn state(&mut self) -> ConnectionState {
        match self.socket().state() {
            TcpState::Closed => ConnectionState::Closed,
            TcpState::Listen => ConnectionState::Listening,
            TcpState::Established => ConnectionState::Established,
            _ => ConnectionState::Transitioning,
        }
    }

    pub(crate) fn is_open(&mut self) -> bool {
        self.socket().is_open()
    }

    pub(crate) fn listen(&mut self, port: u16) -> Result<(), NetworkError> {
        self.socket()
            .listen(port)
            .map_err(|_| NetworkError::ListenFailed)
    }

    pub(crate) fn disable_nagle(&mut self) {
        self.socket().set_nagle_enabled(false);
    }

    pub(crate) fn can_receive(&mut self) -> bool {
        self.socket().can_recv()
    }

    pub(crate) fn can_send(&mut self) -> bool {
        self.socket().can_send()
    }

    pub(crate) fn may_receive(&mut self) -> bool {
        self.socket().may_recv()
    }

    pub(crate) fn may_send(&mut self) -> bool {
        self.socket().may_send()
    }

    pub(crate) fn connection_alive(&mut self) -> bool {
        self.may_receive() || self.may_send()
    }

    pub(crate) fn receive(&mut self, output: &mut [u8]) -> Result<usize, NetworkError> {
        self.socket()
            .recv_slice(output)
            .map_err(|_| NetworkError::Disconnected)
    }

    pub(crate) fn send(&mut self, data: &[u8]) -> Result<usize, NetworkError> {
        self.socket()
            .send_slice(data)
            .map_err(|_| NetworkError::Disconnected)
    }

    pub(crate) fn close(&mut self) {
        self.socket().close();
    }

    pub(crate) fn abort(&mut self) {
        self.socket().abort();
    }

    fn socket(&mut self) -> &mut TcpSocket<'static> {
        self.sockets.get_mut(self.tcp_handle)
    }
}
