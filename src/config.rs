//! Compile-time firmware configuration boundary.

#[derive(Clone, Copy)]
pub(crate) struct Ipv4Address([u8; 4]);

impl Ipv4Address {
    pub(crate) const fn octets(self) -> [u8; 4] {
        self.0
    }
}

#[derive(Clone, Copy)]
pub(crate) struct WifiCredentials {
    ssid: &'static str,
    password: &'static str,
}

impl WifiCredentials {
    pub(crate) const fn ssid(self) -> &'static str {
        self.ssid
    }

    pub(crate) const fn password(self) -> &'static str {
        self.password
    }
}

#[derive(Clone, Copy)]
pub(crate) struct NetworkConfig {
    credentials: WifiCredentials,
    address: Ipv4Address,
    gateway: Ipv4Address,
    subnet_prefix: u8,
}

impl NetworkConfig {
    pub(crate) const fn credentials(self) -> WifiCredentials {
        self.credentials
    }

    pub(crate) const fn address(self) -> Ipv4Address {
        self.address
    }

    pub(crate) const fn gateway(self) -> Ipv4Address {
        self.gateway
    }

    pub(crate) const fn subnet_prefix(self) -> u8 {
        self.subnet_prefix
    }
}

pub(crate) const NETWORK: NetworkConfig = NetworkConfig {
    credentials: WifiCredentials {
        ssid: env!("WIFI_SSID"),
        password: env!("WIFI_PASSWORD"),
    },
    address: Ipv4Address(parse_ipv4(env!("STATIC_IP"))),
    gateway: Ipv4Address(parse_ipv4(env!("GATEWAY_IP"))),
    subnet_prefix: 24,
};

const fn parse_ipv4(input: &str) -> [u8; 4] {
    let bytes = input.as_bytes();
    let mut output = [0_u8; 4];
    let mut octet_index = 0_usize;
    let mut value = 0_u16;
    let mut has_digit = false;
    let mut index = 0_usize;

    while index < bytes.len() {
        let byte = bytes[index];
        if byte == b'.' {
            assert!(has_digit, "bad ip: missing digit before dot");
            assert!(octet_index < 3, "bad ip: too many octets");
            output[octet_index] = value as u8;
            octet_index += 1;
            value = 0;
            has_digit = false;
        } else if byte >= b'0' && byte <= b'9' {
            has_digit = true;
            value = value * 10 + (byte - b'0') as u16;
            assert!(value <= 255, "bad ip: octet greater than 255");
        } else {
            panic!("bad ip: invalid character");
        }
        index += 1;
    }

    assert!(has_digit, "bad ip: missing final digit");
    assert!(octet_index == 3, "bad ip: not four octets");
    output[octet_index] = value as u8;
    output
}
