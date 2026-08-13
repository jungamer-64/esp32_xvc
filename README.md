# ESP32 XVC server

Bare-metal Xilinx Virtual Cable (XVC) server for the original dual-core ESP32.
It exposes the ESP32 JTAG pins over TCP port 2542 through Wi-Fi.

## Hardware

The firmware is intentionally specific to the original Xtensa ESP32 and uses
direct ESP32 register access. The JTAG pin mapping is:

| Signal | ESP32 GPIO |
| --- | ---: |
| TCK | 18 |
| TMS | 23 |
| TDI | 19 |
| TDO | 34 |

Connect ESP32 ground to the target ground. GPIO34 is input-only, which is
appropriate for TDO.

## Prerequisites

Install Rust, then install the Espressif Xtensa toolchain and flashing tool:

```powershell
cargo install espup --locked
espup install
cargo install espflash --locked
```

The project selects the `esp` toolchain and `xtensa-esp32-none-elf` target
automatically.

## Configuration

Set the build-time network values in the shell. Shell values override the safe
placeholder defaults in `.cargo/config.toml`:

```powershell
$env:WIFI_SSID = "your-ssid"
$env:WIFI_PASSWORD = "your-password"
$env:STATIC_IP = "192.168.1.100"
$env:GATEWAY_IP = "192.168.1.1"
```

The subnet prefix is currently fixed at `/24` in the firmware. Do not commit
real Wi-Fi credentials.

## Build and flash

```powershell
cargo check
cargo build --release
cargo run --release
```

`cargo run` flashes the connected ESP32 and opens the serial monitor. Add
`--features xvc-log` when detailed XVC logging is needed:

```powershell
cargo run --release --features xvc-log
```

After the firmware reports that the server is ready, connect Vivado to
`TCP:<STATIC_IP>:2542`.

Large-shift streaming is required for Vivado compatibility. It is a core
protocol behavior, not an optional optimization.
