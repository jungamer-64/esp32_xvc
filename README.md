# ESP32 XVC server

Bare-metal Xilinx Virtual Cable (XVC) server for the original dual-core ESP32.
It exposes the ESP32 JTAG pins over TCP port 2542 through Wi-Fi.

> [!CAUTION]
> **DO NOT REMOVE LARGE-SHIFT STREAMING OR REPLACE IT WITH A FIXED-BUFFER-ONLY
> IMPLEMENTATION.** Vivado may ignore the shift limit reported by `getinfo:`
> and send a request larger than the normal XVC receive buffer. Large-shift
> streaming is mandatory protocol behavior: it must retain or reconstruct TMS,
> execute JTAG incrementally with bounded RAM, return TDO incrementally, and
> preserve timeout, disconnect, abort, and Core0/Core1 synchronization behavior.

## Architecture

- `main` is only the firmware entrypoint; `app` composes the subsystems.
- `config` and `logging` own external configuration and diagnostics.
- `network` owns the async Wi-Fi recovery task and the fixed-IPv4
  `embassy-net` stack. XVC owns the only TCP socket.
- `jtag` exclusively owns the pins, Core1 worker, atomic mailbox, and unsafe
  pointer boundary. Sequence-tagged completion signals and an in-flight guard
  keep borrowed buffers unavailable until Core1 finishes or acknowledges an
  abort, including when an async operation is cancelled.
- `xvc` owns the async TCP session, wire decoder, buffered commands, explicit
  progress timeouts, and mandatory bounded-memory large-shift streaming.

Core0 runs Embassy on `esp-rtos`; Core1 remains dedicated to JTAG bit-banging.
The firmware uses `esp-radio` only through its async Wi-Fi interface and has no
direct `smoltcp` integration or synchronous compatibility path.

XVC code invokes JTAG through validated `reset` and `shift` operations; it does
not access GPIO registers or the Core1 mailbox directly.

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
cargo check --locked
cargo build --release --locked
cargo run --release
```

`cargo run` flashes the connected ESP32 and opens the serial monitor. Add
`--features xvc-log` when detailed XVC logging is needed:

```powershell
cargo run --release --features xvc-log
```

The verbose TDO register diagnostic is disabled during normal startup. Enable
it explicitly when diagnosing the GPIO34 input path:

```powershell
cargo run --release --features tdo-diagnostic
```

After the firmware reports that the server is ready, connect Vivado to
`TCP:<STATIC_IP>:2542`.
