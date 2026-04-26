# SVEA-PX4-MODULE-ENCODER

ESP32-C6 firmware for SVEA encoder + PX4 Wi-Fi bridge.

## What Is Included

- ESP-IDF project for ESP32-C6 (`idf.py build` compatible)
- UART <-> UDP bridge for PX4 MAVLink
- Encoder edge counting on 2 GPIO inputs
- Binary encoder telemetry UDP stream with CRC32

## Requirements

- VS Code + Dev Containers extension
- Docker Desktop
- ESP32-C6 board connected to host machine

## Quick Start

1. Open this folder in VS Code.
2. Run: `Dev Containers: Reopen in Container`.
3. Inside container terminal:

```bash
idf.py set-target esp32c6
idf.py build
```
mavlink start -d /dev/ttyS1 -b 921600 -m onboard -f on px4

Use a normal shell session (`zsh` or a new terminal tab). Do not run `source ./zsh` (that file does not exist).

## Flashing

### Recommended on macOS with Docker Desktop

Build inside the dev container:

```bash
idf.py set-target esp32c6
idf.py build
```

Flash from the host terminal (outside the dev container):

```bash
(cd build && python -m esptool --chip esp32c6 --port /dev/cu.usbmodem11201 --baud 460800 write-flash @flash_args)
```

Monitor serial output from host:

```bash
python -m serial.tools.miniterm /dev/cu.usbmodem11201 115200
```

or with PlatformIO monitor:

```bash
pio device monitor -p /dev/cu.usbmodem11201 -b 115200
```

### Direct USB port (Linux and Windows setups where passthrough works)

Use your board serial device directly from the environment that can access USB:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

or

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

On Windows use your COM port:

```powershell
idf.py -p COM3 flash monitor
```

## Configure Default Port (Optional)

Set `idf.port` in VS Code user/workspace settings to your preferred value:

- Linux direct USB: `/dev/ttyUSB0` or `/dev/ttyACM0`
- Windows direct USB: `COM3`
- macOS host flashing is done with `esptool` from host terminal

## Runtime Behavior

- Uses `esp32-wifi-manager` (captive portal + credential persistence in NVS):
  - Tries previously saved STA network first
  - If STA is unavailable, starts AP + captive portal for Wi-Fi setup
  - Default provisioning AP SSID: `SVEA-PX4-MODULE`
  - Default provisioning AP password: `sveabridge`
- Opens UDP port `14550` (MAVLink bridge):
  - UDP -> UART: forwards datagrams to PX4 UART
  - UART -> UDP: forwards PX4 MAVLink bytes back to last UDP sender
- Publishes encoder packets to UDP port `14660` (same peer IP as MAVLink sender)

## Wiring

PX4 side (Clicker4 STM32F7):
- `PA2` (`USART2_TX`) -> ESP32-C6 RX
- `PA3` (`USART2_RX`) <- ESP32-C6 TX
- Common GND
- 3.3V logic only

ESP32-C6 defaults in `main.c`:
- UART TX GPIO `16` (`D6` on XIAO ESP32C6)
- UART RX GPIO `17` (`D7` on XIAO ESP32C6)
- Encoder GPIOs `4` (left), `5` (right)

Change these macros in `main/main.c` if your board uses other pins.

## PX4 Setup

Use the USART2-mapped device as MAVLink serial endpoint (board-specific `/dev/ttySx`):

```sh
mavlink start -d /dev/ttyS1 -b 921600 -m onboard
```

On QGroundControl/companion side connect UDP to:
- `10.10.0.1:14550` (when connected to provisioning AP)

## Encoder Packet Format (UDP 14660)

Packed struct `encoder_packet_t` (little-endian) from `main.c`:
- `magic` (`0x434E4553`, "SENC")
- `version`
- `payload_len`
- `seq`
- `uptime_ms`
- `left_count`, `right_count`
- `left_delta`, `right_delta`
- `left_mps`, `right_mps`
- `linear_mps`
- `yaw_rate_rps`
- `crc32` (computed over all prior packet bytes)
