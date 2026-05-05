# SVEA-PX4-MODULE-ENCODER

ESP32-C6 firmware for a pure UART encoder peripheral for PX4.

## What Is Included

- ESP-IDF project for ESP32-C6
- Encoder edge counting on 2 GPIO inputs
- Periodic wheel distance publishing to PX4 over UART using a dedicated framed protocol (CRC protected)

## Source Layout

- `main/main.c`: startup and task orchestration
- `main/bridge_io.c`: UART initialization and shared utility functions
- `main/encoder.c`: GPIO ISR edge counting and wheel-kinematics computation
- `main/peripheral_frame.c`: UART frame packing + CRC16
- `main/peripheral_frame.h`: peripheral frame contract
- `main/svea_config.h`: pin/baud/kinematics configuration macros

## Build

### In dev container / Docker Compose

```bash
docker compose run --rm idf zsh -lc "idf.py build"
```

### First-time target setup (or after changing target)

```bash
docker compose run --rm idf zsh -lc "idf.py set-target esp32c6"
```

## Flash

### Recommended (host esptool)

Build first:

```bash
docker compose run --rm idf zsh -lc "idf.py build"
```

Then flash from host using generated flash args:

```bash
cd build
python -m esptool --chip esp32c6 --port /dev/cu.usbmodem11201 --baud 460800 write-flash @flash_args
```

Replace port with your device path.

### Alternative (inside IDF environment with direct USB access)

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

(or `/dev/ttyACM0`, or `COMx` on Windows)

## Runtime Behavior

- No Wi-Fi, no UDP gateway, no captive portal
- UART-only peripheral behavior
- Sends framed wheel distance samples to PX4 at configured publish rate

Frame payload fields:

- `sequence` (`uint32`)
- `time_ms` (`uint32`)
- `left_distance_m` (`float32`)
- `right_distance_m` (`float32`)

## Wiring

PX4 side (Clicker4 STM32F7):
- `PB6` (`USART1_TX`, `/dev/ttyS0`) -> ESP32-C6 RX
- `PB7` (`USART1_RX`, `/dev/ttyS0`) <- ESP32-C6 TX
- Common GND
- 3.3V logic only

Do not use `PA2/PA3` for this peripheral; that port is used by the NSH serial console.

ESP32-C6 defaults in `main/svea_config.h`:
- UART TX GPIO `16` (`D6` on XIAO ESP32C6)
- UART RX GPIO `17` (`D7` on XIAO ESP32C6)
- Encoder GPIOs `4` (left), `5` (right)

## Encoder Emulation

`main/svea_config.h`:
- `ENCODER_EMULATION_ENABLE 1` to enable synthetic ticks for bringup.
