# SVEA-PX4-MODULE-ENCODER

ESP32-C6 firmware for SVEA encoder + PX4 Wi-Fi bridge.

## What Is Included

- ESP-IDF project for ESP32-C6 (`idf.py build` compatible)
- UART <-> UDP bridge for PX4 MAVLink
- Encoder edge counting on 2 GPIO inputs
- Encoder MAVLink `ODOMETRY` injection into PX4 UART

## Source Layout

- `main/main.c`: startup, Wi-Fi manager callbacks, task orchestration
- `main/bridge_io.c`: UART/UDP bridge and runtime stats
- `main/encoder.c`: GPIO ISR edge counting and wheel-kinematics computation
- `main/mavlink_odometry.c`: MAVLink `ODOMETRY` packaging via generated C library API
- `main/svea_config.h`: pin/baud/kinematics configuration macros
- `components/mavlink`: generated MAVLink C headers used for packing/transmission
- `components/mavlink_repo`: upstream MAVLink repo clone (message definitions + `pymavlink` generator)

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

Use a normal shell session (`zsh` or a new terminal tab). Do not run `source ./zsh` (that file does not exist).

Regenerate MAVLink headers used by this module:

```bash
PYTHONPATH=components/mavlink_repo .venv/bin/python components/mavlink_repo/pymavlink/tools/mavgen.py \
  --lang C --wire-protocol 2.0 \
  --output components/mavlink \
  components/mavlink_repo/message_definitions/v1.0/common.xml
```

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
- Publishes encoder-derived MAVLink `ODOMETRY` into PX4 UART at 50 Hz

## Wiring

PX4 side (Clicker4 STM32F7):
- `PA2` (`USART2_TX`) -> ESP32-C6 RX
- `PA3` (`USART2_RX`) <- ESP32-C6 TX
- Common GND
- 3.3V logic only

ESP32-C6 defaults in `main/svea_config.h`:
- UART TX GPIO `16` (`D6` on XIAO ESP32C6)
- UART RX GPIO `17` (`D7` on XIAO ESP32C6)
- Encoder GPIOs `4` (left), `5` (right)

Change these macros in `main/svea_config.h` if your board uses other pins.

## PX4 Setup

Use the USART2-mapped device as MAVLink serial endpoint (board-specific `/dev/ttySx`) from PX4 NSH:

```sh
mavlink start -d /dev/ttyS1 -b 921600 -m onboard -f
```

Verify link state:

```sh
mavlink status
```

Verify odometry messages are entering PX4 uORB (topic name depends on PX4 version/build):

```sh
listener vehicle_odometry 5
listener vehicle_visual_odometry 5
```

On QGroundControl/companion side connect UDP to:
- `10.10.0.1:14550` (when connected to provisioning AP)

## Encoder to PX4 Integration

Encoder ticks are processed on ESP32 and emitted as MAVLink `ODOMETRY` on the same UART already used for PX4 MAVLink input.

- Message: `ODOMETRY` (ID `331`, MAVLink 2 framing)
- Estimator type: `MAV_ESTIMATOR_TYPE_NAIVE`
- Velocity frame: `MAV_FRAME_BODY_FRD`
- Encoded velocity:
  - `vx = linear_mps`
  - `vy = 0`
  - `vz = 0`
  - `yawspeed = yaw_rate_rps`

Pose fields are intentionally published as `NaN` so PX4 treats this as velocity-only odometry input.

## Encoder Emulation (No Hardware Encoder Needed)

In `main/svea_config.h`:

- Set `ENCODER_EMULATION_ENABLE` to `1`
- Tune:
  - `ENCODER_EMU_LINEAR_MPS`
  - `ENCODER_EMU_YAW_RATE_RPS`

When enabled, GPIO ISR counting is disabled and synthetic wheel ticks are generated every publish period.  
These synthetic ticks go through the exact same pipeline as real ticks:

1. tick counters
2. wheel kinematics
3. MAVLink `ODOMETRY` packing
4. UART TX into PX4
