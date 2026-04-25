# SVEA-PX4-MODULE-ENCODER

ESP32-C6 module workspace for SVEA encoder + PX4 bridge development.

This repository is intentionally focused on environment setup first.

## What Is Included

- ESP-IDF project scaffold (`idf.py build` compatible)
- VS Code Dev Container for ESP-IDF
- `zsh` configured inside the container
- Cross-platform flashing workflow (Linux/macOS/Windows)

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

## Notes

- No MAVLink functionality is implemented yet.
- Next step after environment validation: add UART transport and encoder task skeleton.
