# SVEA-PX4-Autopilot-MCU-Peripheral

ESP32-C6 firmware for a SVEA peripheral MCU that publishes PX4-uORB-aligned data over UART.

The current implemented example module is `encoder` (publishes wheel topics).

## Architecture

The project is intentionally split into layers:

- `main/core/`
  - Runtime context and shared counters/locks.
  - `peripheral_context.[ch]`
- `main/transport/`
  - Raw UART setup and byte writes only.
  - `peripheral_uart.[ch]`
- `main/protocol/`
  - Frame format, CRC, and topic-id multiplexing.
  - `peripheral_protocol.[ch]`
- `main/topics/`
  - uORB-aligned topic payload APIs (typed publish helpers).
  - `peripheral_topics.[ch]`
- `main/modules/`
  - Feature modules that produce data.
  - `encoder/encoder_module.[ch]`
- `main/main.c`
  - Boot, wiring, and module startup.

This keeps modules independent from framing/UART details.

## Current Wire Protocol (compatible with PX4 `svea_peripheral_mcu`)

Frame bytes:

- `magic0` = `0x53` (`'S'`)
- `magic1` = `0x45` (`'E'`)
- `version` = `2`
- `topic_id` = `uint8`
- `payload_len` = `uint8`
- `payload[payload_len]`
- `crc16_ccitt` over `[version, topic_id, payload_len, payload...]` (little-endian in frame)

Current topic IDs:

- `1` -> `wheel_distance`
- `2` -> `wheel_encoders`

## Encoder Example (Reference Module)

`modules/encoder/encoder_module.c` shows the expected pattern for adding modules:

1. Collect or synthesize sensor data.
2. Populate a typed payload struct from `topics/peripheral_topics.h`.
3. Publish through `peripheral_topic_publish_*()`.
4. Fail hard on publish errors.

It publishes both:

- `wheel_distance` payload (`sequence`, `time_ms`, left/right distance)
- `wheel_encoders` payload (`sequence`, `time_ms`, wheel speeds/angles)

## How To Add A New Module

1. Create `main/modules/<name>/<name>_module.[ch]`.
2. Add topic payload + publish function to `main/topics/peripheral_topics.[ch]`.
3. Assign a new `topic_id` in `main/protocol/peripheral_protocol.h`.
4. Use `peripheral_protocol_send()` only from topic-layer helpers.
5. Start module from `main/main.c`.
6. Add new source file(s) in `main/CMakeLists.txt`.

Note: This repo is protocol/uORB-aligned on MCU side. PX4 still needs matching decode+publish mapping for any brand-new topic class.

## Build

### Docker Compose

```bash
docker compose run --rm idf zsh -lc "idf.py build"
```


## Flash

Build first:

```bash
docker compose run --rm idf zsh -lc "idf.py build"
```

Create a venv for python and run (this can be done via `F1` + `Select Interpreter`)
```bash
pip install esptool
```

Then flash from host:

```bash
cd build
python -m esptool --chip esp32c6 --port /dev/serial/by-id/*Espressif* --baud 460800 write-flash @flash_args
cd ..
```



##### May be needed, set target (first time)

```bash
docker compose run --rm idf zsh -lc "idf.py set-target esp32c6"
```


## Monitor 
```bash
pio device monitor --port /dev/serial/by-id/*Espressif* --baud 115200
```

## Wiring (PX4 Clicker4 STM32F7)

- PX4 `PB6` (`USART1_TX`, `/dev/ttyS0`) -> ESP RX (`D7` / GPIO17)
- PX4 `PB7` (`USART1_RX`, `/dev/ttyS0`) <- ESP TX (`D6` / GPIO16)
- Common GND
- 3.3V logic only

Do not use `PA2/PA3` for this peripheral link (NSH console path).

## Configuration

See `main/svea_config.h`:

- UART config (port/pins/baud/buffer sizes)
- Encoder GPIO and kinematic constants
- `ENCODER_EMULATION_ENABLE` (synthetic encoder ticks for bring-up)
