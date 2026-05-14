#pragma once

#include "driver/uart.h"

/* ESP32-C6 UART connected to PX4 USART1 (/dev/ttyS0):
 *   PX4 PB6 (USART1_TX) -> ESP RX
 *   PX4 PB7 (USART1_RX) <- ESP TX
 */
#define BRIDGE_UART_NUM UART_NUM_1
#define BRIDGE_UART_TX_GPIO 16 /* XIAO D6 */
#define BRIDGE_UART_RX_GPIO 17 /* XIAO D7 */
#define BRIDGE_UART_BAUDRATE 115200
#define BRIDGE_UART_RX_BUF 4096
#define BRIDGE_UART_TX_BUF 4096

#define ENCODER_LEFT_GPIO 22  /* XIAO D4 / SDA */
#define ENCODER_RIGHT_GPIO 23 /* XIAO D5 / SCL */
#define ENCODER_TICKS_PER_REV 80.0f
#define ENCODER_WHEEL_DIAM_M 0.115f
#define ENCODER_WHEELBASE_M 0.32f
#define ENCODER_PUBLISH_MS 20
#define ENCODER_SPEED_SCALE 0.93f
#define ENCODER_MIN_SPEED_MPS 0.01f
#define ENCODER_MAX_SPEED_MPS 5.0f

/* Set to 1 to force emulation and ignore runtime GPIO activity detection. */
#define ENCODER_EMULATION_ENABLE 0
/* Probe window at boot before deciding emulation mode. */
#define ENCODER_ACTIVITY_PROBE_MS 300
/* While emulating, switch to real encoder mode if pin activity is detected. */
#define ENCODER_AUTO_SWITCH_TO_REAL_ENABLE 1
/* Simple band-pass plausibility filter for encoder-derived wheel speed. */
#define ENCODER_EDGE_FILTER_ENABLE 1
/* Log raw encoder GPIO levels and counters once per second. */
#define ENCODER_DEBUG_LOG_ENABLE 1
#define ENCODER_DEBUG_LOG_MS 1000
/* Emulation motion profile is intentionally hardcoded in encoder.c for
 * communication/integration bringup validation.
 */
