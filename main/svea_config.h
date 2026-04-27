#pragma once

#include "driver/uart.h"

/* ESP32-C6 UART connected to PX4 USART2:
 *   PX4 PA2 (TX) -> ESP RX
 *   PX4 PA3 (RX) <- ESP TX
 */
#define BRIDGE_UART_NUM             UART_NUM_1
#define BRIDGE_UART_TX_GPIO         16  /* XIAO D6 */
#define BRIDGE_UART_RX_GPIO         17  /* XIAO D7 */
#define BRIDGE_UART_BAUDRATE        921600
#define BRIDGE_UART_RX_BUF          4096
#define BRIDGE_UART_TX_BUF          4096

#define MAVLINK_UDP_LISTEN_PORT     14550

#define ENCODER_LEFT_GPIO           4
#define ENCODER_RIGHT_GPIO          5
#define ENCODER_TICKS_PER_REV       80.0f
#define ENCODER_WHEEL_DIAM_M        0.115f
#define ENCODER_WHEELBASE_M         0.32f
#define ENCODER_PUBLISH_MS          20
#define ENCODER_SPEED_SCALE         0.93f

/* Set to 1 to bypass GPIO encoder edges and inject synthetic wheel ticks. */
#define ENCODER_EMULATION_ENABLE    0
/* Target kinematics for synthetic encoder data when emulation is enabled. */
#define ENCODER_EMU_LINEAR_MPS      0.60f
#define ENCODER_EMU_YAW_RATE_RPS    0.35f

/* MAVLink identity of encoder source injected towards PX4. */
#define ENCODER_MAV_SYS_ID          42
#define ENCODER_MAV_COMP_ID         197 /* MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY */
