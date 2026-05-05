#pragma once

#include <stdint.h>

typedef enum {
    PERIPHERAL_MSG_ID_WHEEL_DISTANCE = 1,
    PERIPHERAL_MSG_ID_WHEEL_ENCODERS = 2,
} peripheral_msg_id_t;

typedef struct {
    uint32_t sequence;
    uint32_t time_ms;
    float left_distance_m;
    float right_distance_m;
} peripheral_wheel_distance_sample_t;

typedef struct {
    uint32_t sequence;
    uint32_t time_ms;
    float right_wheel_speed_rad_s;
    float left_wheel_speed_rad_s;
    float right_wheel_angle_rad;
    float left_wheel_angle_rad;
} peripheral_wheel_encoders_sample_t;

int peripheral_wheel_distance_send_uart(int uart_num, const peripheral_wheel_distance_sample_t *sample);
int peripheral_wheel_encoders_send_uart(int uart_num, const peripheral_wheel_encoders_sample_t *sample);
