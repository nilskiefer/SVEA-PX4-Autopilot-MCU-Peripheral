#pragma once

#include <stdint.h>

typedef struct {
    uint32_t sequence;
    uint32_t time_ms;
    float left_distance_m;
    float right_distance_m;
} peripheral_wheel_distance_sample_t;

int peripheral_wheel_distance_send_uart(int uart_num, const peripheral_wheel_distance_sample_t *sample);
