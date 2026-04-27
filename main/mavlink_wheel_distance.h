#pragma once

#include <stdint.h>

typedef struct {
    uint64_t time_usec;
    double left_distance_m;
    double right_distance_m;
} mavlink_wheel_distance_sample_t;

int mavlink_wheel_distance_send_uart(
    int uart_num,
    uint8_t sysid,
    uint8_t compid,
    const mavlink_wheel_distance_sample_t *sample);
