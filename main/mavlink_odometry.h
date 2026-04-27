#pragma once

#include <stdint.h>

typedef struct {
    uint64_t time_usec;
    uint8_t reset_counter;
    float linear_mps;
    float yaw_rate_rps;
} mavlink_odometry_sample_t;

int mavlink_odometry_send_uart(
    int uart_num,
    uint8_t seq,
    uint8_t sysid,
    uint8_t compid,
    const mavlink_odometry_sample_t *sample);
