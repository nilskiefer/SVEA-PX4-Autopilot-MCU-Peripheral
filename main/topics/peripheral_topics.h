#pragma once

#include <stdint.h>

#include "peripheral_context.h"

// uORB-aligned payload for PX4 topic `wheel_distance`.
typedef struct __attribute__((packed)) {
    uint32_t sequence;
    uint32_t time_ms;
    float left_distance_m;
    float right_distance_m;
} peripheral_topic_wheel_distance_t;

// uORB-aligned payload for PX4 topic `wheel_encoders`.
typedef struct __attribute__((packed)) {
    uint32_t sequence;
    uint32_t time_ms;
    float right_wheel_speed_rad_s;
    float left_wheel_speed_rad_s;
    float right_wheel_angle_rad;
    float left_wheel_angle_rad;
} peripheral_topic_wheel_encoders_t;

int peripheral_topic_publish_wheel_distance(peripheral_context_t *ctx, const peripheral_topic_wheel_distance_t *msg);
int peripheral_topic_publish_wheel_encoders(peripheral_context_t *ctx, const peripheral_topic_wheel_encoders_t *msg);
