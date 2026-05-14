#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "peripheral_context.h"

typedef struct {
    peripheral_context_t *ctx;
    portMUX_TYPE ticks_lock;
    volatile int32_t left_ticks;
    volatile int32_t right_ticks;
    volatile int32_t physical_left_ticks;
    volatile int32_t physical_right_ticks;
    volatile uint32_t rejected_left_edges;
    volatile uint32_t rejected_right_edges;
    volatile uint32_t last_left_edge_us;
    volatile uint32_t last_right_edge_us;
    uint32_t edge_min_interval_us;
    uint32_t emulate_start_ms;
    uint32_t last_debug_ms;
    uint32_t sequence;
    int32_t prev_left_ticks;
    int32_t prev_right_ticks;
    int32_t prev_activity_left_ticks;
    int32_t prev_activity_right_ticks;
    float emulate_left_frac_ticks;
    float emulate_right_frac_ticks;
    bool use_emulation;
    bool force_emulation;
} encoder_module_t;

void encoder_module_init(encoder_module_t *module, peripheral_context_t *ctx);
void encoder_module_start(encoder_module_t *module);
