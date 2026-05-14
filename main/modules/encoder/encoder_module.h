#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "peripheral_context.h"

typedef struct {
    peripheral_context_t *ctx;
    portMUX_TYPE ticks_lock;
    volatile int32_t left_ticks;
    volatile int32_t right_ticks;
    uint32_t emulate_start_ms;
    uint32_t sequence;
    int32_t prev_left_ticks;
    int32_t prev_right_ticks;
    float emulate_left_frac_ticks;
    float emulate_right_frac_ticks;
    bool use_emulation;
    bool force_emulation;
    int last_left_level;
    int last_right_level;
} encoder_module_t;

void encoder_module_init(encoder_module_t *module, peripheral_context_t *ctx);
void encoder_module_start(encoder_module_t *module);
