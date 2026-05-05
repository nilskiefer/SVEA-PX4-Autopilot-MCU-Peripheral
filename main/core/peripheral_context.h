#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    SemaphoreHandle_t uart_tx_lock;

    volatile uint32_t frames_tx;
    volatile uint32_t bytes_tx;
    volatile uint32_t wheel_distance_tx;
    volatile uint32_t wheel_encoders_tx;
} peripheral_context_t;

void peripheral_context_init(peripheral_context_t *ctx);
uint32_t peripheral_now_ms(void);
