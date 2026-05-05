#pragma once

#include <stddef.h>
#include <stdint.h>

#include "peripheral_context.h"

void peripheral_uart_init(void);
int peripheral_uart_write(peripheral_context_t *ctx, const uint8_t *data, size_t len);
