#pragma once

#include "bridge_state.h"

void encoder_gpio_init(bridge_state_t *state);
void encoder_publish_task(void *arg);
