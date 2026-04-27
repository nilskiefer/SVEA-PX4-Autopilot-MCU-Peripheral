#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bridge_state.h"

void bridge_state_init(bridge_state_t *state);
void bridge_uart_init(void);
void bridge_udp_init(bridge_state_t *state);

bool bridge_get_peer_addr(bridge_state_t *state, struct sockaddr_in *out);
void bridge_set_peer_addr(bridge_state_t *state, const struct sockaddr_in *peer);

void udp_to_uart_task(void *arg);
void uart_to_udp_task(void *arg);
void bridge_stats_task(void *arg);

uint32_t now_ms(void);
