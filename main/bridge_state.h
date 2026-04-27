#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"

typedef struct {
    int udp_sock;

    SemaphoreHandle_t peer_lock;
    SemaphoreHandle_t uart_tx_lock;
    bool udp_peer_valid;
    struct sockaddr_in udp_peer_addr;

    portMUX_TYPE encoder_lock;
    volatile int32_t left_ticks;
    volatile int32_t right_ticks;

    volatile uint32_t udp_to_uart_pkts;
    volatile uint32_t udp_to_uart_bytes;
    volatile uint32_t uart_to_udp_pkts;
    volatile uint32_t uart_to_udp_bytes;
    volatile uint32_t uart_to_udp_drop_no_peer_pkts;
    volatile uint32_t uart_to_udp_drop_no_peer_bytes;
    volatile uint32_t uart_mavlink_frames;
    volatile uint32_t uart_mavlink_heartbeats;
    volatile uint32_t encoder_to_px4_pkts;

    uint32_t last_waiting_peer_log_ms;
    uint32_t last_no_peer_drop_log_ms;
} bridge_state_t;
