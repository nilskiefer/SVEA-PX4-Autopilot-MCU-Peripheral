#include "bridge_io.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "svea_common.h"
#include "svea_config.h"

void bridge_state_init(bridge_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->udp_sock = -1;
    state->encoder_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
}

uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void bridge_uart_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate = BRIDGE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    SVEA_CHECK(uart_driver_install(BRIDGE_UART_NUM, BRIDGE_UART_RX_BUF, BRIDGE_UART_TX_BUF, 0, NULL, 0));
    SVEA_CHECK(uart_param_config(BRIDGE_UART_NUM, &uart_cfg));
    SVEA_CHECK(uart_set_pin(BRIDGE_UART_NUM, BRIDGE_UART_TX_GPIO, BRIDGE_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(SVEA_TAG, "UART bridge up: uart=%d tx=%d rx=%d baud=%d",
             BRIDGE_UART_NUM, BRIDGE_UART_TX_GPIO, BRIDGE_UART_RX_GPIO, BRIDGE_UART_BAUDRATE);
}

void bridge_udp_init(bridge_state_t *state)
{
    state->udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (state->udp_sock < 0) {
        ESP_LOGE(SVEA_TAG, "socket() failed errno=%d", errno);
        abort();
    }

    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = 100000
    };
    if (setsockopt(state->udp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        ESP_LOGE(SVEA_TAG, "setsockopt(SO_RCVTIMEO) failed errno=%d", errno);
        abort();
    }

    struct sockaddr_in bind_addr = {0};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(MAVLINK_UDP_LISTEN_PORT);

    if (bind(state->udp_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(SVEA_TAG, "bind() failed errno=%d", errno);
        abort();
    }

    ESP_LOGI(SVEA_TAG, "UDP bridge listening on port %d", MAVLINK_UDP_LISTEN_PORT);
}

bool bridge_get_peer_addr(bridge_state_t *state, struct sockaddr_in *out)
{
    bool valid = false;

    if (xSemaphoreTake(state->peer_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(SVEA_TAG, "peer lock failed");
        abort();
    }

    if (state->udp_peer_valid) {
        *out = state->udp_peer_addr;
        valid = true;
    }

    if (xSemaphoreGive(state->peer_lock) != pdTRUE) {
        ESP_LOGE(SVEA_TAG, "peer unlock failed");
        abort();
    }

    return valid;
}

void bridge_set_peer_addr(bridge_state_t *state, const struct sockaddr_in *peer)
{
    if (xSemaphoreTake(state->peer_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(SVEA_TAG, "peer lock failed");
        abort();
    }

    const bool first_peer = !state->udp_peer_valid;
    state->udp_peer_addr = *peer;
    state->udp_peer_valid = true;

    if (xSemaphoreGive(state->peer_lock) != pdTRUE) {
        ESP_LOGE(SVEA_TAG, "peer unlock failed");
        abort();
    }

    if (first_peer) {
        const uint32_t ip = ntohl(peer->sin_addr.s_addr);
        ESP_LOGI(SVEA_TAG, "first UDP peer: %u.%u.%u.%u:%u",
                 (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF,
                 ntohs(peer->sin_port));
    }
}

void udp_to_uart_task(void *arg)
{
    bridge_state_t *state = (bridge_state_t *)arg;
    uint8_t buf[1472];

    for (;;) {
        struct sockaddr_in src = {0};
        socklen_t src_len = sizeof(src);

        ssize_t n = recvfrom(state->udp_sock, buf, sizeof(buf), 0, (struct sockaddr *)&src, &src_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            ESP_LOGE(SVEA_TAG, "recvfrom() failed errno=%d", errno);
            abort();
        }

        bridge_set_peer_addr(state, &src);
        if (xSemaphoreTake(state->uart_tx_lock, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(SVEA_TAG, "uart tx lock failed");
            abort();
        }
        int written = uart_write_bytes(BRIDGE_UART_NUM, (const char *)buf, n);
        if (xSemaphoreGive(state->uart_tx_lock) != pdTRUE) {
            ESP_LOGE(SVEA_TAG, "uart tx unlock failed");
            abort();
        }
        if (written != n) {
            ESP_LOGE(SVEA_TAG, "uart_write_bytes short write=%d expected=%d", written, (int)n);
            abort();
        }

        state->udp_to_uart_pkts++;
        state->udp_to_uart_bytes += (uint32_t)n;
    }
}

void uart_to_udp_task(void *arg)
{
    bridge_state_t *state = (bridge_state_t *)arg;
    uint8_t buf[512];

    for (;;) {
        int n = uart_read_bytes(BRIDGE_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n <= 0) {
            continue;
        }

        struct sockaddr_in peer = {0};
        if (!bridge_get_peer_addr(state, &peer)) {
            continue;
        }

        ssize_t sent = sendto(state->udp_sock, buf, n, 0, (struct sockaddr *)&peer, sizeof(peer));
        if (sent != n) {
            ESP_LOGE(SVEA_TAG, "sendto(uart->udp) failed sent=%d expected=%d errno=%d", (int)sent, n, errno);
            abort();
        }

        state->uart_to_udp_pkts++;
        state->uart_to_udp_bytes += (uint32_t)n;
    }
}

void bridge_stats_task(void *arg)
{
    bridge_state_t *state = (bridge_state_t *)arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        struct sockaddr_in peer = {0};
        const bool has_peer = bridge_get_peer_addr(state, &peer);

        if (has_peer) {
            const uint32_t ip = ntohl(peer.sin_addr.s_addr);
            ESP_LOGI(SVEA_TAG,
                     "stats: udp->uart pkts=%" PRIu32 " bytes=%" PRIu32
                     " | uart->udp pkts=%" PRIu32 " bytes=%" PRIu32
                     " | enc->px4 pkts=%" PRIu32 " | peer=%u.%u.%u.%u:%u",
                     state->udp_to_uart_pkts, state->udp_to_uart_bytes,
                     state->uart_to_udp_pkts, state->uart_to_udp_bytes,
                     state->encoder_to_px4_pkts,
                     (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF,
                     ntohs(peer.sin_port));
        } else {
            ESP_LOGI(SVEA_TAG,
                     "stats: udp->uart pkts=%" PRIu32 " bytes=%" PRIu32
                     " | uart->udp pkts=%" PRIu32 " bytes=%" PRIu32
                     " | enc->px4 pkts=%" PRIu32 " | peer=none",
                     state->udp_to_uart_pkts, state->udp_to_uart_bytes,
                     state->uart_to_udp_pkts, state->uart_to_udp_bytes,
                     state->encoder_to_px4_pkts);
        }
    }
}
