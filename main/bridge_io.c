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

#define WAITING_FOR_PEER_LOG_INTERVAL_MS 5000U
#define DROP_NO_PEER_LOG_INTERVAL_MS 2000U

typedef struct {
    uint8_t stx;
    uint8_t payload_len;
    uint8_t incompat_flags;
    uint8_t header_pos;
    uint8_t bytes_remaining;
    uint8_t sysid;
    uint8_t compid;
    uint32_t msgid;
    bool in_frame;
} mavlink_sniffer_t;

static bool mavlink_sniffer_feed(mavlink_sniffer_t *sniffer, uint8_t b, uint32_t *msgid, uint8_t *sysid, uint8_t *compid)
{
    const uint8_t MAVLINK_STX_V1 = 0xFE;
    const uint8_t MAVLINK_STX_V2 = 0xFD;
    const uint8_t MAVLINK_IFLAG_SIGNED = 0x01;

    if (!sniffer->in_frame) {
        if (b == MAVLINK_STX_V1 || b == MAVLINK_STX_V2) {
            memset(sniffer, 0, sizeof(*sniffer));
            sniffer->in_frame = true;
            sniffer->stx = b;
        }
        return false;
    }

    if (sniffer->header_pos == 0) {
        sniffer->payload_len = b;
        sniffer->header_pos++;
        return false;
    }

    if (sniffer->stx == MAVLINK_STX_V2) {
        if (sniffer->header_pos == 1) {
            sniffer->incompat_flags = b;
            sniffer->header_pos++;
            return false;
        }
        if (sniffer->header_pos == 2) {
            sniffer->header_pos++; /* compat_flags */
            return false;
        }
        if (sniffer->header_pos == 3) {
            sniffer->header_pos++; /* seq */
            return false;
        }
        if (sniffer->header_pos == 4) {
            sniffer->sysid = b;
            sniffer->header_pos++;
            return false;
        }
        if (sniffer->header_pos == 5) {
            sniffer->compid = b;
            sniffer->header_pos++;
            return false;
        }
        if (sniffer->header_pos == 6) {
            sniffer->msgid = (uint32_t)b;
            sniffer->header_pos++;
            return false;
        }
        if (sniffer->header_pos == 7) {
            sniffer->msgid |= ((uint32_t)b << 8);
            sniffer->header_pos++;
            return false;
        }
        if (sniffer->header_pos == 8) {
            sniffer->msgid |= ((uint32_t)b << 16);
            sniffer->header_pos++;
            sniffer->bytes_remaining = sniffer->payload_len + 2 +
                                       ((sniffer->incompat_flags & MAVLINK_IFLAG_SIGNED) ? 13 : 0);
            if (sniffer->bytes_remaining == 0) {
                sniffer->in_frame = false;
                return false;
            }
            return false;
        }
    } else {
        /* MAVLink v1 header after STX: len, seq, sysid, compid, msgid */
        if (sniffer->header_pos == 1) {
            sniffer->header_pos++; /* seq */
            return false;
        }
        if (sniffer->header_pos == 2) {
            sniffer->sysid = b;
            sniffer->header_pos++;
            return false;
        }
        if (sniffer->header_pos == 3) {
            sniffer->compid = b;
            sniffer->header_pos++;
            return false;
        }
        if (sniffer->header_pos == 4) {
            sniffer->msgid = (uint32_t)b;
            sniffer->header_pos++;
            sniffer->bytes_remaining = sniffer->payload_len + 2;
            if (sniffer->bytes_remaining == 0) {
                sniffer->in_frame = false;
                return false;
            }
            return false;
        }
    }

    if (sniffer->bytes_remaining > 0) {
        sniffer->bytes_remaining--;
        if (sniffer->bytes_remaining == 0) {
            *msgid = sniffer->msgid;
            *sysid = sniffer->sysid;
            *compid = sniffer->compid;
            sniffer->in_frame = false;
            return true;
        }
    } else {
        sniffer->in_frame = false;
    }

    return false;
}

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
                struct sockaddr_in peer = {0};
                if (!bridge_get_peer_addr(state, &peer)) {
                    const uint32_t now = now_ms();
                    if (now - state->last_waiting_peer_log_ms >= WAITING_FOR_PEER_LOG_INTERVAL_MS) {
                        ESP_LOGW(SVEA_TAG,
                                 "waiting for first UDP peer on %u (send any UDP packet to set peer)",
                                 (unsigned)MAVLINK_UDP_LISTEN_PORT);
                        state->last_waiting_peer_log_ms = now;
                    }
                }
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
    mavlink_sniffer_t sniffer = {0};
    bool first_uart_mavlink_logged = false;
    bool first_uart_heartbeat_logged = false;
    const uint32_t MAVLINK_MSG_ID_HEARTBEAT = 0U;

    for (;;) {
        int n = uart_read_bytes(BRIDGE_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n <= 0) {
            continue;
        }

        for (int i = 0; i < n; i++) {
            uint32_t msgid = 0;
            uint8_t sysid = 0;
            uint8_t compid = 0;
            if (mavlink_sniffer_feed(&sniffer, buf[i], &msgid, &sysid, &compid)) {
                state->uart_mavlink_frames++;

                if (!first_uart_mavlink_logged) {
                    ESP_LOGI(SVEA_TAG,
                             "UART MAVLink detected: first frame msgid=%" PRIu32 " sysid=%u compid=%u",
                             msgid, sysid, compid);
                    first_uart_mavlink_logged = true;
                }

                if (msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    state->uart_mavlink_heartbeats++;

                    if (!first_uart_heartbeat_logged) {
                        ESP_LOGI(SVEA_TAG,
                                 "PX4 heartbeat seen on UART: sysid=%u compid=%u",
                                 sysid, compid);
                        first_uart_heartbeat_logged = true;
                    }
                }
            }
        }

        struct sockaddr_in peer = {0};
        if (!bridge_get_peer_addr(state, &peer)) {
            state->uart_to_udp_drop_no_peer_pkts++;
            state->uart_to_udp_drop_no_peer_bytes += (uint32_t)n;

            const uint32_t now = now_ms();
            if (now - state->last_no_peer_drop_log_ms >= DROP_NO_PEER_LOG_INTERVAL_MS) {
                ESP_LOGW(SVEA_TAG,
                         "dropping UART->UDP while peer=none: dropped_pkts=%" PRIu32 " dropped_bytes=%" PRIu32,
                         state->uart_to_udp_drop_no_peer_pkts, state->uart_to_udp_drop_no_peer_bytes);
                state->last_no_peer_drop_log_ms = now;
            }
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
                     " | uart_drop_no_peer pkts=%" PRIu32 " bytes=%" PRIu32
                     " | uart_mav frames=%" PRIu32 " hb=%" PRIu32
                     " | enc->px4 pkts=%" PRIu32 " | peer=%u.%u.%u.%u:%u",
                     state->udp_to_uart_pkts, state->udp_to_uart_bytes,
                     state->uart_to_udp_pkts, state->uart_to_udp_bytes,
                     state->uart_to_udp_drop_no_peer_pkts, state->uart_to_udp_drop_no_peer_bytes,
                     state->uart_mavlink_frames, state->uart_mavlink_heartbeats,
                     state->encoder_to_px4_pkts,
                     (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF,
                     ntohs(peer.sin_port));
        } else {
            ESP_LOGI(SVEA_TAG,
                     "stats: udp->uart pkts=%" PRIu32 " bytes=%" PRIu32
                     " | uart->udp pkts=%" PRIu32 " bytes=%" PRIu32
                     " | uart_drop_no_peer pkts=%" PRIu32 " bytes=%" PRIu32
                     " | uart_mav frames=%" PRIu32 " hb=%" PRIu32
                     " | enc->px4 pkts=%" PRIu32 " | peer=none",
                     state->udp_to_uart_pkts, state->udp_to_uart_bytes,
                     state->uart_to_udp_pkts, state->uart_to_udp_bytes,
                     state->uart_to_udp_drop_no_peer_pkts, state->uart_to_udp_drop_no_peer_bytes,
                     state->uart_mavlink_frames, state->uart_mavlink_heartbeats,
                     state->encoder_to_px4_pkts);
        }
    }
}
