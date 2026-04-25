#include <arpa/inet.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rom_crc.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "wifi_manager.h"

static const char *TAG = "SVEA_BRIDGE";

#define SVEA_CHECK(x) do {                                       \
        esp_err_t __err_rc = (x);                                \
        if (__err_rc != ESP_OK) {                                \
            ESP_LOGE(TAG, "%s failed: %s", #x, esp_err_to_name(__err_rc)); \
            abort();                                              \
        }                                                         \
    } while (0)

/* ---------- Build-time defaults (adjust to your board wiring) ---------- */
/* ESP32-C6 UART connected to PX4 USART2:
 *   PX4 PA2 (TX) -> ESP RX
 *   PX4 PA3 (RX) <- ESP TX
 */
#define BRIDGE_UART_NUM             UART_NUM_1
#define BRIDGE_UART_TX_GPIO         16  /* XIAO D6 */
#define BRIDGE_UART_RX_GPIO         17  /* XIAO D7 */
#define BRIDGE_UART_BAUDRATE        921600
#define BRIDGE_UART_RX_BUF          4096
#define BRIDGE_UART_TX_BUF          4096

#define MAVLINK_UDP_LISTEN_PORT     14550
#define ENCODER_UDP_PORT            14660

#define ENCODER_LEFT_GPIO           4
#define ENCODER_RIGHT_GPIO          5
#define ENCODER_TICKS_PER_REV       80.0f
#define ENCODER_WHEEL_DIAM_M        0.115f
#define ENCODER_WHEELBASE_M         0.32f
#define ENCODER_PUBLISH_MS          20
#define ENCODER_SPEED_SCALE         0.93f

typedef struct __attribute__((packed)) {
    uint32_t magic;      /* 'SENC' */
    uint16_t version;    /* 1 */
    uint16_t payload_len;
    uint32_t seq;
    uint32_t uptime_ms;
    int32_t left_count;
    int32_t right_count;
    int32_t left_delta;
    int32_t right_delta;
    float left_mps;
    float right_mps;
    float linear_mps;
    float yaw_rate_rps;
    uint32_t crc32;
} encoder_packet_t;

static int s_udp_sock = -1;
static bool s_udp_peer_valid = false;
static struct sockaddr_in s_udp_peer_addr = {0};
static SemaphoreHandle_t s_peer_lock = NULL;
static portMUX_TYPE s_encoder_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t s_left_ticks = 0;
static volatile int32_t s_right_ticks = 0;
static volatile uint32_t s_udp_to_uart_pkts = 0;
static volatile uint32_t s_udp_to_uart_bytes = 0;
static volatile uint32_t s_uart_to_udp_pkts = 0;
static volatile uint32_t s_uart_to_udp_bytes = 0;
static volatile uint32_t s_encoder_pkts = 0;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void cb_wifi_sta_got_ip(void *pv_parameter)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)pv_parameter;

    if (event) {
        ESP_LOGI(TAG, "STA got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    } else {
        ESP_LOGI(TAG, "STA got IP");
    }
}

static void cb_wifi_sta_disconnected(void *pv_parameter)
{
    wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)pv_parameter;

    if (disc) {
        ESP_LOGW(TAG, "STA disconnected: reason=%u rssi=%d", disc->reason, disc->rssi);
    } else {
        ESP_LOGW(TAG, "STA disconnected");
    }
}

static void bridge_uart_init(void)
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

    ESP_LOGI(TAG, "UART bridge up: uart=%d tx=%d rx=%d baud=%d",
             BRIDGE_UART_NUM, BRIDGE_UART_TX_GPIO, BRIDGE_UART_RX_GPIO, BRIDGE_UART_BAUDRATE);
}

static void bridge_udp_init(void)
{
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_udp_sock < 0) {
        ESP_LOGE(TAG, "socket() failed errno=%d", errno);
        abort();
    }

    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = 100000
    };
    if (setsockopt(s_udp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        ESP_LOGE(TAG, "setsockopt(SO_RCVTIMEO) failed errno=%d", errno);
        abort();
    }

    struct sockaddr_in bind_addr = {0};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(MAVLINK_UDP_LISTEN_PORT);

    if (bind(s_udp_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed errno=%d", errno);
        abort();
    }

    ESP_LOGI(TAG, "UDP bridge listening on port %d", MAVLINK_UDP_LISTEN_PORT);
}

static bool get_peer_addr(struct sockaddr_in *out)
{
    bool valid = false;

    if (xSemaphoreTake(s_peer_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "peer lock failed");
        abort();
    }

    if (s_udp_peer_valid) {
        *out = s_udp_peer_addr;
        valid = true;
    }

    if (xSemaphoreGive(s_peer_lock) != pdTRUE) {
        ESP_LOGE(TAG, "peer unlock failed");
        abort();
    }

    return valid;
}

static void set_peer_addr(const struct sockaddr_in *peer)
{
    if (xSemaphoreTake(s_peer_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "peer lock failed");
        abort();
    }

    const bool first_peer = !s_udp_peer_valid;
    s_udp_peer_addr = *peer;
    s_udp_peer_valid = true;

    if (xSemaphoreGive(s_peer_lock) != pdTRUE) {
        ESP_LOGE(TAG, "peer unlock failed");
        abort();
    }

    if (first_peer) {
        const uint32_t ip = ntohl(peer->sin_addr.s_addr);
        ESP_LOGI(TAG, "first UDP peer: %u.%u.%u.%u:%u",
                 (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF,
                 ntohs(peer->sin_port));
    }
}

static void IRAM_ATTR encoder_gpio_isr(void *arg)
{
    uint32_t pin = (uint32_t)(uintptr_t)arg;

    portENTER_CRITICAL_ISR(&s_encoder_lock);
    if (pin == ENCODER_LEFT_GPIO) {
        s_left_ticks++;
    } else if (pin == ENCODER_RIGHT_GPIO) {
        s_right_ticks++;
    }
    portEXIT_CRITICAL_ISR(&s_encoder_lock);
}

static void encoder_gpio_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << ENCODER_LEFT_GPIO) | (1ULL << ENCODER_RIGHT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    SVEA_CHECK(gpio_config(&cfg));
    SVEA_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    SVEA_CHECK(gpio_isr_handler_add(ENCODER_LEFT_GPIO, encoder_gpio_isr, (void *)(uintptr_t)ENCODER_LEFT_GPIO));
    SVEA_CHECK(gpio_isr_handler_add(ENCODER_RIGHT_GPIO, encoder_gpio_isr, (void *)(uintptr_t)ENCODER_RIGHT_GPIO));

    ESP_LOGI(TAG, "Encoder GPIO ISR up: left=%d right=%d", ENCODER_LEFT_GPIO, ENCODER_RIGHT_GPIO);
}

static void udp_to_uart_task(void *arg)
{
    (void)arg;
    uint8_t buf[1472];

    for (;;) {
        struct sockaddr_in src = {0};
        socklen_t src_len = sizeof(src);

        ssize_t n = recvfrom(s_udp_sock, buf, sizeof(buf), 0, (struct sockaddr *)&src, &src_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            ESP_LOGE(TAG, "recvfrom() failed errno=%d", errno);
            abort();
        }

        set_peer_addr(&src);
        int written = uart_write_bytes(BRIDGE_UART_NUM, (const char *)buf, n);
        if (written != n) {
            ESP_LOGE(TAG, "uart_write_bytes short write=%d expected=%d", written, (int)n);
            abort();
        }

        s_udp_to_uart_pkts++;
        s_udp_to_uart_bytes += (uint32_t)n;
    }
}

static void uart_to_udp_task(void *arg)
{
    (void)arg;
    uint8_t buf[512];

    for (;;) {
        int n = uart_read_bytes(BRIDGE_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n <= 0) {
            continue;
        }

        struct sockaddr_in peer = {0};
        if (!get_peer_addr(&peer)) {
            continue;
        }

        ssize_t sent = sendto(s_udp_sock, buf, n, 0, (struct sockaddr *)&peer, sizeof(peer));
        if (sent != n) {
            ESP_LOGE(TAG, "sendto(uart->udp) failed sent=%d expected=%d errno=%d", (int)sent, n, errno);
            abort();
        }

        s_uart_to_udp_pkts++;
        s_uart_to_udp_bytes += (uint32_t)n;
    }
}

static void encoder_publish_task(void *arg)
{
    (void)arg;
    const float wheel_circ = (float)M_PI * ENCODER_WHEEL_DIAM_M;
    const float meters_per_tick = wheel_circ / ENCODER_TICKS_PER_REV;
    const float dt_s = (float)ENCODER_PUBLISH_MS / 1000.0f;
    uint32_t seq = 0;
    int32_t prev_left = 0;
    int32_t prev_right = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ENCODER_PUBLISH_MS));

        int32_t left = 0;
        int32_t right = 0;
        portENTER_CRITICAL(&s_encoder_lock);
        left = s_left_ticks;
        right = s_right_ticks;
        portEXIT_CRITICAL(&s_encoder_lock);

        const int32_t d_left = left - prev_left;
        const int32_t d_right = right - prev_right;
        prev_left = left;
        prev_right = right;

        const float left_mps = ((float)d_left * meters_per_tick / dt_s) * ENCODER_SPEED_SCALE;
        const float right_mps = ((float)d_right * meters_per_tick / dt_s) * ENCODER_SPEED_SCALE;
        const float linear_mps = 0.5f * (left_mps + right_mps);
        const float yaw_rate = (left_mps - right_mps) / ENCODER_WHEELBASE_M;

        struct sockaddr_in peer = {0};
        if (!get_peer_addr(&peer)) {
            continue;
        }

        encoder_packet_t pkt = {0};
        pkt.magic = 0x434E4553U; /* "SENC" */
        pkt.version = 1;
        pkt.payload_len = sizeof(pkt);
        pkt.seq = seq++;
        pkt.uptime_ms = now_ms();
        pkt.left_count = left;
        pkt.right_count = right;
        pkt.left_delta = d_left;
        pkt.right_delta = d_right;
        pkt.left_mps = left_mps;
        pkt.right_mps = right_mps;
        pkt.linear_mps = linear_mps;
        pkt.yaw_rate_rps = yaw_rate;
        pkt.crc32 = esp_rom_crc32_le(0, (const uint8_t *)&pkt, sizeof(pkt) - sizeof(pkt.crc32));

        peer.sin_port = htons(ENCODER_UDP_PORT);
        ssize_t sent = sendto(s_udp_sock, &pkt, sizeof(pkt), 0, (struct sockaddr *)&peer, sizeof(peer));
        if (sent != (ssize_t)sizeof(pkt)) {
            ESP_LOGE(TAG, "sendto(encoder) failed sent=%d expected=%d errno=%d",
                     (int)sent, (int)sizeof(pkt), errno);
            abort();
        }

        s_encoder_pkts++;
    }
}

static void bridge_stats_task(void *arg)
{
    (void)arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        struct sockaddr_in peer = {0};
        const bool has_peer = get_peer_addr(&peer);

        if (has_peer) {
            const uint32_t ip = ntohl(peer.sin_addr.s_addr);
            ESP_LOGI(TAG,
                     "stats: udp->uart pkts=%" PRIu32 " bytes=%" PRIu32
                     " | uart->udp pkts=%" PRIu32 " bytes=%" PRIu32
                     " | enc_pkts=%" PRIu32 " | peer=%u.%u.%u.%u:%u",
                     s_udp_to_uart_pkts, s_udp_to_uart_bytes,
                     s_uart_to_udp_pkts, s_uart_to_udp_bytes,
                     s_encoder_pkts,
                     (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF,
                     ntohs(peer.sin_port));
        } else {
            ESP_LOGI(TAG,
                     "stats: udp->uart pkts=%" PRIu32 " bytes=%" PRIu32
                     " | uart->udp pkts=%" PRIu32 " bytes=%" PRIu32
                     " | enc_pkts=%" PRIu32 " | peer=none",
                     s_udp_to_uart_pkts, s_udp_to_uart_bytes,
                     s_uart_to_udp_pkts, s_uart_to_udp_bytes,
                     s_encoder_pkts);
        }
    }
}

void app_main(void)
{
    for (int i = 10; i > 0; i--) {
        ESP_LOGI(TAG, "BOOT DELAY: starting in %d s", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "BOOT: app_main entered");
    s_peer_lock = xSemaphoreCreateMutex();
    if (s_peer_lock == NULL) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed");
        abort();
    }

    wifi_manager_start();
    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_wifi_sta_got_ip);
    wifi_manager_set_callback(WM_EVENT_STA_DISCONNECTED, &cb_wifi_sta_disconnected);
    ESP_LOGI(TAG, "Wi-Fi manager started (captive portal/AP fallback enabled)");

    bridge_uart_init();
    bridge_udp_init();
    encoder_gpio_init();

    BaseType_t rc = xTaskCreate(udp_to_uart_task, "udp_to_uart", 4096, NULL, 18, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate udp_to_uart failed");
        abort();
    }

    rc = xTaskCreate(uart_to_udp_task, "uart_to_udp", 4096, NULL, 18, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate uart_to_udp failed");
        abort();
    }

    rc = xTaskCreate(encoder_publish_task, "encoder_pub", 4096, NULL, 12, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate encoder_pub failed");
        abort();
    }

    rc = xTaskCreate(bridge_stats_task, "bridge_stats", 4096, NULL, 10, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate bridge_stats failed");
        abort();
    }

    ESP_LOGI(TAG, "SVEA ESP32-C6 bridge started");
}
