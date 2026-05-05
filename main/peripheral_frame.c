#include "peripheral_frame.h"

#include <inttypes.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "svea_common.h"

#define FRAME_MAGIC0 0x53U
#define FRAME_MAGIC1 0x45U
#define FRAME_VERSION 1U
#define FRAME_PAYLOAD_LEN 16U

typedef struct __attribute__((packed)) {
    uint32_t sequence;
    uint32_t time_ms;
    float left_distance_m;
    float right_distance_m;
} wheel_payload_t;

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

int peripheral_wheel_distance_send_uart(int uart_num, const peripheral_wheel_distance_sample_t *sample)
{
    static uint32_t last_debug_ms = 0;
    uint8_t frame[2 + 1 + 1 + FRAME_PAYLOAD_LEN + 2] = {0};
    wheel_payload_t payload = {0};

    payload.sequence = sample->sequence;
    payload.time_ms = sample->time_ms;
    payload.left_distance_m = (float)sample->left_distance_m;
    payload.right_distance_m = (float)sample->right_distance_m;

    frame[0] = FRAME_MAGIC0;
    frame[1] = FRAME_MAGIC1;
    frame[2] = FRAME_VERSION;
    frame[3] = FRAME_PAYLOAD_LEN;
    memcpy(&frame[4], &payload, sizeof(payload));

    const uint16_t crc = crc16_ccitt(&frame[2], 2 + FRAME_PAYLOAD_LEN);
    frame[4 + FRAME_PAYLOAD_LEN] = (uint8_t)(crc & 0xFF);
    frame[4 + FRAME_PAYLOAD_LEN + 1] = (uint8_t)(crc >> 8);

    const int frame_len = (int)sizeof(frame);
    const int written = uart_write_bytes(uart_num, (const char *)frame, frame_len);

    if (written != frame_len) {
        ESP_LOGE(SVEA_TAG, "uart_write_bytes(peripheral_frame) short write=%d expected=%d", written, frame_len);
    }

    const uint32_t now = sample->time_ms;
    if ((now - last_debug_ms) >= 1000U) {
        last_debug_ms = now;
        ESP_LOGI(SVEA_TAG,
                 "tx frame: len=%d seq=%" PRIu32 " t_ms=%" PRIu32 " left=%.3f right=%.3f bytes=%02X %02X %02X %02X %02X %02X",
                 written,
                 sample->sequence,
                 sample->time_ms,
                 (float)sample->left_distance_m,
                 (float)sample->right_distance_m,
                 frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]);
    }

    return written;
}
