#include "peripheral_frame.h"

#include <inttypes.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "svea_common.h"

#define FRAME_MAGIC0 0x53U
#define FRAME_MAGIC1 0x45U
#define FRAME_VERSION 2U
#define FRAME_PAYLOAD_MAX_LEN 64U

typedef struct __attribute__((packed)) {
    uint32_t sequence;
    uint32_t time_ms;
    float left_distance_m;
    float right_distance_m;
} wheel_payload_t;

typedef struct __attribute__((packed)) {
    uint32_t sequence;
    uint32_t time_ms;
    float right_wheel_speed_rad_s;
    float left_wheel_speed_rad_s;
    float right_wheel_angle_rad;
    float left_wheel_angle_rad;
} wheel_encoders_payload_t;

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

static int peripheral_send_uart(int uart_num, peripheral_msg_id_t msg_id, const void *payload, uint8_t payload_len)
{
    uint8_t frame[2 + 1 + 1 + 1 + FRAME_PAYLOAD_MAX_LEN + 2] = {0};

    if (payload == NULL || payload_len == 0 || payload_len > FRAME_PAYLOAD_MAX_LEN) {
        ESP_LOGE(SVEA_TAG, "invalid peripheral payload: id=%u len=%u", (unsigned)msg_id, (unsigned)payload_len);
        return -1;
    }

    frame[0] = FRAME_MAGIC0;
    frame[1] = FRAME_MAGIC1;
    frame[2] = FRAME_VERSION;
    frame[3] = (uint8_t)msg_id;
    frame[4] = payload_len;
    memcpy(&frame[5], payload, payload_len);

    const uint16_t crc = crc16_ccitt(&frame[2], (uint16_t)(3U + payload_len));
    const int crc_idx = 5 + payload_len;
    frame[crc_idx] = (uint8_t)(crc & 0xFF);
    frame[crc_idx + 1] = (uint8_t)(crc >> 8);

    const int frame_len = crc_idx + 2;
    const int written = uart_write_bytes(uart_num, (const char *)frame, frame_len);

    if (written != frame_len) {
        ESP_LOGE(SVEA_TAG, "uart_write_bytes(peripheral_frame) short write=%d expected=%d id=%u",
                 written, frame_len, (unsigned)msg_id);
    }

    return written;
}

int peripheral_wheel_distance_send_uart(int uart_num, const peripheral_wheel_distance_sample_t *sample)
{
    static uint32_t last_debug_ms = 0;
    wheel_payload_t payload = {0};

    payload.sequence = sample->sequence;
    payload.time_ms = sample->time_ms;
    payload.left_distance_m = (float)sample->left_distance_m;
    payload.right_distance_m = (float)sample->right_distance_m;

    const int written = peripheral_send_uart(uart_num,
                                             PERIPHERAL_MSG_ID_WHEEL_DISTANCE,
                                             &payload,
                                             (uint8_t)sizeof(payload));

    const uint32_t now = sample->time_ms;
    if ((now - last_debug_ms) >= 1000U) {
        last_debug_ms = now;
        ESP_LOGI(SVEA_TAG,
                 "tx wheel_distance: len=%d seq=%" PRIu32 " t_ms=%" PRIu32 " left=%.3f right=%.3f",
                 written,
                 sample->sequence,
                 sample->time_ms,
                 (float)sample->left_distance_m,
                 (float)sample->right_distance_m);
    }

    return written;
}

int peripheral_wheel_encoders_send_uart(int uart_num, const peripheral_wheel_encoders_sample_t *sample)
{
    static uint32_t last_debug_ms = 0;
    wheel_encoders_payload_t payload = {0};

    payload.sequence = sample->sequence;
    payload.time_ms = sample->time_ms;
    payload.right_wheel_speed_rad_s = sample->right_wheel_speed_rad_s;
    payload.left_wheel_speed_rad_s = sample->left_wheel_speed_rad_s;
    payload.right_wheel_angle_rad = sample->right_wheel_angle_rad;
    payload.left_wheel_angle_rad = sample->left_wheel_angle_rad;

    const int written = peripheral_send_uart(uart_num,
                                             PERIPHERAL_MSG_ID_WHEEL_ENCODERS,
                                             &payload,
                                             (uint8_t)sizeof(payload));

    const uint32_t now = sample->time_ms;
    if ((now - last_debug_ms) >= 1000U) {
        last_debug_ms = now;
        ESP_LOGI(SVEA_TAG,
                 "tx wheel_encoders: len=%d seq=%" PRIu32 " t_ms=%" PRIu32 " wr=%.3f wl=%.3f ar=%.3f al=%.3f",
                 written,
                 sample->sequence,
                 sample->time_ms,
                 (double)sample->right_wheel_speed_rad_s,
                 (double)sample->left_wheel_speed_rad_s,
                 (double)sample->right_wheel_angle_rad,
                 (double)sample->left_wheel_angle_rad);
    }

    return written;
}
