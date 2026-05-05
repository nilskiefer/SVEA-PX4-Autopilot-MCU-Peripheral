#include "peripheral_protocol.h"

#include <string.h>

#include "esp_log.h"
#include "peripheral_uart.h"
#include "svea_common.h"

#define PERIPHERAL_FRAME_MAGIC0 0x53U
#define PERIPHERAL_FRAME_MAGIC1 0x45U
#define PERIPHERAL_FRAME_VERSION 2U
#define PERIPHERAL_FRAME_PAYLOAD_MAX_LEN 64U

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

int peripheral_protocol_send(peripheral_context_t *ctx,
                             peripheral_uorb_topic_id_t topic_id,
                             const void *payload,
                             uint8_t payload_len)
{
    uint8_t frame[2 + 1 + 1 + 1 + PERIPHERAL_FRAME_PAYLOAD_MAX_LEN + 2] = {0};

    if (ctx == NULL) {
        ESP_LOGE(SVEA_TAG, "null context");
        return -1;
    }

    if (topic_id == 0) {
        ESP_LOGE(SVEA_TAG, "invalid topic id=0");
        return -1;
    }

    if (payload == NULL || payload_len == 0 || payload_len > PERIPHERAL_FRAME_PAYLOAD_MAX_LEN) {
        ESP_LOGE(SVEA_TAG, "invalid payload: topic=%u len=%u", (unsigned)topic_id, (unsigned)payload_len);
        return -1;
    }

    frame[0] = PERIPHERAL_FRAME_MAGIC0;
    frame[1] = PERIPHERAL_FRAME_MAGIC1;
    frame[2] = PERIPHERAL_FRAME_VERSION;
    frame[3] = (uint8_t)topic_id;
    frame[4] = payload_len;
    memcpy(&frame[5], payload, payload_len);

    const uint16_t crc = crc16_ccitt(&frame[2], (uint16_t)(3U + payload_len));
    const int crc_idx = 5 + payload_len;
    frame[crc_idx] = (uint8_t)(crc & 0xFF);
    frame[crc_idx + 1] = (uint8_t)(crc >> 8);

    const int frame_len = crc_idx + 2;
    const int written = peripheral_uart_write(ctx, frame, (size_t)frame_len);

    if (written < 0) {
        return -1;
    }

    ctx->frames_tx++;
    ctx->bytes_tx += (uint32_t)written;

    return written;
}
