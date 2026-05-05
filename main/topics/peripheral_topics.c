#include "peripheral_topics.h"

#include <inttypes.h>

#include "esp_log.h"
#include "peripheral_protocol.h"
#include "svea_common.h"

int peripheral_topic_publish_wheel_distance(peripheral_context_t *ctx, const peripheral_topic_wheel_distance_t *msg)
{
    static uint32_t last_debug_ms = 0;

    if (ctx == NULL || msg == NULL) {
        ESP_LOGE(SVEA_TAG, "wheel_distance publish null input");
        return -1;
    }

    const int written = peripheral_protocol_send(ctx, PERIPHERAL_UORB_TOPIC_WHEEL_DISTANCE, msg, (uint8_t)sizeof(*msg));

    if (written < 0) {
        return -1;
    }

    ctx->wheel_distance_tx++;

    if ((msg->time_ms - last_debug_ms) >= 1000U) {
        last_debug_ms = msg->time_ms;
        ESP_LOGI(SVEA_TAG,
                 "tx wheel_distance: seq=%" PRIu32 " t_ms=%" PRIu32 " left=%.3f right=%.3f",
                 msg->sequence, msg->time_ms, (double)msg->left_distance_m, (double)msg->right_distance_m);
    }

    return written;
}

int peripheral_topic_publish_wheel_encoders(peripheral_context_t *ctx, const peripheral_topic_wheel_encoders_t *msg)
{
    static uint32_t last_debug_ms = 0;

    if (ctx == NULL || msg == NULL) {
        ESP_LOGE(SVEA_TAG, "wheel_encoders publish null input");
        return -1;
    }

    const int written = peripheral_protocol_send(ctx, PERIPHERAL_UORB_TOPIC_WHEEL_ENCODERS, msg, (uint8_t)sizeof(*msg));

    if (written < 0) {
        return -1;
    }

    ctx->wheel_encoders_tx++;

    if ((msg->time_ms - last_debug_ms) >= 1000U) {
        last_debug_ms = msg->time_ms;
        ESP_LOGI(SVEA_TAG,
                 "tx wheel_encoders: seq=%" PRIu32 " t_ms=%" PRIu32 " wr=%.3f wl=%.3f ar=%.3f al=%.3f",
                 msg->sequence, msg->time_ms,
                 (double)msg->right_wheel_speed_rad_s, (double)msg->left_wheel_speed_rad_s,
                 (double)msg->right_wheel_angle_rad, (double)msg->left_wheel_angle_rad);
    }

    return written;
}
