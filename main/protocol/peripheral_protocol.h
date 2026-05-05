#pragma once

#include <stdint.h>

#include "peripheral_context.h"

typedef enum {
    PERIPHERAL_UORB_TOPIC_WHEEL_DISTANCE = 1,
    PERIPHERAL_UORB_TOPIC_WHEEL_ENCODERS = 2,
} peripheral_uorb_topic_id_t;

int peripheral_protocol_send(peripheral_context_t *ctx,
                             peripheral_uorb_topic_id_t topic_id,
                             const void *payload,
                             uint8_t payload_len);
