#include "peripheral_context.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "svea_common.h"

void peripheral_context_init(peripheral_context_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));

    ctx->uart_tx_lock = xSemaphoreCreateMutex();
    if (ctx->uart_tx_lock == NULL) {
        ESP_LOGE(SVEA_TAG, "xSemaphoreCreateMutex uart_tx_lock failed");
        abort();
    }
}

uint32_t peripheral_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}
