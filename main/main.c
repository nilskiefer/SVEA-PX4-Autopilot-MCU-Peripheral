#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoder_module.h"
#include "peripheral_context.h"
#include "peripheral_uart.h"
#include "svea_common.h"

static peripheral_context_t s_ctx;
static encoder_module_t s_encoder_module;

void app_main(void)
{
    for (int i = 5; i > 0; i--) {
        ESP_LOGI(SVEA_TAG, "BOOT DELAY: starting in %d s", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(SVEA_TAG, "BOOT: app_main entered");
    peripheral_context_init(&s_ctx);
    peripheral_uart_init();

    encoder_module_init(&s_encoder_module, &s_ctx);
    encoder_module_start(&s_encoder_module);

    ESP_LOGI(SVEA_TAG, "SVEA peripheral MCU started (module=encoder)");
}
