#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bridge_io.h"
#include "bridge_state.h"
#include "encoder.h"
#include "svea_common.h"

static bridge_state_t s_state;

void app_main(void) {
    for (int i = 5; i > 0; i--) {
        ESP_LOGI(SVEA_TAG, "BOOT DELAY: starting in %d s", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(SVEA_TAG, "BOOT: app_main entered");

    bridge_state_init(&s_state);
    s_state.peer_lock = xSemaphoreCreateMutex();
    if (s_state.peer_lock == NULL) {
        ESP_LOGE(SVEA_TAG, "xSemaphoreCreateMutex failed");
        abort();
    }
    s_state.uart_tx_lock = xSemaphoreCreateMutex();
    if (s_state.uart_tx_lock == NULL) {
        ESP_LOGE(SVEA_TAG, "xSemaphoreCreateMutex uart_tx_lock failed");
        abort();
    }

    bridge_uart_init();
    encoder_gpio_init(&s_state);

    BaseType_t rc = xTaskCreate(encoder_publish_task, "encoder_pub", 4096, &s_state, 12, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(SVEA_TAG, "xTaskCreate encoder_pub failed");
        abort();
    }
    ESP_LOGI(SVEA_TAG, "SVEA encoder module started (UART encoder publisher only)");
}
