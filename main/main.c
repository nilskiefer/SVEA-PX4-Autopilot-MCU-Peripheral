#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SVEA_ENCODER";

void app_main(void)
{
    while (1) {
        ESP_LOGI(TAG, "ESP-IDF environment ready (encoder module scaffold)");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
