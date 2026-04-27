#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_manager.h"

#include "bridge_io.h"
#include "bridge_state.h"
#include "encoder.h"
#include "svea_common.h"

static bridge_state_t s_state;

static void cb_wifi_sta_got_ip(void *pv_parameter)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)pv_parameter;

    if (event) {
        ESP_LOGI(SVEA_TAG, "STA got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    } else {
        ESP_LOGI(SVEA_TAG, "STA got IP");
    }
}

static void cb_wifi_sta_disconnected(void *pv_parameter)
{
    wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)pv_parameter;

    if (disc) {
        ESP_LOGW(SVEA_TAG, "STA disconnected: reason=%u rssi=%d", disc->reason, disc->rssi);
    } else {
        ESP_LOGW(SVEA_TAG, "STA disconnected");
    }
}

void app_main(void)
{
    for (int i = 10; i > 0; i--) {
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

    wifi_manager_start();
    wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_wifi_sta_got_ip);
    wifi_manager_set_callback(WM_EVENT_STA_DISCONNECTED, &cb_wifi_sta_disconnected);
    ESP_LOGI(SVEA_TAG, "Wi-Fi manager started (captive portal/AP fallback enabled)");

    bridge_uart_init();
    bridge_udp_init(&s_state);
    encoder_gpio_init(&s_state);

    BaseType_t rc = xTaskCreate(udp_to_uart_task, "udp_to_uart", 4096, &s_state, 18, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(SVEA_TAG, "xTaskCreate udp_to_uart failed");
        abort();
    }

    rc = xTaskCreate(uart_to_udp_task, "uart_to_udp", 4096, &s_state, 18, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(SVEA_TAG, "xTaskCreate uart_to_udp failed");
        abort();
    }

    rc = xTaskCreate(encoder_publish_task, "encoder_pub", 4096, &s_state, 12, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(SVEA_TAG, "xTaskCreate encoder_pub failed");
        abort();
    }

    rc = xTaskCreate(bridge_stats_task, "bridge_stats", 4096, &s_state, 10, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(SVEA_TAG, "xTaskCreate bridge_stats failed");
        abort();
    }

    ESP_LOGI(SVEA_TAG, "SVEA ESP32-C6 bridge started");
}
