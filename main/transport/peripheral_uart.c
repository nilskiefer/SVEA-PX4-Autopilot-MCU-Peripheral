#include "peripheral_uart.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "svea_common.h"
#include "svea_config.h"

void peripheral_uart_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate = BRIDGE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    SVEA_CHECK(uart_driver_install(BRIDGE_UART_NUM, BRIDGE_UART_RX_BUF, BRIDGE_UART_TX_BUF, 0, NULL, 0));
    SVEA_CHECK(uart_param_config(BRIDGE_UART_NUM, &uart_cfg));
    SVEA_CHECK(uart_set_pin(BRIDGE_UART_NUM, BRIDGE_UART_TX_GPIO, BRIDGE_UART_RX_GPIO,
                            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(SVEA_TAG, "UART up: uart=%d tx=%d rx=%d baud=%d",
             BRIDGE_UART_NUM, BRIDGE_UART_TX_GPIO, BRIDGE_UART_RX_GPIO, BRIDGE_UART_BAUDRATE);
}

int peripheral_uart_write(peripheral_context_t *ctx, const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0 || len > INT32_MAX) {
        ESP_LOGE(SVEA_TAG, "invalid UART write len=%u", (unsigned)len);
        return -1;
    }

    if (xSemaphoreTake(ctx->uart_tx_lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(SVEA_TAG, "uart tx lock failed");
        abort();
    }

    const int written = uart_write_bytes(BRIDGE_UART_NUM, (const char *)data, (uint32_t)len);

    if (xSemaphoreGive(ctx->uart_tx_lock) != pdTRUE) {
        ESP_LOGE(SVEA_TAG, "uart tx unlock failed");
        abort();
    }

    if (written != (int)len) {
        ESP_LOGE(SVEA_TAG, "uart_write_bytes short write=%d expected=%u", written, (unsigned)len);
        return -1;
    }

    return written;
}
