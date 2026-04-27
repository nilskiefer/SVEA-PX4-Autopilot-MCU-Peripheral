#pragma once

#include <stdlib.h>

#include "esp_err.h"
#include "esp_log.h"

#define SVEA_TAG "SVEA_BRIDGE"

#define SVEA_CHECK(x) do {                                                     \
        esp_err_t __err_rc = (x);                                              \
        if (__err_rc != ESP_OK) {                                              \
            ESP_LOGE(SVEA_TAG, "%s failed: %s", #x, esp_err_to_name(__err_rc)); \
            abort();                                                            \
        }                                                                       \
    } while (0)
