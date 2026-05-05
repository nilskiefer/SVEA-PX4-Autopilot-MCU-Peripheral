#include "encoder_module.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "peripheral_context.h"
#include "peripheral_topics.h"
#include "svea_common.h"
#include "svea_config.h"

#if !ENCODER_EMULATION_ENABLE
static encoder_module_t *s_isr_module = NULL;

static void IRAM_ATTR encoder_gpio_isr(void *arg)
{
    const uint32_t pin = (uint32_t)(uintptr_t)arg;
    encoder_module_t *module = s_isr_module;

    if (module == NULL) {
        return;
    }

    portENTER_CRITICAL_ISR(&module->ticks_lock);
    if (pin == ENCODER_LEFT_GPIO) {
        module->left_ticks++;
    } else if (pin == ENCODER_RIGHT_GPIO) {
        module->right_ticks++;
    }
    portEXIT_CRITICAL_ISR(&module->ticks_lock);
}
#endif

static void encoder_emulate_profile(float t_s, float *linear_mps, float *yaw_rate_rps)
{
    const float two_pi = 2.0f * (float)M_PI;
    *linear_mps = 0.70f * sinf(two_pi * 0.08f * t_s) + 0.18f * sinf(two_pi * 0.27f * t_s);
    *yaw_rate_rps = 0.85f * sinf(two_pi * 0.13f * t_s + 0.9f);
}

static void encoder_emulate_ticks(encoder_module_t *module, float meters_per_tick, float dt_s)
{
    const float t_s = (float)(peripheral_now_ms() - module->emulate_start_ms) * 0.001f;
    float linear_mps = 0.0f;
    float yaw_rate_rps = 0.0f;
    encoder_emulate_profile(t_s, &linear_mps, &yaw_rate_rps);

    const float left_mps = linear_mps + (yaw_rate_rps * ENCODER_WHEELBASE_M * 0.5f);
    const float right_mps = linear_mps - (yaw_rate_rps * ENCODER_WHEELBASE_M * 0.5f);

    const float left_tick_delta_f = (left_mps * dt_s) / (meters_per_tick * ENCODER_SPEED_SCALE) + module->emulate_left_frac_ticks;
    const float right_tick_delta_f = (right_mps * dt_s) / (meters_per_tick * ENCODER_SPEED_SCALE) + module->emulate_right_frac_ticks;

    const int32_t left_tick_delta = (int32_t)left_tick_delta_f;
    const int32_t right_tick_delta = (int32_t)right_tick_delta_f;

    module->emulate_left_frac_ticks = left_tick_delta_f - (float)left_tick_delta;
    module->emulate_right_frac_ticks = right_tick_delta_f - (float)right_tick_delta;

    portENTER_CRITICAL(&module->ticks_lock);
    module->left_ticks += left_tick_delta;
    module->right_ticks += right_tick_delta;
    portEXIT_CRITICAL(&module->ticks_lock);
}

static void encoder_publish_task(void *arg)
{
    encoder_module_t *module = (encoder_module_t *)arg;
    const float two_pi = 2.0f * (float)M_PI;
    const float wheel_circ = (float)M_PI * ENCODER_WHEEL_DIAM_M;
    const float meters_per_tick = wheel_circ / ENCODER_TICKS_PER_REV;
    const float dt_s = (float)ENCODER_PUBLISH_MS / 1000.0f;
    const float radians_per_tick = (two_pi / ENCODER_TICKS_PER_REV) * ENCODER_SPEED_SCALE;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ENCODER_PUBLISH_MS));

#if ENCODER_EMULATION_ENABLE
        encoder_emulate_ticks(module, meters_per_tick, dt_s);
#endif

        int32_t left_ticks = 0;
        int32_t right_ticks = 0;

        portENTER_CRITICAL(&module->ticks_lock);
        left_ticks = module->left_ticks;
        right_ticks = module->right_ticks;
        portEXIT_CRITICAL(&module->ticks_lock);

        const int32_t d_left = left_ticks - module->prev_left_ticks;
        const int32_t d_right = right_ticks - module->prev_right_ticks;
        module->prev_left_ticks = left_ticks;
        module->prev_right_ticks = right_ticks;

        const float right_speed_rad_s = ((float)d_right * radians_per_tick) / dt_s;
        const float left_speed_rad_s = ((float)d_left * radians_per_tick) / dt_s;
        const float right_angle_rad = (float)right_ticks * radians_per_tick;
        const float left_angle_rad = (float)left_ticks * radians_per_tick;
        const uint32_t sequence = module->sequence++;
        const uint32_t time_ms = peripheral_now_ms();

        peripheral_topic_wheel_distance_t wheel_distance = {
            .sequence = sequence,
            .time_ms = time_ms,
            .left_distance_m = (float)left_ticks * meters_per_tick * ENCODER_SPEED_SCALE,
            .right_distance_m = (float)right_ticks * meters_per_tick * ENCODER_SPEED_SCALE,
        };

        peripheral_topic_wheel_encoders_t wheel_encoders = {
            .sequence = sequence,
            .time_ms = time_ms,
            .right_wheel_speed_rad_s = right_speed_rad_s,
            .left_wheel_speed_rad_s = left_speed_rad_s,
            .right_wheel_angle_rad = right_angle_rad,
            .left_wheel_angle_rad = left_angle_rad,
        };

        if (peripheral_topic_publish_wheel_distance(module->ctx, &wheel_distance) < 0 ||
            peripheral_topic_publish_wheel_encoders(module->ctx, &wheel_encoders) < 0) {
            ESP_LOGE(SVEA_TAG, "failed to publish encoder topics");
            abort();
        }
    }
}

void encoder_module_init(encoder_module_t *module, peripheral_context_t *ctx)
{
    if (module == NULL || ctx == NULL) {
        ESP_LOGE(SVEA_TAG, "encoder init null input");
        abort();
    }

    memset(module, 0, sizeof(*module));
    module->ctx = ctx;
    module->ticks_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    module->emulate_start_ms = peripheral_now_ms();

#if ENCODER_EMULATION_ENABLE
    ESP_LOGW(SVEA_TAG, "encoder emulation enabled (no GPIO ISR)");
    return;
#else
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << ENCODER_LEFT_GPIO) | (1ULL << ENCODER_RIGHT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    SVEA_CHECK(gpio_config(&cfg));
    SVEA_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    s_isr_module = module;
    SVEA_CHECK(gpio_isr_handler_add(ENCODER_LEFT_GPIO, encoder_gpio_isr, (void *)(uintptr_t)ENCODER_LEFT_GPIO));
    SVEA_CHECK(gpio_isr_handler_add(ENCODER_RIGHT_GPIO, encoder_gpio_isr, (void *)(uintptr_t)ENCODER_RIGHT_GPIO));
    ESP_LOGI(SVEA_TAG, "encoder GPIO ISR up: left=%d right=%d", ENCODER_LEFT_GPIO, ENCODER_RIGHT_GPIO);
#endif
}

void encoder_module_start(encoder_module_t *module)
{
    if (module == NULL || module->ctx == NULL) {
        ESP_LOGE(SVEA_TAG, "encoder start invalid module");
        abort();
    }

    BaseType_t rc = xTaskCreate(encoder_publish_task, "encoder_pub", 4096, module, 12, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(SVEA_TAG, "xTaskCreate encoder_pub failed");
        abort();
    }
}
