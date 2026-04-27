#include "encoder.h"

#include <math.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "mavlink_wheel_distance.h"
#include "svea_common.h"
#include "svea_config.h"
#include "bridge_io.h"

typedef struct {
    bridge_state_t *state;
    uint32_t emulate_start_ms;
    int32_t prev_left;
    int32_t prev_right;
    float emulate_left_frac_ticks;
    float emulate_right_frac_ticks;
} encoder_task_state_t;

static encoder_task_state_t s_encoder_task_state;
static bridge_state_t *s_state = NULL;

#if ENCODER_EMULATION_ENABLE
static void encoder_emulate_profile(float t_s, float *linear_mps, float *yaw_rate_rps)
{
    const float two_pi = 2.0f * (float)M_PI;
    /* Hardcoded dynamic pattern for bringup confidence:
     * - forward/reverse transitions
     * - varying yaw
     * - multiple frequencies to stress packet path
     */
    *linear_mps = 0.70f * sinf(two_pi * 0.08f * t_s) + 0.18f * sinf(two_pi * 0.27f * t_s);
    *yaw_rate_rps = 0.85f * sinf(two_pi * 0.13f * t_s + 0.9f);
}

static void encoder_emulate_ticks(bridge_state_t *state, encoder_task_state_t *task, float meters_per_tick, float dt_s)
{
    const float t_s = (float)(now_ms() - task->emulate_start_ms) * 0.001f;
    float linear_mps = 0.0f;
    float yaw_rate_rps = 0.0f;
    encoder_emulate_profile(t_s, &linear_mps, &yaw_rate_rps);

    const float left_mps = linear_mps + (yaw_rate_rps * ENCODER_WHEELBASE_M * 0.5f);
    const float right_mps = linear_mps - (yaw_rate_rps * ENCODER_WHEELBASE_M * 0.5f);

    const float left_tick_delta_f = (left_mps * dt_s) / (meters_per_tick * ENCODER_SPEED_SCALE) + task->emulate_left_frac_ticks;
    const float right_tick_delta_f = (right_mps * dt_s) / (meters_per_tick * ENCODER_SPEED_SCALE) + task->emulate_right_frac_ticks;

    const int32_t left_tick_delta = (int32_t)left_tick_delta_f;
    const int32_t right_tick_delta = (int32_t)right_tick_delta_f;

    task->emulate_left_frac_ticks = left_tick_delta_f - (float)left_tick_delta;
    task->emulate_right_frac_ticks = right_tick_delta_f - (float)right_tick_delta;

    portENTER_CRITICAL(&state->encoder_lock);
    state->left_ticks += left_tick_delta;
    state->right_ticks += right_tick_delta;
    portEXIT_CRITICAL(&state->encoder_lock);
}
#endif

static void IRAM_ATTR encoder_gpio_isr(void *arg)
{
    uint32_t pin = (uint32_t)(uintptr_t)arg;

    portENTER_CRITICAL_ISR(&s_state->encoder_lock);
    if (pin == ENCODER_LEFT_GPIO) {
        s_state->left_ticks++;
    } else if (pin == ENCODER_RIGHT_GPIO) {
        s_state->right_ticks++;
    }
    portEXIT_CRITICAL_ISR(&s_state->encoder_lock);
}

void encoder_gpio_init(bridge_state_t *state)
{
    s_state = state;

#if ENCODER_EMULATION_ENABLE
    ESP_LOGW(SVEA_TAG, "Encoder emulation enabled: GPIO ISR disabled (hardcoded dynamic profile)");
    return;
#endif

    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << ENCODER_LEFT_GPIO) | (1ULL << ENCODER_RIGHT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    SVEA_CHECK(gpio_config(&cfg));
    SVEA_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    SVEA_CHECK(gpio_isr_handler_add(ENCODER_LEFT_GPIO, encoder_gpio_isr, (void *)(uintptr_t)ENCODER_LEFT_GPIO));
    SVEA_CHECK(gpio_isr_handler_add(ENCODER_RIGHT_GPIO, encoder_gpio_isr, (void *)(uintptr_t)ENCODER_RIGHT_GPIO));

    ESP_LOGI(SVEA_TAG, "Encoder GPIO ISR up: left=%d right=%d", ENCODER_LEFT_GPIO, ENCODER_RIGHT_GPIO);
}

void encoder_publish_task(void *arg)
{
    bridge_state_t *state = (bridge_state_t *)arg;
    const float wheel_circ = (float)M_PI * ENCODER_WHEEL_DIAM_M;
    const float meters_per_tick = wheel_circ / ENCODER_TICKS_PER_REV;
    const float dt_s = (float)ENCODER_PUBLISH_MS / 1000.0f;
    encoder_task_state_t *task = &s_encoder_task_state;
    task->state = state;
    task->emulate_start_ms = now_ms();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ENCODER_PUBLISH_MS));

#if ENCODER_EMULATION_ENABLE
        encoder_emulate_ticks(state, task, meters_per_tick, dt_s);
#endif

        int32_t left = 0;
        int32_t right = 0;
        portENTER_CRITICAL(&state->encoder_lock);
        left = state->left_ticks;
        right = state->right_ticks;
        portEXIT_CRITICAL(&state->encoder_lock);

        const int32_t d_left = left - task->prev_left;
        const int32_t d_right = right - task->prev_right;
        task->prev_left = left;
        task->prev_right = right;

        (void)d_left;
        (void)d_right;

        const double left_distance_m = (double)left * (double)meters_per_tick * (double)ENCODER_SPEED_SCALE;
        const double right_distance_m = (double)right * (double)meters_per_tick * (double)ENCODER_SPEED_SCALE;

        mavlink_wheel_distance_sample_t sample = {
            .time_usec = (uint64_t)now_ms() * 1000ULL,
            .left_distance_m = left_distance_m,
            .right_distance_m = right_distance_m,
        };

        if (xSemaphoreTake(state->uart_tx_lock, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(SVEA_TAG, "uart tx lock failed");
            abort();
        }
        int written = mavlink_wheel_distance_send_uart(
            BRIDGE_UART_NUM,
            ENCODER_MAV_SYS_ID,
            ENCODER_MAV_COMP_ID,
            &sample);
        if (xSemaphoreGive(state->uart_tx_lock) != pdTRUE) {
            ESP_LOGE(SVEA_TAG, "uart tx unlock failed");
            abort();
        }

        if (written <= 0) {
            ESP_LOGE(SVEA_TAG, "Failed to publish wheel distance over MAVLink");
            abort();
        }

        state->encoder_to_px4_pkts++;
    }
}
