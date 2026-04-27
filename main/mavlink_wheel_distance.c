#include "mavlink_wheel_distance.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "mavlink/common/mavlink.h"
#include "svea_common.h"

int mavlink_wheel_distance_send_uart(
    int uart_num,
    uint8_t sysid,
    uint8_t compid,
    const mavlink_wheel_distance_sample_t *sample)
{
    mavlink_message_t msg;
    uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
    double distance[16] = {0.0};
    distance[0] = sample->left_distance_m;
    distance[1] = sample->right_distance_m;

    mavlink_msg_wheel_distance_pack(
        sysid,
        compid,
        &msg,
        sample->time_usec,
        2, /* left + right */
        distance);

    const uint16_t packet_len = mavlink_msg_to_send_buffer(tx_buf, &msg);
    const int written = uart_write_bytes(uart_num, (const char *)tx_buf, packet_len);

    if (written != (int)packet_len) {
        ESP_LOGE(SVEA_TAG, "uart_write_bytes(wheel_distance) short write=%d expected=%d", written, (int)packet_len);
    }

    return written;
}
