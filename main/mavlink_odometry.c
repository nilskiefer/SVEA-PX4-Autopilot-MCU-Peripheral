#include "mavlink_odometry.h"

#include <math.h>
#include <stddef.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "mavlink/common/mavlink.h"
#include "svea_common.h"

int mavlink_odometry_send_uart(
    int uart_num,
    uint8_t seq,
    uint8_t sysid,
    uint8_t compid,
    const mavlink_odometry_sample_t *sample)
{
    (void)seq;

    mavlink_message_t msg;
    uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];

    float q[4] = {NAN, NAN, NAN, NAN};
    float pose_covariance[21];
    float velocity_covariance[21];

    for (size_t i = 0; i < 21; i++) {
        pose_covariance[i] = NAN;
        velocity_covariance[i] = NAN;
    }

    mavlink_msg_odometry_pack(
        sysid,
        compid,
        &msg,
        sample->time_usec,
        MAV_FRAME_LOCAL_NED,
        MAV_FRAME_BODY_FRD,
        NAN,
        NAN,
        NAN,
        q,
        sample->linear_mps,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        sample->yaw_rate_rps,
        pose_covariance,
        velocity_covariance,
        sample->reset_counter,
        MAV_ESTIMATOR_TYPE_NAIVE,
        0);

    uint16_t packet_len = mavlink_msg_to_send_buffer(tx_buf, &msg);
    int written = uart_write_bytes(uart_num, (const char *)tx_buf, packet_len);

    if (written != (int)packet_len) {
        ESP_LOGE(SVEA_TAG, "uart_write_bytes(odometry) short write=%d expected=%d", written, (int)packet_len);
    }

    return written;
}
