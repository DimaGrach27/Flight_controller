//
// Created by Dmytro Hrachov on 05.05.2026.
//

#include "FlightController/logger.h"

#include <cstdint>

#include "main.h"
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"

Logger::Logger(UART_HandleTypeDef& huart2)
    : m_huart2(huart2)
{

}

void Logger::SendFlightLogCsv(const FlightLogSample& sample)
{
    const uint32_t nowMs = HAL_GetTick();

    if (nowMs - m_lastDebugMs < LOG_PERIOD_MS)
    {
        return;
    }

    m_lastDebugMs = nowMs;

    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    auto sendNamed = [&](const char* name, float value)
    {
        mavlink_msg_named_value_float_pack(
            1,
            1,
            &msg,
            nowMs,
            name,
            value
        );

        const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        HAL_UART_Transmit(&m_huart2, buffer, len, 100);
    };

    sendNamed("+++++", 1); //start log
    sendNamed("time_ms", static_cast<float>(sample.timeMs));
    sendNamed("imu_seq", static_cast<float>(sample.imuSeq));
    sendNamed("cont_seq", static_cast<float>(sample.controlSeq));
    sendNamed("log_seq", static_cast<float>(sample.logSeq));
    sendNamed("dt", sample.dt);
    sendNamed("imu_dt", sample.imuDt);
    sendNamed("hal_dt", sample.halDt);
    sendNamed("rc_thr", sample.rcThrottle);
    sendNamed("rc_roll", sample.rcRoll);
    sendNamed("rc_pitch", sample.rcPitch);
    sendNamed("rc_yaw", sample.rcYaw);
    sendNamed("t_roll", sample.targetRollRateDegSec);
    sendNamed("t_pitch", sample.targetPitchRateDegSec);
    sendNamed("t_yaw", sample.targetYawRateDegSec);
    sendNamed("g_roll", sample.gyroRollDegSec);
    sendNamed("g_pitch", sample.gyroPitchDegSec);
    sendNamed("g_yaw", sample.gyroYawDegSec);
    sendNamed("est_roll", sample.estimatedRollDeg);
    sendNamed("est_pitch", sample.estimatedPitchDeg);
    sendNamed("c_roll", sample.controlRoll);
    sendNamed("c_pitch", sample.controlPitch);
    sendNamed("c_yaw", sample.controlYaw);
    sendNamed("m1", sample.motorM1);
    sendNamed("m2", sample.motorM2);
    sendNamed("m3", sample.motorM3);
    sendNamed("m4", sample.motorM4);
    sendNamed("-----", 0); //end log
}
