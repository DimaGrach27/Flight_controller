//
// Created by Dmytro Hrachov on 05.05.2026.
//

#include "../../../Inc/FlightController/DebugLogs/logger.h"

#include <cstdint>

#include "main.h"
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"

Logger::Logger(UART_HandleTypeDef& huart2)
    : m_huart2(huart2)
{

}

FlightLogSample& Logger::GetLogSample()
{
    return m_logSample;
}

void Logger::SendFlightLogCsv()
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
    sendNamed("f_mode", m_logSample.flightMode);
    sendNamed("armed", m_logSample.armed);
    sendNamed("time_ms", static_cast<float>(m_logSample.timeMs));
    sendNamed("imu_seq", static_cast<float>(m_logSample.imuSeq));
    sendNamed("cont_seq", static_cast<float>(m_logSample.controlSeq));
    sendNamed("log_seq", static_cast<float>(m_logSample.logSeq));
    sendNamed("dt", m_logSample.dt);
    sendNamed("imu_dt", m_logSample.imuDt);
    sendNamed("hal_dt", m_logSample.halDt);
    sendNamed("rc_thr", m_logSample.rcThrottle);
    sendNamed("rc_roll", m_logSample.rcRoll);
    sendNamed("rc_pitch", m_logSample.rcPitch);
    sendNamed("rc_yaw", m_logSample.rcYaw);
    sendNamed("t_roll", m_logSample.targetRollRateDegSec);
    sendNamed("t_pitch", m_logSample.targetPitchRateDegSec);
    sendNamed("t_yaw", m_logSample.targetYawRateDegSec);
    sendNamed("g_roll", m_logSample.gyroRollDegSec);
    sendNamed("g_pitch", m_logSample.gyroPitchDegSec);
    sendNamed("g_yaw", m_logSample.gyroYawDegSec);
    sendNamed("a_roll", m_logSample.accelRoll);
    sendNamed("a_pitch", m_logSample.accelPitch);
    sendNamed("cor_roll", m_logSample.correctedRoll);
    sendNamed("cor_pitch", m_logSample.correctedPitch);
    sendNamed("err_roll", m_logSample.angleErrorRoll);
    sendNamed("err_pitch", m_logSample.angleErrorPitch);
    sendNamed("est_roll", m_logSample.estimatedRollDeg);
    sendNamed("est_pitch", m_logSample.estimatedPitchDeg);
    sendNamed("c_roll", m_logSample.controlRoll);
    sendNamed("c_pitch", m_logSample.controlPitch);
    sendNamed("c_yaw", m_logSample.controlYaw);
    sendNamed("m1", m_logSample.motorM1);
    sendNamed("m2", m_logSample.motorM2);
    sendNamed("m3", m_logSample.motorM3);
    sendNamed("m4", m_logSample.motorM4);
    sendNamed("-----", 0); //end log
}
