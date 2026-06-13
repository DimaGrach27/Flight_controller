//
// Created by Dmytro Hrachov on 05.05.2026.
//

#include "../../../Inc/FlightController/DebugLogs/logger.h"

#include <cstdint>

#include "main.h"
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"

Logger::Logger(UsbDebugConsole& debugConsole)
    : m_debugConsole(debugConsole)
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
        // HAL_UART_Transmit(&m_huart2, buffer, len, 100);
        m_debugConsole.WriteBytes(buffer, len);
    };

    sendNamed("+++++", 1); //start log
    sendNamed("f_mode", m_logSample.flightMode);
    sendNamed("armed", m_logSample.armed);
    sendNamed("fs", m_logSample.failsafe);
    sendNamed("fs_rsn", m_logSample.failsafeReason);
    sendNamed("arm_deny", m_logSample.armDenyReason);
    sendNamed("stop_rsn", m_logSample.controlStopReason);
    sendNamed("time_ms", static_cast<float>(m_logSample.timeMs));
    sendNamed("imu_seq", static_cast<float>(m_logSample.imuSeq));
    sendNamed("cont_seq", static_cast<float>(m_logSample.controlSeq));
    sendNamed("log_seq", static_cast<float>(m_logSample.logSeq));
    sendNamed("dt", m_logSample.dt);
    sendNamed("imu_dt", m_logSample.imuDt);
    sendNamed("hal_dt", m_logSample.halDt);
    sendNamed("ctrl_dt", m_logSample.controlDt);
    sendNamed("rc_thr", m_logSample.rcThrottle);
    sendNamed("rc_roll", m_logSample.rcRoll);
    sendNamed("rc_pitch", m_logSample.rcPitch);
    sendNamed("rc_yaw", m_logSample.rcYaw);
    sendNamed("rc_age", m_logSample.rcAgeMs);
    sendNamed("rc_valid", m_logSample.rcValid);
    sendNamed("rc_arm", m_logSample.rcArmSwitch);
    sendNamed("rc_angle", m_logSample.rcAngleSwitch);
    sendNamed("t_roll", m_logSample.targetRollRateDegSec);
    sendNamed("t_pitch", m_logSample.targetPitchRateDegSec);
    sendNamed("t_yaw", m_logSample.targetYawRateDegSec);
    sendNamed("g_roll", m_logSample.gyroRollDegSec);
    sendNamed("g_pitch", m_logSample.gyroPitchDegSec);
    sendNamed("g_yaw", m_logSample.gyroYawDegSec);
    sendNamed("a_roll", m_logSample.accelRoll);
    sendNamed("a_pitch", m_logSample.accelPitch);
    sendNamed("a_yaw", m_logSample.accelYaw);
    sendNamed("cor_roll", m_logSample.correctedRoll);
    sendNamed("cor_pitch", m_logSample.correctedPitch);
    sendNamed("cor_yaw", m_logSample.correctedYaw);
    sendNamed("err_roll", m_logSample.angleErrorRoll);
    sendNamed("err_pitch", m_logSample.angleErrorPitch);
    sendNamed("err_yaw", m_logSample.angleErrorYaw);
    sendNamed("est_roll", m_logSample.estimatedRollDeg);
    sendNamed("est_pitch", m_logSample.estimatedPitchDeg);
    sendNamed("est_yaw", m_logSample.estimatedYawDeg);
    sendNamed("st_valid", m_logSample.stateValid);
    sendNamed("c_roll", m_logSample.controlRoll);
    sendNamed("c_pitch", m_logSample.controlPitch);
    sendNamed("c_yaw", m_logSample.controlYaw);
    sendNamed("m1", m_logSample.motorM1);
    sendNamed("m2", m_logSample.motorM2);
    sendNamed("m3", m_logSample.motorM3);
    sendNamed("m4", m_logSample.motorM4);
    sendNamed("m_min", m_logSample.motorMin);
    sendNamed("m_max", m_logSample.motorMax);
    sendNamed("m_span", m_logSample.motorSpan);
    sendNamed("thr_lim", m_logSample.throttleLimit);
    sendNamed("thr_out", m_logSample.throttleLimited);
    sendNamed("rp", m_logSample.PID_P_roll);
    sendNamed("ri", m_logSample.PID_I_roll);
    sendNamed("rd", m_logSample.PID_D_roll);
    sendNamed("re", m_logSample.PID_E_roll);
    sendNamed("rs", m_logSample.PID_S_roll);
    sendNamed("pp", m_logSample.PID_P_pitch);
    sendNamed("pi", m_logSample.PID_I_pitch);
    sendNamed("pd", m_logSample.PID_D_pitch);
    sendNamed("pe", m_logSample.PID_E_pitch);
    sendNamed("ps", m_logSample.PID_S_pitch);
    sendNamed("yp", m_logSample.PID_P_yaw);
    sendNamed("yi", m_logSample.PID_I_yaw);
    sendNamed("yd", m_logSample.PID_D_yaw);
    sendNamed("ye", m_logSample.PID_E_yaw);
    sendNamed("ys", m_logSample.PID_S_yaw);
    sendNamed("bat_v", m_logSample.batteryVoltage);
    sendNamed("cell_v", m_logSample.batteryCellVoltage);
    sendNamed("bat_a", m_logSample.batteryCurrent);
    sendNamed("bat_pct", m_logSample.batteryPercent);
    sendNamed("bat_st", m_logSample.batteryState);
    sendNamed("bat_warn", m_logSample.batteryWarnings);
    sendNamed("bat_flt", m_logSample.batteryFaults);
    sendNamed("-----", 0); //end log
}
