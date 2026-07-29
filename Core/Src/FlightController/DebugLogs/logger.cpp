//
// Created by Dmytro Hrachov on 05.05.2026.
//

#include "../../../Inc/FlightController/DebugLogs/logger.h"

#include <cstdint>
#include <cstring>

#include "main.h"

namespace
{
    constexpr uint16_t LogSync = 0xA55A;
    constexpr uint8_t LogVersion = 1;
    constexpr uint8_t LogTypeFlightSample = 1;

#pragma pack(push, 1)
    struct BinaryLogHeader
    {
        uint16_t sync = LogSync;
        uint8_t version = LogVersion;
        uint8_t type = LogTypeFlightSample;
        uint16_t length = 0;
        uint32_t sequence = 0;
    };

    struct BinaryFlightLogSampleV1
    {
        uint32_t timeMs = 0;
        uint32_t imuSeq = 0;
        uint32_t controlSeq = 0;
        uint32_t logSeq = 0;

        float flightMode = 0.0f;
        float armed = 0.0f;
        float failsafe = 0.0f;
        float failsafeReason = 0.0f;
        float armDenyReason = 0.0f;
        float controlStopReason = 0.0f;

        float dt = 0.0f;
        float imuDt = 0.0f;
        float halDt = 0.0f;
        float controlDt = 0.0f;

        float rcThrottle = 0.0f;
        float rcRoll = 0.0f;
        float rcPitch = 0.0f;
        float rcYaw = 0.0f;
        float rcAgeMs = 0.0f;
        float rcValid = 0.0f;
        float rcArmSwitch = 0.0f;
        float rcAngleSwitch = 0.0f;

        float targetRollRateDegSec = 0.0f;
        float targetPitchRateDegSec = 0.0f;
        float targetYawRateDegSec = 0.0f;
        float gyroRollDegSec = 0.0f;
        float gyroPitchDegSec = 0.0f;
        float gyroYawDegSec = 0.0f;
        float accelRoll = 0.0f;
        float accelPitch = 0.0f;
        float accelYaw = 0.0f;
        float correctedRoll = 0.0f;
        float correctedPitch = 0.0f;
        float correctedYaw = 0.0f;
        float angleErrorRoll = 0.0f;
        float angleErrorPitch = 0.0f;
        float angleErrorYaw = 0.0f;
        float estimatedRollDeg = 0.0f;
        float estimatedPitchDeg = 0.0f;
        float estimatedYawDeg = 0.0f;
        float stateValid = 0.0f;

        float controlRoll = 0.0f;
        float controlPitch = 0.0f;
        float controlYaw = 0.0f;
        float motorM1 = 0.0f;
        float motorM2 = 0.0f;
        float motorM3 = 0.0f;
        float motorM4 = 0.0f;
        float motorMin = 0.0f;
        float motorMax = 0.0f;
        float motorSpan = 0.0f;
        float throttleLimit = 0.0f;
        float throttleLimited = 0.0f;

        float PID_P_roll = 0.0f;
        float PID_I_roll = 0.0f;
        float PID_D_roll = 0.0f;
        float PID_E_roll = 0.0f;
        float PID_S_roll = 0.0f;
        float PID_P_pitch = 0.0f;
        float PID_I_pitch = 0.0f;
        float PID_D_pitch = 0.0f;
        float PID_E_pitch = 0.0f;
        float PID_S_pitch = 0.0f;
        float PID_P_yaw = 0.0f;
        float PID_I_yaw = 0.0f;
        float PID_D_yaw = 0.0f;
        float PID_E_yaw = 0.0f;
        float PID_S_yaw = 0.0f;

        float batteryVoltage = 0.0f;
        float batteryCellVoltage = 0.0f;
        float batteryCurrent = 0.0f;
        float batteryPercent = 0.0f;
        float batteryState = 0.0f;
        float batteryWarnings = 0.0f;
        float batteryFaults = 0.0f;
    };
#pragma pack(pop)

    uint16_t Crc16Ccitt(const uint8_t* data, uint16_t size)
    {
        uint16_t crc = 0xFFFF;

        for (uint16_t i = 0; i < size; ++i)
        {
            crc ^= static_cast<uint16_t>(data[i]) << 8U;

            for (uint8_t bit = 0; bit < 8U; ++bit)
            {
                crc = (crc & 0x8000U) != 0U
                    ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                    : static_cast<uint16_t>(crc << 1U);
            }
        }

        return crc;
    }

    BinaryFlightLogSampleV1 ToBinarySample(const FlightLogSample& sample)
    {
        BinaryFlightLogSampleV1 binary{};

        binary.timeMs = sample.timeMs;
        binary.imuSeq = sample.imuSeq;
        binary.controlSeq = sample.controlSeq;
        binary.logSeq = sample.logSeq;

        binary.flightMode = sample.flightMode;
        binary.armed = sample.armed;
        binary.failsafe = sample.failsafe;
        binary.failsafeReason = sample.failsafeReason;
        binary.armDenyReason = sample.armDenyReason;
        binary.controlStopReason = sample.controlStopReason;

        binary.dt = sample.dt;
        binary.imuDt = sample.imuDt;
        binary.halDt = sample.halDt;
        binary.controlDt = sample.controlDt;

        binary.rcThrottle = sample.rcThrottle;
        binary.rcRoll = sample.rcRoll;
        binary.rcPitch = sample.rcPitch;
        binary.rcYaw = sample.rcYaw;
        binary.rcAgeMs = sample.rcAgeMs;
        binary.rcValid = sample.rcValid;
        binary.rcArmSwitch = sample.rcArmSwitch;
        binary.rcAngleSwitch = sample.rcAngleSwitch;

        binary.targetRollRateDegSec = sample.targetRollRateDegSec;
        binary.targetPitchRateDegSec = sample.targetPitchRateDegSec;
        binary.targetYawRateDegSec = sample.targetYawRateDegSec;
        binary.gyroRollDegSec = sample.gyroRollDegSec;
        binary.gyroPitchDegSec = sample.gyroPitchDegSec;
        binary.gyroYawDegSec = sample.gyroYawDegSec;
        binary.accelRoll = sample.accelRoll;
        binary.accelPitch = sample.accelPitch;
        binary.accelYaw = sample.accelYaw;
        binary.correctedRoll = sample.correctedRoll;
        binary.correctedPitch = sample.correctedPitch;
        binary.correctedYaw = sample.correctedYaw;
        binary.angleErrorRoll = sample.angleErrorRoll;
        binary.angleErrorPitch = sample.angleErrorPitch;
        binary.angleErrorYaw = sample.angleErrorYaw;
        binary.estimatedRollDeg = sample.estimatedRollDeg;
        binary.estimatedPitchDeg = sample.estimatedPitchDeg;
        binary.estimatedYawDeg = sample.estimatedYawDeg;
        binary.stateValid = sample.stateValid;

        binary.controlRoll = sample.controlRoll;
        binary.controlPitch = sample.controlPitch;
        binary.controlYaw = sample.controlYaw;
        binary.motorM1 = sample.motorM1;
        binary.motorM2 = sample.motorM2;
        binary.motorM3 = sample.motorM3;
        binary.motorM4 = sample.motorM4;
        binary.motorMin = sample.motorMin;
        binary.motorMax = sample.motorMax;
        binary.motorSpan = sample.motorSpan;
        binary.throttleLimit = sample.throttleLimit;
        binary.throttleLimited = sample.throttleLimited;

        binary.PID_P_roll = sample.PID_P_roll;
        binary.PID_I_roll = sample.PID_I_roll;
        binary.PID_D_roll = sample.PID_D_roll;
        binary.PID_E_roll = sample.PID_E_roll;
        binary.PID_S_roll = sample.PID_S_roll;
        binary.PID_P_pitch = sample.PID_P_pitch;
        binary.PID_I_pitch = sample.PID_I_pitch;
        binary.PID_D_pitch = sample.PID_D_pitch;
        binary.PID_E_pitch = sample.PID_E_pitch;
        binary.PID_S_pitch = sample.PID_S_pitch;
        binary.PID_P_yaw = sample.PID_P_yaw;
        binary.PID_I_yaw = sample.PID_I_yaw;
        binary.PID_D_yaw = sample.PID_D_yaw;
        binary.PID_E_yaw = sample.PID_E_yaw;
        binary.PID_S_yaw = sample.PID_S_yaw;

        binary.batteryVoltage = sample.batteryVoltage;
        binary.batteryCellVoltage = sample.batteryCellVoltage;
        binary.batteryCurrent = sample.batteryCurrent;
        binary.batteryPercent = sample.batteryPercent;
        binary.batteryState = sample.batteryState;
        binary.batteryWarnings = sample.batteryWarnings;
        binary.batteryFaults = sample.batteryFaults;

        return binary;
    }
}

Logger::Logger(UsbDebugConsole& debugConsole)
    : m_debugConsole(debugConsole)
{

}

FlightLogSample& Logger::GetLogSample()
{
    return m_logSample;
}

void Logger::SetBinaryUsbEnabled(bool enabled)
{
    m_binaryUsbEnabled = enabled;
}

bool Logger::IsBinaryUsbEnabled() const
{
    return m_binaryUsbEnabled;
}

void Logger::SendFlightLogBinary()
{
    if (!m_binaryUsbEnabled)
    {
        return;
    }

    const uint32_t nowMs = HAL_GetTick();

    if (nowMs - m_lastDebugMs < LOG_PERIOD_MS)
    {
        return;
    }

    m_lastDebugMs = nowMs;

    const BinaryFlightLogSampleV1 payload = ToBinarySample(m_logSample);
    BinaryLogHeader header{};
    header.length = sizeof(payload);
    header.sequence = m_logSample.logSeq;

    constexpr uint16_t FrameSize =
        sizeof(BinaryLogHeader) +
        sizeof(BinaryFlightLogSampleV1) +
        sizeof(uint16_t);

    uint8_t frame[FrameSize]{};
    uint16_t offset = 0;

    std::memcpy(&frame[offset], &header, sizeof(header));
    offset += sizeof(header);

    std::memcpy(&frame[offset], &payload, sizeof(payload));
    offset += sizeof(payload);

    const uint16_t crc = Crc16Ccitt(frame, offset);
    std::memcpy(&frame[offset], &crc, sizeof(crc));

    m_debugConsole.WriteBytes(frame, sizeof(frame));
}
