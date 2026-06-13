//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "MavlinkBridge.h"

#include "MathUtils.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <unordered_map>

#include "SerialPort_USB.h"

NAMESPACE_BEGIN
namespace
{
    constexpr uint16_t LogSync = 0xA55A;
    constexpr uint8_t LogSync0 = static_cast<uint8_t>(LogSync & 0xFFU);
    constexpr uint8_t LogSync1 = static_cast<uint8_t>((LogSync >> 8U) & 0xFFU);
    constexpr uint8_t LogVersion = 1;
    constexpr uint8_t LogTypeFlightSample = 1;

#pragma pack(push, 1)
    struct BinaryLogHeader
    {
        uint16_t sync = 0;
        uint8_t version = 0;
        uint8_t type = 0;
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

    uint16_t Crc16Ccitt(const uint8_t* data, const uint16_t size)
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
}

MavlinkBridge::MavlinkBridge()
{
    m_serial = std::make_unique<SerialPort_USB>();
}

bool MavlinkBridge::Open(const std::string& port, int baud, LogCallback callback)
{
    m_logCallback = callback;
    return m_serial->Open(port, baud);
}

void MavlinkBridge::Poll()
{
    uint8_t buffer[512];

    while (true)
    {
        int n = m_serial->Read(buffer, sizeof(buffer));

        if (n <= 0)
            break;

        for (int i = 0; i < n; ++i)
        {
            ParseRxByte(buffer[i]);
        }
    }
}

void MavlinkBridge::SendHilSensor(
    uint64_t timeUsec,
    double rollRad,
    double pitchRad,
    double yawRad,
    double rollRateRad,
    double pitchRateRad,
    double yawRateRad
)
{
    (void)yawRad; // Поки yaw angle не потрібен для accel gravity vector

    if (!m_serial->IsOpen())
        return;

    constexpr double g = 9.80665;

    /*
        Спрощена модель gravity vector для roll/pitch.

        STM32 зазвичай рахує:
        roll  = atan2(accel.y, accel.z)
        pitch = atan2(-accel.x, sqrt(accel.y^2 + accel.z^2))

        Yaw не змінює напрямок гравітації в body frame для цих формул,
        тому accel лишається залежним тільки від roll/pitch.
    */

    float xacc = static_cast<float>(-std::sin(pitchRad) * g);
    float yacc = static_cast<float>( std::sin(rollRad) * std::cos(pitchRad) * g);
    float zacc = static_cast<float>( std::cos(rollRad) * std::cos(pitchRad) * g);

    float xgyro = static_cast<float>(rollRateRad);
    float ygyro = static_cast<float>(pitchRateRad);
    float zgyro = static_cast<float>(yawRateRad);

    mavlink_message_t msg;
    uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_hil_sensor_pack(
        255,
        1,
        &msg,
        timeUsec,
        xacc,
        yacc,
        zacc,
        xgyro,
        ygyro,
        zgyro,
        0.0f,
        0.0f,
        0.0f,
        1013.25f,
        0.0f,
        0.0f,
        25.0f,
        0xFFFF,
        0
    );

    uint16_t len = mavlink_msg_to_send_buffer(txBuffer, &msg);

    m_serial->Write(txBuffer, len);
}

void MavlinkBridge::SendHilSensorFromImu(uint64_t timeUsec, const ImuData& imuData)
{
    if (!m_serial->IsOpen())
        return;

    mavlink_message_t msg;
    uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];

    // printf("[MavlinkBridge]Hil IMU accelX = %f; accelY = %f; accelZ = %f; gyroX = %f; gyroY = %f; gyroZ = %f\n",
    //     imuData.accelX, imuData.accelY, imuData.accelZ, imuData.gyroX, imuData.gyroY, imuData.gyroZ);

    mavlink_msg_hil_sensor_pack(
        255,
        1,
        &msg,
        timeUsec,
        static_cast<float>(imuData.accelX),
        static_cast<float>(imuData.accelY),
        static_cast<float>(imuData.accelZ),
        static_cast<float>(imuData.gyroX),
        static_cast<float>(imuData.gyroY),
        static_cast<float>(imuData.gyroZ),
        0.0f,
        0.0f,
        0.0f,
        1013.25f,
        0.0f,
        0.0f,
        25.0f,
        0xFFFF,
        0
    );

    uint16_t len = mavlink_msg_to_send_buffer(txBuffer, &msg);

    m_serial->Write(txBuffer, len);
}

void MavlinkBridge::SendManualControl(
    bool armStatus,
    bool acroMode,
    int roll,
    int pitch,
    int throttle,
    int yaw)
{
    mavlink_message_t msg;
    uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];

    int16_t x = static_cast<int16_t>(Clamp(pitch, -1000, 1000));
    int16_t y = static_cast<int16_t>(Clamp(roll, -1000, 1000));
    int16_t z = static_cast<int16_t>(Clamp(throttle, 0, 1000));
    int16_t r = static_cast<int16_t>(Clamp(yaw, -1000, 1000));

    uint16_t buttons = 0;
    if (armStatus)
    {
        buttons |= (1u << 1);
    }

    if (acroMode)
    {
        buttons |= (1u << 2);
    }

    mavlink_msg_manual_control_pack(
        255,
        1,
        &msg,
        1,      // target system: Nucleo system_id
        x,
        y,
        z,
        r,
        buttons,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    );

    uint16_t len = mavlink_msg_to_send_buffer(txBuffer, &msg);
    m_serial->Write(txBuffer, len);
}

const MotorOutputs& MavlinkBridge::Motors() const
{
    return motors_;
}

static double NowWallSec()
{
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

void MavlinkBridge::HandleMessage(const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
        {
            mavlink_servo_output_raw_t servo{};
            mavlink_msg_servo_output_raw_decode(&msg, &servo);

            motors_.m1 = PwmToMotor(servo.servo1_raw);
            motors_.m2 = PwmToMotor(servo.servo2_raw);
            motors_.m3 = PwmToMotor(servo.servo3_raw);
            motors_.m4 = PwmToMotor(servo.servo4_raw);

            const double nowWallSec = NowWallSec();

            ++m_servoRxCount;

            if (m_lastServoWallSec > 0.0)
            {
                const double dt = nowWallSec - m_lastServoWallSec;
                m_servoDtMin = std::min(m_servoDtMin, dt);
                m_servoDtMax = std::max(m_servoDtMax, dt);

                if (dt > 0.030)
                {
                    m_servoMaxCount++;
                }
            }

            m_lastServoWallSec = nowWallSec;
            // printf("[MavlinkBridge] Get servo output\n");
            break;
        }
        default:
            break;
    }
}

void MavlinkBridge::ParseRxByte(const uint8_t byte)
{
    switch (m_binaryLogParseState)
    {
        case BinaryLogParseState::WaitSync0:
        {
            if (byte == LogSync0)
            {
                m_binaryLogFrame.clear();
                m_binaryLogFrame.push_back(byte);
                m_binaryLogParseState = BinaryLogParseState::WaitSync1;
                return;
            }

            FeedMavlinkByte(byte);
            return;
        }

        case BinaryLogParseState::WaitSync1:
        {
            if (byte == LogSync1)
            {
                m_binaryLogFrame.push_back(byte);
                m_binaryLogParseState = BinaryLogParseState::ReadFrame;
                return;
            }

            FeedMavlinkByte(LogSync0);
            ResetBinaryLogParser();
            ParseRxByte(byte);
            return;
        }

        case BinaryLogParseState::ReadFrame:
        {
            m_binaryLogFrame.push_back(byte);

            if (m_binaryLogFrame.size() == sizeof(BinaryLogHeader))
            {
                BinaryLogHeader header{};
                std::memcpy(&header, m_binaryLogFrame.data(), sizeof(header));

                if (header.sync != LogSync ||
                    header.version != LogVersion ||
                    header.type != LogTypeFlightSample ||
                    header.length != sizeof(BinaryFlightLogSampleV1))
                {
                    for (const uint8_t frameByte : m_binaryLogFrame)
                    {
                        FeedMavlinkByte(frameByte);
                    }

                    ResetBinaryLogParser();
                    return;
                }

                m_expectedBinaryLogFrameSize =
                    static_cast<uint16_t>(sizeof(BinaryLogHeader) + header.length + sizeof(uint16_t));
            }

            if (m_expectedBinaryLogFrameSize != 0U &&
                m_binaryLogFrame.size() >= m_expectedBinaryLogFrameSize)
            {
                TryHandleBinaryLogFrame();
                ResetBinaryLogParser();
            }

            return;
        }
    }
}

void MavlinkBridge::FeedMavlinkByte(const uint8_t byte)
{
    mavlink_message_t msg;
    mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
    {
        HandleMessage(msg);
    }
}

void MavlinkBridge::ResetBinaryLogParser()
{
    m_binaryLogParseState = BinaryLogParseState::WaitSync0;
    m_binaryLogFrame.clear();
    m_expectedBinaryLogFrameSize = 0;
}

void MavlinkBridge::TryHandleBinaryLogFrame()
{
    if (!m_logCallback)
    {
        return;
    }

    if (m_binaryLogFrame.size() < sizeof(BinaryLogHeader) + sizeof(uint16_t))
    {
        return;
    }

    const uint16_t frameWithoutCrcSize =
        static_cast<uint16_t>(m_binaryLogFrame.size() - sizeof(uint16_t));

    uint16_t receivedCrc = 0;
    std::memcpy(
        &receivedCrc,
        m_binaryLogFrame.data() + frameWithoutCrcSize,
        sizeof(receivedCrc)
    );

    const uint16_t calculatedCrc = Crc16Ccitt(
        m_binaryLogFrame.data(),
        frameWithoutCrcSize
    );

    if (receivedCrc != calculatedCrc)
    {
        return;
    }

    BinaryLogHeader header{};
    std::memcpy(&header, m_binaryLogFrame.data(), sizeof(header));

    if (header.length != sizeof(BinaryFlightLogSampleV1))
    {
        return;
    }

    BinaryFlightLogSampleV1 sample{};
    std::memcpy(
        &sample,
        m_binaryLogFrame.data() + sizeof(BinaryLogHeader),
        sizeof(sample)
    );

    std::unordered_map<std::string, float> fields;
    fields.reserve(81);

    fields["f_mode"] = sample.flightMode;
    fields["armed"] = sample.armed;
    fields["fs"] = sample.failsafe;
    fields["fs_rsn"] = sample.failsafeReason;
    fields["arm_deny"] = sample.armDenyReason;
    fields["stop_rsn"] = sample.controlStopReason;
    fields["time_ms"] = static_cast<float>(sample.timeMs);
    fields["imu_seq"] = static_cast<float>(sample.imuSeq);
    fields["cont_seq"] = static_cast<float>(sample.controlSeq);
    fields["log_seq"] = static_cast<float>(sample.logSeq);
    fields["dt"] = sample.dt;
    fields["imu_dt"] = sample.imuDt;
    fields["hal_dt"] = sample.halDt;
    fields["ctrl_dt"] = sample.controlDt;
    fields["rc_thr"] = sample.rcThrottle;
    fields["rc_roll"] = sample.rcRoll;
    fields["rc_pitch"] = sample.rcPitch;
    fields["rc_yaw"] = sample.rcYaw;
    fields["rc_age"] = sample.rcAgeMs;
    fields["rc_valid"] = sample.rcValid;
    fields["rc_arm"] = sample.rcArmSwitch;
    fields["rc_angle"] = sample.rcAngleSwitch;
    fields["t_roll"] = sample.targetRollRateDegSec;
    fields["t_pitch"] = sample.targetPitchRateDegSec;
    fields["t_yaw"] = sample.targetYawRateDegSec;
    fields["g_roll"] = sample.gyroRollDegSec;
    fields["g_pitch"] = sample.gyroPitchDegSec;
    fields["g_yaw"] = sample.gyroYawDegSec;
    fields["a_roll"] = sample.accelRoll;
    fields["a_pitch"] = sample.accelPitch;
    fields["a_yaw"] = sample.accelYaw;
    fields["cor_roll"] = sample.correctedRoll;
    fields["cor_pitch"] = sample.correctedPitch;
    fields["cor_yaw"] = sample.correctedYaw;
    fields["err_roll"] = sample.angleErrorRoll;
    fields["err_pitch"] = sample.angleErrorPitch;
    fields["err_yaw"] = sample.angleErrorYaw;
    fields["est_roll"] = sample.estimatedRollDeg;
    fields["est_pitch"] = sample.estimatedPitchDeg;
    fields["est_yaw"] = sample.estimatedYawDeg;
    fields["st_valid"] = sample.stateValid;
    fields["c_roll"] = sample.controlRoll;
    fields["c_pitch"] = sample.controlPitch;
    fields["c_yaw"] = sample.controlYaw;
    fields["m1"] = sample.motorM1;
    fields["m2"] = sample.motorM2;
    fields["m3"] = sample.motorM3;
    fields["m4"] = sample.motorM4;
    fields["m_min"] = sample.motorMin;
    fields["m_max"] = sample.motorMax;
    fields["m_span"] = sample.motorSpan;
    fields["thr_lim"] = sample.throttleLimit;
    fields["thr_out"] = sample.throttleLimited;
    fields["rp"] = sample.PID_P_roll;
    fields["ri"] = sample.PID_I_roll;
    fields["rd"] = sample.PID_D_roll;
    fields["re"] = sample.PID_E_roll;
    fields["rs"] = sample.PID_S_roll;
    fields["pp"] = sample.PID_P_pitch;
    fields["pi"] = sample.PID_I_pitch;
    fields["pd"] = sample.PID_D_pitch;
    fields["pe"] = sample.PID_E_pitch;
    fields["ps"] = sample.PID_S_pitch;
    fields["yp"] = sample.PID_P_yaw;
    fields["yi"] = sample.PID_I_yaw;
    fields["yd"] = sample.PID_D_yaw;
    fields["ye"] = sample.PID_E_yaw;
    fields["ys"] = sample.PID_S_yaw;
    fields["bat_v"] = sample.batteryVoltage;
    fields["cell_v"] = sample.batteryCellVoltage;
    fields["bat_a"] = sample.batteryCurrent;
    fields["bat_pct"] = sample.batteryPercent;
    fields["bat_st"] = sample.batteryState;
    fields["bat_warn"] = sample.batteryWarnings;
    fields["bat_flt"] = sample.batteryFaults;

    m_logCallback(fields);
}

double MavlinkBridge::PwmToMotor(uint16_t pwm)
{
    double value = (static_cast<double>(pwm) - 1000.0) / 1000.0;
    return Clamp(value, 0.0, 1.0);
}
NAMESPACE_END
