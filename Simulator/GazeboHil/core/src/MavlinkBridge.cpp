//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "MavlinkBridge.h"

#include "MathUtils.h"

#include <cmath>
#include <iostream>

#include "SerialPort_UART.h"

NAMESPACE_BEGIN
MavlinkBridge::MavlinkBridge()
{
    m_serial = std::make_unique<SerialPort_UART>();
}

bool MavlinkBridge::Open(const std::string& port, int baud, std::function<void(const mavlink_named_value_float_t&)> callback)
{
    m_callback = callback;
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
            mavlink_message_t msg;
            mavlink_status_t status;

            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
            {
                HandleMessage(msg);
            }
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
            printf("[MavlinkBridge] Get servo output\n");
            break;
        }
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
        {
            mavlink_named_value_float_t named_value_float{};
            mavlink_msg_named_value_float_decode(&msg, &named_value_float);

            if (m_callback)
            {
                m_callback(named_value_float);
            }
            // printf("[MavlinkBridge] %s %f\n", named_value_float.name, named_value_float.value);
            break;
        }

        default:
            break;
    }
}

double MavlinkBridge::PwmToMotor(uint16_t pwm)
{
    double value = (static_cast<double>(pwm) - 1000.0) / 1000.0;
    return Clamp(value, 0.0, 1.0);
}
NAMESPACE_END
