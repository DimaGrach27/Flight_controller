//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "MavlinkBridge.h"

#include "MathUtils.h"

#include <cmath>
#include <iostream>

NAMESPACE_BEGIN
bool MavlinkBridge::Open(const std::string& port, int baud)
{
    return serial_.Open(port, baud);
}

void MavlinkBridge::Poll()
{
    uint8_t buffer[512];

    while (true)
    {
        int n = serial_.Read(buffer, sizeof(buffer));

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
    double rollRateRad,
    double pitchRateRad
)
{
    if (!serial_.IsOpen())
        return;

    constexpr double g = 9.80665;

    /*
    Спрощена модель gravity vector для roll/pitch.

    STM32 зазвичай рахує:
    roll  = atan2(accel.y, accel.z)
    pitch = atan2(-accel.x, sqrt(accel.y^2 + accel.z^2))

    Тому даємо сумісні accel:
    */

    float xacc = static_cast<float>(-std::sin(pitchRad) * g);
    float yacc = static_cast<float>( std::sin(rollRad) * std::cos(pitchRad) * g);
    float zacc = static_cast<float>( std::cos(rollRad) * std::cos(pitchRad) * g);

    float xgyro = static_cast<float>(rollRateRad);
    float ygyro = static_cast<float>(pitchRateRad);
    float zgyro = 0.0f;

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

    serial_.Write(txBuffer, len);
}

void MavlinkBridge::SendManualControl(
    bool armStatus,
    double roll,
    double pitch,
    double throttle,
    double yaw)
{
    mavlink_message_t msg;
    uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];

    int16_t x = static_cast<int16_t>(Clamp(pitch, -1.0, 1.0) * 1000.0);
    int16_t y = static_cast<int16_t>(Clamp(roll, -1.0, 1.0) * 1000.0);
    int16_t z = static_cast<int16_t>(Clamp(throttle, 0.0, 1.0) * 1000.0);
    int16_t r = static_cast<int16_t>(Clamp(yaw, -1.0, 1.0) * 1000.0);
    uint16_t buttons = 0;
    if (armStatus)
    {
        buttons |= (1u << 6);
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
    serial_.Write(txBuffer, len);
}

const MotorOutputs& MavlinkBridge::Motors() const
{
    return motors_;
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