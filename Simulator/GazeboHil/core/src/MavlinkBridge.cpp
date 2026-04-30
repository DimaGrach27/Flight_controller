//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "MavlinkBridge.h"

#include "MathUtils.h"

#include <cmath>

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
    double angleRad,
    double angularVelocityRad
)
{
    if (!serial_.IsOpen())
        return;

    constexpr double g = 9.80665;

    float xacc = 0.0f;
    float yacc = static_cast<float>(std::sin(angleRad) * g);
    float zacc = static_cast<float>(std::cos(angleRad) * g);

    float xgyro = static_cast<float>(angularVelocityRad);
    float ygyro = 0.0f;
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

            motors_.left = PwmToMotor(servo.servo1_raw);
            motors_.right = PwmToMotor(servo.servo2_raw);

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