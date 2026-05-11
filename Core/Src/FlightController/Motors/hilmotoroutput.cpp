//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Motors/hilmotoroutput.h"

#include "main.h"
#include "mavlink/common/mavlink.h"

HilMotorOutput::HilMotorOutput(UART_HandleTypeDef& serialUart)
    : m_serialUart(serialUart)
{
}

bool HilMotorOutput::Init()
{
    m_lastCommand = {};
    m_initialized = true;
    return true;
}

void HilMotorOutput::Write(const MotorCommand& command)
{
    if (!m_initialized)
    {
        return;
    }

    m_lastCommand = command;
    m_lastCommand.Clamp01();

    SendServoOutput(m_lastCommand);
}

void HilMotorOutput::StopAll()
{
    m_lastCommand = {};

    SendServoOutput(m_lastCommand);
}

const MotorCommand& HilMotorOutput::GetLastCommand() const
{
    return m_lastCommand;
}

void HilMotorOutput::SendServoOutput(const MotorCommand &command)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    const uint16_t leftPwmFront = (1000 + command.m1 * 1000);
    const uint16_t rightPwmFront = (1000 + command.m2 * 1000);
    const uint16_t rightPwmBack = (1000 + command.m3 * 1000);
    const uint16_t leftPwmBack = (1000 + command.m4 * 1000);

    mavlink_msg_servo_output_raw_pack(
        1,
        1,
        &msg,
        HAL_GetTick() * 1000ULL,
        0,
        leftPwmFront,
        rightPwmFront,
        rightPwmBack,
        leftPwmBack,
        0,
        0,
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

    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    HAL_UART_Transmit(&m_serialUart, buffer, len, 100);
}
