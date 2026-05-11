//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Motors/pwmmotoroutput.h"

PwmMotorOutput::PwmMotorOutput(const std::array<MotorChannel, 4>& channels)
    : m_channels(channels)
{
    m_config =
    {
        .minPulseUs = 1000,
        .maxPulseUs = 2000,
        .stopPulseUs = 1000,
        .idlePulseUs = 1050,

        .useIdleWhenArmed = false
    };
}

bool PwmMotorOutput::Init()
{
    for (const MotorChannel& motor : m_channels)
    {
        if (motor.timer == nullptr || motor.channel == 0)
        {
            m_initialized = false;
            return false;
        }

        const HAL_StatusTypeDef status = HAL_TIM_PWM_Start(
            motor.timer,
            motor.channel
        );

        if (status != HAL_OK)
        {
            m_initialized = false;
            return false;
        }
    }

    StopAll();

    m_initialized = true;
    return true;
}

void PwmMotorOutput::Write(const MotorCommand& command)
{
    if (!m_initialized)
    {
        return;
    }

    WriteMotor(0, command.m1);
    WriteMotor(1, command.m2);
    WriteMotor(2, command.m3);
    WriteMotor(3, command.m4);
}

void PwmMotorOutput::StopAll()
{
    for (uint8_t i = 0; i < m_channels.size(); ++i)
    {
        WritePulseUs(i, m_config.stopPulseUs);
    }
}

void PwmMotorOutput::WriteMotor(uint8_t index, float normalizedValue)
{
    const uint16_t pulseUs = NormalizeToPulseUs(normalizedValue);
    WritePulseUs(index, pulseUs);
}

void PwmMotorOutput::WritePulseUs(uint8_t index, uint16_t pulseUs)
{
    if (index >= m_channels.size())
    {
        return;
    }

    const MotorChannel& motor = m_channels[index];

    if (motor.timer == nullptr)
    {
        return;
    }

    /*
        ВАЖЛИВО:
        Це працює напряму тільки якщо timer налаштований так,
        що 1 timer tick = 1 microsecond.

        Наприклад:
        timer clock = 84 MHz
        prescaler = 84 - 1
        tick = 1 MHz = 1 us
    */
    __HAL_TIM_SET_COMPARE(
        motor.timer,
        motor.channel,
        pulseUs
    );
}

uint16_t PwmMotorOutput::NormalizeToPulseUs(float value) const
{
    value = MathUtils::Clamp01(value);

    const uint16_t range =
        static_cast<uint16_t>(m_config.maxPulseUs - m_config.minPulseUs);

    return static_cast<uint16_t>(
        static_cast<float>(m_config.minPulseUs) +
        value * static_cast<float>(range)
    );
}