//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/RcInput/rcinput.h"

#include "FlightController/Utils/mathutils.h"

RcInput::RcInput(IRcReceiver& receiver)
    : m_receiver(receiver)
{
}

bool RcInput::Init()
{
    m_command = {};

    if (!m_receiver.Init())
    {
        m_initialized = false;
        return false;
    }

    m_initialized = true;
    return true;
}

bool RcInput::Update(uint32_t nowUs)
{
    if (!m_initialized)
    {
        m_command.valid = false;
        m_command.failsafe = true;
        return false;
    }

    m_receiver.Update(nowUs);

    RcRawFrame frame{};

    if (m_receiver.ReadFrame(frame) && frame.valid && !frame.failsafe && frame.timestampUs != m_lastConsumedFrameUs)
    {
        const RcCommand newCommand = ConvertFrameToCommand(frame);

        if (newCommand.valid && !newCommand.failsafe)
        {
            m_command = newCommand;

            m_lastValidFrameUs = frame.timestampUs;
            m_lastConsumedFrameUs = frame.timestampUs;

            return true;
        }
    }

    const uint32_t timeSinceLastFrameUs = nowUs - m_lastValidFrameUs;

    if (timeSinceLastFrameUs > m_failsafeTimeoutUs)
    {
        m_command.throttle = 0.0f;
        m_command.roll = 0.0f;
        m_command.pitch = 0.0f;
        m_command.yaw = 0.0f;

        m_command.armSwitch = false;
        m_command.angleModeSwitch = false;
        m_command.acroModeSwitch = false;

        m_command.failsafe = true;
        m_command.valid = false;
        m_command.timestampUs = nowUs;

        return false;
    }

    return m_command.valid;
}

const RcCommand& RcInput::GetCommand() const
{
    return m_command;
}

RcCommand RcInput::ConvertFrameToCommand(const RcRawFrame& frame) const
{
    RcCommand command{};

    if (frame.channelCount <= m_modeChannel)
    {
        command.valid = false;
        command.failsafe = true;
        command.timestampUs = frame.timestampUs;
        return command;
    }

    command.roll = NormalizeCenteredChannel(frame.channels[m_rollChannel]);

    /*
        Pitch часто треба інвертувати.
        На пульті stick forward зазвичай має давати pitch forward.
        У математиці контролера може знадобитись знак '-'.
        Поки ставлю мінус, бо для дронів це часто зручніше.
    */
    command.pitch = -NormalizeCenteredChannel(frame.channels[m_pitchChannel]);
    command.throttle = NormalizeThrottleChannel(frame.channels[m_throttleChannel]);
    command.yaw = -NormalizeCenteredChannel(frame.channels[m_yawChannel]);

    command.roll = MathUtils::ApplyDeadband(command.roll, 0.025f);
    command.pitch = MathUtils::ApplyDeadband(command.pitch, 0.025f);
    command.throttle = MathUtils::ApplyDeadband(command.throttle, 0.025f);
    command.yaw = MathUtils::ApplyDeadband(command.yaw, 0.025f);

    command.armSwitch = IsSwitchHigh(frame.channels[m_armChannel]);

    command.angleModeSwitch = IsSwitchHigh(frame.channels[m_modeChannel]);
    command.acroModeSwitch = !command.angleModeSwitch;

    command.failsafe = frame.failsafe;
    command.valid = frame.valid && !frame.failsafe;
    command.timestampUs = frame.timestampUs;

    return command;
}

float RcInput::NormalizeCenteredChannel(uint16_t value) const
{
    float normalized = 0.0f;

    if (value >= m_channelMid)
    {
        const uint16_t range = m_channelMax - m_channelMid;

        if (range == 0)
        {
            return 0.0f;
        }

        normalized =
            static_cast<float>(value - m_channelMid) /
            static_cast<float>(range);
    }
    else
    {
        const uint16_t range = m_channelMid - m_channelMin;

        if (range == 0)
        {
            return 0.0f;
        }

        normalized =
            -static_cast<float>(m_channelMid - value) /
            static_cast<float>(range);
    }

    return MathUtils::Clamp(normalized, -1.0f, 1.0f);
}

float RcInput::NormalizeThrottleChannel(uint16_t value) const
{
    if (value <= m_channelMin)
    {
        return 0.0f;
    }

    if (value >= m_channelMax)
    {
        return 1.0f;
    }

    const uint16_t range = m_channelMax - m_channelMin;

    if (range == 0)
    {
        return 0.0f;
    }

    const float normalized =
        static_cast<float>(value - m_channelMin) /
        static_cast<float>(range);

    return MathUtils::Clamp01(normalized);
}

bool RcInput::IsSwitchHigh(uint16_t value) const
{
    return value > m_channelMid;
}