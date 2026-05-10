//
// Created by Dmytro Hrachov on 10.05.2026.
//

#include "FlightController/pidautotune.h"

#include "FlightController/mathutils.h"

#include <cmath>

void PidAutoTune::Start()
{
    m_state = State::Running;

    m_timeSec = 0.0f;
    m_output = m_config.relayAmplitude;

    m_lastRate = 0.0f;
    m_positivePeak = 0.0f;
    m_negativePeak = 0.0f;

    m_havePositivePeak = false;
    m_haveNegativePeak = false;

    m_lastPositiveCrossTime = -1.0f;
    m_periodSum = 0.0f;
    m_periodCount = 0;

    m_amplitudeSum = 0.0f;
    m_amplitudeCount = 0;

    m_result = {};
}

void PidAutoTune::Stop()
{
    m_state = State::Idle;
    m_output = 0.0f;
}

float PidAutoTune::Update(float measuredRateRadSec, float dt)
{
    if (m_state != State::Running)
        return 0.0f;

    m_timeSec += dt;

    if (m_timeSec > m_config.timeoutSec)
    {
        m_state = State::Failed;
        m_output = 0.0f;
        return 0.0f;
    }

    if (std::fabs(measuredRateRadSec) > m_config.maxSafeRate)
    {
        m_state = State::Failed;
        m_output = 0.0f;
        return 0.0f;
    }

    UpdatePeaks(measuredRateRadSec);
    UpdateRelay(measuredRateRadSec);
    DetectPeriod(measuredRateRadSec);

    m_lastRate = measuredRateRadSec;

    if (m_periodCount >= m_config.periodsToCollect)
    {
        CalculateGains();
        m_state = State::Finished;
        m_output = 0.0f;
        return 0.0f;
    }

    return m_output;
}

PidAutoTune::State PidAutoTune::GetState() const
{
    return m_state;
}

bool PidAutoTune::IsFinished() const
{
    return m_state == State::Finished;
}

bool PidAutoTune::IsFailed() const
{
    return m_state == State::Failed;
}

PidGains PidAutoTune::GetResult() const
{
    return m_result;
}

void PidAutoTune::UpdateRelay(float measuredRateRadSec)
{
    // Якщо rate став позитивним — даємо негативну корекцію.
    // Якщо rate став негативним — даємо позитивну корекцію.
    // Hysteresis потрібен, щоб output не смикався біля нуля.
    if (measuredRateRadSec > m_config.hysteresis)
    {
        m_output = -m_config.relayAmplitude;
    }
    else if (measuredRateRadSec < -m_config.hysteresis)
    {
        m_output = m_config.relayAmplitude;
    }
}

void PidAutoTune::UpdatePeaks(float measuredRateRadSec)
{
    if (measuredRateRadSec > m_positivePeak)
    {
        m_positivePeak = measuredRateRadSec;
        m_havePositivePeak = true;
    }

    if (measuredRateRadSec < m_negativePeak)
    {
        m_negativePeak = measuredRateRadSec;
        m_haveNegativePeak = true;
    }
}

void PidAutoTune::DetectPeriod(float measuredRateRadSec)
{
    const bool crossedUp =
            m_lastRate <= 0.0f &&
            measuredRateRadSec > 0.0f;

    if (!crossedUp)
        return;

    if (m_lastPositiveCrossTime >= 0.0f)
    {
        const float period = m_timeSec - m_lastPositiveCrossTime;

        if (period > 0.05f)
        {
            const float amplitude = CalculateCurrentAmplitude();

            if (amplitude >= m_config.minOscillationAmplitude)
            {
                m_periodSum += period;
                m_periodCount++;

                m_amplitudeSum += amplitude;
                m_amplitudeCount++;
            }
        }
    }

    m_lastPositiveCrossTime = m_timeSec;

    // Після повного циклу очищаємо піки для наступного періоду.
    m_positivePeak = 0.0f;
    m_negativePeak = 0.0f;
    m_havePositivePeak = false;
    m_haveNegativePeak = false;
}

float PidAutoTune::CalculateCurrentAmplitude() const
{
    if (!m_havePositivePeak || !m_haveNegativePeak)
        return 0.0f;

    return 0.5f * (m_positivePeak - m_negativePeak);
}

void PidAutoTune::CalculateGains()
{
    if (m_periodCount == 0 || m_amplitudeCount == 0)
    {
        m_state = State::Failed;
        return;
    }

    const float Tu = m_periodSum / static_cast<float>(m_periodCount);
    const float a = m_amplitudeSum / static_cast<float>(m_amplitudeCount);
    const float d = m_config.relayAmplitude;

    if (Tu <= 0.0f || a <= 0.0f)
    {
        m_state = State::Failed;
        return;
    }

    // Relay feedback:
    // Ku = 4d / (pi * a)
    const float Ku = (4.0f * d) / (3.1415926f * a);

    // Для дрона я б НЕ брав агресивний Ziegler-Nichols.
    // Нижче більш м'які коефіцієнти для rate loop.
    PidGains gains;
    gains.kp = 0.35f * Ku;
    gains.ki = 0.50f * Ku / Tu;
    gains.kd = 0.06f * Ku * Tu;

    gains.kp = MathUtils::Clamp(gains.kp, 0.0f, m_config.maxKp);
    gains.ki = MathUtils::Clamp(gains.ki, 0.0f, m_config.maxKi);
    gains.kd = MathUtils::Clamp(gains.kd, 0.0f, m_config.maxKd);

    m_result = gains;
}
