//
// Created by Dmytro Hrachov on 10.05.2026.
//

#include "FlightController/Utils/lowpassfilter.h"

#include <cmath>

LowPassFilter::LowPassFilter()
{
}

void LowPassFilter::Init(float cutoffHz)
{
    m_cutoffHz = cutoffHz;
    m_state = 0.0f;
    m_initialized = false;
}

float LowPassFilter::Update(float input, float dt)
{
    if (dt <= 0.0f)
    {
        return m_state;
    }

    if (!m_initialized)
    {
        m_state = input;
        m_initialized = true;
        return m_state;
    }

    const float alpha = ComputeAlpha(dt);

    m_state = m_state + alpha * (input - m_state);

    return m_state;
}

void LowPassFilter::Reset(float value)
{
    m_state = value;
    m_initialized = true;
}

bool LowPassFilter::IsInitialized() const
{
    return m_initialized;
}

float LowPassFilter::ComputeAlpha(float dt) const
{
    /*
        RC low-pass:
        alpha = dt / (RC + dt)
        RC = 1 / (2*pi*cutoff)
    */
    const float rc = 1.0f / (2.0f * M_PI * m_cutoffHz);

    return dt / (rc + dt);
}