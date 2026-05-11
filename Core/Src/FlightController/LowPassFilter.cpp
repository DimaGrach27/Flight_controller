//
// Created by Dmytro Hrachov on 10.05.2026.
//

#include "FlightController/LowPassFilter.h"

LowPassFilter::LowPassFilter(float alpha)
    : m_alpha(alpha)
{
}

float LowPassFilter::Update(float value)
{
    m_state = m_alpha * m_state + (1.0f - m_alpha) * value;
    return m_state;
}

void LowPassFilter::Reset(float value)
{
    m_state = value;
}
