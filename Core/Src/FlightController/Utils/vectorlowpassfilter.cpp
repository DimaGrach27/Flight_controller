//
// Created by Dmytro Hrachov on 12.05.2026.
//
#include "FlightController/Utils/vectorlowpassfilter.h"

#include "FlightController/Math/Vector3f.h"

Vector3LowPassFilter::Vector3LowPassFilter()
{
}

void Vector3LowPassFilter::Init(float cutoffHz)
{
    m_xFilter.Init(cutoffHz);
    m_yFilter.Init(cutoffHz);
    m_zFilter.Init(cutoffHz);
}

Vector3f Vector3LowPassFilter::Update(const Vector3f& input, float dt)
{
    Vector3f output{};

    output.x = m_xFilter.Update(input.x, dt);
    output.y = m_yFilter.Update(input.y, dt);
    output.z = m_zFilter.Update(input.z, dt);

    return output;
}

void Vector3LowPassFilter::Reset(const Vector3f& value)
{
    m_xFilter.Reset(value.x);
    m_yFilter.Reset(value.y);
    m_zFilter.Reset(value.z);
}