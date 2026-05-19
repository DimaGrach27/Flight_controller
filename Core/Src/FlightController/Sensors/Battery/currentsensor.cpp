//
// Created by Dmytro Hrachov on 18.05.2026.
//
#include "FlightController/Sensors/Battery/currentsensor.h"

CurrentSensor::CurrentSensor()
{
    m_config =
    {
        .vref = 3.3f,
        // .offsetV = 0.50f, // 0A 0.34V; 0.5A 0.49V; 1.0A 0.61V
        .offsetV = 0.034f, // 0.03f
        // .voltsPerAmp = 0.066f,
        .voltsPerAmp = 0.028f, // 0.0267f
        .filterAlpha = 0.05f
    };
}

void CurrentSensor::Update(uint16_t adcRaw)
{
    const float voltage = AdcToVoltage(adcRaw);
    m_currentA = VoltageToCurrent(voltage);

    if (!m_initialized)
    {
        m_filteredCurrentA = m_currentA;
        m_initialized = true;
        return;
    }

    m_filteredCurrentA += m_config.filterAlpha * (m_currentA - m_filteredCurrentA);
}

float CurrentSensor::GetCurrentA() const
{
    return m_currentA;
}

float CurrentSensor::GetFilteredCurrentA() const
{
    return m_filteredCurrentA;
}

float CurrentSensor::AdcToVoltage(uint16_t adcRaw) const
{
    constexpr float adcMax = 4095.0f;
    return static_cast<float>(adcRaw) * m_config.vref / adcMax;
}

float CurrentSensor::VoltageToCurrent(float voltage) const
{
    float current = (voltage - m_config.offsetV) / m_config.voltsPerAmp;

    if (current < 0.0f)
    {
        current = 0.0f;
    }

    return current;
}