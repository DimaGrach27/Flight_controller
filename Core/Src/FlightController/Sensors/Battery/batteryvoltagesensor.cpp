//
// Created by Dmytro Hrachov on 17.05.2026.
//

#include "FlightController/Sensors/Battery/batteryvoltagesensor.h"
namespace
{
    constexpr float kCalibrationFactor = 15.2f / 15.26f; //this value was calibrated with measuring the real voltage
}

BatteryVoltageSensor::BatteryVoltageSensor()
{

}

bool BatteryVoltageSensor::Init()
{
    m_adcRaw = 0;
    m_voltageRaw = 0.0f;
    m_voltageFiltered = 0.0f;

    m_initialized = false;
    m_valid = false;

    return true;
}

bool BatteryVoltageSensor::Update(uint32_t adcRaw)
{
    m_adcRaw = adcRaw;
    m_voltageRaw = ConvertRawToBatteryVoltage(m_adcRaw);

    if (!m_initialized)
    {
        m_voltageFiltered = m_voltageRaw;
        m_initialized = true;
    }
    else
    {
        m_voltageFiltered += kFilterAlpha * (m_voltageRaw - m_voltageFiltered);
    }

    m_valid = true;
    return true;
}

float BatteryVoltageSensor::ConvertRawToBatteryVoltage(uint32_t raw) const
{
    const float adcVoltage = static_cast<float>(raw) * kVdda / kAdcMax;
    return adcVoltage * kDividerRatio * kCalibrationFactor;
}

float BatteryVoltageSensor::GetVoltageRaw() const
{
    return m_voltageRaw;
}

float BatteryVoltageSensor::GetVoltageFiltered() const
{
    return m_voltageFiltered;
}

uint32_t BatteryVoltageSensor::GetAdcRaw() const
{
    return m_adcRaw;
}

bool BatteryVoltageSensor::IsValid() const
{
    return m_valid;
}
