//
// Created by Dmytro Hrachov on 17.05.2026.
//

#include "FlightController/Sensors/BatteryVoltage/batteryvoltagesensor.h"
namespace
{
    constexpr float kCalibrationFactor = 15.2f / 15.26f; //this value was calibrated with measuring the real voltage
}

BatteryVoltageSensor::BatteryVoltageSensor(ADC_HandleTypeDef& adc)
    : m_adc(adc)
{
}

bool BatteryVoltageSensor::Init()
{
    m_adcRaw = 0;
    m_voltageRaw = 0.0f;
    m_voltageFiltered = 0.0f;

    m_initialized = false;

    return true;
}

bool BatteryVoltageSensor::Update()
{
    const uint32_t raw = ReadAdcRawAveraged();

    if (raw == 0)
    {
        m_valid = false;
        return false;
    }

    m_adcRaw = raw;
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

uint32_t BatteryVoltageSensor::ReadAdcRawAveraged()
{
    uint32_t sum = 0;

    for (uint32_t i = 0; i < kSampleCount; ++i)
    {
        if (HAL_ADC_Start(&m_adc) != HAL_OK)
        {
            HAL_ADC_Stop(&m_adc);
            return 0;
        }

        if (HAL_ADC_PollForConversion(&m_adc, 10) != HAL_OK)
        {
            HAL_ADC_Stop(&m_adc);
            return 0;
        }

        sum += HAL_ADC_GetValue(&m_adc);

        HAL_ADC_Stop(&m_adc);
    }

    return sum / kSampleCount;
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
