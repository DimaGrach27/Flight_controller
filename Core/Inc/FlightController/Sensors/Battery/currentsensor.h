//
// Created by Dmytro Hrachov on 18.05.2026.
//
#pragma once

#include <cstdint>

class CurrentSensor
{
public:
    struct Config
    {
        float vref = 3.3f;
        float offsetV = 0.003f;
        float voltsPerAmp = 0.025f;
        float filterAlpha = 0.05f;
    };

    explicit CurrentSensor();

    void Update(uint32_t adcRaw);

    float GetCurrentA() const;
    float GetFilteredCurrentA() const;

private:
    float AdcToVoltage(uint32_t adcRaw) const;
    float VoltageToCurrent(float voltage) const;

private:
    Config m_config{};
    float m_currentA = 0.0f;
    float m_filteredCurrentA = 0.0f;
    bool m_initialized = false;
};