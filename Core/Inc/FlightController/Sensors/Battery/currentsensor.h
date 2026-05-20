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
        float zeroCalibrationAlpha = 0.02f;

        float maxValidCurrentA = 120.0f;
    };

    explicit CurrentSensor();

    void StartZeroCalibration();
    void StopZeroCalibration();

    void Update(uint16_t adcRaw);

    float GetCurrentA() const;
    float GetFilteredCurrentA() const;
    float GetOffsetV() const;

private:
    float AdcToVoltage(uint16_t adcRaw) const;
    float VoltageToCurrent(float voltage) const;

private:
    Config m_config{};

    float m_offsetV = 0.0f;

    float m_currentA = 0.0f;
    float m_filteredCurrentA = 0.0f;

    bool m_initialized = false;
    bool m_zeroCalibrationActive = false;
};