//
// Created by Dmytro Hrachov on 17.05.2026.
//
#pragma once

#include "main.h"
#include <cstdint>

class BatteryVoltageSensor
{
public:
    explicit BatteryVoltageSensor();

    bool Init();
    bool Update(uint32_t adcRaw);

    float GetVoltageRaw() const;
    float GetVoltageFiltered() const;
    uint32_t GetAdcRaw() const;
    bool IsValid() const;

private:
    float ConvertRawToBatteryVoltage(uint32_t raw) const;

private:
    static constexpr float kVdda = 3.3f;
    static constexpr float kAdcMax = 4095.0f;

    static constexpr float kRTop = 100000.0f;
    static constexpr float kRBottom = 20000.0f;
    static constexpr float kDividerRatio = (kRTop + kRBottom) / kRBottom;

    static constexpr float kFilterAlpha = 0.05f;

private:
    uint32_t m_adcRaw = 0;


    float m_voltageRaw = 0.0f;
    float m_voltageFiltered = 0.0f;

    bool m_initialized = false;
    bool m_valid = false;
};
