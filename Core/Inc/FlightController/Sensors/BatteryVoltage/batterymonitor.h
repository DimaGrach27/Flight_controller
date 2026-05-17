//
// Created by Dmytro Hrachov on 17.05.2026.
//
#pragma once

#include <cstdint>

#include "FlightController/datastructs.h"

enum class BatteryCellDetectState
{
    Unknown,
    Detecting,
    Detected,
    Failed
};

enum class CellCountMode
{
    Auto,
    Manual
};

struct BatteryConfig
{
    CellCountMode cellCountMode = CellCountMode::Auto;
    uint8_t manualCellCount = 4;
};

class BatteryMonitor
{
public:
    explicit BatteryMonitor(uint8_t cellCount);

    void Init();
    void Update(float batteryVoltage, float currentA, bool armed, const uint32_t nowUs);

    BatteryData GetBatteryData() const;
    BatteryCellDetectState GetCellDetectState() const;

    uint8_t GetCellCount() const;

    float GetVoltage() const;
    float GetCellVoltage() const;
    float GetCurrentA() const;
    float GetConsumedMah() const;

    bool CanArm() const;

private:
    void UpdateCellDetection(float batteryVoltage, float dtUs, bool armed);
    uint8_t DetectCellCount(float batteryVoltage) const;
    BatteryState EvaluateVoltageState(float cellVoltage, const float dtUs);
    float ComputeDtSeconds(uint32_t nowUs);

private:

private:
    uint8_t m_cellCount = 0;

    float m_consumedMah = 0.0f;

    BatteryData m_batteryData = {};
    BatteryCellDetectState m_cellDetectState = BatteryCellDetectState::Unknown;
    BatteryConfig m_batteryConfig = {};

    float m_detectTimerUs = 0;
    float m_lowTimerUs = 0;
    float m_criticalTimerUs = 0;
    float m_emergencyTimerUs = 0;

    uint32_t m_lastUpdateUs = 0;
    bool m_hasLastUpdate = false;
};
