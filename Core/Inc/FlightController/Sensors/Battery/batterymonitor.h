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
    struct Config
    {
        float minValidVoltageV = 6.0f;
        float maxValidVoltageV = 30.0f;

        float maxPlausibleCurrentA = 120.0f;

        float currentWarningA = 25.0f;

        float currentLimitStartA = 35.0f;
        float currentLimitFullA = 55.0f;
        float minThrottleLimit = 0.55f;

        float instantOverCurrentA = 70.0f;
        float instantOverCurrentDelaySec = 0.2f; // 20 ms

        float sustainedOverCurrentA = 45.0f;
        float sustainedOverCurrentDelaySec = 3.0f; // 3 sec

        float batteryCapacityMah = 850.0f;
        float capacityWarningMah = 550.0f;
        float capacityCriticalMah = 700.0f;
    };

    struct Warnings
    {
        bool currentHigh = false;
        bool capacityWarning = false;
        bool capacityCritical = false;
        bool sustainedCurrentHigh = false;
    };

    struct Faults
    {
        bool voltageSensorInvalid = false;
        bool currentSensorInvalid = false;
        bool instantOverCurrent = false;
        bool sustainedOverCurrent = false;
    };

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
    float GetThrottleLimit() const;

    const Warnings& GetWarnings() const;
    const Faults& GetFaults() const;

    bool HasCriticalFault() const;
    bool CanArm() const;

private:
    void UpdateCellDetection(float batteryVoltage, float dtSeconds, bool armed);
    uint8_t DetectCellCount(float batteryVoltage) const;
    BatteryState EvaluateVoltageState(float cellVoltage, const float dtSeconds);
    void EvaluateCurrentState(bool armed, float currentA, const float dtSeconds);
    float ComputeDtSeconds(uint32_t nowUs);
    uint8_t ComputeBatteryPercentage() const;
    float CalculateThrottleLimit(float currentA) const;

private:
    uint8_t m_cellCount = 0;

    float m_consumedMah = 0.0f;

    Config m_config{};
    BatteryData m_batteryData = {};
    BatteryCellDetectState m_cellDetectState = BatteryCellDetectState::Unknown;
    BatteryConfig m_batteryConfig = {};
    Warnings m_warnings{};
    Faults m_faults{};

    float m_detectTimerSec = 0.0f;
    float m_lowTimerSec = 0.0f;
    float m_criticalTimerSec = 0.0f;
    float m_emergencyTimerSec = 0.0f;
    float m_throttleLimit = 1.0f;
    float m_instantOverCurrentTimeSec = 0;
    float m_sustainedOverCurrentTimeSec = 0;

    uint32_t m_lastUpdateUs = 0;
    bool m_hasLastUpdate = false;
};
