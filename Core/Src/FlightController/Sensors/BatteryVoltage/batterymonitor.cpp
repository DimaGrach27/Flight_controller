//
// Created by Dmytro Hrachov on 17.05.2026.
//

#include "FlightController/Sensors/BatteryVoltage/batterymonitor.h"

#include "FlightController/Utils/mathutils.h"
#include <cmath>

namespace
{
    constexpr float kMaxCellVoltage = 4.20f;
    constexpr float kLowCellVoltage = 3.60f;
    constexpr float kCriticalCellVoltage = 3.50f;
    constexpr float kEmergencyCellVoltage = 3.40f;
    constexpr float kBatteryPresentVoltage = 5.0f;

    constexpr uint8_t kMinCells = 1;
    constexpr uint8_t kMaxCells = 6;

    constexpr float kNominalDetectCellVoltage = 3.80f;
    constexpr float kMinValidCellVoltage = 3.00f;
    constexpr float kMaxValidCellVoltage = 4.40f;

    constexpr float kVoltageHysteresis = 0.05f;

    constexpr float kLowDelaySec = 2.0f;
    constexpr float kCriticalDelaySec = 1.5f;
    constexpr float kEmergencyDelaySec = 1.0f;
    constexpr float kDetectDelaySec = 1.0f;

    constexpr float MinDtSeconds = 0.000001f;
    constexpr float MaxDtSeconds = 0.5f;
}

BatteryMonitor::BatteryMonitor(const uint8_t cellCount)
{
    m_batteryConfig = {
        .cellCountMode = CellCountMode::Auto,
        .manualCellCount = cellCount
    };
}

void BatteryMonitor::Init()
{
    m_batteryData = {};
}

void BatteryMonitor::Update(const float batteryVoltage, const float currentA, const bool armed, const uint32_t nowUs)
{
    m_batteryData.voltage_V = batteryVoltage;
    m_batteryData.current_A = currentA;

    const float dtSeconds = ComputeDtSeconds(nowUs);

    UpdateCellDetection(m_batteryData.voltage_V, dtSeconds, armed);

    if (m_cellCount == 0)
    {
        m_batteryData.cellVoltage_V = 0.0f;
        m_batteryData.state = BatteryState::Unknown;
        m_batteryData.valid = false;
        return;
    }

    m_batteryData.cellVoltage_V = m_batteryData.voltage_V / static_cast<float>(m_cellCount);

    if (m_batteryData.current_A > 0.0f)
    {
        const float dtHours = static_cast<float>(dtSeconds) / 3600.0f;
        m_consumedMah += m_batteryData.current_A * 1000.0f * dtHours;
    }

    m_batteryData.state = EvaluateVoltageState(m_batteryData.cellVoltage_V, dtSeconds);

    m_batteryData.timestampUs = nowUs;
    m_batteryData.lowVoltage = m_batteryData.state == BatteryState::Low;
    m_batteryData.criticalVoltage = m_batteryData.state == BatteryState::Critical;
    m_batteryData.percentage = ComputeBatteryPercentage();

    m_batteryData.valid = true;
}

BatteryData BatteryMonitor::GetBatteryData() const
{
    return m_batteryData;
}

BatteryCellDetectState BatteryMonitor::GetCellDetectState() const
{
    return m_cellDetectState;
}

uint8_t BatteryMonitor::GetCellCount() const
{
    return m_cellCount;
}

void BatteryMonitor::UpdateCellDetection(const float batteryVoltage, const float dtSeconds, const bool armed)
{
    if (m_cellDetectState == BatteryCellDetectState::Detected)
    {
        return;
    }

    if (armed)
    {
        return;
    }

    m_cellDetectState = BatteryCellDetectState::Detecting;
    m_detectTimerSec += dtSeconds;

    if (m_detectTimerSec < kDetectDelaySec)
    {
        return;
    }

    const uint8_t detectedCells = DetectCellCount(batteryVoltage);

    if (detectedCells == 0)
    {
        m_cellDetectState = BatteryCellDetectState::Failed;
        return;
    }

    m_cellCount = detectedCells;
    m_cellDetectState = BatteryCellDetectState::Detected;
}


uint8_t BatteryMonitor::DetectCellCount(const float batteryVoltage) const
{
    if (batteryVoltage <= 0.0f || batteryVoltage < kBatteryPresentVoltage)
    {
        return 0;
    }

    const int estimatedCells = static_cast<int>(std::round(batteryVoltage / kNominalDetectCellVoltage));

    if (estimatedCells < kMinCells || estimatedCells > kMaxCells)
    {
        return 0;
    }

    const float cellVoltage = batteryVoltage / static_cast<float>(estimatedCells);

    if (cellVoltage < kMinValidCellVoltage ||
        cellVoltage > kMaxValidCellVoltage)
    {
        return 0;
    }

    return static_cast<uint8_t>(estimatedCells);
}

BatteryState BatteryMonitor::EvaluateVoltageState(const float cellVoltage, const float dtSeconds)
{
    if (cellVoltage < kEmergencyCellVoltage)
    {
        m_emergencyTimerSec += dtSeconds;
    }
    else if (cellVoltage > kEmergencyCellVoltage + kVoltageHysteresis)
    {
        m_emergencyTimerSec = 0.0f;
    }

    if (cellVoltage < kCriticalCellVoltage)
    {
        m_criticalTimerSec += dtSeconds;
    }
    else if (cellVoltage > kCriticalCellVoltage + kVoltageHysteresis)
    {
        m_criticalTimerSec = 0.0f;
    }

    if (cellVoltage < kLowCellVoltage)
    {
        m_lowTimerSec += dtSeconds;
    }
    else if (cellVoltage > kLowCellVoltage + kVoltageHysteresis)
    {
        m_lowTimerSec = 0.0f;
    }

    if (m_emergencyTimerSec >= kEmergencyDelaySec)
    {
        return BatteryState::Emergency;
    }

    if (m_criticalTimerSec >= kCriticalDelaySec)
    {
        return BatteryState::Critical;
    }

    if (m_lowTimerSec >= kLowDelaySec)
    {
        return BatteryState::Low;
    }

    return BatteryState::Normal;
}

float BatteryMonitor::ComputeDtSeconds(const uint32_t nowUs)
{
    if (!m_hasLastUpdate)
    {
        m_lastUpdateUs = nowUs;
        m_hasLastUpdate = true;
        return 0.0f;
    }

    const uint32_t dtUs = nowUs - m_lastUpdateUs;
    const float dtSeconds = static_cast<float>(dtUs) / 1000000.0f;

    m_lastUpdateUs = nowUs;

    if (dtSeconds < MinDtSeconds)
    {
        return 0.0f;
    }

    if (dtSeconds > MaxDtSeconds)
    {
        return 0.0f;
    }

    return dtSeconds;
}

float BatteryMonitor::ComputeBatteryPercentage() const
{
    const float maxBatteryVoltage = m_cellCount * kMaxCellVoltage;
    float batteryPercentage = m_batteryData.voltage_V / maxBatteryVoltage;

    batteryPercentage = MathUtils::Clamp01(batteryPercentage);

    return batteryPercentage;
}

float BatteryMonitor::GetVoltage() const
{
    return m_batteryData.voltage_V;
}

float BatteryMonitor::GetCellVoltage() const
{
    return m_batteryData.cellVoltage_V;
}

float BatteryMonitor::GetCurrentA() const
{
    return m_batteryData.current_A;
}

float BatteryMonitor::GetConsumedMah() const
{
    return m_consumedMah;
}

bool BatteryMonitor::CanArm() const
{
    if (m_cellDetectState != BatteryCellDetectState::Detected)
    {
        return false;
    }

    return m_batteryData.state == BatteryState::Normal || m_batteryData.state == BatteryState::Low;
}