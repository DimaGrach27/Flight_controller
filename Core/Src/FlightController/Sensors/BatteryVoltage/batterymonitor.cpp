//
// Created by Dmytro Hrachov on 17.05.2026.
//

#include "FlightController/Sensors/BatteryVoltage/batterymonitor.h"

#include "FlightController/Utils/mathutils.h"
#include <cmath>

namespace
{
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

    constexpr uint32_t kLowDelayUs = 200000;
    constexpr uint32_t kCriticalDelayUs = 150000;
    constexpr uint32_t kEmergencyDelayUs = 100000;
    constexpr uint32_t kDetectDelayUs = 100000;

    constexpr float MinDtSeconds = 0.000001f;
    constexpr float MaxDtSeconds = 0.05f;
}

BatteryMonitor::BatteryMonitor(uint8_t cellCount)
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

void BatteryMonitor::Update(float batteryVoltage, float currentA, bool armed, const uint32_t nowUs)
{
    m_batteryData.voltage_V = batteryVoltage;
    m_batteryData.current_A = currentA;

    const float dtMs = ComputeDtSeconds(nowUs);

    UpdateCellDetection(m_batteryData.voltage_V, dtMs, armed);

    if (m_cellCount == 0)
    {
        m_batteryData.cellVoltage_V = 0.0f;
        m_batteryData.state = BatteryState::Unknown;
        return;
    }

    m_batteryData.cellVoltage_V = m_batteryData.voltage_V / static_cast<float>(m_cellCount);

    if (m_batteryData.current_A > 0.0f)
    {
        const float dtHours = static_cast<float>(dtMs) / 3600000.0f;
        m_consumedMah += m_batteryData.current_A * 1000.0f * dtHours;
    }

    m_batteryData.state = EvaluateVoltageState(m_batteryData.cellVoltage_V, dtMs);
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

void BatteryMonitor::UpdateCellDetection(float batteryVoltage, float dtUs, bool armed)
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
    m_detectTimerUs += dtUs;

    if (m_detectTimerUs < kDetectDelayUs)
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


uint8_t BatteryMonitor::DetectCellCount(float batteryVoltage) const
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

BatteryState BatteryMonitor::EvaluateVoltageState(float cellVoltage, const float dtUs)
{
    if (cellVoltage < kEmergencyCellVoltage)
    {
        m_emergencyTimerUs += dtUs;
    }
    else if (cellVoltage > kEmergencyCellVoltage + kVoltageHysteresis)
    {
        m_emergencyTimerUs = 0;
    }

    if (cellVoltage < kCriticalCellVoltage)
    {
        m_criticalTimerUs += dtUs;
    }
    else if (cellVoltage > kCriticalCellVoltage + kVoltageHysteresis)
    {
        m_criticalTimerUs = 0;
    }

    if (cellVoltage < kLowCellVoltage)
    {
        m_lowTimerUs += dtUs;
    }
    else if (cellVoltage > kLowCellVoltage + kVoltageHysteresis)
    {
        m_lowTimerUs = 0;
    }

    if (m_emergencyTimerUs >= kEmergencyDelayUs)
    {
        return BatteryState::Emergency;
    }

    if (m_criticalTimerUs >= kCriticalDelayUs)
    {
        return BatteryState::Critical;
    }

    if (m_lowTimerUs >= kLowDelayUs)
    {
        return BatteryState::Low;
    }

    return BatteryState::Normal;
}

float BatteryMonitor::ComputeDtSeconds(uint32_t nowUs)
{
    if (!m_hasLastUpdate)
    {
        m_lastUpdateUs = nowUs;
        m_hasLastUpdate = true;
        return 0.0f;
    }

    const uint32_t dtUs = nowUs - m_lastUpdateUs;
    const float dt = static_cast<float>(dtUs) / 1000000.0f;

    m_lastUpdateUs = nowUs;

    if (dt < MinDtSeconds)
    {
        return 0.0f;
    }

    if (dt > MaxDtSeconds)
    {
        return 0.0f;
    }

    return dt;
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