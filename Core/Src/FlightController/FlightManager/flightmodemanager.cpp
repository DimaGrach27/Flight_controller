//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/FlightManager/flightmodemanager.h"


FlightModeManager::FlightModeManager()
{
}

void FlightModeManager::Init()
{
    m_state = {};

    m_state.mode = FlightMode::Acro;
    m_state.armState = ArmState::Disarmed;
    m_state.failsafe = true;
    m_state.throttleLow = false;
    m_state.canArm = false;
    m_state.timestampUs = 0;

    m_previousArmSwitch = false;
}

void FlightModeManager::Update(const RcCommand& rcCommand, uint32_t nowUs)
{
    Update(rcCommand,
        {0.0f, 0.0f, 0.0f, 0, false, false, BatteryState::Normal, nowUs, true},
        true,
        nowUs);
}

void FlightModeManager::Update(const RcCommand& rcCommand, const BatteryData& batterData, bool batteryValid, uint32_t nowUs)
{
    m_state.timestampUs = nowUs;

    m_state.failsafe = rcCommand.failsafe || !rcCommand.valid;
    m_state.throttleLow = IsThrottleLow(rcCommand.throttle);
    m_state.canArm = CanArmFromCommand(rcCommand) && CanArmFromBattery(batterData) && batteryValid;

    const bool armRisingEdge = rcCommand.armSwitch && !m_previousArmSwitch;

    if (m_state.failsafe)
    {
        m_state.armState = ArmState::Disarmed;
        m_state.mode = FlightMode::Acro;
        m_previousArmSwitch = rcCommand.armSwitch;
        return;
    }

    if (rcCommand.angleModeSwitch)
    {
        m_state.mode = FlightMode::Angle;
    }
    else
    {
        m_state.mode = FlightMode::Acro;
    }

    /*
        Якщо arm switch low — завжди disarmed.
    */
    if (!rcCommand.armSwitch)
    {
        m_state.armState = ArmState::Disarmed;
        m_previousArmSwitch = rcCommand.armSwitch;
        return;
    }

    /*
        Якщо вже armed і switch high — лишаємось armed.
        Throttle може бути будь-який.
    */
    if (m_state.armState == ArmState::Armed)
    {
        m_previousArmSwitch = rcCommand.armSwitch;
        return;
    }

    /*
        Якщо disarmed, дозволяємо arm тільки по rising edge,
        тільки при throttle low і без failsafe.
    */
    if (armRisingEdge && m_state.canArm)
    {
        m_state.armState = ArmState::Armed;
    }

    m_previousArmSwitch = rcCommand.armSwitch;
}

const FlightModeState& FlightModeManager::GetState() const
{
    return m_state;
}

bool FlightModeManager::IsArmed() const
{
    return m_state.armState == ArmState::Armed;
}

bool FlightModeManager::IsFailsafe() const
{
    return m_state.failsafe;
}

FlightMode FlightModeManager::GetMode() const
{
    return m_state.mode;
}

bool FlightModeManager::IsThrottleLow(float throttle) const
{
    return throttle <= m_throttleLowThreshold;
}

bool FlightModeManager::CanArmFromCommand(const RcCommand& rcCommand) const
{
    if (!rcCommand.valid)
    {
        return false;
    }

    if (rcCommand.failsafe)
    {
        return false;
    }

    if (!rcCommand.armSwitch)
    {
        return false;
    }

    if (!IsThrottleLow(rcCommand.throttle))
    {
        return false;
    }

    return true;
}

bool FlightModeManager::CanArmFromBattery(const BatteryData &batterData) const
{
    if (batterData.state == BatteryState::Unknown
        || batterData.state == BatteryState::Critical
        || batterData.state == BatteryState::Emergency
        )
    {
        return false;
    }

    return true;
}
