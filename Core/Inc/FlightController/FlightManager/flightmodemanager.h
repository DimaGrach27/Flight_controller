//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "FlightController/datastructs.h"
#include "FlightController/FlightManager/flightmode.h"

#include <cstdint>

class FlightModeManager
{
public:
    FlightModeManager();

    void Init();

    void Update(const RcCommand& rcCommand, uint32_t nowUs);

    const FlightModeState& GetState() const;

    bool IsArmed() const;
    bool IsFailsafe() const;
    FlightMode GetMode() const;

private:
    bool IsThrottleLow(float throttle) const;
    bool CanArmFromCommand(const RcCommand& rcCommand) const;

private:
    FlightModeState m_state{};

    bool m_previousArmSwitch = false;

    float m_throttleLowThreshold = 0.05f;
};