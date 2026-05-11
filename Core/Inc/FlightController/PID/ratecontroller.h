//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include <cstdint>

#include "pidcontroller.h"
#include "FlightController/datastructs.h"
#include "FlightController/structs.h"

class RateController
{
public:
    RateController();

    void Init();

    ControlOutput Update(
        const RcCommand& rcCommand,
        const VehicleState& state,
        uint32_t nowUs
    );

    void Reset();

private:
    float ComputeDtSeconds(uint32_t nowUs);

private:
    PidController m_rollPid;
    PidController m_pitchPid;
    PidController m_yawPid;

    uint32_t m_lastUpdateUs = 0;
    bool m_hasLastUpdate = false;

    float m_maxRollRate_rads = 0.0f;
    float m_maxPitchRate_rads = 0.0f;
    float m_maxYawRate_rads = 0.0f;
};
