//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include <cstdint>

#include "pidcontroller.h"
#include "FlightController/datastructs.h"
#include "FlightController/structs.h"
#include "ratesetpointgenerator.h"

class RateController
{
public:
    RateController();

    void Init();

    ControlOutput Update(
        const RateSetpoint& rateSetpoint,
        const VehicleState& state,
        uint32_t nowUs
    );

    void Reset();

    const RateData& GetRateData();

private:
    // float ComputeDtSeconds(uint32_t nowUs);

private:
    PidController m_rollPid;
    PidController m_pitchPid;
    PidController m_yawPid;

    uint32_t m_lastUpdateUs = 0;
    bool m_hasLastUpdate = false;

    RateData m_rateData;
};
