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

    const RateData& GetRateData();

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

    int m_rollTargetSign = 1;
    int m_pitchTargetSign = 1;
    int m_yawTargetSign = 1;

    int m_rollFeedbackSign = 1;
    int m_pitchFeedbackSign = 1;
    int m_yawFeedbackSign = 1;

    RateData m_rateData;
};
