//
// Created by Dmytro Hrachov on 29.07.2026.
//
#pragma once

#include <cstdint>

#include "pidcontroller.h"
#include "FlightController/datastructs.h"

class AngleController
{
public:
    AngleController();

    void Init();

    RateTargets Update(
        const RcCommand& rcCommand,
        const VehicleState& state,
        uint32_t nowUs
    );

    void Reset();

    const AngleData& GetAngleData() const;

private:
    float ComputeDtSeconds(uint32_t nowUs);

private:
    PidController m_rollPid;
    PidController m_pitchPid;

    uint32_t m_lastUpdateUs = 0;
    bool m_hasLastUpdate = false;

    float m_maxRollAngleRad = 0.0f;
    float m_maxPitchAngleRad = 0.0f;
    float m_maxRollRateRadS = 0.0f;
    float m_maxPitchRateRadS = 0.0f;
    float m_maxYawRateRadS = 0.0f;

    int m_rollTargetSign = 1;
    int m_pitchTargetSign = 1;
    int m_yawTargetSign = 1;

    int m_rollFeedbackSign = 1;
    int m_pitchFeedbackSign = 1;

    AngleData m_angleData;
};
