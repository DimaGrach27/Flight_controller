//
// Created by Dmytro Hrachov.
//
#pragma once

#include "FlightController/datastructs.h"

class AngleController
{
public:
    AngleController();

    void Init();

    RateSetpoint Update(
        const RcCommand& rcCommand,
        const VehicleState& state
    ) const;

private:
    float m_maxAngleRad = 0.0f;
    float m_maxRollPitchRateRadS = 0.0f;
    float m_maxYawRateRadS = 0.0f;
    float m_angleKp = 0.0f;
};
