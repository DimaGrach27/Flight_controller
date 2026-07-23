//
// Created by Dmytro Hrachov.
//

#include "FlightController/PID/anglecontroller.h"

#include <algorithm>

namespace
{
    constexpr float DegToRad = 0.01745329252f;

    constexpr float MaxAngleDeg = 45.0f;
    constexpr float MaxRollPitchRateDegS = 360.0f;
    constexpr float MaxYawRateDegS = 180.0f;

    constexpr float AngleKp = 6.0f;
}

AngleController::AngleController()
{
}

void AngleController::Init()
{
    m_maxAngleRad = MaxAngleDeg * DegToRad;
    m_maxRollPitchRateRadS = MaxRollPitchRateDegS * DegToRad;
    m_maxYawRateRadS = MaxYawRateDegS * DegToRad;
    m_angleKp = AngleKp;
}

RateSetpoint AngleController::Update(
    const RcCommand& rcCommand,
    const VehicleState& state
) const
{
    RateSetpoint setpoint{};

    if (!rcCommand.valid || rcCommand.failsafe || !state.valid)
    {
        return setpoint;
    }

    const float targetRollRad = rcCommand.roll * m_maxAngleRad;
    const float targetPitchRad = rcCommand.pitch * m_maxAngleRad;

    const float rollAngleErrorRad = targetRollRad - state.rollRad;
    const float pitchAngleErrorRad = targetPitchRad - state.pitchRad;

    setpoint.rollRadS = std::clamp(
        rollAngleErrorRad * m_angleKp,
        -m_maxRollPitchRateRadS,
        m_maxRollPitchRateRadS
    );

    setpoint.pitchRadS = std::clamp(
        pitchAngleErrorRad * m_angleKp,
        -m_maxRollPitchRateRadS,
        m_maxRollPitchRateRadS
    );

    setpoint.yawRadS = std::clamp(
        rcCommand.yaw * m_maxYawRateRadS,
        -m_maxYawRateRadS,
        m_maxYawRateRadS
    );

    setpoint.valid = true;
    return setpoint;
}
