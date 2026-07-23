//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/PID/ratecontroller.h"

#include <algorithm>

#include "FlightController/Utils/mathutils.h"

namespace
{
    constexpr float DegToRad = 0.01745329252f;

    constexpr float MaxRollRate_dps = 360.0f;
    constexpr float MaxPitchRate_dps = 360.0f;
    constexpr float MaxYawRate_dps = 180.0f;

    constexpr float MinDtSeconds = 0.000001f;
    constexpr float MaxDtSeconds = 0.05f;
}

RateController::RateController()
{
}

void RateController::Init()
{
    m_rollPid.Init(0.065f, 0.045f, 0.0015f);
    m_pitchPid.Init(0.065f, 0.045f, 0.0015f);
    m_yawPid.Init(0.090f, 0.025f, 0.0000f);

    m_rollPid.SetOutputLimit(-0.65f, 0.65f);
    m_pitchPid.SetOutputLimit(-0.65f, 0.65f);
    m_yawPid.SetOutputLimit(-0.35f, 0.35f);

    m_rollPid.SetIntegralLimit(-0.25f, 0.25f);
    m_pitchPid.SetIntegralLimit(-0.25f, 0.25f);
    m_yawPid.SetIntegralLimit(-0.15f, 0.15f);

    m_maxRollRate_rads = MaxRollRate_dps * DegToRad;
    m_maxPitchRate_rads = MaxPitchRate_dps * DegToRad;
    m_maxYawRate_rads = MaxYawRate_dps * DegToRad;

    Reset();
}

RateSetpoint RateController::CreateAcroSetpoint(const RcCommand& rcCommand) const
{
    RateSetpoint setpoint{};

    if (!rcCommand.valid || rcCommand.failsafe)
    {
        return setpoint;
    }

    setpoint.rollRadS = std::clamp(
        rcCommand.roll * m_maxRollRate_rads,
        -m_maxRollRate_rads,
        m_maxRollRate_rads
    );

    setpoint.pitchRadS = std::clamp(
        rcCommand.pitch * m_maxPitchRate_rads,
        -m_maxPitchRate_rads,
        m_maxPitchRate_rads
    );

    setpoint.yawRadS = std::clamp(
        rcCommand.yaw * m_maxYawRate_rads,
        -m_maxYawRate_rads,
        m_maxYawRate_rads
    );

    setpoint.valid = true;
    return setpoint;
}

ControlOutput RateController::Update(
    const RateSetpoint& setpoint,
    const VehicleState& state,
    uint32_t nowUs
)
{
    ControlOutput output{};

    if (!setpoint.valid || !state.valid)
    {
        Reset();
        return output;
    }

    const float dt = ComputeDtSeconds(nowUs);

    if (dt <= 0.0f)
    {
        return output;
    }

    const float targetRollRateRadS = std::clamp(
        setpoint.rollRadS,
        -m_maxRollRate_rads,
        m_maxRollRate_rads
    );

    const float targetPitchRateRadS = std::clamp(
        setpoint.pitchRadS,
        -m_maxPitchRate_rads,
        m_maxPitchRate_rads
    );

    const float targetYawRateRadS = std::clamp(
        setpoint.yawRadS,
        -m_maxYawRate_rads,
        m_maxYawRate_rads
    );

    output.roll = m_rollPid.Update(
        targetRollRateRadS,
        state.rollRateRadS,
        dt
    );

    output.pitch = m_pitchPid.Update(
        targetPitchRateRadS,
        state.pitchRateRadS,
        dt
    );

    output.yaw = m_yawPid.Update(
        targetYawRateRadS,
        state.yawRateRadS,
        dt
    );

    constexpr float RollPitchDeadband = 0.015f;
    constexpr float YawDeadband = 0.005f;

    output.roll = MathUtils::ApplyDeadband(output.roll, RollPitchDeadband);
    output.pitch = MathUtils::ApplyDeadband(output.pitch, RollPitchDeadband);
    output.yaw = MathUtils::ApplyDeadband(output.yaw, YawDeadband);

    m_rateData.targetRollRad = targetRollRateRadS;
    m_rateData.targetPitchRad = targetPitchRateRadS;
    m_rateData.targetYawRad = targetYawRateRadS;

    return output;
}

void RateController::Reset()
{
    m_rollPid.Reset();
    m_pitchPid.Reset();
    m_yawPid.Reset();

    m_lastUpdateUs = 0;
    m_hasLastUpdate = false;
    m_rateData = {};
}

const RateData& RateController::GetRateData()
{
    return m_rateData;
}

float RateController::ComputeDtSeconds(uint32_t nowUs)
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
