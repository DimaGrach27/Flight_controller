//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/PID/ratecontroller.h"

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
    /*
        Стартові PID gains дуже обережні.

        Для реального дрона ці значення треба тюнити.
        Для симулятора вони теж можуть бути іншими.
    */
    m_rollPid.Init(0.016f, 0.0f, 0.0f);
    m_pitchPid.Init(0.016f, 0.0f, 0.0f);
    m_yawPid.Init(0.012f, 0.0f, 0.0f);

    m_rollPid.SetOutputLimit(-0.4f, 0.4f);
    m_pitchPid.SetOutputLimit(-0.4f, 0.4f);
    m_yawPid.SetOutputLimit(-0.2f, 0.2f);

    m_rollPid.SetIntegralLimit(-0.2f, 0.2f);
    m_pitchPid.SetIntegralLimit(-0.2f, 0.2f);
    m_yawPid.SetIntegralLimit(-0.1f, 0.1f);

    m_maxRollRate_rads = MaxRollRate_dps * DegToRad;
    m_maxPitchRate_rads = MaxPitchRate_dps * DegToRad;
    m_maxYawRate_rads = MaxYawRate_dps * DegToRad;

    Reset();
}

ControlOutput RateController::Update(
    const RcCommand& rcCommand,
    const VehicleState& state,
    uint32_t nowUs
)
{
    ControlOutput output{};

    if (!rcCommand.valid || rcCommand.failsafe || !state.valid)
    {
        Reset();
        return output;
    }

    const float dt = ComputeDtSeconds(nowUs);

    if (dt <= 0.0f)
    {
        return output;
    }

    const float targetRollRate_rads = rcCommand.roll * m_maxRollRate_rads;
    const float targetPitchRate_rads = rcCommand.pitch * m_maxPitchRate_rads;
    const float targetYawRate_rads = rcCommand.yaw * m_maxYawRate_rads;

    output.roll = m_rollPid.Update(targetRollRate_rads,state.rollRateRadS, dt);
    output.pitch = m_pitchPid.Update(targetPitchRate_rads,state.pitchRateRadS, dt);
    output.yaw = m_yawPid.Update(targetYawRate_rads,state.yawRateRadS, dt);

    // constexpr float ROLL_PITCH_DEADBAND = 0.005f;
    // constexpr float YAW_DEADBAND = 0.005f;

    // output.roll = MathUtils::ApplyDeadband(output.roll, ROLL_PITCH_DEADBAND);
    // output.pitch = MathUtils::ApplyDeadband(output.pitch, ROLL_PITCH_DEADBAND);
    // output.yaw = MathUtils::ApplyDeadband(output.yaw, YAW_DEADBAND);

    // output.roll = -output.roll;
    // output.pitch = -output.pitch;
    // output.yaw = -output.yaw;
    // output.roll = 0.0f;
    // output.pitch = 0.0f;
    // output.yaw = 0.02f;

    m_rateData.targetRollRad = targetRollRate_rads;
    m_rateData.targetPitchRad = targetPitchRate_rads;
    m_rateData.targetYawRad = targetYawRate_rads;

    return output;
}

void RateController::Reset()
{
    m_rollPid.Reset();
    m_pitchPid.Reset();
    m_yawPid.Reset();

    m_lastUpdateUs = 0;
    m_hasLastUpdate = false;
}

const RateData & RateController::GetRateData()
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
