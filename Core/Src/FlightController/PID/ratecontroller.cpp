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
    constexpr float MaxAngleModeRoll_deg = 25.0f;
    constexpr float MaxAngleModePitch_deg = 25.0f;
    constexpr float AngleModeRateGain = 4.0f;

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
#if NOT_USE_HIL
    m_rollPid.Init(0.065f, 0.045f, 0.0015f);
    m_pitchPid.Init(0.065f, 0.045f, 0.0015f);
    m_yawPid.Init(0.090f, 0.025f, 0.0000f);

    m_rollPid.SetOutputLimit(-0.65f, 0.65f);
    m_pitchPid.SetOutputLimit(-0.65f, 0.65f);
    m_yawPid.SetOutputLimit(-0.35f, 0.35f);

    m_rollPid.SetIntegralLimit(-0.25f, 0.25f);
    m_pitchPid.SetIntegralLimit(-0.25f, 0.25f);
    m_yawPid.SetIntegralLimit(-0.15f, 0.15f);
#else
    m_rollPid.Init(0.050f, 0.0f, 0.0f);
    m_pitchPid.Init(0.050f, 0.0f, 0.0f);
    m_yawPid.Init(0.020f, 0.0f, 0.0f);

    m_rollPid.SetOutputLimit(-0.16f, 0.16f);
    m_pitchPid.SetOutputLimit(-0.16f, 0.16f);
    m_yawPid.SetOutputLimit(-0.07f, 0.07f);

    m_rollPid.SetIntegralLimit(0.0f, 0.0f);
    m_pitchPid.SetIntegralLimit(0.0f, 0.0f);
    m_yawPid.SetIntegralLimit(0.0f, 0.0f);
#endif

    m_maxRollRate_rads = MaxRollRate_dps * DegToRad;
    m_maxPitchRate_rads = MaxPitchRate_dps * DegToRad;
    m_maxYawRate_rads = MaxYawRate_dps * DegToRad;

#if NOT_USE_HIL
    m_rollTargetSign = 1;
    m_pitchTargetSign = 1;
    m_yawTargetSign = 1;

    m_rollFeedbackSign = 1;
    m_pitchFeedbackSign = 1;
    m_yawFeedbackSign = 1;
#else
    m_rollTargetSign = 1;
    m_pitchTargetSign = 1;
    m_yawTargetSign = 1;

    m_rollFeedbackSign = 1;
    m_pitchFeedbackSign = 1;
    m_yawFeedbackSign = 1;
#endif

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

    const float targetRollRate_rads =
        rcCommand.roll * m_rollTargetSign * m_maxRollRate_rads;

    const float targetPitchRate_rads =
        rcCommand.pitch * m_pitchTargetSign * m_maxPitchRate_rads;

    const float targetYawRate_rads =
        rcCommand.yaw * m_yawTargetSign * m_maxYawRate_rads;

    return UpdateRateTargets(
        targetRollRate_rads,
        targetPitchRate_rads,
        targetYawRate_rads,
        state,
        nowUs
    );
}

ControlOutput RateController::UpdateAngleMode(
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

    const float maxRollAngle_rads = MaxAngleModeRoll_deg * DegToRad;
    const float maxPitchAngle_rads = MaxAngleModePitch_deg * DegToRad;

    const float targetRollAngle_rads =
        rcCommand.roll * m_rollTargetSign * maxRollAngle_rads;

    const float targetPitchAngle_rads =
        rcCommand.pitch * m_pitchTargetSign * maxPitchAngle_rads;

    const float measuredRollAngle_rads =
        state.rollRad * m_rollFeedbackSign;

    const float measuredPitchAngle_rads =
        state.pitchRad * m_pitchFeedbackSign;

    const float targetRollRate_rads = MathUtils::Clamp(
        (targetRollAngle_rads - measuredRollAngle_rads) * AngleModeRateGain,
        -m_maxRollRate_rads,
        m_maxRollRate_rads
    );

    const float targetPitchRate_rads = MathUtils::Clamp(
        (targetPitchAngle_rads - measuredPitchAngle_rads) * AngleModeRateGain,
        -m_maxPitchRate_rads,
        m_maxPitchRate_rads
    );

    const float targetYawRate_rads =
        rcCommand.yaw * m_yawTargetSign * m_maxYawRate_rads;

    return UpdateRateTargets(
        targetRollRate_rads,
        targetPitchRate_rads,
        targetYawRate_rads,
        state,
        nowUs
    );
}

ControlOutput RateController::UpdateRateTargets(
    const float targetRollRate_rads,
    const float targetPitchRate_rads,
    const float targetYawRate_rads,
    const VehicleState& state,
    uint32_t nowUs
)
{
    ControlOutput output{};

    const float dt = ComputeDtSeconds(nowUs);

    if (dt <= 0.0f)
    {
        m_rateData.dt = 0.0f;
        return output;
    }

    const float measuredRollRate_rads =
        state.rollRateRadS * m_rollFeedbackSign;

    const float measuredPitchRate_rads =
        state.pitchRateRadS * m_pitchFeedbackSign;

    const float measuredYawRate_rads =
        state.yawRateRadS * m_yawFeedbackSign;

    m_rateData.targetRollRad = targetRollRate_rads;
    m_rateData.targetPitchRad = targetPitchRate_rads;
    m_rateData.targetYawRad = targetYawRate_rads;
    m_rateData.measuredRollRad = measuredRollRate_rads;
    m_rateData.measuredPitchRad = measuredPitchRate_rads;
    m_rateData.measuredYawRad = measuredYawRate_rads;
    m_rateData.dt = dt;

    output.roll = m_rollPid.Update(
        targetRollRate_rads,
        measuredRollRate_rads,
        dt
    );

    output.pitch = m_pitchPid.Update(
        targetPitchRate_rads,
        measuredPitchRate_rads,
        dt
    );

    output.yaw = m_yawPid.Update(
        targetYawRate_rads,
        measuredYawRate_rads,
        dt
    );

#if NOT_USE_HIL
    constexpr float ROLL_PITCH_DEADBAND = 0.015f;
    constexpr float YAW_DEADBAND = 0.005f;
#else
    constexpr float ROLL_PITCH_DEADBAND = 0.002f;
    constexpr float YAW_DEADBAND = 0.002f;
#endif

    output.roll = MathUtils::ApplyDeadband(output.roll, ROLL_PITCH_DEADBAND);
    output.pitch = MathUtils::ApplyDeadband(output.pitch, ROLL_PITCH_DEADBAND);
    output.yaw = MathUtils::ApplyDeadband(output.yaw, YAW_DEADBAND);

    // output.roll = -output.roll;
    // output.pitch = -output.pitch;
    // output.yaw = -output.yaw;
    // output.roll = 0.0f;
    // output.pitch = 0.0f;
    // output.yaw = 0.02f;

    m_rateData.rollPid = m_rollPid.GetDebugData();
    m_rateData.pitchPid = m_pitchPid.GetDebugData();
    m_rateData.yawPid = m_yawPid.GetDebugData();

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
