//
// Created by Dmytro Hrachov on 29.07.2026.
//

#include "FlightController/PID/anglecontroller.h"

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

AngleController::AngleController()
{
}

void AngleController::Init()
{
    m_rollPid.Init(AngleModeRateGain, 0.0f, 0.0f);
    m_pitchPid.Init(AngleModeRateGain, 0.0f, 0.0f);

    m_maxRollAngleRad = MaxAngleModeRoll_deg * DegToRad;
    m_maxPitchAngleRad = MaxAngleModePitch_deg * DegToRad;
    m_maxRollRateRadS = MaxRollRate_dps * DegToRad;
    m_maxPitchRateRadS = MaxPitchRate_dps * DegToRad;
    m_maxYawRateRadS = MaxYawRate_dps * DegToRad;

    m_rollPid.SetOutputLimit(-m_maxRollRateRadS, m_maxRollRateRadS);
    m_pitchPid.SetOutputLimit(-m_maxPitchRateRadS, m_maxPitchRateRadS);

    m_rollPid.SetIntegralLimit(0.0f, 0.0f);
    m_pitchPid.SetIntegralLimit(0.0f, 0.0f);

#if NOT_USE_HIL
    m_rollTargetSign = 1;
    m_pitchTargetSign = 1;
    m_yawTargetSign = 1;

    m_rollFeedbackSign = 1;
    m_pitchFeedbackSign = 1;
#else
    m_rollTargetSign = 1;
    m_pitchTargetSign = 1;
    m_yawTargetSign = 1;

    m_rollFeedbackSign = 1;
    m_pitchFeedbackSign = 1;
#endif

    Reset();
}

RateTargets AngleController::Update(
    const RcCommand& rcCommand,
    const VehicleState& state,
    uint32_t nowUs
)
{
    RateTargets rateTargets{};

    if (!rcCommand.valid || rcCommand.failsafe || !state.valid)
    {
        Reset();
        return rateTargets;
    }

    const float dt = ComputeDtSeconds(nowUs);

    if (dt <= 0.0f)
    {
        m_angleData.dt = 0.0f;
        return rateTargets;
    }

    const float targetRollAngleRad =
        rcCommand.roll * m_rollTargetSign * m_maxRollAngleRad;

    const float targetPitchAngleRad =
        rcCommand.pitch * m_pitchTargetSign * m_maxPitchAngleRad;

    const float measuredRollAngleRad =
        state.rollRad * m_rollFeedbackSign;

    const float measuredPitchAngleRad =
        state.pitchRad * m_pitchFeedbackSign;

    rateTargets.rollRateRadS = m_rollPid.Update(
        targetRollAngleRad,
        measuredRollAngleRad,
        dt
    );

    rateTargets.pitchRateRadS = m_pitchPid.Update(
        targetPitchAngleRad,
        measuredPitchAngleRad,
        dt
    );

    rateTargets.yawRateRadS =
        rcCommand.yaw * m_yawTargetSign * m_maxYawRateRadS;
    rateTargets.valid = true;

    m_angleData.targetRollRad = targetRollAngleRad;
    m_angleData.targetPitchRad = targetPitchAngleRad;
    m_angleData.measuredRollRad = measuredRollAngleRad;
    m_angleData.measuredPitchRad = measuredPitchAngleRad;
    m_angleData.dt = dt;
    m_angleData.rollPid = m_rollPid.GetDebugData();
    m_angleData.pitchPid = m_pitchPid.GetDebugData();

    return rateTargets;
}

void AngleController::Reset()
{
    m_rollPid.Reset();
    m_pitchPid.Reset();

    m_lastUpdateUs = 0;
    m_hasLastUpdate = false;
    m_angleData = {};
}

const AngleData& AngleController::GetAngleData() const
{
    return m_angleData;
}

float AngleController::ComputeDtSeconds(uint32_t nowUs)
{
    if (!m_hasLastUpdate)
    {
        m_lastUpdateUs = nowUs;
        m_hasLastUpdate = true;
        return 0.0f;
    }

    const uint32_t dtUs = nowUs - m_lastUpdateUs;
    m_lastUpdateUs = nowUs;

    float dt = static_cast<float>(dtUs) * 0.000001f;
    dt = MathUtils::Clamp(dt, MinDtSeconds, MaxDtSeconds);

    return dt;
}
