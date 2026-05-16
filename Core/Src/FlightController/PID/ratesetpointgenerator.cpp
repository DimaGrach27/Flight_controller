//
// Created by Dmytro Hrachov on 16.05.2026.
//

#include "../../../Inc/FlightController/PID/ratesetpointgenerator.h"

#include <cmath>

#include "FlightController/Utils/computing.h"
#include "FlightController/Utils/mathutils.h"

namespace
{
    constexpr float DegToRad = 0.01745329252f;

    constexpr float RcExpoRollPitch = 0.30f;
    constexpr float RcExpoYaw = 0.20f;

    constexpr float RollPitchMaxSlewRate_radss = 30.0f;
    constexpr float YawMaxSlewRate_radss = 20.0f;
}

RateSetpointGenerator::RateSetpointGenerator()
    : m_maxRollRateRps(300.0f * DegToRad)
    , m_maxPitchRateRps(300.0f * DegToRad)
    , m_maxYawRateRps(180.0f * DegToRad)
    , m_deadband(0.03f)
{
}

void RateSetpointGenerator::Update(const RcCommand& rc, const uint32_t nowUs)
{
    if (!rc.valid || rc.failsafe)
    {
        m_lastRateSetpoint.rollRps = 0.0f;
        m_lastRateSetpoint.pitchRps = 0.0f;
        m_lastRateSetpoint.yawRps = 0.0f;
        m_lastRateSetpoint.throttle = 0.0f;
        m_lastRateSetpoint.valid = false;
    }

    const float roll = ApplyDeadband(rc.roll, m_deadband);
    const float pitch = ApplyDeadband(rc.pitch, m_deadband);
    const float yaw = ApplyDeadband(rc.yaw, m_deadband);

    const float rollCmd = ApplyRcExpo(roll, RcExpoRollPitch);
    const float pitchCmd = ApplyRcExpo(pitch, RcExpoRollPitch);
    const float yawCmd = ApplyRcExpo(yaw, RcExpoYaw);

    const float desiredRollRate_rads = LimitRateByStick(rollCmd, m_maxRollRateRps);
    const float desiredPitchRate_rads = LimitRateByStick(pitchCmd, m_maxPitchRateRps);
    const float desiredYawRate_rads = LimitRateByStick(yawCmd, m_maxYawRateRps);

    const float dt = Computing::ComputeDtSeconds(nowUs, m_hasUpdate, m_lastUpdateTimeUs);

    const float targetRollRate_rads = SlewRateTarget(desiredRollRate_rads, m_lastRateSetpoint.rollRps, dt, RollPitchMaxSlewRate_radss);
    const float targetPitchRate_rads = SlewRateTarget(desiredPitchRate_rads, m_lastRateSetpoint.pitchRps, dt, RollPitchMaxSlewRate_radss);
    const float targetYawRate_rads = SlewRateTarget(desiredYawRate_rads, m_lastRateSetpoint.yawRps, dt, YawMaxSlewRate_radss);

    m_lastRateSetpoint.rollRps = targetRollRate_rads;
    m_lastRateSetpoint.pitchRps = targetPitchRate_rads;
    m_lastRateSetpoint.yawRps = targetYawRate_rads;
    m_lastRateSetpoint.throttle = rc.throttle;
    m_lastRateSetpoint.valid = true;
}

const RateSetpoint RateSetpointGenerator::GetRateSetpoint()
{
    return m_lastRateSetpoint;
}

float RateSetpointGenerator::ApplyDeadband(float value, float deadband) const
{
    if (std::fabs(value) < deadband)
    {
        return 0.0f;
    }

    return value;
}

float RateSetpointGenerator::ApplyRcExpo(float input, float expo) const
{
    const float x3 = input * input * input;
    return input * (1.0f - expo) + x3 * expo;
}

float RateSetpointGenerator::LimitRateByStick(float normalizedStick, float maxRate) const
{
    const float absStick = (normalizedStick >= 0.0f) ? normalizedStick : -normalizedStick;
    const float authority = 0.35f + 0.65f * absStick;

    return normalizedStick * maxRate * authority;
}

float RateSetpointGenerator::SlewRateTarget(float target, float previous, float dt, float maxSlewRate) const
{
    const float maxStep = maxSlewRate * dt;
    const float delta = target - previous;
    const float limitedDelta = MathUtils::Clamp(delta, -maxStep, maxStep);

    return previous + limitedDelta;
}