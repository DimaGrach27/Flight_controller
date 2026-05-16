//
// Created by Dmytro Hrachov on 16.05.2026.
//
#pragma once
#include "FlightController/datastructs.h"

struct RateSetpoint
{
    float rollRps;
    float pitchRps;
    float yawRps;
    float throttle;
    bool valid;
};

class RateSetpointGenerator
{
public:
    RateSetpointGenerator();

    void Update(const RcCommand& rc, const uint32_t nowUs);
    const RateSetpoint GetRateSetpoint();

private:
    float ApplyDeadband(float value, float deadband) const;
    float ApplyRcExpo(float input, float expo) const;
    float LimitRateByStick(float normalizedStick, float maxRate) const;
    float SlewRateTarget(float target, float previous, float dt, float maxSlewRate) const;

private:
    RateSetpoint m_lastRateSetpoint{};

    uint32_t m_lastUpdateTimeUs;
    bool m_hasUpdate;

    float m_maxRollRateRps;
    float m_maxPitchRateRps;
    float m_maxYawRateRps;
    float m_deadband;
};
