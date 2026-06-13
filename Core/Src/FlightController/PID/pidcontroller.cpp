//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/PID/pidcontroller.h"

#include "FlightController/Utils/mathutils.h"

PidController::PidController()
{
}

void PidController::Init(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;

    Reset();
}

float PidController::Update(float target, float measured, float dt)
{
    if (dt <= 0.0f)
    {
        m_debugData = {};
        m_debugData.target = target;
        m_debugData.measured = measured;
        return 0.0f;
    }

    const float error = target - measured;

    m_integral += error * dt;
    m_integral = MathUtils::Clamp(m_integral, m_minIntegral, m_maxIntegral);

    float derivative = 0.0f;

    if (m_hasPreviousError)
    {
        derivative = (error - m_previousError) / dt;
    }

    m_previousError = error;
    m_hasPreviousError = true;

    const float p = m_kp * error;
    const float i = m_ki * m_integral;
    const float d = m_kd * derivative;
    const float unclampedOutput = p + i + d;
    const float output = MathUtils::Clamp(unclampedOutput, m_minOutput, m_maxOutput);

    m_debugData.target = target;
    m_debugData.measured = measured;
    m_debugData.error = error;
    m_debugData.p = p;
    m_debugData.i = i;
    m_debugData.d = d;
    m_debugData.output = output;
    m_debugData.unclampedOutput = unclampedOutput;
    m_debugData.integral = m_integral;
    m_debugData.saturated = output != unclampedOutput;

    return output;
}

void PidController::Reset()
{
    m_integral = 0.0f;
    m_previousError = 0.0f;
    m_hasPreviousError = false;
    m_debugData = {};
}

void PidController::SetGains(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void PidController::SetOutputLimit(float minOutput, float maxOutput)
{
    m_minOutput = minOutput;
    m_maxOutput = maxOutput;
}

void PidController::SetIntegralLimit(float minIntegral, float maxIntegral)
{
    m_minIntegral = minIntegral;
    m_maxIntegral = maxIntegral;
}

const PidDebugData& PidController::GetDebugData() const
{
    return m_debugData;
}
