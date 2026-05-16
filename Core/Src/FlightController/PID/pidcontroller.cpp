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
        return 0.0f;
    }

    const float error = target - measured;

    m_integral += error * dt;
    m_integral = MathUtils::Clamp(m_integral, m_minIntegral, m_maxIntegral);

    float derivative = 0.0f;

    if (m_hasPreviousError)
    {
        derivative = (measured - m_previousError) / dt;
    }

    m_previousError = measured;
    m_hasPreviousError = true;

    const float output =
        m_kp * error +
        m_ki * m_integral +
        -m_kd * derivative;

    return MathUtils::Clamp(output, m_minOutput, m_maxOutput);
}

void PidController::Reset()
{
    m_integral = 0.0f;
    m_previousError = 0.0f;
    m_hasPreviousError = false;
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