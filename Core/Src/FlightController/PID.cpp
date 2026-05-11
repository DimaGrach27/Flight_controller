//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "FlightController/PID.h"

#include "../../Inc/FlightController/Utils/mathutils.h"

float PID_Controller::Update(PID *pid, const float target, const float measured, const float dt)
{
    if (dt <= 0.000001f)
    {
        return 0.0f;
    }

    const float error = target - measured;

    pid->integrator += error * dt;
    pid->integrator = MathUtils::Clamp(pid->integrator, -pid->integratorLimit, pid->integratorLimit);

    const float derivative = (error - pid->previousError) / dt;
    pid->previousError = error;

    const float pidOut = pid->kp * error
         + pid->ki * pid->integrator
         + pid->kd * derivative;

    return pidOut;
}

float PID_Controller::UpdateAngleWithGyroD(PID *pid,
    const float targetAngleDeg, const float measuredAngleDeg, const float gyroDegPerSec,
    const float dt)
{
    float error = targetAngleDeg - measuredAngleDeg;

    pid->integrator += error * dt;
    pid->integrator = MathUtils::Clamp(
        pid->integrator,
        -pid->integratorLimit,
        pid->integratorLimit
    );

    const float p = pid->kp * error;
    const float i = pid->ki * pid->integrator;
    const float d = -pid->kd * gyroDegPerSec;

    const float pidOut = p + i + d;

    return pidOut;
}
