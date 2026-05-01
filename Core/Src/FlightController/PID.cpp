//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "FlightController/PID.h"

#include "FlightController/mathutils.h"

float PID_Controller::Update(PID *pid, float target, float measured, float dt)
{
    if (dt <= 0.000001f)
    {
        return 0.0f;
    }

    float error = target - measured;

    pid->integrator += error * dt;
    pid->integrator = MathUtils::Clamp(pid->integrator, -pid->integratorLimit, pid->integratorLimit);

    float derivative = (error - pid->previousError) / dt;
    pid->previousError = error;

    return pid->kp * error
         + pid->ki * pid->integrator
         + pid->kd * derivative;
}

float PID_Controller::UpdateAngleWithGyroD(PID *pid,
    float targetAngleDeg, float measuredAngleDeg, float gyroDegPerSec,
    float dt)
{
    float error = targetAngleDeg - measuredAngleDeg;

    pid->integrator += error * dt;
    pid->integrator = MathUtils::Clamp(
        pid->integrator,
        -pid->integratorLimit,
        pid->integratorLimit
    );

    float p = pid->kp * error;
    float i = pid->ki * pid->integrator;
    float d = -pid->kd * gyroDegPerSec;

    return p + i + d;
}
