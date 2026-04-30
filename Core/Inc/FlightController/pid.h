//
// Created by Dmytro Hrachov on 29.04.2026.
//

#pragma once


typedef struct
{
    float kp;
    float ki;
    float kd;

    float integrator;
    float previousError;

    float integratorLimit;
} PID;

float PID_Update(PID *pid, float target, float measured, float dt);
float PID_UpdateAngleWithGyroD(PID *pid, float targetAngleDeg, float measuredAngleDeg, float gyroDegPerSec, float dt);
