//
// Created by Dmytro Hrachov on 01.05.2026.
//
#pragma once
#include <cstdint>

struct  PID
{
    float kp;
    float ki;
    float kd;

    float integrator;
    float previousError;

    float integratorLimit;
};

class PID_Controller
{
public:
    static int16_t Update(PID *pid, float target, float measured, float dt);
    static int16_t UpdateAngleWithGyroD(PID *pid, float targetAngleDeg, float measuredAngleDeg, float gyroDegPerSec, float dt);
};