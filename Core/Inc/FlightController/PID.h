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

public:
    inline void SetGains(const float gainsKp, const float gainsKi, const float gainsKd)
    {
        this->kp = gainsKp;
        this->ki = gainsKi;
        this->kd = gainsKd;
    }
};

class PID_Controller
{
public:
    static float Update(PID *pid, float target, float measured, float dt);
    static float UpdateAngleWithGyroD(PID *pid, float targetAngleDeg, float measuredAngleDeg, float gyroDegPerSec, float dt);
};