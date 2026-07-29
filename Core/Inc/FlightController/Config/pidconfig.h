//
// Created by Codex on 29.07.2026.
//
#pragma once

struct PidAxisGains
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

struct PidConfig
{
    PidAxisGains rateRoll;
    PidAxisGains ratePitch;
    PidAxisGains rateYaw;

    PidAxisGains angleRoll;
    PidAxisGains anglePitch;
};
