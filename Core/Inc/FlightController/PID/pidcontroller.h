//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "FlightController/structs.h"

class PidController
{
public:
    PidController();

    void Init(float kp, float ki, float kd);

    float Update(float target, float measured, float dt);

    void Reset();

    void SetGains(float kp, float ki, float kd);
    void SetOutputLimit(float minOutput, float maxOutput);
    void SetIntegralLimit(float minIntegral, float maxIntegral);

    const PidDebugData& GetDebugData() const;

private:
    float m_kp = 0.0f;
    float m_ki = 0.0f;
    float m_kd = 0.0f;

    float m_integral = 0.0f;
    float m_previousError = 0.0f;

    float m_minOutput = -1.0f;
    float m_maxOutput = 1.0f;

    float m_minIntegral = -0.3f;
    float m_maxIntegral = 0.3f;

    bool m_hasPreviousError = false;

    PidDebugData m_debugData{};
};
