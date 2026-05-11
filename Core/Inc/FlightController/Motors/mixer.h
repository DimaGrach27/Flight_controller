//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "FlightController/structs.h"
#include "FlightController/Motors/motorcommand.h"

class Mixer
{
public:
    Mixer();

    void Init();

    MotorCommand Mix(float throttle, const ControlOutput& control) const;
    void Desaturate(MotorCommand& motors) const;

private:
    float m_minThrottle = 0.05f;
    float m_idleThrottle = 0.06f;
};
