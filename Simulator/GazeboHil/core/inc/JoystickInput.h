//
// Created by Dmytro Hrachov on 02.05.2026.
//
#pragma once

#include "GlobalDef.h"

NAMESPACE_BEGIN
struct ManualControl
{
    int roll = 0;      // -1000..1000
    int pitch = 0;     // -1000..1000
    int throttle = 50;  // 0..1000
    int yaw = 0;       // -1000..1000
    bool arm = false;
    bool acroMode = false;

    bool valid = false;
};

class JoystickInput
{
public:
    bool Init(int joystickIndex = 0);
    void Shutdown();

    void Poll();

    const ManualControl& Control() const;

private:
    double Axis(int index) const;
    double ApplyDeadzone(double value, double deadzone);
    double ApplyExpo(double value, double expo);
    double NormalizeThrottle(double value);
    int QuantizeAxis(double value) const;

private:
    void* m_joystick = nullptr;
    ManualControl m_control;
};
NAMESPACE_END
