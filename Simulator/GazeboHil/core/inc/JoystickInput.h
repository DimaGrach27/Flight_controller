//
// Created by Dmytro Hrachov on 02.05.2026.
//
#pragma once

#include "GlobalDef.h"

NAMESPACE_BEGIN
struct ManualControl
{
    double roll = 0.0;      // -1..1
    double pitch = 0.0;     // -1..1
    double throttle = 0.5;  // 0..1
    double yaw = 0.0;       // -1..1
    bool arm = false;
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
    static double ApplyDeadzone(double value, double deadzone);
    static double NormalizeThrottle(double value);

private:
    void* m_joystick = nullptr;
    ManualControl m_control;
};
NAMESPACE_END
