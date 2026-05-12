//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Motors/mixer.h"

#include "FlightController/Motors/motorcommand.h"
#include "FlightController/Utils/mathutils.h"

Mixer::Mixer()
{
}

void Mixer::Init()
{
}

MotorCommand Mixer::Mix(float throttle, const ControlOutput& control) const
{
    throttle = MathUtils::Clamp01(throttle);

    MotorCommand motors{};

    /*
        Motor layout:

              front

           m3       m1
        front-left front-right

           m2       m4
        rear-left  rear-right

              rear

        m1 = front right
        m2 = rear left
        m3 = front left
        m4 = rear right

        Це X-frame mixer.
    */

    if (throttle <= m_minThrottle)
    {
        motors.m1 = m_idleThrottle;
        motors.m2 = m_idleThrottle;
        motors.m3 = m_idleThrottle;
        motors.m4 = m_idleThrottle;

        return motors;
    }

    motors.m1 = throttle - control.roll + control.pitch + control.yaw; // front right
    motors.m2 = throttle + control.roll + control.pitch - control.yaw; // rear left
    motors.m3 = throttle + control.roll - control.pitch + control.yaw; // front left
    motors.m4 = throttle - control.roll - control.pitch - control.yaw; // rear right

    Desaturate(motors);

    motors.Clamp01();

    return motors;
}

void Mixer::Desaturate(MotorCommand &motors) const
{
    const float minMotor = motors.Min();
    const float maxMotor = motors.Max();

    /*
        Якщо всі мотори вище 1.0, зміщуємо вниз.
    */
    if (maxMotor > 1.0f)
    {
        motors.Add(1.0f - maxMotor);
    }

    /*
        Якщо якийсь мотор нижче 0.0, зміщуємо вгору.
    */
    if (minMotor < 0.0f)
    {
        motors.Add(-minMotor);
    }
}
