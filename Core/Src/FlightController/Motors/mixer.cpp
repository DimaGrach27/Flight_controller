//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Motors/mixer.h"

#include "FlightController/Motors/motorcommand.h"
#include "FlightController/Utils/mathutils.h"

#include <cmath>

Mixer::Mixer()
{
}

void Mixer::Init()
{
    m_controlDirectionConfig = {
        .rollSign = 1,
        .pitchSign = -1,
        .yawSign = -1,
    };
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

    float roll = control.roll * m_controlDirectionConfig.rollSign;
    float pitch = control.pitch * m_controlDirectionConfig.pitchSign;
    float yaw = control.yaw * m_controlDirectionConfig.yawSign;


    float correction[4] =
    {
        -roll + pitch + yaw,
         roll - pitch + yaw,
         roll + pitch - yaw,
        -roll - pitch - yaw
    };

    //find max correction
    float maxCorrectionAbs = 0.0f;

    for (float value : correction)
    {
        const float absValue = std::abs(value);
        if (absValue > maxCorrectionAbs)
        {
            maxCorrectionAbs = absValue;
        }
    }

    constexpr float MaxOutput = 1.0f;
    constexpr float MinOutput = 0.0f;

    //find headroom for corrections
    const float upperHeadroom = MaxOutput - throttle;
    const float lowerHeadroom = throttle - MinOutput;
    const float availableHeadroom = upperHeadroom < lowerHeadroom ? upperHeadroom : lowerHeadroom;

    float correctionScale = 1.0f;

    if (maxCorrectionAbs > availableHeadroom && maxCorrectionAbs > 0.000001f)
    {
        correctionScale = availableHeadroom / maxCorrectionAbs;
    }

    float mixed[4] =
    {
        throttle + correction[0] * correctionScale,
        throttle + correction[1] * correctionScale,
        throttle + correction[2] * correctionScale,
        throttle + correction[3] * correctionScale
    };


    //saturate
    float minMixed = mixed[0];
    float maxMixed = mixed[0];

    for (float value : mixed)
    {
        if (value < minMixed)
        {
            minMixed = value;
        }

        if (value > maxMixed)
        {
            maxMixed = value;
        }
    }

    if (maxMixed > MaxOutput)
    {
        const float shift = maxMixed - MaxOutput;

        for (float& value : mixed)
        {
            value -= shift;
        }
    }

    minMixed = mixed[0];

    for (float value : mixed)
    {
        if (value < minMixed)
        {
            minMixed = value;
        }
    }

    if (minMixed < MinOutput)
    {
        const float shift = MinOutput - minMixed;

        for (float& value : mixed)
        {
            value += shift;
        }
    }

    // motors.m1 = throttle - roll + pitch + yaw; // front right
    // motors.m2 = throttle + roll - pitch + yaw; // rear left
    // motors.m3 = throttle + roll + pitch - yaw; // front left
    // motors.m4 = throttle - roll - pitch - yaw; // rear right

    //яв треба поправити щоб повертався в ту сторону

    // Desaturate(motors);

    motors.m1 = MathUtils::Clamp(mixed[0], MinOutput, MaxOutput);
    motors.m2 = MathUtils::Clamp(mixed[1], MinOutput, MaxOutput);
    motors.m3 = MathUtils::Clamp(mixed[2], MinOutput, MaxOutput);
    motors.m4 = MathUtils::Clamp(mixed[3], MinOutput, MaxOutput);

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
