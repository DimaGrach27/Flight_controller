//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include "FlightController/Utils/mathutils.h"

struct MotorCommand
{
    float m1 = 0.0f;
    float m2 = 0.0f;
    float m3 = 0.0f;
    float m4 = 0.0f;

    void Clamp01()
    {
        m1 = MathUtils::Clamp01(m1);
        m2 = MathUtils::Clamp01(m2);
        m3 = MathUtils::Clamp01(m3);
        m4 = MathUtils::Clamp01(m4);
    }

    float Min() const
    {
        float value = m1;

        if (m2 < value)
        {
            value = m2;
        }

        if (m3 < value)
        {
            value = m3;
        }

        if (m4 < value)
        {
            value = m4;
        }

        return value;
    }

    float Max() const
    {
        float value = m1;

        if (m2 > value)
        {
            value = m2;
        }

        if (m3 > value)
        {
            value = m3;
        }

        if (m4 > value)
        {
            value = m4;
        }

        return value;
    }

    void Add(float value)
    {
        m1 += value;
        m2 += value;
        m3 += value;
        m4 += value;
    }
};
