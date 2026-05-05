//
// Created by Dmytro Hrachov on 01.05.2026.
//
#include "FlightController/mathutils.h"

#include <cmath>

namespace MathUtils
{
    float Clamp(const float value, const float minValue, const float maxValue)
    {
        if (value < minValue)
            return minValue;

        if (value > maxValue)
            return maxValue;

        return value;
    }

    int16_t Clamp(const int16_t value, const int16_t minValue, const int16_t maxValue)
    {
        if (value < minValue)
            return minValue;

        if (value > maxValue)
            return maxValue;

        return value;
    }

    uint16_t Clamp(const uint16_t value, const uint16_t minValue, const uint16_t maxValue)
    {
        if (value < minValue)
            return minValue;

        if (value > maxValue)
            return maxValue;

        return value;
    }

    float Clamp01(const float value)
    {
        constexpr float minValue = 0.0f;
        constexpr float maxValue = 1.0f;

        if (value < minValue)
            return minValue;

        if (value > maxValue)
            return maxValue;

        return value;
    }

    float ApplyDeadband(const float input, const float deadband)
    {
        if (std::abs(input) < deadband)
            return 0.0f;

        return input;
    }

    int16_t ApplyDeadband(const int16_t input, const int16_t deadband)
    {
        if (std::abs(input) < deadband)
            return 0;

        return input;
    }

    uint16_t ApplyDeadband(const uint16_t input, const uint16_t deadband)
    {
        if (std::abs(input) < deadband)
            return 0;

        return input;
    }
}
