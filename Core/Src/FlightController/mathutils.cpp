//
// Created by Dmytro Hrachov on 01.05.2026.
//
#include "FlightController/mathutils.h"

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
}