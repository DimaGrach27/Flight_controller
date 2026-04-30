//
// Created by Dmytro Hrachov on 29.04.2026.
//

#include "FlightController/helpers.h"

float clamp(float x, float minValue, float maxValue)
{
    if (x < minValue)
        return minValue;

    if (x > maxValue)
        return maxValue;

    return x;
}