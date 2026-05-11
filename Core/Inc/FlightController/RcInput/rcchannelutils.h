//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include <cstdint>

#include "FlightController/Utils/mathutils.h"

namespace RcChannelUtils
{
    inline uint16_t FromNormalizedCentered(
        float value,
        uint16_t minValue,
        uint16_t midValue,
        uint16_t maxValue
    )
    {
        value = MathUtils::Clamp(value, -1.0f, 1.0f);

        if (value >= 0.0f)
        {
            return static_cast<uint16_t>(
                static_cast<float>(midValue) +
                value * static_cast<float>(maxValue - midValue)
            );
        }

        return static_cast<uint16_t>(
            static_cast<float>(midValue) +
            value * static_cast<float>(midValue - minValue)
        );
    }

    inline uint16_t FromNormalizedThrottle(
        float value,
        uint16_t minValue,
        uint16_t maxValue
    )
    {
        value = MathUtils::Clamp01(value);

        return static_cast<uint16_t>(
            static_cast<float>(minValue) +
            value * static_cast<float>(maxValue - minValue)
        );
    }

    inline uint16_t FromSwitch(bool enabled, uint16_t minValue, uint16_t maxValue)
    {
        return enabled ? maxValue : minValue;
    }
}
