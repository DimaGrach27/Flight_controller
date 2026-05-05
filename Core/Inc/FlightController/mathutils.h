//
// Created by Dmytro Hrachov on 01.05.2026.
//
#pragma once
#include <cstdint>

namespace MathUtils
{
    float Clamp(const float value, const float minValue, const float maxValue);
    int16_t Clamp(const int16_t value, const int16_t minValue, const int16_t maxValue);
    uint16_t Clamp(const uint16_t value, const uint16_t minValue, const uint16_t maxValue);
    float Clamp01(const float value);
}
