//
// Created by Dmytro Hrachov on 16.05.2026.
//
#pragma once
#include <cstdint>

namespace Computing
{
    constexpr float MinDtSeconds = 0.000001f;
    constexpr float MaxDtSeconds = 0.05f;

    inline float ComputeDtSeconds(const uint32_t nowUs, bool& outHasUpdate, uint32_t& outLastDt)
    {
        if (!outHasUpdate)
        {
            outLastDt = nowUs;
            outHasUpdate = true;
            return 0.0f;
        }

        const uint32_t dtUs = nowUs - outLastDt;
        const float dt = static_cast<float>(dtUs) / 1000000.0f;

        outLastDt = nowUs;

        if (dt < MinDtSeconds)
        {
            return 0.0f;
        }

        if (dt > MaxDtSeconds)
        {
            return 0.0f;
        }

        return dt;
    }
}
