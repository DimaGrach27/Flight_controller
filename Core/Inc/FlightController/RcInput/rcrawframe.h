//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include <array>
#include <cstdint>

struct RcRawFrame
{
    static constexpr uint8_t MaxChannels = 16;

    std::array<uint16_t, MaxChannels> channels{};

    uint8_t channelCount = 0;

    uint32_t timestampUs = 0;

    bool failsafe = true;
    bool valid = false;
};