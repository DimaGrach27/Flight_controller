//
// Created by Codex on 29.07.2026.
//
#pragma once

#include <cstdint>

#include "FlightController/Config/pidconfig.h"

class FlashConfigStorage
{
public:
    enum class Status : uint8_t
    {
        Ok = 0,
        Invalid = 1,
        FlashError = 2,
    };

    Status LoadPidConfig(PidConfig& config) const;
    Status SavePidConfig(const PidConfig& config) const;

private:
    static uint32_t Crc32(const uint8_t* data, uint32_t size);
};
