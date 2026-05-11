//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include <cstdint>

enum class TaskID : uint8_t
{
    Imu = 0,
    Rc = 1,
    Battery = 2,
    Barometer = 3,
    Control = 4,
    Telemetry = 5,

    INVALID = 254,
    COUNT
};
