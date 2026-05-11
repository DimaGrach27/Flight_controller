//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include <cstdint>

enum class FlightMode : uint8_t
{
    Acro = 0,
    Angle = 1,
};

enum class ArmState : uint8_t
{
    Disarmed = 0,
    Armed
};

struct FlightModeState
{
    FlightMode mode = FlightMode::Acro;
    ArmState armState = ArmState::Disarmed;

    bool failsafe = true;
    bool throttleLow = false;
    bool canArm = false;

    uint32_t timestampUs = 0;
};