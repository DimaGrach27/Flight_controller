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

enum class FailsafeReason : uint8_t
{
    None = 0,
    RcInvalid = 1,
    RcFailsafe = 2,
};

enum class ArmDenyReason : uint8_t
{
    None = 0,
    RcInvalid = 1,
    RcFailsafe = 2,
    ArmSwitchLow = 3,
    ThrottleHigh = 4,
    BatteryInvalid = 5,
    BatteryUnsafe = 6,
};

static const char* EnumToChar_FlightMode(const FlightMode value)
{
    switch (value)
    {
        case FlightMode::Acro:
            return "ACRO";
        case FlightMode::Angle:
            return "ANGLE";
    }

    return "UNKNOWN";
}

static const char* EnumToChar_ArmState(const ArmState value)
{
    switch (value)
    {
        case ArmState::Disarmed:
            return "DISARMED";
        case ArmState::Armed:
            return "ARMED";
    }

    return "UNKNOWN";
}

struct FlightModeState
{
    FlightMode mode = FlightMode::Acro;
    ArmState armState = ArmState::Disarmed;

    bool failsafe = true;
    FailsafeReason failsafeReason = FailsafeReason::RcInvalid;
    ArmDenyReason armDenyReason = ArmDenyReason::RcInvalid;
    bool throttleLow = false;
    bool canArm = false;

    uint32_t timestampUs = 0;
};
