#pragma once

#include <string>
#include <unordered_map>

#include "MavlinkBridge.h"
#include "GlobalDef.h"

NAMESPACE_BEGIN
struct TelemetryOverlayState
{
    double simTimeSec = 0.0;
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double yawDeg = 0.0;
    double rollRateDegSec = 0.0;
    double pitchRateDegSec = 0.0;
    double yawRateDegSec = 0.0;
    double rollTorque = 0.0;
    double pitchTorque = 0.0;
    double yawTorque = 0.0;
    bool hasAttitude = false;
    bool hasTorque = false;
};

std::string FormatTelemetryOverlay(
    const std::unordered_map<std::string, float>& fields,
    const MotorOutputs& motors,
    const TelemetryOverlayState& state
);
NAMESPACE_END
