#include "TelemetryOverlay.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

NAMESPACE_BEGIN
namespace
{
    float Field(
        const std::unordered_map<std::string, float>& fields,
        const char* key,
        const float fallback = 0.0f)
    {
        const auto it = fields.find(key);
        return it == fields.end() ? fallback : it->second;
    }

    bool HasField(const std::unordered_map<std::string, float>& fields, const char* key)
    {
        return fields.find(key) != fields.end();
    }

    const char* ArmedText(const float armed)
    {
        return armed >= 0.5f ? "ARM" : "SAFE";
    }

    const char* FlightModeText(const float mode)
    {
        return mode >= 0.5f ? "ACRO" : "ANGLE";
    }

    int PercentFromMotor(const double value)
    {
        return static_cast<int>(std::round(std::clamp(value, 0.0, 1.0) * 100.0));
    }

    std::string Bar(const double value, const int width = 10)
    {
        const int filled = static_cast<int>(std::round(std::clamp(value, 0.0, 1.0) * width));
        std::string out;
        out.reserve(static_cast<size_t>(width));

        for (int i = 0; i < width; ++i)
        {
            out += i < filled ? '#' : '.';
        }

        return out;
    }

    void WriteMotor(std::ostringstream& out, const char* name, const double value)
    {
        out
            << name
            << " "
            << std::setw(3)
            << PercentFromMotor(value)
            << "% ["
            << Bar(value, 8)
            << "]";
    }
}

std::string FormatTelemetryOverlay(
    const std::unordered_map<std::string, float>& fields,
    const MotorOutputs& motors,
    const TelemetryOverlayState& state)
{
    const bool hasFirmwareAttitude = HasField(fields, "est_roll");
    const float rollDeg = hasFirmwareAttitude
        ? Field(fields, "est_roll")
        : static_cast<float>(state.rollDeg);
    const float pitchDeg = hasFirmwareAttitude
        ? Field(fields, "est_pitch")
        : static_cast<float>(state.pitchDeg);
    const float yawDeg = hasFirmwareAttitude
        ? Field(fields, "est_yaw")
        : static_cast<float>(state.yawDeg);

    const float rollRateDegSec = HasField(fields, "g_roll")
        ? Field(fields, "g_roll")
        : static_cast<float>(state.rollRateDegSec);
    const float pitchRateDegSec = HasField(fields, "g_pitch")
        ? Field(fields, "g_pitch")
        : static_cast<float>(state.pitchRateDegSec);
    const float yawRateDegSec = HasField(fields, "g_yaw")
        ? Field(fields, "g_yaw")
        : static_cast<float>(state.yawRateDegSec);

    const float armed = Field(fields, "armed", 0.0f);
    const float failsafe = Field(fields, "fs", 0.0f);
    const float flightMode = Field(fields, "f_mode", 0.0f);
    const float batteryVoltage = Field(fields, "bat_v", 0.0f);
    const float cellVoltage = Field(fields, "cell_v", 0.0f);
    const float batteryPercent = Field(fields, "bat_pct", 0.0f);
    const float rcAgeMs = Field(fields, "rc_age", 0.0f);

    std::ostringstream out;
    out << std::fixed << std::setprecision(1);

    out
        << "FC GOGGLES OSD\n"
        << "TIME "
        << std::setw(7)
        << state.simTimeSec
        << "s  "
        << ArmedText(armed)
        << "  "
        << FlightModeText(flightMode);

    if (failsafe >= 0.5f)
    {
        out << "  FAILSAFE";
    }

    out
        << "\nBAT  "
        << std::setw(5)
        << batteryVoltage
        << "V  CELL "
        << std::setw(4)
        << cellVoltage
        << "V  "
        << std::setw(3)
        << static_cast<int>(std::round(batteryPercent))
        << "%";

    out
        << "\nATT  R "
        << std::setw(6)
        << rollDeg
        << "  P "
        << std::setw(6)
        << pitchDeg
        << "  Y "
        << std::setw(6)
        << yawDeg;

    out
        << "\nGYRO R "
        << std::setw(6)
        << rollRateDegSec
        << "  P "
        << std::setw(6)
        << pitchRateDegSec
        << "  Y "
        << std::setw(6)
        << yawRateDegSec
        << " deg/s";

    out
        << "\nRC   THR "
        << std::setw(5)
        << Field(fields, "rc_thr")
        << "  R "
        << std::setw(5)
        << Field(fields, "rc_roll")
        << "  P "
        << std::setw(5)
        << Field(fields, "rc_pitch")
        << "  Y "
        << std::setw(5)
        << Field(fields, "rc_yaw")
        << "  AGE "
        << std::setw(5)
        << rcAgeMs
        << "ms";

    out << "\nMOT  ";
    WriteMotor(out, "M1", motors.m1);
    out << "  ";
    WriteMotor(out, "M2", motors.m2);
    out << "\n     ";
    WriteMotor(out, "M3", motors.m3);
    out << "  ";
    WriteMotor(out, "M4", motors.m4);

    if (state.hasTorque)
    {
        out
            << "\nTRQ  R "
            << std::setw(6)
            << state.rollTorque
            << "  P "
            << std::setw(6)
            << state.pitchTorque
            << "  Y "
            << std::setw(6)
            << state.yawTorque;
    }

    if (HasField(fields, "truth_z"))
    {
        out
            << "\nPOS  X "
            << std::setw(6)
            << Field(fields, "truth_x")
            << "  Y "
            << std::setw(6)
            << Field(fields, "truth_y")
            << "  Z "
            << std::setw(6)
            << Field(fields, "truth_z");
    }

    return out.str();
}
NAMESPACE_END
