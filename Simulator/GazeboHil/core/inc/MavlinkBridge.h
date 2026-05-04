//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include <unordered_map>

#include "CsvLogger.h"
#include "mavlink/common/mavlink.h"
#include "GlobalDef.h"
#include "Structs.h"
#include "SerialPort.h"

NAMESPACE_BEGIN
struct MotorOutputs
{
    double m1 = 0.5;
    double m2 = 0.5;
    double m3 = 0.5;
    double m4 = 0.5;
};

class MavlinkBridge
{
public:
    bool Open(const std::string& port, int baud);

    void Poll();

    void SendHilSensor(
        uint64_t timeUsec,
        double rollRad,
        double pitchRad,
        double yawRad,
        double rollRateRad,
        double pitchRateRad,
        double yawRateRad
    );

    void SendHilSensorFromImu(
        uint64_t timeUsec,
        const ImuData& imuData
    );

    void SendManualControl(
        bool armStatus,
        bool acroMode,
        double roll,
        double pitch,
        double throttle,
        double yaw
    );

    const MotorOutputs& Motors() const;

private:
    void HandleMessage(const mavlink_message_t& msg);

    static double PwmToMotor(uint16_t pwm);

    void HandleNamedValueFloat(const mavlink_named_value_float_t& value);
private:
    SerialPort serial_;
    MotorOutputs motors_;
    CsvLogger m_csvLogger;

    std::unordered_map<std::string, float> m_currentLogFields;
    bool m_isCollectingLogSample = false;
};
NAMESPACE_END