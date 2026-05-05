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
    bool Open(const std::string& port, int baud, std::function<void(const mavlink_named_value_float_t&)> callback);

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
        int roll,
        int pitch,
        int throttle,
        int yaw
    );

    const MotorOutputs& Motors() const;

    uint32_t m_servoRxCount = 0;
    uint32_t m_servoMaxCount = 0;
    double m_lastServoWallSec = 0.0;
    double m_servoDtMin = 999.0;
    double m_servoDtMax = 0.0;

private:
    void HandleMessage(const mavlink_message_t& msg);

    static double PwmToMotor(uint16_t pwm);

private:
    SerialPort serial_;
    MotorOutputs motors_;

    std::function<void(const mavlink_named_value_float_t&)> m_callback;
};
NAMESPACE_END