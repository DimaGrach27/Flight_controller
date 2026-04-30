//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include <cstdint>

#include "mavlink/common/mavlink.h"
#include "GlobalDef.h"
#include "SerialPort.h"

NAMESPACE_BEGIN
struct MotorOutputs
{
    double left = 0.5;
    double right = 0.5;
};

class MavlinkBridge
{
public:
    bool Open(const std::string& port, int baud);

    void Poll();

    void SendHilSensor(
        uint64_t timeUsec,
        double angleRad,
        double angularVelocityRad
    );

    const MotorOutputs& Motors() const;

private:
    void HandleMessage(const mavlink_message_t& msg);

    static double PwmToMotor(uint16_t pwm);

private:
    SerialPort serial_;
    MotorOutputs motors_;
};
NAMESPACE_END