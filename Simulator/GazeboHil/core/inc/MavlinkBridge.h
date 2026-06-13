//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <unordered_map>

#include "CsvLogger.h"
#include "mavlink/common/mavlink.h"
#include "GlobalDef.h"
#include "Structs.h"
#include "ISerialPort.h"

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
    using LogCallback = std::function<void(const std::unordered_map<std::string, float>&)>;

    MavlinkBridge();

    bool Open(const std::string& port, int baud, LogCallback callback);

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
    enum class BinaryLogParseState
    {
        WaitSync0,
        WaitSync1,
        ReadFrame,
    };

    void HandleMessage(const mavlink_message_t& msg);
    void ParseRxByte(uint8_t byte);
    void FeedMavlinkByte(uint8_t byte);
    void ResetBinaryLogParser();
    void TryHandleBinaryLogFrame();

    static double PwmToMotor(uint16_t pwm);

private:
    std::unique_ptr<ISerialPort> m_serial = nullptr;
    MotorOutputs motors_;

    LogCallback m_logCallback;

    BinaryLogParseState m_binaryLogParseState = BinaryLogParseState::WaitSync0;
    std::vector<uint8_t> m_binaryLogFrame;
    uint16_t m_expectedBinaryLogFrameSize = 0;
};
NAMESPACE_END
