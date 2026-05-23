//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "FlightController/RcInput/rcreceiver.h"
#include "FlightController/RcInput/rcrawframe.h"
#include "FlightController/datastructs.h"

#include <cstdint>

class RcInput
{
public:
    explicit RcInput(RcReceiver& receiver);

    bool Init();

    bool Update(uint32_t nowUs);

    const RcCommand& GetCommand() const;

private:
    RcCommand ConvertFrameToCommand(const RcRawFrame& frame) const;

    float NormalizeCenteredChannel(uint16_t value) const;
    float NormalizeThrottleChannel(uint16_t value) const;

    bool IsSwitchHigh(uint16_t value) const;

private:
    RcReceiver& m_receiver;

    RcCommand m_command{};

    uint32_t m_lastValidFrameUs = 0;
    uint32_t m_lastConsumedFrameUs = 0;

    bool m_initialized = false;

    /*
        CRSF/SBUS-like range.
        Потім можна винести в конфіг.
    */
    uint16_t m_channelMin = 172;
    uint16_t m_channelMid = 992;
    uint16_t m_channelMax = 1811;

    uint32_t m_failsafeTimeoutUs = 200000; // 200 ms

    uint8_t m_rollChannel = 0;
    uint8_t m_pitchChannel = 1;
    uint8_t m_throttleChannel = 2;
    uint8_t m_yawChannel = 3;

    uint8_t m_armChannel = 4;
    uint8_t m_modeChannel = 5;
};
