//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include <cstdint>

#include "rcrawframe.h"
#include "FlightController/Protocols/uartbytestrem.h"

class CrsfRcReceiver
{
public:
    CrsfRcReceiver(UartByteStream& byteStream);

    bool Init();
    bool Update(uint32_t nowUs);
    bool ReadFrame(RcRawFrame& outFrame);

private:
    void PushByte(uint8_t byte);
    void PushBytes(const uint8_t* data, uint16_t size);

    bool TryParseBuffer(uint32_t nowUs);
    bool TryParseFrameAt(uint8_t offset, uint32_t nowUs);

    bool DecodeRcChannels(
        const uint8_t* payload,
        uint8_t payloadSize,
        uint32_t nowUs
    );

    uint16_t ReadPacked11BitChannel(
        const uint8_t* payload,
        uint8_t payloadSize,
        uint8_t channelIndex
    ) const;

    uint8_t ComputeCrc8DvbS2(const uint8_t* data, uint8_t size) const;

    bool IsKnownAddress(uint8_t address) const;
    void RemoveBytesFromBuffer(uint8_t count);

private:
    static constexpr uint8_t MaxFrameSize = 64;
    static constexpr uint8_t MaxParserBufferSize = 96;
    static constexpr uint16_t TempReadBufferSize = 64;

private:
    UartByteStream& m_byteStream;

    RcRawFrame m_latestFrame{};

    uint8_t m_parserBuffer[MaxParserBufferSize]{};
    uint8_t m_parserCount = 0;

    bool m_hasFrame = false;
    bool m_initialized = false;
};
