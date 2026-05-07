//
// Created by Dmytro Hrachov on 07.05.2026.
//
#pragma once

#include <cstdint>
#include <array>

class CrsfReceiver
{
public:
    static constexpr uint8_t CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8;
    static constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
    static constexpr uint8_t CRSF_MAX_FRAME_SIZE = 64;

    struct Channels
    {
        std::array<uint16_t, 16> raw{};
        std::array<uint16_t, 16> us{};
        bool valid = false;
    };

public:
    bool ProcessByte(uint8_t byte);
    const Channels& GetChannels() const;
    float NormalizeStick(uint16_t us);
    float NormalizeThrottle(uint16_t us);

private:
    enum class State
    {
        WaitAddress,
        ReadLength,
        ReadPayload
    };

private:
    void Reset();
    static uint8_t Crc8DvbS2(const uint8_t* data, uint8_t len);

    void ParseFrame();

    void DecodeChannels(const uint8_t* payload);
    static uint16_t RawToUs(uint16_t raw);



private:
    State m_state = State::WaitAddress;

    std::array<uint8_t, CRSF_MAX_FRAME_SIZE + 2> m_buffer{};
    uint8_t m_index = 0;
    uint8_t m_frameLength = 0;

    Channels m_channels{};
};