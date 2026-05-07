//
// Created by Dmytro Hrachov on 07.05.2026.
//

#include "FlightController/crsfreceiver.h"

bool CrsfReceiver::ProcessByte(uint8_t byte)
{
    switch (m_state)
    {
        case State::WaitAddress:
            if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER)
            {
                m_buffer[0] = byte;
                m_index = 1;
                m_state = State::ReadLength;
            }
            break;

        case State::ReadLength:
            m_frameLength = byte;

            // frameLength рахує: type + payload + crc
            if (m_frameLength < 2 || m_frameLength > CRSF_MAX_FRAME_SIZE)
            {
                Reset();
                return false;
            }

            m_buffer[m_index++] = byte;
            m_state = State::ReadPayload;
            break;

        case State::ReadPayload:
            m_buffer[m_index++] = byte;

            // total bytes = address + length + frameLength
            if (m_index >= static_cast<uint8_t>(m_frameLength + 2))
            {
                ParseFrame();
                Reset();
                return true;
            }
            break;
    }

    return false;
}

const CrsfReceiver::Channels& CrsfReceiver::GetChannels() const
{
    return m_channels;
}

void CrsfReceiver::Reset()
{
    m_state = State::WaitAddress;
    m_index = 0;
    m_frameLength = 0;
}

uint8_t CrsfReceiver::Crc8DvbS2(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; ++i)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x80)
                crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
            else
                crc <<= 1;
        }
    }

    return crc;
}

void CrsfReceiver::ParseFrame()
{
    const uint8_t type = m_buffer[2];

    // CRC рахується по type + payload, без address/length.
    const uint8_t crcReceived = m_buffer[m_index - 1];
    const uint8_t crcCalculated = Crc8DvbS2(&m_buffer[2], m_frameLength - 1);

    if (crcReceived != crcCalculated)
        return;

    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        const uint8_t* payload = &m_buffer[3];
        DecodeChannels(payload);
    }
}

void CrsfReceiver::DecodeChannels(const uint8_t* payload)
{
    // 16 каналів * 11 біт = 176 біт = 22 байти
    uint32_t bitIndex = 0;

    for (int ch = 0; ch < 16; ++ch)
    {
        uint32_t value = 0;

        for (int bit = 0; bit < 11; ++bit)
        {
            const uint32_t byteIndex = (bitIndex + bit) / 8;
            const uint32_t bitInByte = (bitIndex + bit) % 8;

            if (payload[byteIndex] & (1 << bitInByte))
            {
                value |= (1 << bit);
            }
        }

        m_channels.raw[ch] = static_cast<uint16_t>(value);
        m_channels.us[ch] = RawToUs(static_cast<uint16_t>(value));

        bitIndex += 11;
    }

    m_channels.valid = true;
}

uint16_t CrsfReceiver::RawToUs(uint16_t raw)
{
    // CRSF raw -> приблизний PWM us
    return static_cast<uint16_t>(((static_cast<int32_t>(raw) - 992) * 5 / 8) + 1500);
}

float CrsfReceiver::NormalizeStick(uint16_t us)
{
    constexpr float center = 1500.0f;
    constexpr float range = 500.0f;

    float value = (static_cast<float>(us) - center) / range;

    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;

    return value * 1000;
}

float CrsfReceiver::NormalizeThrottle(uint16_t us)
{
    constexpr float minUs = 1000.0f;
    constexpr float maxUs = 2000.0f;

    float value = (static_cast<float>(us) - minUs) / (maxUs - minUs);

    if (value > 1.0f) value = 1.0f;
    if (value < 0.0f) value = 0.0f;

    return value * 1000;
}
