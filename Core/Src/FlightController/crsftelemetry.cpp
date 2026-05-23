//
// Created by Dmytro Hrachov on 07.05.2026.
//

#include "FlightController/crsftelemetry.h"

#include <cstring>

#include "main.h"

CrsfTelemetry::CrsfTelemetry(UartByteStream& byteStream)
    :m_byteStream(byteStream)
{

}

void CrsfTelemetry::Init()
{
}

void CrsfTelemetry::SendBattery(float voltage, float current, uint32_t consumedMah, uint8_t remainingPercent)
{
    uint8_t payload[8]{};

    /*
        CRSF battery sensor payload зазвичай:
        voltage:      uint16, 0.1V
        current:      uint16, 0.1A
        capacity:     uint24, mAh
        remaining:    uint8, %
    */

    const uint16_t voltageDeciVolt = static_cast<uint16_t>(voltage * 10.0f);
    const uint16_t currentDeciAmp  = static_cast<uint16_t>(current * 10.0f);

    payload[0] = static_cast<uint8_t>(voltageDeciVolt >> 8);
    payload[1] = static_cast<uint8_t>(voltageDeciVolt & 0xFF);

    payload[2] = static_cast<uint8_t>(currentDeciAmp >> 8);
    payload[3] = static_cast<uint8_t>(currentDeciAmp & 0xFF);

    payload[4] = static_cast<uint8_t>((consumedMah >> 16) & 0xFF);
    payload[5] = static_cast<uint8_t>((consumedMah >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>(consumedMah & 0xFF);

    payload[7] = remainingPercent;

    SendFrame(CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BATTERY_SENSOR, payload, sizeof(payload));
}

void CrsfTelemetry::SendFlightMode(const char *text)
{
    if (text == nullptr)
        return;

    /*
        Payload для CRSF_FLIGHT_MODE:
        char[] null-terminated string
    */

    constexpr uint8_t maxTextLength = 16;

    char payload[maxTextLength + 1]{};

    std::strncpy(payload, text, maxTextLength);
    payload[maxTextLength] = '\0';

    const uint8_t payloadSize = static_cast<uint8_t>(std::strlen(payload) + 1);

    SendFrame(
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_FRAMETYPE_FLIGHT_MODE,
        reinterpret_cast<const uint8_t*>(payload),
        payloadSize
    );
}

void CrsfTelemetry::SendFrame(uint8_t address, uint8_t type, const uint8_t* payload, uint8_t payloadSize)
{
    /*
        length = type + payload + crc
    */
    const uint8_t length = payloadSize + 2;

    uint8_t frame[CRSF_MAX_FRAME_SIZE]{};
    uint8_t index = 0;

    frame[index++] = address;
    frame[index++] = length;
    frame[index++] = type;

    for (uint8_t i = 0; i < payloadSize; ++i)
    {
        frame[index++] = payload[i];
    }

    const uint8_t crc = Crc8DvbS2(&frame[2], length - 1);
    frame[index++] = crc;

    SendData(frame, index);
}

void CrsfTelemetry::SendByte(uint8_t byte)
{
    m_byteStream.Write(&byte, 1);

    // HAL_UART_Transmit(m_huart1, &byte, 1, 1);
}

void CrsfTelemetry::SendData(const uint8_t* data, uint8_t size)
{
    m_byteStream.Write(data, size);
    // HAL_UART_Transmit(m_huart1, const_cast<uint8_t*>(data), size, 10);
}

uint8_t CrsfTelemetry::Crc8DvbS2(const uint8_t* data, uint8_t len)
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
