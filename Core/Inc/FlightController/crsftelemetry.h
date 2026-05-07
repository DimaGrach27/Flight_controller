//
// Created by Dmytro Hrachov on 07.05.2026.
//
#pragma once

#include <cstdint>
#include "main.h"

class CrsfTelemetry
{
public:
    using SendByteCallback = void(*)(uint8_t byte);

    // explicit CrsfTelemetry(SendByteCallback SendByte)
    //     : m_sendByte(SendByte)
    // {
    // }

    void Init(UART_HandleTypeDef& huart1);

    void SendBattery(float voltage, float current, uint32_t consumedMah, uint8_t remainingPercent);
    void SendFlightMode(const char* text);

private:
    static constexpr uint8_t CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8;
    static constexpr uint8_t CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08;
    static constexpr uint8_t CRSF_FRAMETYPE_FLIGHT_MODE = 0x21;
    static constexpr uint8_t CRSF_MAX_FRAME_SIZE = 64;

private:
    void SendFrame(uint8_t address, uint8_t type, const uint8_t* payload, uint8_t payloadSize);
    void SendByte(uint8_t byte);
    void SendData(const uint8_t *data, uint8_t size);

    static uint8_t Crc8DvbS2(const uint8_t* data, uint8_t len);

private:
    SendByteCallback m_sendByte = nullptr;
    UART_HandleTypeDef* m_huart1 = nullptr;
};
