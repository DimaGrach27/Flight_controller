//
// Created by Dmytro Hrachov on 10.05.2026.
//
#pragma once

#include <cstdint>

#include "main.h"

#include "FlightController/datastructs.h"
#include "FlightController/Protocols/spidmabus.h"

class IMU_MPU6000
{
public:
    static constexpr uint8_t RawFrameSize = 14;
    static constexpr uint8_t SpiFrameSize = RawFrameSize + 1;

    explicit IMU_MPU6000(SpiDmaBus& spiBus);

    bool Init();

    bool StartReadRaw();
    bool IsReadComplete() const;
    bool HasError() const;

    void Reset();

    bool ReadRaw(ImuRawData& outRawData, uint32_t nowUs);
    bool Read(ImuSample& outData, uint32_t nowUs);

private:
    bool CheckDeviceId() const;
    bool ConfigureDevice();

    int16_t MakeInt16(uint8_t high, uint8_t low) const;

private:
    SpiDmaBus& m_spiBus;

    bool m_initialized = false;

    float m_accelScale_mps2 = 0.0f;
    float m_gyroScale_rads = 0.0f;

    uint8_t m_txBuffer[SpiFrameSize] = {};
    uint8_t m_rxBuffer[SpiFrameSize] = {};
};
