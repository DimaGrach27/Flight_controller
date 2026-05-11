//
// Created by Dmytro Hrachov on 10.05.2026.
//
#pragma once

#include <cstdint>

#include "main.h"

#include "FlightController/datastructs.h"
#include "FlightController/Protocols/ispibus.h"

class IMU_Lsm6ds3
{
public:
    explicit IMU_Lsm6ds3(ISpiBus& spiBus);

    bool Init();

    bool ReadRaw(ImuRawData& outRawData, uint32_t nowUs);
    bool Read(ImuData& outData, uint32_t nowUs);

private:
    bool CheckDeviceId();
    bool ConfigureDevice();

    int16_t ReadInt16Le(const uint8_t* buffer, uint8_t lowIndex) const;

private:
    ISpiBus& m_spiBus;

    bool m_initialized = false;

    float m_accelScale_mps2 = 0.0f;
    float m_gyroScale_rads = 0.0f;
};
