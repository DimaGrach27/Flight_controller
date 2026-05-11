//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#define USE_REAL_IMU 1

#if USE_REAL_IMU
#include "FlightController/Sensors/imu_driver_lsm6ds3.h"
#else
#include "FlightController/Sensors/imu_driver_hil.h"
#endif

class Imu_Driver
{
public:
    Imu_Driver(ISpiBus& iSpiBus);
    ~Imu_Driver();

    bool Init();

    bool ReadRaw(ImuRawData& outRawData, uint32_t nowUs);
    bool Read(ImuData& outData, uint32_t nowUs);

private:

#if USE_REAL_IMU
    IMU_Lsm6ds3* m_driverReal = nullptr; //real driver
#else
    IMU_Driver_Hil* m_driverHil = nullptr; //HIL driver
#endif
};
