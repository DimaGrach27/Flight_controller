//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once

#include "FlightController/globaldef.h"
#include "imu_driver_hil.h"

#if NOT_USE_HIL
#if FC_IMU_USE_MPU6000
#include "FlightController/Sensors/IMU/imu_driver_mpu6000.h"
using RealImuDriver = IMU_MPU6000;
#else
#include "FlightController/Sensors/IMU/imu_driver_lsm6ds3.h"
using RealImuDriver = IMU_Lsm6ds3;
#endif
#else
#include "imu_driver_hil.h"
#endif

class Imu_Driver
{
public:
#if NOT_USE_HIL
    Imu_Driver(RealImuDriver& realImuDriver);
#else
    Imu_Driver(IMU_Driver_Hil& imu_driver_hil);
#endif

    ~Imu_Driver();

    bool Init();

    bool ReadRaw(ImuRawData& outRawData, uint32_t nowUs);
    bool Read(ImuSample& outData, uint32_t nowUs);

private:

#if NOT_USE_HIL
    RealImuDriver& m_driverReal; //real driver
#else
    IMU_Driver_Hil& m_driverHil; //HIL driver
#endif
};
