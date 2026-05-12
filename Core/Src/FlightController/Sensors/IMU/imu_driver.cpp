//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "../../../../Inc/FlightController/Sensors/IMU/imu_driver.h"


#if NOT_USE_HIL
Imu_Driver::Imu_Driver(IMU_Lsm6ds3& imu_lsm6_ds3)
    : m_driverReal(imu_lsm6_ds3)
{

}
#else
Imu_Driver::Imu_Driver(IMU_Driver_Hil &imu_driver_hil)
    : m_driverHil(imu_driver_hil)
{

}
#endif

bool Imu_Driver::Init()
{
#if NOT_USE_HIL
    return m_driverReal.Init();
#else
    return m_driverHil.Init();
#endif

    return false;
}

bool Imu_Driver::ReadRaw(ImuRawData &outRawData, uint32_t nowUs)
{
#if NOT_USE_HIL
    return m_driverReal.ReadRaw(outRawData, nowUs);
#else
    return m_driverHil.ReadRaw(outRawData, nowUs);
#endif

    return false;
}

bool Imu_Driver::Read(ImuSample &outData, uint32_t nowUs)
{
#if NOT_USE_HIL
    return m_driverReal.Read(outData, nowUs);
#else
    return m_driverHil.Read(outData, nowUs);
#endif

    return false;
}
