//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Sensors/imu_driver.h"


Imu_Driver::Imu_Driver(ISpiBus& iSpiBus)
{
#if USE_REAL_IMU
    m_driverReal = new IMU_Lsm6ds3(iSpiBus);
#else
    m_driverHil = new IMU_Driver_Hil();
#endif
}

Imu_Driver::~Imu_Driver()
{
#if USE_REAL_IMU
    delete m_driverReal;
#else
    delete m_driverHil;
#endif
}

bool Imu_Driver::Init()
{
#if USE_REAL_IMU
    return m_driverReal->Init();
#else
    return m_driverHil->Init();
#endif

    return false;
}

bool Imu_Driver::ReadRaw(ImuRawData &outRawData, uint32_t nowUs)
{
#if USE_REAL_IMU
    return m_driverReal->ReadRaw(outRawData, nowUs);
#else
    return m_driverHil->ReadRaw(outRawData, nowUs);
#endif

    return false;
}

bool Imu_Driver::Read(ImuData &outData, uint32_t nowUs)
{
#if USE_REAL_IMU
    return m_driverReal->Read(outData, nowUs);
#else
    return m_driverHil->Read(outData, nowUs);
#endif

    return false;
}
