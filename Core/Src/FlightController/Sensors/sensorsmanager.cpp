//
// Created by Dmytro Hrachov on 11.05.2026.
//

#include "FlightController/Sensors/sensorsmanager.h"

SensorsManager::SensorsManager(Imu_Sensor& imuSensor)
    : m_imuSensor(imuSensor)
{
}

SensorsManager::~SensorsManager()
{

}

bool SensorsManager::Init()
{
    if (!m_imuSensor.Init())
    {
        //TODO: add log for failed init IMU
        return false;
    }

    return true;
}

void SensorsManager::UpdateImu(uint32_t nowUs)
{
    m_imuSensor.Update(nowUs);
}

const ImuSample& SensorsManager::GetImuData() const
{
    return m_imuSensor.GetData();
}