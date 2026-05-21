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

bool SensorsManager::UpdateImu(uint32_t nowUs)
{
    return m_imuSensor.Update(nowUs);
}

const ImuSample& SensorsManager::GetImuData() const
{
    return m_imuSensor.GetData();
}

bool SensorsManager::IsImuReady() const
{
    return m_imuSensor.IsGyroCalibrated();
}

void SensorsManager::StartGyroCalibration(uint16_t sampleCount)
{
    m_imuSensor.StartGyroCalibration(sampleCount);
}

void SensorsManager::StartLevelAccelCalibration(uint16_t sampleCount, float expectedAccelZ_mps2)
{
    m_imuSensor.StartLevelAccelCalibration(sampleCount, expectedAccelZ_mps2);
}
