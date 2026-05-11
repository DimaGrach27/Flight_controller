//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include <cstdint>

#include "FlightController/Sensors/imu_sensor.h"
#include "FlightController/datastructs.h"

class SensorsManager
{
public:
    SensorsManager(Imu_Sensor& imuSensor);
    ~SensorsManager();

    bool Init();
    void UpdateImu(uint32_t nowUs);

    const ImuSample& GetImuData() const;

private:
    Imu_Sensor& m_imuSensor;
};
