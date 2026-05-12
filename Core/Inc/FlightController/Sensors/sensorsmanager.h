//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include <cstdint>

#include "IMU/imu_sensor.h"
#include "FlightController/datastructs.h"

class SensorsManager
{
public:
    SensorsManager(Imu_Sensor& imuSensor);
    ~SensorsManager();

    bool Init();
    void UpdateImu(uint32_t nowUs);

    const ImuSample& GetImuData() const;

    bool IsImuReady() const;

    void StartGyroCalibration(uint16_t sampleCount);
    void StartLevelAccelCalibration(uint16_t sampleCount, float expectedAccelZ_mps2);

private:
    Imu_Sensor& m_imuSensor;
};
