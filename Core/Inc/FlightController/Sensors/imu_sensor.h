//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include "FlightController/Sensors/imu_driver.h"
#include "FlightController/datastructs.h"

class Imu_Sensor
{
public:
    explicit Imu_Sensor(Imu_Driver& driver);
    ~Imu_Sensor();

    bool Init();

    bool Update(uint32_t nowUs);

    const ImuSample& GetData() const;

    void SetGyroOffset(float x_rads, float y_rads, float z_rads);
    void SetAccelOffset(float x_mps2, float y_mps2, float z_mps2);

private:
    ImuSample ApplyCalibration(const ImuSample& sample) const;
    bool ValidateSample(const ImuSample& sample) const;

private:
    Imu_Driver& m_driver;

    ImuSample m_data{};

    float m_accelOffsetX_mps2 = 0.0f;
    float m_accelOffsetY_mps2 = 0.0f;
    float m_accelOffsetZ_mps2 = 0.0f;

    float m_gyroOffsetX_rads = 0.0f;
    float m_gyroOffsetY_rads = 0.0f;
    float m_gyroOffsetZ_rads = 0.0f;

    bool m_initialized = false;
};
