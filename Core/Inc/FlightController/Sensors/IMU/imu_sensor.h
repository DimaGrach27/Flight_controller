//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include "imucalibarator.h"
#include "imu_driver.h"
#include "FlightController/datastructs.h"
#include "FlightController/Utils/vectorlowpassfilter.h"

class Imu_Sensor
{
public:
    explicit Imu_Sensor(Imu_Driver& driver);
    ~Imu_Sensor();

    bool Init();

    bool Update(uint32_t nowUs);

    const ImuSample& GetData() const;

    void StartGyroCalibration(uint16_t sampleCount);
    void StartLevelAccelCalibration(uint16_t sampleCount, float expectedAccelZ_mps2);

    bool IsGyroCalibrated() const;
    bool IsAccelCalibrated() const;

    void SetGyroOffset(float x_rads, float y_rads, float z_rads);
    void SetAccelOffset(float x_mps2, float y_mps2, float z_mps2);

private:
    ImuSample ApplyCalibration(const ImuSample& sample) const;
    ImuSample ApplyFiltering(const ImuSample& sample);

    bool ValidateSample(const ImuSample& sample) const;

    // float ComputeDtSeconds(uint32_t timestampUs);

private:
    Imu_Driver& m_driver;

    ImuSample m_data{};

    ImuCalibrator m_calibrator;
    ImuCalibrationData m_dataCalibration{};

    Vector3LowPassFilter m_accelFilter;
    Vector3LowPassFilter m_gyroFilter;

    uint32_t m_lastFilterUpdateUs = 0;
    bool m_hasLastFilterUpdate = false;

    float m_accelOffsetX_mps2 = 0.0f;
    float m_accelOffsetY_mps2 = 0.0f;
    float m_accelOffsetZ_mps2 = 0.0f;

    float m_gyroOffsetX_rads = 0.0f;
    float m_gyroOffsetY_rads = 0.0f;
    float m_gyroOffsetZ_rads = 0.0f;

    bool m_initialized = false;
    bool m_enableFiltering = true;
};
