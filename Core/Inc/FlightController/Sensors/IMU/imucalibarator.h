//
// Created by Dmytro Hrachov on 12.05.2026.
//
#pragma once

#include <cstdint>

#include "FlightController/datastructs.h"
#include "FlightController/Math/vector3f.h"

struct ImuCalibrationData
{
    Vector3f gyroBias_rads{};
    Vector3f accelBias_mps2{};

    bool gyroCalibrated = false;
    bool accelCalibrated = false;
};

class ImuCalibrator
{
public:
    ImuCalibrator();

    void Init();

    void StartGyroCalibration(uint16_t sampleCount);
    void StartLevelAccelCalibration(uint16_t sampleCount, float expectedAccelZ_mps2);

    void Update(const ImuSample& sample);

    bool IsGyroCalibrationRunning() const;
    bool IsAccelCalibrationRunning() const;

    bool IsGyroCalibrated() const;
    bool IsAccelCalibrated() const;

    const ImuCalibrationData& GetCalibrationData() const;

private:
    void ResetGyroAccumulation();
    void ResetAccelAccumulation();

private:
    ImuCalibrationData m_calibration{};

    Vector3f m_gyroSum{};
    Vector3f m_accelSum{};

    uint16_t m_gyroSampleTarget = 0;
    uint16_t m_accelSampleTarget = 0;

    uint16_t m_gyroSampleCount = 0;
    uint16_t m_accelSampleCount = 0;

    float m_expectedAccelZ_mps2 = 9.80665f;

    bool m_gyroCalibrationRunning = false;
    bool m_accelCalibrationRunning = false;
};