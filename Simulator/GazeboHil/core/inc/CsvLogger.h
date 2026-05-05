//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include "GlobalDef.h"

#include <fstream>
#include <string>

NAMESPACE_BEGIN
struct FlightLogSample
{
    uint32_t timeMs = 0;
    uint32_t imuSeq = 0;
    uint32_t controlSeq = 0;
    uint32_t logSeq = 0;

    float dt = 0.0f;
    float imuDt = 0.0f;
    float halDt = 0.0f;

    float rcThrottle = 0.0f;
    float rcRoll = 0.0f;
    float rcPitch = 0.0f;
    float rcYaw = 0.0f;

    float targetRollRateDegSec = 0.0f;
    float targetPitchRateDegSec = 0.0f;
    float targetYawRateDegSec = 0.0f;

    float gyroRollDegSec = 0.0f;
    float gyroPitchDegSec = 0.0f;
    float gyroYawDegSec = 0.0f;

    float estimatedRollDeg = 0.0f;
    float estimatedPitchDeg = 0.0f;

    float controlRoll = 0.0f;
    float controlPitch = 0.0f;
    float controlYaw = 0.0f;

    float motorM1 = 0.0f;
    float motorM2 = 0.0f;
    float motorM3 = 0.0f;
    float motorM4 = 0.0f;
};

class CsvLogger
{
public:
    const std::string HEADER_LOG_SHORT = "time,angle_deg,gyro_deg_s,left_motor,right_motor,torque";
    const std::string HEADER_LOG_FLIGHT_SAMPLE = "time_ms,imu_seq,cont_seq,log_seq,dt,imu_dt,hal_dt,rc_thr,rc_roll,rc_pitch,rc_yaw,t_roll,t_pitch,t_yaw,g_roll,g_pitch,g_yaw,est_roll,est_pitch,c_roll,c_pitch,c_yaw,m1,m2,m3,m4";

    bool Open(const std::string& path, const std::string& header);
    void Close();

    bool IsOpen() const;

    void WriteHeader(const std::string& header);

    void Log(
        double timeSec,
        double angleDeg,
        double gyroDegSec,
        double leftMotor,
        double rightMotor,
        double torque
    );

    void Log(FlightLogSample sample);
    void Log(std::unordered_map<std::string, float> map_log);

private:
    std::ofstream file_;
};
NAMESPACE_END