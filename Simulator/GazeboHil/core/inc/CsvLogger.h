//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include "GlobalDef.h"

#include <fstream>
#include <string>

NAMESPACE_BEGIN
class CsvLogger
{
public:
    const std::string HEADER_LOG_SHORT = "time,angle_deg,gyro_deg_s,left_motor,right_motor,torque";
    const std::string HEADER_LOG_FLIGHT_SAMPLE = "time_ms,imu_seq,cont_seq,log_seq,dt,imu_dt,hal_dt,rc_thr,rc_roll,rc_pitch,rc_yaw,t_roll,t_pitch,t_yaw,g_roll,g_pitch,g_yaw,a_roll,a_pitch,cor_roll,cor_pitch,err_roll,err_pitch,est_roll,est_pitch,c_roll,c_pitch,c_yaw,m1,m2,m3,m4";

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

    void Log(std::unordered_map<std::string, float> map_log);

private:
    std::ofstream m_file;
};
NAMESPACE_END