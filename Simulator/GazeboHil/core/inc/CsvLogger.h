//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include "GlobalDef.h"

#include <fstream>
#include <string>
#include <unordered_map>

NAMESPACE_BEGIN
class CsvLogger
{
public:
    const std::string HEADER_LOG_SHORT = "truth_x,truth_y,truth_z,truth_vx,truth_vy,truth_vz,time,angle_deg,gyro_deg_s,left_motor,right_motor,torque";
    const std::string HEADER_LOG_FLIGHT_SAMPLE =
        "truth_x,truth_y,truth_z,truth_vx,truth_vy,truth_vz,"
        "f_mode,armed,fs,fs_rsn,arm_deny,stop_rsn,"
        "time_ms,imu_seq,cont_seq,log_seq,dt,imu_dt,hal_dt,ctrl_dt,"
        "rc_thr,rc_roll,rc_pitch,rc_yaw,rc_age,rc_valid,rc_arm,rc_angle,"
        "t_roll,t_pitch,t_yaw,g_roll,g_pitch,g_yaw,a_roll,a_pitch,a_yaw,"
        "cor_roll,cor_pitch,cor_yaw,err_roll,err_pitch,err_yaw,"
        "est_roll,est_pitch,est_yaw,st_valid,"
        "c_roll,c_pitch,c_yaw,m1,m2,m3,m4,m_min,m_max,m_span,thr_lim,thr_out,"
        "rp,ri,rd,re,rs,pp,pi,pd,pe,ps,yp,yi,yd,ye,ys,"
        "bat_v,cell_v,bat_a,bat_pct,bat_st,bat_warn,bat_flt";

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
