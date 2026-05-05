//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "CsvLogger.h"

NAMESPACE_BEGIN
bool CsvLogger::Open(const std::string& path, const std::string& header)
{
    Close();

    file_.open(path);

    if (!file_.is_open())
        return false;

    WriteHeader(header);

    return true;
}

void CsvLogger::Close()
{
    if (file_.is_open())
        file_.close();
}

bool CsvLogger::IsOpen() const
{
    return file_.is_open();
}

void CsvLogger::WriteHeader(const std::string& header)
{
    if (!file_.is_open())
        return;

    file_ << header << "\n";
}

void CsvLogger::Log(
    double timeSec,
    double angleDeg,
    double gyroDegSec,
    double leftMotor,
    double rightMotor,
    double torque
)
{
    if (!file_.is_open())
        return;

    file_
        << timeSec << ","
        << angleDeg << ","
        << gyroDegSec << ","
        << leftMotor << ","
        << rightMotor << ","
        << torque
        << "\n";
}

void CsvLogger::Log(FlightLogSample sample)
{
    if (!file_.is_open())
        return;

    file_
        << sample.timeMs << ','
        << sample.imuSeq << ','
        << sample.controlSeq << ','
        << sample.logSeq << ','
        << sample.dt << ','
        << sample.imuDt << ','
        << sample.halDt << ','
        << sample.rcThrottle << ','
        << sample.rcRoll << ','
        << sample.rcPitch << ','
        << sample.rcYaw << ','
        << sample.targetRollRateDegSec << ','
        << sample.targetPitchRateDegSec << ','
        << sample.targetYawRateDegSec << ','
        << sample.gyroRollDegSec << ','
        << sample.gyroPitchDegSec << ','
        << sample.gyroYawDegSec << ','
        << sample.estimatedRollDeg << ','
        << sample.estimatedPitchDeg << ','
        << sample.controlRoll << ','
        << sample.controlPitch << ','
        << sample.controlYaw << ','
        << sample.motorM1 << ','
        << sample.motorM2 << ','
        << sample.motorM3 << ','
        << sample.motorM4 << ','
        << '\n';

    file_.flush();
}

void CsvLogger::Log(std::unordered_map<std::string, float> map_log)
{
    if (!file_.is_open())
        return;

    file_
        << map_log.at("time_ms") << ','
        << map_log.at("imu_seq") << ','
        << map_log.at("cont_seq") << ','
        << map_log.at("log_seq") << ','
        << map_log.at("dt") << ','
        << map_log.at("imu_dt") << ','
        << map_log.at("hal_dt") << ','
        << map_log.at("rc_thr") << ','
        << map_log.at("rc_roll") << ','
        << map_log.at("rc_pitch") << ','
        << map_log.at("rc_yaw") << ','
        << map_log.at("t_roll") << ','
        << map_log.at("t_pitch") << ','
        << map_log.at("t_yaw") << ','
        << map_log.at("g_roll") << ','
        << map_log.at("g_pitch") << ','
        << map_log.at("g_yaw") << ','
        << map_log.at("est_roll") << ','
        << map_log.at("est_pitch") << ','
        << map_log.at("c_roll") << ','
        << map_log.at("c_pitch") << ','
        << map_log.at("c_yaw") << ','
        << map_log.at("m1") << ','
        << map_log.at("m2") << ','
        << map_log.at("m3") << ','
        << map_log.at("m4")
        << '\n';

    file_.flush();
}

NAMESPACE_END
