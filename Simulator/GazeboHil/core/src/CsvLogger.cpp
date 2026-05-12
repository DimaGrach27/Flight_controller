//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "CsvLogger.h"

NAMESPACE_BEGIN
bool CsvLogger::Open(const std::string& path, const std::string& header)
{
    Close();

    m_file.open(path);

    if (!m_file.is_open())
        return false;

    WriteHeader(header);

    return true;
}

void CsvLogger::Close()
{
    if (m_file.is_open())
        m_file.close();
}

bool CsvLogger::IsOpen() const
{
    return m_file.is_open();
}

void CsvLogger::WriteHeader(const std::string& header)
{
    if (!m_file.is_open())
        return;

    m_file << header << "\n";
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
    if (!m_file.is_open())
        return;

    m_file
        << timeSec << ","
        << angleDeg << ","
        << gyroDegSec << ","
        << leftMotor << ","
        << rightMotor << ","
        << torque
        << "\n";
}

void CsvLogger::Log(std::unordered_map<std::string, float> map_log)
{
    if (!m_file.is_open())
        return;

    try
    {
        m_file
            << map_log.at("truth_x") << ','
            << map_log.at("truth_y") << ','
            << map_log.at("truth_z") << ','
            << map_log.at("truth_vx") << ','
            << map_log.at("truth_vy") << ','
            << map_log.at("truth_vz") << ','
            << map_log.at("f_mode") << ','
            << map_log.at("armed") << ','
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
            << map_log.at("a_roll") << ','
            << map_log.at("a_pitch") << ','
            << map_log.at("cor_roll") << ','
            << map_log.at("cor_pitch") << ','
            << map_log.at("err_roll") << ','
            << map_log.at("err_pitch") << ','
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

        m_file.flush();
    }
    catch (std::exception& e)
    {
        printf("CsvLogger::Log: std::exception: %s\n", e.what());
    }
}

NAMESPACE_END
