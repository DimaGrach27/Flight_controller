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

    constexpr const char* fields[] =
    {
        "truth_x", "truth_y", "truth_z", "truth_vx", "truth_vy", "truth_vz",
        "f_mode", "armed", "fs", "fs_rsn", "arm_deny", "stop_rsn",
        "time_ms", "imu_seq", "cont_seq", "log_seq", "dt", "imu_dt", "hal_dt", "ctrl_dt",
        "rc_thr", "rc_roll", "rc_pitch", "rc_yaw", "rc_age", "rc_valid", "rc_arm", "rc_angle",
        "t_roll", "t_pitch", "t_yaw", "g_roll", "g_pitch", "g_yaw", "a_roll", "a_pitch", "a_yaw",
        "cor_roll", "cor_pitch", "cor_yaw", "err_roll", "err_pitch", "err_yaw",
        "est_roll", "est_pitch", "est_yaw", "st_valid",
        "c_roll", "c_pitch", "c_yaw", "m1", "m2", "m3", "m4", "m_min", "m_max", "m_span",
        "thr_lim", "thr_out",
        "rp", "ri", "rd", "re", "rs", "pp", "pi", "pd", "pe", "ps", "yp", "yi", "yd", "ye", "ys",
        "bat_v", "cell_v", "bat_a", "bat_pct", "bat_st", "bat_warn", "bat_flt",
    };

    constexpr size_t fieldCount = sizeof(fields) / sizeof(fields[0]);

    for (size_t i = 0; i < fieldCount; ++i)
    {
        const auto it = map_log.find(fields[i]);

        if (it != map_log.end())
        {
            m_file << it->second;
        }
        else
        {
            m_file << 0.0f;
        }

        if (i + 1U < fieldCount)
        {
            m_file << ',';
        }
    }

    m_file << '\n';
    m_file.flush();
}

NAMESPACE_END
