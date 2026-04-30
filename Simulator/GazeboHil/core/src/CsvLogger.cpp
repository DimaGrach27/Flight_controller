//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "CsvLogger.h"

NAMESPACE_BEGIN
bool CsvLogger::Open(const std::string& path)
{
    Close();

    file_.open(path);

    if (!file_.is_open())
        return false;

    WriteHeader();

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

void CsvLogger::WriteHeader()
{
    if (!file_.is_open())
        return;

    file_ << "time,angle_deg,gyro_deg_s,left_motor,right_motor,torque\n";
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
NAMESPACE_END