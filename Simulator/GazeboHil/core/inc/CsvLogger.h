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
    bool Open(const std::string& path);
    void Close();

    bool IsOpen() const;

    void WriteHeader();

    void Log(
        double timeSec,
        double angleDeg,
        double gyroDegSec,
        double leftMotor,
        double rightMotor,
        double torque
    );

private:
    std::ofstream file_;
};
NAMESPACE_END