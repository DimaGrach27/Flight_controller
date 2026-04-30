//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include <string>
#include "GlobalDef.h"

NAMESPACE_BEGIN
class SerialPort
{
public:
    SerialPort() = default;
    ~SerialPort();

    bool Open(const std::string& path, int baud);
    void Close();

    bool IsOpen() const;

    int Read(uint8_t* buffer, size_t maxLen);
    bool Write(const uint8_t* data, size_t len);

private:
    int fd_ = -1;
};
NAMESPACE_END