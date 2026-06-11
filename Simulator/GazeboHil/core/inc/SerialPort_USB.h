//
// Created by Dmytro Hrachov on 07.06.2026.
//
#pragma once

#include <string>

#include "GlobalDef.h"
#include "ISerialPort.h"

NAMESPACE_BEGIN
class SerialPort_USB : public ISerialPort
{
    ~SerialPort_USB() override;

    bool Open(const std::string &path, int baud) override;
    void Close() override;

    bool IsOpen() const override;

    int Read(uint8_t* buffer, size_t maxLen) override;
    bool Write(const uint8_t* data, size_t len) override;

private:
    void CloseInternal();

private:
    int m_fd = -1;
};
NAMESPACE_END
