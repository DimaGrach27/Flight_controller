//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include <string>
#include "GlobalDef.h"
#include "ISerialPort.h"

NAMESPACE_BEGIN
class SerialPort_UART : public ISerialPort
{
public:
    ~SerialPort_UART() override;

    bool Open(const std::string &path, int baud) override;
    void Close() override;


    bool IsOpen() const override;

    int Read(uint8_t* buffer, size_t maxLen) override;
    bool Write(const uint8_t* data, size_t len) override;

private:
    void CloseInternal();

private:
    int fd_ = -1;
};
NAMESPACE_END