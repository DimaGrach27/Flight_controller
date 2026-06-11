//
// Created by Dmytro Hrachov on 07.06.2026.
//
#pragma once

#include <cstdint>
#include "GlobalDef.h"

NAMESPACE_BEGIN
class ISerialPort
{
public:
    virtual ~ISerialPort() = default;

    virtual bool Open(const std::string &path, int baud) = 0;
    virtual void Close() = 0;

    virtual bool IsOpen() const = 0;

    virtual int Read(uint8_t* buffer, size_t maxLen) = 0;
    virtual bool Write(const uint8_t* data, size_t len) = 0;
};
NAMESPACE_END
