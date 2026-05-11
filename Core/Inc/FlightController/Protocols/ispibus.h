//
// Created by Dmytro Hrachov on 11.05.2026.
//
#pragma once
#include <cstdint>

class ISpiBus
{
public:
    virtual ~ISpiBus() = default;

    virtual bool WriteRegister(uint8_t reg, uint8_t value, const uint8_t writeMask) = 0;
    virtual bool ReadRegisters(uint8_t startReg, uint8_t* buffer, uint16_t size, const uint8_t spiReadBit) = 0;
};
