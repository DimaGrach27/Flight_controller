//
// Created by Codex on 29.07.2026.
//

#include "FlightController/Config/flashconfigstorage.h"

#include "stm32f4xx_hal.h"

namespace
{
    constexpr uint32_t ConfigFlashAddress = 0x080E0000U;
    constexpr uint32_t ConfigFlashSector = FLASH_SECTOR_11;
    constexpr uint32_t ConfigMagic = 0x50494443U; // "PIDC"
    constexpr uint32_t ConfigVersion = 1U;

    struct StoredPidConfig
    {
        uint32_t magic = ConfigMagic;
        uint32_t version = ConfigVersion;
        uint32_t payloadSize = sizeof(PidConfig);
        PidConfig pidConfig{};
        uint32_t crc = 0U;
    };

    static_assert((sizeof(StoredPidConfig) % sizeof(uint32_t)) == 0U, "Stored config must be word aligned");
}

FlashConfigStorage::Status FlashConfigStorage::LoadPidConfig(PidConfig& config) const
{
    const auto* stored = reinterpret_cast<const StoredPidConfig*>(ConfigFlashAddress);

    if (stored->magic != ConfigMagic ||
        stored->version != ConfigVersion ||
        stored->payloadSize != sizeof(PidConfig))
    {
        return Status::Invalid;
    }

    const uint32_t crc = Crc32(
        reinterpret_cast<const uint8_t*>(&stored->pidConfig),
        sizeof(stored->pidConfig));

    if (crc != stored->crc)
    {
        return Status::Invalid;
    }

    config = stored->pidConfig;
    return Status::Ok;
}

FlashConfigStorage::Status FlashConfigStorage::SavePidConfig(const PidConfig& config) const
{
    StoredPidConfig stored{};
    stored.pidConfig = config;
    stored.crc = Crc32(
        reinterpret_cast<const uint8_t*>(&stored.pidConfig),
        sizeof(stored.pidConfig));

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return Status::FlashError;
    }

    FLASH_EraseInitTypeDef erase{};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = ConfigFlashSector;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sectorError = 0U;
    if (HAL_FLASHEx_Erase(&erase, &sectorError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return Status::FlashError;
    }

    const uint32_t* words = reinterpret_cast<const uint32_t*>(&stored);
    const uint32_t wordCount = sizeof(stored) / sizeof(uint32_t);

    for (uint32_t i = 0; i < wordCount; ++i)
    {
        const uint32_t address = ConfigFlashAddress + (i * sizeof(uint32_t));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, words[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return Status::FlashError;
        }
    }

    HAL_FLASH_Lock();
    return Status::Ok;
}

uint32_t FlashConfigStorage::Crc32(const uint8_t* data, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFFU;

    for (uint32_t i = 0; i < size; ++i)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8U; ++bit)
        {
            const uint32_t mask = (crc & 1U) != 0U ? 0xEDB88320U : 0U;
            crc = (crc >> 1U) ^ mask;
        }
    }

    return ~crc;
}
