//
// Created by Dmytro Hrachov on 20.05.2026.
//
#pragma once

#include <array>
#include <cstdint>
#include <cstddef>

enum class UsbDebugConsoleCommand : uint8_t
{
    Unknown,

    Status,
    IMU_Status,
    Battery_Status,
    CalibrateAccel,
    CalibrateGyro,
    Reboot,

    COUNT
};

class UsbDebugConsole
{
public:
    static constexpr uint16_t RxBufferSize = 512;
    static constexpr uint16_t TxBufferSize = 1024;
    static constexpr uint16_t CommandBufferSize = 128;

    void Init();
    void Update(uint32_t nowUs);

    void OnUsbReceived(const uint8_t* data, uint32_t size);
    void OnTransmitComplete();

    void Write(const char* text);
    void WriteLine(const char* text);
    void WriteBytes(const uint8_t* data, uint16_t size);

private:
    void ProcessRx();
    void ProcessCommand(const char* command);

    bool PushRx(uint8_t byte);
    bool PopRx(uint8_t& byte);

    bool PushTx(uint8_t byte);
    bool PopTx(uint8_t& byte);

    void TryStartTransmit();

private:
    using Callback = void (*)(void* context);

    struct DebugCommand
    {
        UsbDebugConsoleCommand command = UsbDebugConsoleCommand::Unknown;
        Callback callback = nullptr;
        void* context = nullptr;
    };

private:
    uint8_t m_rxBuffer[RxBufferSize]{};
    volatile uint16_t m_rxHead = 0;
    volatile uint16_t m_rxTail = 0;

    uint8_t m_txBuffer[TxBufferSize]{};
    volatile uint16_t m_txHead = 0;
    volatile uint16_t m_txTail = 0;

    char m_commandBuffer[CommandBufferSize]{};
    uint16_t m_commandLength = 0;

    bool m_txBusy = false;
    uint8_t m_usbTxChunk[64]{};

    uint32_t m_lastStatusUs = 0;

    std::array<DebugCommand, static_cast<uint8_t>(UsbDebugConsoleCommand::COUNT)> m_commandCallbacks;
};
