//
// Created by Dmytro Hrachov on 20.05.2026.
//

#include "FlightController/DebugLogs/usbdebugconsole.h"

#include <cstring>
#include <cstdio>

#include "FlightController/flightcontroller_entry.h"
#include "FlightController/globaldef.h"

extern "C"
{
#include "usbd_cdc_if.h"
}

UsbDebugConsole g_usbDebugConsole;

void UsbDebugConsole::Init()
{
    m_rxHead = 0;
    m_rxTail = 0;
    m_txHead = 0;
    m_txTail = 0;
    m_commandLength = 0;
    m_txBusy = false;

    WriteLine("\r\nUSB debug console ready");
    WriteLine("Type: help");
    Write("> ");
}

void UsbDebugConsole::Update(uint32_t nowUs)
{
    ProcessRx();

    if (!m_txBusy)
    {
        TryStartTransmit();
    }

    // Optional heartbeat message, disabled by default.
    // Не спамити в fast loop.
    (void)nowUs;
}

void UsbDebugConsole::OnUsbReceived(const uint8_t* data, uint32_t size)
{
    if (data == nullptr || size == 0)
    {
        return;
    }

    for (uint32_t i = 0; i < size; ++i)
    {
        PushRx(data[i]);
    }
}

void UsbDebugConsole::OnTransmitComplete()
{
    m_txBusy = false;
}

void UsbDebugConsole::Write(const char* text)
{
    if (text == nullptr)
    {
        return;
    }

    while (*text != '\0')
    {
        PushTx(static_cast<uint8_t>(*text));
        ++text;
    }

    TryStartTransmit();
}

void UsbDebugConsole::WriteLine(const char* text)
{
    Write(text);
    Write("\r\n");
}

void UsbDebugConsole::WriteBytes(const uint8_t* data, uint16_t size)
{
    if (data == nullptr || size == 0)
    {
        return;
    }

    const uint16_t used = m_txHead >= m_txTail
        ? static_cast<uint16_t>(m_txHead - m_txTail)
        : static_cast<uint16_t>(TxBufferSize - m_txTail + m_txHead);
    const uint16_t free = static_cast<uint16_t>(TxBufferSize - used - 1U);

    if (free < size)
    {
        return;
    }

    for (uint16_t i = 0; i < size; ++i)
    {
        PushTx(data[i]);
    }

    TryStartTransmit();
}

void UsbDebugConsole::ShowFlightStatus(const FlightModeState &state)
{
    char line[160]{};
    std::snprintf(
        line,
        sizeof(line),
        "Status: FM=%s ARM=%s FS=%u CanArm=%u",
        EnumToChar_FlightMode(state.mode),
        EnumToChar_ArmState(state.armState),
        state.failsafe ? 1U : 0U,
        state.canArm ? 1U : 0U);

    WriteLine(line);
}

void UsbDebugConsole::ShowBatteryStatus(const BatteryData &batteryData)
{
    char line[160]{};
    std::snprintf(
        line,
        sizeof(line),
        "Battery: %.2f V; %.2f A; %u%%; cell %.2f V; status=%s",
        static_cast<double>(batteryData.voltage_V),
        static_cast<double>(batteryData.current_A),
        static_cast<unsigned>(batteryData.percentage),
        static_cast<double>(batteryData.cellVoltage_V),
        EnumToChar_BatteryState(batteryData.state));

    WriteLine(line);
}

void UsbDebugConsole::ProcessRx()
{
    uint8_t byte = 0;

    while (PopRx(byte))
    {
#if !NOT_USE_HIL
        flight_controller_MavlinkParseByte(byte);
        continue;
#endif

        if (byte == '\r' || byte == '\n')
        {
            if (m_commandLength > 0)
            {
                m_commandBuffer[m_commandLength] = '\0';
                Write("\r\n");
                ProcessCommand(m_commandBuffer);
                m_commandLength = 0;
            }

            Write("> ");
            continue;
        }

        if (byte == 0x08 || byte == 0x7F) // Backspace
        {
            if (m_commandLength > 0)
            {
                --m_commandLength;
                Write("\b \b");
            }

            continue;
        }

        if (m_commandLength < CommandBufferSize - 1)
        {
            m_commandBuffer[m_commandLength++] = static_cast<char>(byte);

            // Echo назад у термінал
            uint8_t echo = byte;
            WriteBytes(&echo, 1);
        }
    }
}

void UsbDebugConsole::ProcessCommand(const char* command)
{
    if (command == nullptr)
    {
        return;
    }

    if (std::strcmp(command, "help") == 0)
    {
        WriteLine("Commands:");
        WriteLine("  help");
        WriteLine("  status");
        WriteLine("  imu");
        WriteLine("  battery");
        WriteLine("  log status");
        WriteLine("  log start");
        WriteLine("  log stop");
        WriteLine("  pid show");
        WriteLine("  pid set <rate|angle> <axis> <kp> <ki> <kd>");
        WriteLine("  pid save");
        WriteLine("  pid load");
        WriteLine("  pid defaults");
        WriteLine("  reboot");
        return;
    }

    if (std::strcmp(command, "pid") == 0 || std::strncmp(command, "pid ", 4) == 0)
    {
        UsbDebugConsole_RunDebugTextCommand(command);
        return;
    }

    if (std::strcmp(command, "log") == 0 || std::strncmp(command, "log ", 4) == 0)
    {
        UsbDebugConsole_RunDebugTextCommand(command);
        return;
    }

    if (std::strcmp(command, "status") == 0)
    {
        UsbDebugConsole_RunDebugCommand(static_cast<uint8_t>(UsbDebugConsoleCommand::Status));
        // WriteLine("state: DISARMED");
        // WriteLine("mode: ACRO");
        // WriteLine("loop: OK");
        // WriteLine("usb: OK");
        return;
    }

    if (std::strcmp(command, "imu") == 0)
    {
        UsbDebugConsole_RunDebugCommand(static_cast<uint8_t>(UsbDebugConsoleCommand::IMU_Status));
        return;
    }

    if (std::strcmp(command, "battery") == 0)
    {
        UsbDebugConsole_RunDebugCommand(static_cast<uint8_t>(UsbDebugConsoleCommand::Battery_Status));
        // Потім сюди підставиш реальні VBAT/current.
        // WriteLine("vbat: 0.00 V");
        // WriteLine("current: 0.00 A");
        return;
    }

    if (std::strcmp(command, "reboot") == 0)
    {
        WriteLine("Rebooting...");
        // Краще дати USB трохи часу відправити відповідь.
        // На старті можна не робити delay, а просто залишити команду disabled.
        // NVIC_SystemReset();
        return;
    }

    Write("Unknown command: ");
    WriteLine(command);
}

bool UsbDebugConsole::PushRx(uint8_t byte)
{
    const uint16_t nextHead = static_cast<uint16_t>((m_rxHead + 1) % RxBufferSize);

    if (nextHead == m_rxTail)
    {
        return false;
    }

    m_rxBuffer[m_rxHead] = byte;
    m_rxHead = nextHead;

    return true;
}

bool UsbDebugConsole::PopRx(uint8_t& byte)
{
    if (m_rxTail == m_rxHead)
    {
        return false;
    }

    byte = m_rxBuffer[m_rxTail];
    m_rxTail = static_cast<uint16_t>((m_rxTail + 1) % RxBufferSize);

    return true;
}

bool UsbDebugConsole::PushTx(uint8_t byte)
{
    const uint16_t nextHead = static_cast<uint16_t>((m_txHead + 1) % TxBufferSize);

    if (nextHead == m_txTail)
    {
        return false;
    }

    m_txBuffer[m_txHead] = byte;
    m_txHead = nextHead;

    return true;
}

bool UsbDebugConsole::PopTx(uint8_t& byte)
{
    if (m_txTail == m_txHead)
    {
        return false;
    }

    byte = m_txBuffer[m_txTail];
    m_txTail = static_cast<uint16_t>((m_txTail + 1) % TxBufferSize);

    return true;
}

void UsbDebugConsole::TryStartTransmit()
{
    if (m_txBusy)
    {
        return;
    }

    uint16_t size = 0;
    uint16_t readPos = m_txTail;

    while (size < sizeof(m_usbTxChunk) && readPos != m_txHead)
    {
        m_usbTxChunk[size++] = m_txBuffer[readPos];
        readPos = static_cast<uint16_t>((readPos + 1) % TxBufferSize);
    }

    if (size == 0)
    {
        return;
    }

    const uint8_t result = CDC_Transmit_FS(m_usbTxChunk, size);

    if (result == USBD_OK)
    {
        m_txTail = readPos;
        m_txBusy = true;
    }
    else
    {
        m_txBusy = false;
    }
}
