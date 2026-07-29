//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "FlightController/flightcontroller_entry.h"

#include "FlightController/flightcontroller.h"

#include <new>

#if NOT_USE_HIL
#include "FlightController/Motors/motorchannel.h"
#endif

struct FlightControllerHandler
{
    alignas(FlightController) unsigned char storage[sizeof(FlightController)];
    bool constructed;
};

static FlightControllerHandler g_FlightControllerHandler;

#if NOT_USE_HIL
std::array<MotorChannel, 4> motorChannels;
#endif

static FlightController* GetFlightController(FlightControllerHandler* handle)
{
    return reinterpret_cast<FlightController*>(handle->storage);
}

extern "C" void flight_controller_Create(
    UART_HandleTypeDef* huart1,
    UART_HandleTypeDef* huart2,
    SPI_HandleTypeDef* hspi2,
    TIM_HandleTypeDef* htim1,
    ADC_HandleTypeDef* hadc1)
{
    FlightControllerHandler* handler = &g_FlightControllerHandler;
#if NOT_USE_HIL
    motorChannels = {{
        { htim1, TIM_CHANNEL_1 }, // m1
        { htim1, TIM_CHANNEL_2 }, // m2
        { htim1, TIM_CHANNEL_3 }, // m3
        { htim1, TIM_CHANNEL_4 }  // m4
    }};

    new (handler->storage) FlightController(*huart2, *huart1, *hspi2, *hadc1, *htim1);
#else
    new (handler->storage) FlightController(*huart2, *hadc1);
#endif

    handler->constructed = true;
}

extern "C" void flight_controller_Destroy()
{
    if (!g_FlightControllerHandler.constructed)
        return;

    GetFlightController(&g_FlightControllerHandler)->~FlightController(); // явний виклик деструктора
    g_FlightControllerHandler.constructed = false;
}

extern "C" void flight_controller_Init()
{
    if (!g_FlightControllerHandler.constructed)
        return;

    GetFlightController(&g_FlightControllerHandler)->Init();
}

extern "C" void flight_controller_Heartbeat(void)
{
    if (!g_FlightControllerHandler.constructed)
        return;

    GetFlightController(&g_FlightControllerHandler)->Heartbeat();
}

extern "C" bool flight_controller_MavlinkParseByte(uint8_t byte)
{
    if (!g_FlightControllerHandler.constructed)
        return false;

    return GetFlightController(&g_FlightControllerHandler)->MavlinkParseByte(byte);
}

extern "C" void flight_controller_ParseRcCommandByte(uint8_t byte)
{
    // GetFlightController(&g_FlightControllerHandler)->ParseRcCommandByte(byte);
}

extern "C" void flight_controller_Update()
{
    if (!g_FlightControllerHandler.constructed)
        return;

    GetFlightController(&g_FlightControllerHandler)->Update();
}

//USB_DEBUG_CONSOL
extern "C" void UsbDebugConsole_OnReceived(uint8_t* data, uint32_t size)
{
    if (!g_FlightControllerHandler.constructed)
        return;

    GetFlightController(&g_FlightControllerHandler)->OnUsbReceived(data, size);
}

extern "C" void UsbDebugConsole_OnTransmitComplete()
{
    if (!g_FlightControllerHandler.constructed)
        return;

    GetFlightController(&g_FlightControllerHandler)->OnTransmitUsbComplete();
}

extern "C" void UsbDebugConsole_RunDebugCommand(uint8_t command)
{
    if (!g_FlightControllerHandler.constructed)
        return;

    GetFlightController(&g_FlightControllerHandler)->RunDebugCommand(command);
}

extern "C" void UsbDebugConsole_RunDebugTextCommand(const char* command)
{
    if (!g_FlightControllerHandler.constructed)
        return;

    GetFlightController(&g_FlightControllerHandler)->RunDebugTextCommand(command);
}

//INTERAPT CALLBACKS from HAL
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    GetFlightController(&g_FlightControllerHandler)->TIM_PeriodElapsedCallback(htim);
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    GetFlightController(&g_FlightControllerHandler)->ADC_ConvCpltCallback(hadc);
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    GetFlightController(&g_FlightControllerHandler)->SPI_TxRxCpltCallback(hspi);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    GetFlightController(&g_FlightControllerHandler)->SPI_ErrorCallback(hspi);
}

extern "C" void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef* huart)
{
    GetFlightController(&g_FlightControllerHandler)->UART_RxHalfCpltCallback(huart);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    GetFlightController(&g_FlightControllerHandler)->UART_RxCpltCallback(huart);
}

extern "C" void flight_controller_OnIdleDmaReceive_UART1()
{
    GetFlightController(&g_FlightControllerHandler)->OnIdleRcUart();
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    GetFlightController(&g_FlightControllerHandler)->UART_TxCpltCallback(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    GetFlightController(&g_FlightControllerHandler)->UART_ErrorCallback(huart);
}
