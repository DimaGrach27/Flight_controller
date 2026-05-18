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
    GetFlightController(&g_FlightControllerHandler)->Init();
}

extern "C" void flight_controller_Heartbeat(void)
{
    GetFlightController(&g_FlightControllerHandler)->Heartbeat();
}

extern "C" void flight_controller_MavlinkParseByte(uint8_t byte)
{
    GetFlightController(&g_FlightControllerHandler)->MavlinkParseByte(byte);
}

extern "C" void flight_controller_ParseRcCommandByte(uint8_t byte)
{
    // GetFlightController(&g_FlightControllerHandler)->ParseRcCommandByte(byte);
}

extern "C" void flight_controller_Update()
{
    GetFlightController(&g_FlightControllerHandler)->Update();
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    GetFlightController(&g_FlightControllerHandler)->OnDmaComplete(htim);
}