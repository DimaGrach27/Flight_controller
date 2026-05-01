//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "FlightController/flightcontroller_entry.h"

#include "FlightController/flightcontroller.h"

static FlightController FlightController;

extern "C" void flight_controller_Init(UART_HandleTypeDef* huart2)
{
    FlightController.Init(*huart2);
}

extern "C" void flight_controller_Update(float dt)
{
    FlightController.Update(dt);
}

extern "C" void flight_controller_Heartbeat(void)
{
    FlightController.Heartbeat();
}

extern "C" void flight_controller_MavlinkParseByte(uint8_t byte)
{
    FlightController.MavlinkParseByte(byte);
}