//
// Created by Dmytro Hrachov on 29.04.2026.
//
#pragma once

#include "mavlink/mavlink_types.h"

typedef struct
{
    float x;
    float y;
    float z;
} Vector3;

typedef struct
{
    Vector3 gyro;
    Vector3 accel;
    bool valid;
} SimImuSample;

static SimImuSample g_simImu = {0};

void update_flight_controller(void);
void flight_Update(float dt);

void heartbeat(void);
void MavlinkParseByte(uint8_t byte);
void MavlinkHandleMessage(const mavlink_message_t* msg);
void HandleHilSensor(const mavlink_message_t* msg);
void SendServoOutputRaw(float leftMotor, float rightMotor);