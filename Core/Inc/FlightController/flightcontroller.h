//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include <cstdint>

#include "logger.h"
#include "main.h"

#include "mavlink/common/mavlink.h"
#include "structs.h"

#include "Estimators/stateestimator.h"
#include "FlightManager/flightmodemanager.h"
#include "Motors/imotoroutput.h"
#include "Motors/mixer.h"
#include "PID/ratecontroller.h"
#include "Protocols/stm32spibus.h"
#include "Protocols/stm32uartdmabytestream.h"
#include "RcInput/rcinput.h"
#include "Scheduler/scheduler.h"
#include "Sensors/sensorsmanager.h"
#include "Sensors/BatteryVoltage/batteryvoltagesensor.h"

#if NOT_USE_HIL
#include "Sensors/imu_driver_lsm6ds3.h"
#include "Motors/pwmmotoroutput.h"
#include "RcInput/crsfrcreceiver.h"
#else
#include "Sensors/IMU/imu_driver_hil.h"
#include "Motors/hilmotoroutput.h"
#include "RcInput/hilrcreceiver.h"
#endif

class FlightController
{
public:

#if NOT_USE_HIL
    FlightController(
        UART_HandleTypeDef& serialUart,
        UART_HandleTypeDef& rcUart,
        SPI_HandleTypeDef& spiImuHandler,
        ADC_HandleTypeDef& batterAdc,
        std::array<PwmMotorOutput::MotorChannel, 4> motorChannels);
#else
    FlightController(UART_HandleTypeDef& serialUart, ADC_HandleTypeDef& batterAdc);
#endif

    ~FlightController();

    void Init();
    void Heartbeat();
    void Update();
    void MavlinkParseByte(uint8_t byte);

    uint32_t GetMicros() const;

private:
    void RunControlLoop(uint32_t nowUs);

    void MavlinkHandleMessage(const mavlink_message_t* msg);
    void HandleHilSensor(const mavlink_message_t* msg);
    void HandleRcCommand(const mavlink_message_t* msg);

private:
    Scheduler m_scheduler;

#if NOT_USE_HIL
    Stm32SpiBus m_stm32SpiBusImu;
    Stm32UartDmaByteStream m_stm32UartDmaCrsfRc;
    IMU_Lsm6ds3 m_imuLsm6ds3;
#else
    IMU_Driver_Hil m_imuDriverHil;
#endif
    Imu_Driver m_imuDriver;
    Imu_Sensor m_imuSensor;
    BatteryVoltageSensor m_batteryVoltageSensor;
    SensorsManager m_sensorsManager;

#if NOT_USE_HIL
    CrsfRcReceiver m_crsfRcReceiver;
#else
    HilRcReceiver m_hilRcReceiver;
#endif
    RcInput m_rcInput;

    StateEstimator m_stateEstimator;

    FlightModeManager m_flightModeManager;

    RateController m_rateController;
    Mixer m_mixer;

#if NOT_USE_HIL
    PwmMotorOutput m_pwmMotorOutput;
#else
    HilMotorOutput m_hilMotorOutput;
#endif

    ControlOutput m_lastControlOutput{};
    MotorCommand m_lastMotorCommand{};

#if NOT_USE_HIL
    uint8_t* m_receiveBufferRcCommand;
    const uint16_t m_receiveBufferSizeRcCommand = 512;
#endif

    UART_HandleTypeDef& m_serialUart;
    //DEBUG
    Logger m_logger;
};
