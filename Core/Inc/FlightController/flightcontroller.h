//
// Created by Dmytro Hrachov on 01.05.2026.
//

#pragma once

#include <cstdint>

#include "crsftelemetry.h"
#include "DebugLogs/logger.h"
#include "main.h"

#include "mavlink/common/mavlink.h"
#include "structs.h"
#include "DebugLogs/usbdebugconsole.h"

#include "Estimators/stateestimator.h"
#include "FlightManager/flightmodemanager.h"
#include "Motors/dshotmotoroutput.h"
#include "Motors/imotoroutput.h"
#include "Motors/mixer.h"
#include "PID/ratecontroller.h"
#include "Protocols/stm32spibus.h"
#include "Protocols/stm32uartdmabytestream.h"
#include "RcInput/rcreceiver.h"
#include "RcInput/rcinput.h"
#include "Scheduler/scheduler.h"
#include "Sensors/analoginputs.h"
#include "Sensors/sensorsmanager.h"
#include "Sensors/Battery/batterymonitor.h"
#include "Sensors/Battery/batteryvoltagesensor.h"
#include "Sensors/Battery/currentsensor.h"

#if NOT_USE_HIL
#include "Sensors/IMU/imu_driver_lsm6ds3.h"
#include "Motors/pwmmotoroutput.h"
#include "Motors/motorchannel.h"
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
        TIM_HandleTypeDef& dshotTimer
        // std::array<MotorChannel, 4> motorChannels
        );
#else
    FlightController(UART_HandleTypeDef& serialUart, ADC_HandleTypeDef& batterAdc);
#endif

    ~FlightController();

    void PreInit(UART_HandleTypeDef& serialUart,
        UART_HandleTypeDef& rcUart,
        SPI_HandleTypeDef& spiImuHandler,
        ADC_HandleTypeDef& batterAdc,
        TIM_HandleTypeDef& dshotTimer);
    void Init();
    void Heartbeat();
    void Update();
    void MavlinkParseByte(uint8_t byte);
    void OnDmaComplete(TIM_HandleTypeDef* htim);
    void OnDmaComplete(ADC_HandleTypeDef* hadc);

    void OnIdleRcUart();

    void OnUsbReceived(const uint8_t* data, uint32_t size);
    void OnTransmitUsbComplete();
    void RunDebugCommand(uint8_t command);

    void SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi);
    void SPI_ErrorCallback(SPI_HandleTypeDef* hspi);
    void UART_RxHalfCpltCallback(UART_HandleTypeDef* huart);
    void UART_RxCpltCallback(UART_HandleTypeDef* huart);

    uint32_t GetMicros() const;

private:
    void SendTelemetry(uint32_t nowUs);
    void SendMavlinkMessage(const mavlink_message_t& msg);

    void RunControlLoop(uint32_t nowUs);

    void MavlinkHandleMessage(const mavlink_message_t* msg);
    void HandleHilSensor(const mavlink_message_t* msg);
    void HandleRcCommand(const mavlink_message_t* msg);

private:
    Scheduler m_scheduler;

#if NOT_USE_HIL
    SpiDmaBus m_dmaBusImu;

    // Stm32SpiBus m_stm32SpiBusImu;
    Stm32UartDmaByteStream m_stm32UartDmaCrsfRc;
    UartByteStream m_uartByteStream;
    IMU_Lsm6ds3 m_imuLsm6ds3;
#else
    IMU_Driver_Hil m_imuDriverHil;
#endif
    Imu_Driver m_imuDriver;
    Imu_Sensor m_imuSensor;
    AnalogInputs m_analogInputs;
    CurrentSensor m_currentSensor;
    BatteryVoltageSensor m_batteryVoltageSensor;
    SensorsManager m_sensorsManager;

#if NOT_USE_HIL
    CrsfTelemetry m_crsfTelemetry;
    CrsfRcReceiver m_crsfRcReceiver;
#else
    HilRcReceiver m_hilRcReceiver;
#endif
    RcReceiver m_rcReceiver;
    RcInput m_rcInput;

    StateEstimator m_stateEstimator;

    BatteryMonitor m_batteryMonitor;
    FlightModeManager m_flightModeManager;

    RateController m_rateController;
    Mixer m_mixer;

#if NOT_USE_HIL
    DshotMotorOutput::Config m_dshotMotorOutputConfig{};
    DshotMotorOutput m_dshotMotorOutput;
    // PwmMotorOutput m_pwmMotorOutput;
#else
    HilMotorOutput m_hilMotorOutput;
#endif

    ControlOutput m_lastControlOutput{};
    MotorCommand m_lastMotorCommand{};

    UART_HandleTypeDef& m_serialUart;
    UART_HandleTypeDef& m_rcUart;

    //DEBUG
    Logger m_logger;
    UsbDebugConsole m_debugConsole;
};
