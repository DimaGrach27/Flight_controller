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
#include "PID/anglecontroller.h"
#include "PID/ratecontroller.h"
#include "Protocols/stm32spibus.h"
#include "Protocols/stm32uartdmarxstream.h"
#include "Protocols/stm32uartdmatxstream.h"
#include "RcInput/rcreceiver.h"
#include "RcInput/rcinput.h"
#include "Scheduler/scheduler.h"
#include "Sensors/analoginputs.h"
#include "Sensors/sensorsmanager.h"
#include "Sensors/Battery/batterymonitor.h"
#include "Sensors/Battery/batteryvoltagesensor.h"
#include "Sensors/Battery/currentsensor.h"

#if NOT_USE_HIL
#include "Sensors/IMU/imu_driver.h"
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
    bool MavlinkParseByte(uint8_t byte);
    void TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
    void ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

    void OnIdleRcUart();

    void OnUsbReceived(const uint8_t* data, uint32_t size);
    void OnTransmitUsbComplete();
    void RunDebugCommand(uint8_t command);

    void SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi);
    void SPI_ErrorCallback(SPI_HandleTypeDef* hspi);
    void UART_RxHalfCpltCallback(UART_HandleTypeDef* huart);
    void UART_RxCpltCallback(UART_HandleTypeDef* huart);
    void UART_TxCpltCallback(UART_HandleTypeDef* huart);
    void UART_ErrorCallback(UART_HandleTypeDef* huart);

    uint32_t GetMicros() const;

private:
    enum class ControlStopReason : uint8_t
    {
        None = 0,
        BatteryCriticalFault = 1,
        BatteryImmediateStop = 2,
        Failsafe = 3,
        Disarmed = 4,
        ImuNotReady = 5,
        StateInvalid = 6,
    };

    void StopMotors();
    void StopMotors(ControlStopReason reason);

    void SendTelemetry(uint32_t nowUs);
    void SendMavlinkMessage(const mavlink_message_t& msg);

    void RunControlLoop(uint32_t nowUs);

    void MavlinkHandleMessage(const mavlink_message_t* msg);
    void HandleHilSensor(const mavlink_message_t* msg);
    void HandleRcCommand(const mavlink_message_t* msg);

private:
    Scheduler m_scheduler;

    UsbDebugConsole m_debugConsole;

#if NOT_USE_HIL
    SpiDmaBus m_dmaBusImu;

    // Stm32SpiBus m_stm32SpiBusImu;
    Stm32UartDmaRxStream m_stm32UartRxDmaCrsfRc;
    Stm32UartDmaTxStream m_stm32UartTxDmaCrsfRc;
    UartByteStream m_uartByteStream;
    RealImuDriver m_realImuDriver;
#else
    IMU_Driver_Hil m_imuDriverHil;
    uint32_t m_lastHilEstimatorImuTimestampUs = 0;
    bool m_hasFreshHilImuForControl = false;
#endif
    Imu_Driver m_imuDriver;
    Imu_Sensor m_imuSensor;
#if NOT_USE_HIL
    AnalogInputs m_analogInputs;
    CurrentSensor m_currentSensor;
    BatteryVoltageSensor m_batteryVoltageSensor;
#endif

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
#if NOT_USE_HIL
    BatteryMonitor m_batteryMonitor;
#endif

    FlightModeManager m_flightModeManager;

    AngleController m_angleController;
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
    ControlStopReason m_lastControlStopReason = ControlStopReason::None;
    float m_lastThrottleLimit = 1.0f;
    float m_lastLimitedThrottle = 0.0f;
    uint32_t m_imuSeq = 0;
    uint32_t m_controlSeq = 0;
    uint32_t m_logSeq = 0;

    UART_HandleTypeDef& m_serialUart;
#if NOT_USE_HIL
    UART_HandleTypeDef& m_rcUart;
#endif

    //DEBUG
    Logger m_logger;
};
