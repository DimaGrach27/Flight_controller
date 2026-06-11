//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "FlightController/flightcontroller.h"

#include <algorithm>

#include "main.h"
#include "FlightController/RcInput/rcchannelutils.h"
#include "FlightController/Sensors/IMU/imuaxismapper.h"
#if NOT_USE_HIL
FlightController::FlightController(
    UART_HandleTypeDef& serialUart,
    UART_HandleTypeDef& rcUart,
    SPI_HandleTypeDef& spiImuHandler,
    ADC_HandleTypeDef& batterAdc,
    TIM_HandleTypeDef& dshotTimer
    // std::array<MotorChannel, 4> motorChannels
    )
    : m_scheduler()
    , m_dmaBusImu(&spiImuHandler, CS_SPI2_GPIO_Port, CS_SPI2_Pin)
    , m_stm32UartRxDmaCrsfRc(rcUart)
    , m_stm32UartTxDmaCrsfRc(rcUart)
    , m_uartByteStream(m_stm32UartRxDmaCrsfRc, m_stm32UartTxDmaCrsfRc)
    , m_imuLsm6ds3(m_dmaBusImu)
    , m_imuDriver(m_imuLsm6ds3)
    , m_imuSensor(m_imuDriver)
    , m_analogInputs(batterAdc)
    , m_currentSensor()
    , m_batteryVoltageSensor()
    , m_sensorsManager(m_imuSensor)
    , m_crsfTelemetry(m_uartByteStream)
    , m_crsfRcReceiver(m_uartByteStream)
    , m_rcReceiver(m_crsfRcReceiver)
    , m_rcInput(m_rcReceiver)
    , m_stateEstimator()
    , m_batteryMonitor(4)
    , m_flightModeManager()
    , m_rateController()
    , m_mixer()
    , m_dshotMotorOutput(&dshotTimer, m_dshotMotorOutputConfig)
    // , m_pwmMotorOutput(motorChannels)
    , m_serialUart(serialUart)
    , m_rcUart(rcUart)
    , m_logger(serialUart)
    , m_debugConsole()
{
}
#else
FlightController::FlightController(UART_HandleTypeDef& serialUart, ADC_HandleTypeDef& batterAdc)
    : m_scheduler()
    , m_debugConsole()
    , m_imuDriverHil()
    , m_imuDriver(m_imuDriverHil)
    , m_imuSensor(m_imuDriver)
    , m_sensorsManager(m_imuSensor)
    , m_hilRcReceiver()
    , m_rcReceiver(m_hilRcReceiver)
    , m_rcInput(m_rcReceiver)
    , m_stateEstimator()
    , m_flightModeManager()
    , m_rateController()
    , m_mixer()
    , m_hilMotorOutput(m_debugConsole)
    , m_serialUart(serialUart)
    , m_logger(serialUart)
{
}

#endif

FlightController::~FlightController()
{

}

void FlightController::PreInit(
    UART_HandleTypeDef &serialUart,
    UART_HandleTypeDef &rcUart,
    SPI_HandleTypeDef &spiImuHandler,
    ADC_HandleTypeDef &batterAdc,
    TIM_HandleTypeDef &dshotTimer)
{

}

void FlightController::Init()
{
    m_scheduler.AddTask(TaskID::Imu, 2000);         //500 Hz
    m_scheduler.AddTask(TaskID::Rc, 2000);          //500 Hz
    m_scheduler.AddTask(TaskID::Control, 2000);     //500 Hz
    m_scheduler.AddTask(TaskID::Telemetry, 100000); //10 Hz
    m_scheduler.AddTask(TaskID::Loging, 10000);     //100 Hz
    m_scheduler.AddTask(TaskID::Battery, 100000);   //10 Hz


#if NOT_USE_HIL
    bool analogGood = m_analogInputs.Start();
    if (analogGood)
    {
        //TODO: faild
    }

    m_currentSensor.StartZeroCalibration();
    m_batteryVoltageSensor.Init();
#endif

    if (!m_sensorsManager.Init())
    {
        //TODO: faild
    }

    m_stateEstimator.Init();

    if (!m_rcInput.Init())
    {
        //TODO: faild
    }

#if NOT_USE_HIL
    m_crsfTelemetry.Init();

    m_batteryMonitor.Init();
#endif

    m_flightModeManager.Init();
    m_rateController.Init();
    m_mixer.Init();

#if NOT_USE_HIL
    m_dshotMotorOutput.Init();
    m_dshotMotorOutput.StopAll();
    // m_pwmMotorOutput.Init();
    // m_pwmMotorOutput.StopAll();
#else
    m_hilMotorOutput.Init();
    m_hilMotorOutput.StopAll();
#endif

    m_debugConsole.Init();
}

void FlightController::Heartbeat()
{

    const FlightModeState& state = m_flightModeManager.GetState();
    char modeText[17]{};
    std::snprintf(modeText, sizeof(modeText), "%s %s%s",
        EnumToChar_FlightMode(state.mode),
        state.armState == ArmState::Armed ? "ARM" : "DIS",
        state.failsafe ? " FS" : "");

#if NOT_USE_HIL
    m_crsfTelemetry.SendFlightMode(modeText);
#else
    //TODO:send telemetry to Gazebo
#endif
}

void FlightController::Update()
{
    const uint32_t nowUs = GetMicros();

    m_scheduler.Update(nowUs);

    if (m_scheduler.ConsumeTask(TaskID::Imu))
    {
        m_sensorsManager.UpdateImu(nowUs);

        m_stateEstimator.UpdateImu(m_sensorsManager.GetImuData());
    }

#if NOT_USE_HIL
    if (m_scheduler.ConsumeTask(TaskID::Battery))
    {
        m_analogInputs.Update();
        m_currentSensor.Update(m_analogInputs.GetFilteredRaw(AnalogInputs::Channel::Current));
        m_batteryVoltageSensor.Update(m_analogInputs.GetFilteredRaw(AnalogInputs::Channel::Vbat));

        const float currentVoltage = m_batteryVoltageSensor.GetVoltageFiltered();
        const float currentCurrent = m_currentSensor.GetFilteredCurrentA();
        m_batteryMonitor.Update(
            currentVoltage,
            currentCurrent,
            m_flightModeManager.GetState().armState == ArmState::Armed,
            nowUs);
    }
    const BatteryData& batteryData = m_batteryMonitor.GetBatteryData();
#endif

    if (m_scheduler.ConsumeTask(TaskID::Rc))
    {
        m_rcInput.Update(nowUs);

#if NOT_USE_HIL
        m_flightModeManager.Update(m_rcInput.GetCommand(), batteryData, m_batteryMonitor.CanArm(), nowUs);
#else
        m_flightModeManager.Update(m_rcInput.GetCommand(), nowUs);
#endif
    }

#if NOT_USE_HIL
    //Should disable calibration befor arm
    if (m_flightModeManager.GetState().armState == ArmState::Armed)
    {
        m_currentSensor.StopZeroCalibration();
    }
#endif

    if (m_scheduler.ConsumeTask(TaskID::Control))
    {
        RunControlLoop(nowUs);
    }

    if (m_scheduler.ConsumeTask(TaskID::Telemetry))
    {
        SendTelemetry(nowUs);
    }

    if (m_scheduler.ConsumeTask(TaskID::Loging))
    {
        const ImuSample& m_imu_sample = m_sensorsManager.GetImuData();
        const RcCommand& rcCommand = m_rcInput.GetCommand();
        const VehicleState& state = m_stateEstimator.GetState();
        const FlightModeState& flightModeState = m_flightModeManager.GetState();
        const RateData& rateData = m_rateController.GetRateData();

        // ImuAxisMapper imuAxisMapper = ImuAxisMapper();

        // Vector3 mappedAccel = imuAxisMapper.MapAccel(m_imu_sample.accel_mps2);
        // Vector3 mappedGyro = imuAxisMapper.MapGyro(m_imu_sample.gyro_rads);
        // Vector3 mappedAccel = m_imu_sample.accel_mps2;
        // Vector3 mappedGyro = m_imu_sample.gyro_rads;

        // char logData[128];
        //
        // int len = snprintf(
        //     logData,
        //     sizeof(logData),
        //     "accel: %.3f %.3f %.3f gyro: %.3f %.3f %.3f\r\n",
        //     mappedAccel.x,
        //     mappedAccel.y,
        //     mappedAccel.z,
        //     mappedGyro.x,
        //     mappedGyro.y,
        //     mappedGyro.z
        // );
        //
        // m_debugConsole.WriteLine(logData);

        m_logger.GetLogSample().timeMs = nowUs / 1000.0f;
        m_logger.GetLogSample().imuDt = state.imuDt;

        m_logger.GetLogSample().flightMode = flightModeState.mode == FlightMode::Acro ? 1.0f : 0.0f;
        m_logger.GetLogSample().armed = flightModeState.armState == ArmState::Armed ? 1.0f : 0.0f;

        m_logger.GetLogSample().gyroRollDegSec = m_imu_sample.gyro_rads.x;
        m_logger.GetLogSample().gyroPitchDegSec = m_imu_sample.gyro_rads.y;
        m_logger.GetLogSample().gyroYawDegSec = m_imu_sample.gyro_rads.z;

        m_logger.GetLogSample().accelRoll = m_imu_sample.accel_mps2.x;
        m_logger.GetLogSample().accelPitch = m_imu_sample.accel_mps2.y;
        m_logger.GetLogSample().accelYaw = m_imu_sample.accel_mps2.z;

        m_logger.GetLogSample().targetRollRateDegSec = rateData.targetRollRad;
        m_logger.GetLogSample().targetPitchRateDegSec = rateData.targetPitchRad;
        m_logger.GetLogSample().targetYawRateDegSec = rateData.targetYawRad;

        m_logger.GetLogSample().rcThrottle = rcCommand.throttle;
        m_logger.GetLogSample().rcRoll = rcCommand.roll;
        m_logger.GetLogSample().rcPitch = rcCommand.pitch;
        m_logger.GetLogSample().rcYaw = rcCommand.yaw;

        m_logger.GetLogSample().estimatedRollDeg = state.rollRad;
        m_logger.GetLogSample().estimatedPitchDeg = state.pitchRad;

        m_logger.GetLogSample().controlRoll = m_lastControlOutput.roll;
        m_logger.GetLogSample().controlPitch = m_lastControlOutput.pitch;
        m_logger.GetLogSample().controlYaw = m_lastControlOutput.yaw;

        m_logger.GetLogSample().motorM1 = m_lastMotorCommand.m1;
        m_logger.GetLogSample().motorM2 = m_lastMotorCommand.m2;
        m_logger.GetLogSample().motorM3 = m_lastMotorCommand.m3;
        m_logger.GetLogSample().motorM4 = m_lastMotorCommand.m4;

        m_logger.SendFlightLogCsv();

        m_debugConsole.Update(nowUs);
    }
}

bool FlightController::MavlinkParseByte(uint8_t byte)
{
    mavlink_message_t msg;
    mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
    {
        MavlinkHandleMessage(&msg);

        return true;
    }

    return false;
}

void FlightController::TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#if NOT_USE_HIL
    m_dshotMotorOutput.OnDmaComplete(htim);
#endif

}

void FlightController::ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
#if NOT_USE_HIL
    m_analogInputs.OnDmaComplete(hadc);
#endif
}

void FlightController::OnIdleRcUart()
{
#if NOT_USE_HIL
    m_stm32UartRxDmaCrsfRc.OnIdleIrq();
#endif
}

void FlightController::OnUsbReceived(const uint8_t *data, uint32_t size)
{
    m_debugConsole.OnUsbReceived(data, size);
}

void FlightController::OnTransmitUsbComplete()
{
    m_debugConsole.OnTransmitComplete();
}

void FlightController::RunDebugCommand(uint8_t command)
{
    switch (static_cast<UsbDebugConsoleCommand>(command))
    {
        case UsbDebugConsoleCommand::Status:
        {
            const VehicleState& vehicleState = m_stateEstimator.GetState();
            const FlightModeState& flightMode = m_flightModeManager.GetState();

            m_debugConsole.ShowFlightStatus(flightMode);
            break;
        }

        case UsbDebugConsoleCommand::IMU_Status:
        {
            break;
        }

        case UsbDebugConsoleCommand::Battery_Status:
        {
#if NOT_USE_HIL

            const BatteryData& batteryData = m_batteryMonitor.GetBatteryData();
            m_debugConsole.ShowBatteryStatus(batteryData);
#endif
            break;
        }

        case UsbDebugConsoleCommand::CalibrateAccel:
        {
            break;
        }

        case UsbDebugConsoleCommand::CalibrateGyro:
        {
            break;
        }

        case UsbDebugConsoleCommand::Reboot:
        {
            break;
        }
        default:
            break;
    }
}

void FlightController::SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
#if NOT_USE_HIL
    m_dmaBusImu.OnDmaComplete(hspi);
#endif
}

void FlightController::SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
#if NOT_USE_HIL
    m_dmaBusImu.OnDmaError(hspi);
#endif
}

void FlightController::UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
#if NOT_USE_HIL
    if (huart == &m_rcUart)
    {
        m_stm32UartRxDmaCrsfRc.OnDmaProgressIrq();
    }
#endif
}

void FlightController::UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if NOT_USE_HIL
    if (huart == &m_rcUart)
    {
        m_stm32UartRxDmaCrsfRc.OnDmaProgressIrq();
    }
#endif
}

void FlightController::UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#if NOT_USE_HIL
    if (huart == &m_rcUart)
    {
        m_stm32UartTxDmaCrsfRc.OnTxComplete();
    }
#endif
}

void FlightController::UART_ErrorCallback(UART_HandleTypeDef *huart)
{
#if NOT_USE_HIL
    if (huart == &m_rcUart)
    {
        m_stm32UartTxDmaCrsfRc.OnTxError();
    }
#endif
}

uint32_t FlightController::GetMicros() const
{
    return HAL_GetTick() * 1000U;
}

void FlightController::StopMotors()
{
    m_rateController.Reset();
#if NOT_USE_HIL
    // m_pwmMotorOutput.StopAll();
    m_dshotMotorOutput.StopAll();
#else
    m_hilMotorOutput.StopAll();
#endif
}

void FlightController::RunControlLoop(uint32_t nowUs)
{
    const RcCommand rcCommand = m_rcInput.GetCommand();
    const VehicleState& state = m_stateEstimator.GetState();

#if MOTOR_DIRECT_TEST
    MotorCommand stop{};
    stop.m1 = 0.0f;
    stop.m2 = 0.0f;
    stop.m3 = 0.0f;
    stop.m4 = 0.0f;

    static uint32_t startMs = HAL_GetTick();

    while ((HAL_GetTick() - startMs) < 3000U)
    {
        m_dshotMotorOutput.Write(stop);
        HAL_Delay(2);
    }

    // m_dshotMotorOutput.Write({0, 0, 0, 0});
    m_dshotMotorOutput.Write({rcCommand.throttle, rcCommand.throttle, rcCommand.throttle, rcCommand.throttle});
    return;
#endif

#if NOT_USE_HIL
    if (m_batteryMonitor.HasCriticalFault())
    {
        StopMotors();
        return;
    }

    if (m_batteryMonitor.ShouldStopMotorsImmediately())
    {
        StopMotors();
        return;
    }
#endif

    if (m_flightModeManager.IsFailsafe())
    {
        StopMotors();
        return;
    }

    if (!m_flightModeManager.IsArmed())
    {
        StopMotors();
        return;
    }

    if (!m_sensorsManager.IsImuReady())
    {
        StopMotors();
        return;
    }

    if (!state.valid)
    {
        StopMotors();
        return;
    }

    ControlOutput control{};

    if (m_flightModeManager.GetMode() == FlightMode::Acro)
    {
        control = m_rateController.Update(rcCommand, state, nowUs);
    }
    else
    {
        //TODO: add angle mode
        control = m_rateController.Update(rcCommand, state, nowUs);
    }

    m_lastControlOutput = control;

#if NOT_USE_HIL
    const float limitedThrottle = rcCommand.throttle * m_batteryMonitor.GetThrottleLimit();
#else
    const float limitedThrottle = rcCommand.throttle;
#endif
    const MotorCommand motors = m_mixer.Mix(limitedThrottle, control);

    m_lastMotorCommand = motors;

#if NOT_USE_HIL
    m_dshotMotorOutput.Write(motors);
    // m_pwmMotorOutput.Write(motors);
#else
    m_hilMotorOutput.Write(motors);
#endif
}

void FlightController::MavlinkHandleMessage(const mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HIL_SENSOR:
            HandleHilSensor(msg);
            break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            HandleRcCommand(msg);
            break;
        default:
            break;
    }
}

void FlightController::HandleHilSensor(const mavlink_message_t* msg)
{
#if !NOT_USE_HIL
    mavlink_hil_sensor_t sensor;
    mavlink_msg_hil_sensor_decode(msg, &sensor);

    ImuSample imuSample{};
    imuSample.accel_mps2.x = sensor.xacc;
    imuSample.accel_mps2.y = sensor.yacc;
    imuSample.accel_mps2.z = sensor.zacc;

    imuSample.gyro_rads.x = sensor.xgyro;
    imuSample.gyro_rads.y = sensor.ygyro;
    imuSample.gyro_rads.z = sensor.zgyro;

    imuSample.timestampUs = sensor.time_usec;

    imuSample.valid = true;

    m_imuDriverHil.SetHilData(imuSample);
#endif
}

void FlightController::HandleRcCommand(const mavlink_message_t* msg)
{
#if !NOT_USE_HIL
    mavlink_manual_control_t manual;
    mavlink_msg_manual_control_decode(msg, &manual);
    // x/y/r зазвичай -1000..1000, z 0..1000

    constexpr uint16_t ChannelMin = 172;
    constexpr uint16_t ChannelMid = 992;
    constexpr uint16_t ChannelMax = 1811;

    RcRawFrame frame{};

    float roll = manual.y / 1000.0f;
    float pitch = -manual.x / 1000.0f;
    float throttle = manual.z / 1000.0f;
    float yaw = -manual.r / 1000.0f;

    constexpr uint8_t armedInputMask = 1u << 1;
    constexpr uint8_t flightModeInputMask = 1u << 2;
    bool arm = (manual.buttons & armedInputMask) != 0;
    bool angleMode = (manual.buttons & flightModeInputMask) == 0;

    frame.channelCount = 16;
    frame.channels[0] = RcChannelUtils::FromNormalizedCentered(
        roll,
        ChannelMin,
        ChannelMid,
        ChannelMax
    );

    frame.channels[1] = RcChannelUtils::FromNormalizedCentered(
        pitch,
        ChannelMin,
        ChannelMid,
        ChannelMax
    );

    frame.channels[2] = RcChannelUtils::FromNormalizedThrottle(
        throttle,
        ChannelMin,
        ChannelMax
    );

    frame.channels[3] = RcChannelUtils::FromNormalizedCentered(
        yaw,
        ChannelMin,
        ChannelMid,
        ChannelMax
    );

    frame.channels[4] = RcChannelUtils::FromSwitch(
        arm,
        ChannelMin,
        ChannelMax
    );

    frame.channels[5] = RcChannelUtils::FromSwitch(
        angleMode,
        ChannelMin,
        ChannelMax
    );

    frame.timestampUs = GetMicros();
    frame.failsafe = false;
    frame.valid = true;

    m_hilRcReceiver.SetFrame(frame);
#endif
}

void FlightController::SendTelemetry(uint32_t nowUs)
{
#if NOT_USE_HIL
    (void)nowUs;
    const BatteryData& batteryData = m_batteryMonitor.GetBatteryData();

    Heartbeat();
    const uint32_t consumedMah = static_cast<uint32_t>(std::clamp(m_batteryMonitor.GetConsumedMah(), 0.0f, 16777215.0f));
    m_crsfTelemetry.SendBattery(
        batteryData.voltage_V,
        batteryData.current_A,
        consumedMah,
        batteryData.percentage);
#endif
}

void FlightController::SendMavlinkMessage(const mavlink_message_t& msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    // HAL_UART_Transmit(&m_serialUart, buffer, len, 100);
}