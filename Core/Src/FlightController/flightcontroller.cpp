//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "FlightController/flightcontroller.h"

#include <algorithm>

#include "main.h"
#include "../../Inc/FlightController/Utils/lowpassfilter.h"
#include "FlightController/Utils/mathutils.h"
#include "FlightController/PID.h"
#include "FlightController/RcInput/rcchannelutils.h"

#if NOT_USE_HIL
FlightController::FlightController(
    UART_HandleTypeDef& serialUart,
    UART_HandleTypeDef& rcUart,
    SPI_HandleTypeDef& spiImuHandler,
    std::array<PwmMotorOutput::MotorChannel, 4> motorChannels)
    : m_scheduler()
    , m_stm32SpiBusImu(spiImuHandler, CS_SPI2_GPIO_Port, CS_SPI2_Pin)
    , m_stm32UartDmaCrsfRc(rcUart, m_receiveBufferRcCommand, m_receiveBufferSizeRcCommand)
    , m_imuLsm6ds3(m_stm32SpiBusImu)
    , m_imuDriver(m_imuLsm6ds3)
    , m_imuSensor(m_imuDriver)
    , m_sensorsManager(m_imuSensor)
    , m_stateEstimator()
    , m_crsfRcReceiver(m_stm32UartDmaCrsfRc)
    , m_rcInput(m_crsfRcReceiver)
    , m_flightModeManager()
    , m_rateController()
    , m_mixer()
    , m_pwmMotorOutput(motorChannels)
    , m_serialUart(serialUart)
    , m_logger(serialUart)
{
}
#else
FlightController::FlightController(UART_HandleTypeDef &serialUart)
    : m_scheduler()
    , m_imuDriverHil()
    , m_imuDriver(m_imuDriverHil)
    , m_imuSensor(m_imuDriver)
    , m_sensorsManager(m_imuSensor)
    , m_hilRcReceiver()
    , m_rcInput(m_hilRcReceiver)
    , m_stateEstimator()
    , m_flightModeManager()
    , m_rateController()
    , m_mixer()
    , m_hilMotorOutput(serialUart)
    , m_serialUart(serialUart)
    , m_logger(serialUart)
{
}

#endif

FlightController::~FlightController()
{

}

void FlightController::Init()
{
    m_scheduler.AddTask(TaskID::Imu, 2000);         //500 Hz
    m_scheduler.AddTask(TaskID::Rc, 5000);          //200 Hz
    m_scheduler.AddTask(TaskID::Control, 2000);     //500 Hz
    m_scheduler.AddTask(TaskID::Telemetry, 100000); //10 Hz
    m_scheduler.AddTask(TaskID::Loging, 10000); //100 Hz

    if (!m_sensorsManager.Init())
    {
        return;
    }

    m_stateEstimator.Init();

    if (!m_rcInput.Init())
    {
        return;
    }

    m_flightModeManager.Init();
    m_rateController.Init();
    m_mixer.Init();

#if NOT_USE_HIL
    m_pwmMotorOutput.Init();
    m_pwmMotorOutput.StopAll();
#else
    m_hilMotorOutput.Init();
    m_hilMotorOutput.StopAll();
#endif
}

void FlightController::Heartbeat()
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        1,                      // system_id
        1,                      // component_id
        &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_MANUAL_ARMED,
        0,
        MAV_STATE_ACTIVE
    );

    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    HAL_UART_Transmit(&m_serialUart, buffer, len, 100);
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

    if (m_scheduler.ConsumeTask(TaskID::Rc))
    {
        m_rcInput.Update(nowUs);

        m_flightModeManager.Update(m_rcInput.GetCommand(), nowUs);
    }

    if (m_scheduler.ConsumeTask(TaskID::Control))
    {
        RunControlLoop(nowUs);
    }

    if (m_scheduler.ConsumeTask(TaskID::Telemetry))
    {

    }

    if (m_scheduler.ConsumeTask(TaskID::Loging))
    {
        const ImuSample& m_imu_sample = m_sensorsManager.GetImuData();
        const RcCommand& rcCommand = m_rcInput.GetCommand();
        const VehicleState& state = m_stateEstimator.GetState();
        const FlightModeState& flightModeState = m_flightModeManager.GetState();
        const RateData& rateData = m_rateController.GetRateData();

        m_logger.GetLogSample().timeMs = nowUs / 1000.0f;
        m_logger.GetLogSample().imuDt = state.imuDt;

        m_logger.GetLogSample().flightMode = flightModeState.mode == FlightMode::Acro ? 1.0f : 0.0f;
        m_logger.GetLogSample().armed = flightModeState.armState == ArmState::Armed ? 1.0f : 0.0f;

        m_logger.GetLogSample().gyroRollRadSec = m_imu_sample.gyro_rads.x;
        m_logger.GetLogSample().gyroPitchRadSec = m_imu_sample.gyro_rads.y;
        m_logger.GetLogSample().gyroYawRadSec = m_imu_sample.gyro_rads.z;
        m_logger.GetLogSample().gyroMagnitude = m_imu_sample.gyro_rads.Length();

        m_logger.GetLogSample().accelRoll = m_imu_sample.accel_mps2.x;
        m_logger.GetLogSample().accelPitch = m_imu_sample.accel_mps2.y;
        m_logger.GetLogSample().accelYaw = m_imu_sample.accel_mps2.z;

        m_logger.GetLogSample().targetRollRateRadSec = rateData.targetRollRad;
        m_logger.GetLogSample().targetPitchRateRadSec = rateData.targetPitchRad;
        m_logger.GetLogSample().targetYawRateRadSec = rateData.targetYawRad;

        m_logger.GetLogSample().rcThrottle = rcCommand.throttle;
        m_logger.GetLogSample().rcRoll = rcCommand.roll;
        m_logger.GetLogSample().rcPitch = rcCommand.pitch;
        m_logger.GetLogSample().rcYaw = rcCommand.yaw;

        m_logger.GetLogSample().estimatedRollRad = state.rollRad;
        m_logger.GetLogSample().estimatedPitchRad = state.pitchRad;
        m_logger.GetLogSample().estimatedYawRad = state.yawRad;

        m_logger.GetLogSample().gyroBiasX = state.gyroBias.x;
        m_logger.GetLogSample().gyroBiasY = state.gyroBias.y;
        m_logger.GetLogSample().gyroBiasZ = state.gyroBias.z;
        m_logger.GetLogSample().accelWeight = state.accelWeight;
        m_logger.GetLogSample().accelMagnitude = m_imu_sample.accel_mps2.Length();
        m_logger.GetLogSample().ahrsValid = state.ahrsValid;

        m_logger.GetLogSample().controlRoll = m_lastControlOutput.roll;
        m_logger.GetLogSample().controlPitch = m_lastControlOutput.pitch;
        m_logger.GetLogSample().controlYaw = m_lastControlOutput.yaw;

        m_logger.GetLogSample().motorM1 = m_lastMotorCommand.m1;
        m_logger.GetLogSample().motorM2 = m_lastMotorCommand.m2;
        m_logger.GetLogSample().motorM3 = m_lastMotorCommand.m3;
        m_logger.GetLogSample().motorM4 = m_lastMotorCommand.m4;

        m_logger.SendFlightLogCsv();
    }
}

void FlightController::MavlinkParseByte(uint8_t byte)
{
    mavlink_message_t msg;
    mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
    {
        MavlinkHandleMessage(&msg);
    }
}

uint32_t FlightController::GetMicros() const
{
    return HAL_GetTick() * 1000U;
}

void FlightController::RunControlLoop(uint32_t nowUs)
{
    const RcCommand rcCommand = m_rcInput.GetCommand();
    const VehicleState& state = m_stateEstimator.GetState();

    if (m_flightModeManager.IsFailsafe())
    {
        m_rateController.Reset();
#if NOT_USE_HIL
        m_pwmMotorOutput.StopAll();
#else
        m_hilMotorOutput.StopAll();
#endif
        return;
    }

    if (!m_flightModeManager.IsArmed())
    {
        m_rateController.Reset();
#if NOT_USE_HIL
        m_pwmMotorOutput.StopAll();
#else
        m_hilMotorOutput.StopAll();
#endif
        return;
    }

    if (!m_sensorsManager.IsImuReady())
    {
#if NOT_USE_HIL
        m_pwmMotorOutput.StopAll();
#else
        m_hilMotorOutput.StopAll();
#endif
        m_rateController.Reset();
        return;
    }

    if (!state.valid)
    {
        m_rateController.Reset();
#if NOT_USE_HIL
        m_pwmMotorOutput.StopAll();
#else
        m_hilMotorOutput.StopAll();
#endif
        return;
    }

    ControlOutput control{};

    if (m_flightModeManager.GetMode() == FlightMode::Acro)
    {
        control = m_rateController.Update(rcCommand, state, nowUs);
    }
    else
    {
        /*
            Angle mode додамо пізніше.
            Поки для safety можна або стопати мотори,
            або тимчасово теж використовувати acro.
        */
        control = m_rateController.Update(rcCommand, state, nowUs);
    }

    m_lastControlOutput = control;

    const MotorCommand motors = m_mixer.Mix(rcCommand.throttle, control);

    m_lastMotorCommand = motors;

#if NOT_USE_HIL
    m_pwmMotorOutput.Write(motors);
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