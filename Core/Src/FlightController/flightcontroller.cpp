//
// Created by Dmytro Hrachov on 01.05.2026.
//

#include "FlightController/flightcontroller.h"

#include <algorithm>

#include "main.h"
#include "FlightController/LowPassFilter.h"
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
    m_scheduler.AddTask(TaskID::Imu, 1000);
    m_scheduler.AddTask(TaskID::Rc, 1000);
    m_scheduler.AddTask(TaskID::Control, 1000);
    m_scheduler.AddTask(TaskID::Telemetry, 1000);

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
    m_pwmMotorOutput.StopAll();
#else
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
    const uint32_t nowUs = HAL_GetTick();

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

    const MotorCommand motors = m_mixer.Mix(rcCommand.throttle, control);

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

    float roll = manual.x / 1000.0f;
    float pitch = manual.y / 1000.0f;
    float throttle = manual.z / 1000.0f;
    float yaw = manual.r / 1000.0f;

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

    frame.timestampUs = HAL_GetTick();
    frame.failsafe = false;
    frame.valid = true;

    m_hilRcReceiver.SetFrame(frame);
#endif
}