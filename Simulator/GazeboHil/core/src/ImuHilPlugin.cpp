//
// Created by Dmytro Hrachov on 03.05.2026.
//

#include "ImuHilPlugin.h"

#include "TelemetryOverlay.h"

#include <gz/plugin/Register.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/math/Quaternion.hh>

#include <algorithm>
#include <cmath>
#include <iostream>

NAMESPACE_BEGIN
ImuHilPlugin::~ImuHilPlugin()
{

}

void ImuHilPlugin::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)

{
    (void)entity;
    (void)ecm;
    (void)eventMgr;

    if (sdf->HasElement("imu_topic"))
    {
        m_imuTopic = sdf->Get<std::string>("imu_topic");
    }

    if (sdf->HasElement("motor_topic"))
    {
        m_motorTopic = sdf->Get<std::string>("motor_topic");
    }

    if (sdf->HasElement("telemetry_topic"))
    {
        m_telemetryTopic = sdf->Get<std::string>("telemetry_topic");
    }

    if (sdf->HasElement("telemetry_rate_hz"))
    {
        m_telemetryRateHz = sdf->Get<double>("telemetry_rate_hz");
    }

    if (sdf->HasElement("throttle"))
    {
        m_throttle = sdf->Get<float>("throttle");
    }

    if (sdf->HasElement("serial_port"))
        m_serialPortPath = sdf->Get<std::string>("serial_port");

    if (sdf->HasElement("baud"))
        m_baud = sdf->Get<int>("baud");

    if (sdf->HasElement("use_joystick"))
        m_useJoystick = sdf->Get<bool>("use_joystick");

    if (sdf->HasElement("joystick_index"))
        m_joystickIndex = sdf->Get<int>("joystick_index");

    m_motorPublisher = m_node.Advertise<gz::msgs::Actuators>(m_motorTopic);
    m_telemetryPublisher = m_node.Advertise<gz::msgs::StringMsg>(m_telemetryTopic);

    if (!m_motorPublisher)
    {
        std::cerr
            << "[ImuHilPlugin] Failed to advertise motor topic: "
            << m_motorTopic
            << "\n";
    }
    else
    {
        std::cout
            << "[ImuHilPlugin] Publishing motors to: "
            << m_motorTopic
            << "\n";
    }

    if (!m_telemetryPublisher)
    {
        std::cerr
            << "[ImuHilPlugin] Failed to advertise telemetry topic: "
            << m_telemetryTopic
            << "\n";
    }
    else
    {
        std::cout
            << "[ImuHilPlugin] Publishing telemetry to: "
            << m_telemetryTopic
            << "\n";
    }

    const bool subscribedImu = m_node.Subscribe(
        m_imuTopic,
        &ImuHilPlugin::OnImu,
        this
    );

    const bool subscribedOdometry = m_node.Subscribe(
        m_groundTruthTopic,
        &ImuHilPlugin::OnOdometry,
        this
);

    if (!subscribedImu)
    {
        printf("[ImuHilPlugin] Failed to subscribe IMU topic: %s\n", m_imuTopic.c_str());
    }
    else
    {
        printf("[ImuHilPlugin] Subscribed to IMU topic: %s\n", m_imuTopic.c_str());
    }

    if (!subscribedOdometry)
    {
        printf("[ImuHilPlugin] Failed to subscribe Odometry topic: %s\n", m_groundTruthTopic.c_str());
    }
    else
    {
        printf("[ImuHilPlugin] Subscribed to Odometry topic: %s\n", m_groundTruthTopic.c_str());
    }

    if (m_useJoystick)
    {
        m_joystickInput.Init(m_joystickIndex);
    }

    if (m_mavlinkBridge.Open(m_serialPortPath, m_baud,
        [this](const std::unordered_map<std::string, float>& fields)
                    {
                        HandleBinaryLogFields(fields);
                    }))
    {
        printf("[ImuHilPlugin] Mavlink opened on %s", m_serialPortPath.c_str());
    }

    m_csvLogger.Open("imu_hil_log.csv", m_csvLogger.HEADER_LOG_FLIGHT_SAMPLE);
}

void ImuHilPlugin::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
    (void)ecm;

    if (info.paused)
    {
        return;
    }

    m_mavlinkBridge.Poll();

    double dt = 0.001;

    if (m_hasLastSimTime)
    {
        const auto delta = info.simTime - m_lastSimTime;
        dt = std::chrono::duration<double>(delta).count();
        dt = std::clamp(dt, 0.0001, 0.02);
    }

    m_lastSimTime = info.simTime;
    m_hasLastSimTime = true;

    const ImuData imu = GetLatestImu();

    if (!imu.valid)
    {
        SendMotorSpeeds(0.0, 0.0, 0.0, 0.0);
        return;
    }

    /*
        Тут поки мінімальна перевірка:
        всі мотори однаково.

        Наступним кроком сюди підключимо твій flight controller:

            SensorData sensor;
            sensor.rollRad = attitude_.rollRad;
            sensor.pitchRad = attitude_.pitchRad;
            sensor.rollRateRadSec = imu.gyroX;
            sensor.pitchRateRadSec = imu.gyroY;
            sensor.yawRateRadSec = imu.gyroZ;

            MotorOutputs motors = controller.Update(sensor, rcInput, dt);
    */

    const MotorOutputs& motorsData = m_mavlinkBridge.Motors();

    // SendMotorSpeeds(
    //     ToMotorSpeed(m_throttle),
    //     ToMotorSpeed(m_throttle),
    //     ToMotorSpeed(m_throttle),
    //     ToMotorSpeed(m_throttle)
    // );

    SendMotorSpeeds(
        ToMotorSpeed(motorsData.m1),
        ToMotorSpeed(motorsData.m2),
        ToMotorSpeed(motorsData.m3),
        ToMotorSpeed(motorsData.m4)
    );

    // SendMotorSpeeds(
    //     ToMotorSpeed(0.96), //front right
    //     ToMotorSpeed(0.72), //back left
    //     ToMotorSpeed(0.96), //front left
    //     ToMotorSpeed(0.72)  //back right
    // );
}

void ImuHilPlugin::PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm)
{
    if (info.paused)
        return;

    PrintStatsIfNeeded();

    const ImuData imu = GetLatestImu();
    if (!imu.valid)
    {
        printf("[ImuHilPlugin] ImuData invalid");
        return;
    }

    double simTimeSec = std::chrono::duration<double>(info.simTime).count();

    double dt = 0.001;
    if (m_hasLastSimTime)
    {
        const auto delta = info.simTime - m_lastSimTime;
        dt = std::chrono::duration<double>(delta).count();
        dt = std::clamp(dt, 0.0001, 0.02);
    }

    if (m_lastHilSendSec < 0.0 ||
    simTimeSec - m_lastHilSendSec >= 1.0 / m_hilRateHz)
    {
        m_lastHilSendSec = simTimeSec;

        const double nowWallSec = NowWallSec();

        ++m_hilTxCount;

        if (m_lastHilWallSec > 0.0)
        {
            const double dtHil = nowWallSec - m_lastHilWallSec;
            m_hilDtMin = std::min(m_hilDtMin, dtHil);
            m_hilDtMax = std::max(m_hilDtMax, dtHil);
        }

        m_lastHilWallSec = nowWallSec;

        // printf("[ImuHilPlugin] HilSendSec: %f\n", simTimeSec);
        m_mavlinkBridge.SendHilSensorFromImu(
            static_cast<uint64_t>(simTimeSec * 1000000.0),
            imu
        );
    }

    if (m_useJoystick)
    {
        m_joystickInput.Poll();
        const double now = NowWallSec();

        const ManualControl& control = m_joystickInput.Control();

        // const bool changed = HasInputChanged(control, m_lastManualControl);
        // const bool periodicRefresh = (simTimeSec - m_lastManualSendSec) >= 1.0 / m_manualRateHz;

        if (ShouldSendManual(control) && control.valid)
        {
            m_lastManualSendSec = now;
            m_lastManualControl = control;

            // printf("[Joystick] arm=%d roll=%f pitch=%f yaw=%f throttle=%f\n",
            //     control.arm,
            //     control.roll,
            //     control.pitch,
            //     control.yaw,
            //     control.throttle);
            // std::cout
            // << "[Joystick] "
            // <<  "arm=" << control.arm
            // << "roll=" << control.roll
            // << " pitch=" << control.pitch
            // << " throttle=" << control.throttle
            // << " yaw=" << control.yaw
            // << std::endl;


            ++m_manualTxCount;

            m_mavlinkBridge.SendManualControl(
                control.arm,
                control.acroMode,
                control.roll,
                control.pitch,
                control.throttle,
                control.yaw
            );
        }
    }

    PublishTelemetry(simTimeSec, imu);
}

void ImuHilPlugin::HandleBinaryLogFields(const std::unordered_map<std::string, float>& fields)
{
    m_currentLogFields = fields;

    if (m_latestGroundTruth.valid)
    {
        m_currentLogFields["truth_x"] = m_latestGroundTruth.x;
        m_currentLogFields["truth_y"] = m_latestGroundTruth.y;
        m_currentLogFields["truth_z"] = m_latestGroundTruth.z;
        m_currentLogFields["truth_vx"] = m_latestGroundTruth.vx;
        m_currentLogFields["truth_vy"] = m_latestGroundTruth.vy;
        m_currentLogFields["truth_vz"] = m_latestGroundTruth.vz;
    }

    m_csvLogger.Log(m_currentLogFields);
}

void ImuHilPlugin::PublishTelemetry(const double simTimeSec, const ImuData& imu)
{
    if (!m_telemetryPublisher)
    {
        return;
    }

    if (m_lastTelemetryPubSec >= 0.0 &&
        simTimeSec - m_lastTelemetryPubSec < 1.0 / m_telemetryRateHz)
    {
        return;
    }

    m_lastTelemetryPubSec = simTimeSec;

    TelemetryOverlayState state;
    state.simTimeSec = simTimeSec;
    state.rollRateDegSec = imu.gyroX * 57.2957795;
    state.pitchRateDegSec = imu.gyroY * 57.2957795;
    state.yawRateDegSec = imu.gyroZ * 57.2957795;

    if (m_latestGroundTruth.valid)
    {
        m_currentLogFields["truth_x"] = m_latestGroundTruth.x;
        m_currentLogFields["truth_y"] = m_latestGroundTruth.y;
        m_currentLogFields["truth_z"] = m_latestGroundTruth.z;
        m_currentLogFields["truth_vx"] = m_latestGroundTruth.vx;
        m_currentLogFields["truth_vy"] = m_latestGroundTruth.vy;
        m_currentLogFields["truth_vz"] = m_latestGroundTruth.vz;
    }

    gz::msgs::StringMsg msg;
    msg.set_data(FormatTelemetryOverlay(m_currentLogFields, m_mavlinkBridge.Motors(), state));
    m_telemetryPublisher.Publish(msg);
}

void ImuHilPlugin::OnImu(const gz::msgs::IMU &msg)
{
    ImuData data;

    data.gyroX = msg.angular_velocity().x();
    data.gyroY = msg.angular_velocity().y();
    data.gyroZ = msg.angular_velocity().z();

    data.accelX = msg.linear_acceleration().x();
    data.accelY = msg.linear_acceleration().y();
    data.accelZ = msg.linear_acceleration().z();

    data.valid = true;

    {
        std::lock_guard<std::mutex> lock(m_imuMutex);
        m_latestImu = data;
    }
}

void ImuHilPlugin::OnOdometry(const gz::msgs::Odometry &msg)
{
    m_latestGroundTruth.x = msg.pose().position().x();
    m_latestGroundTruth.y = msg.pose().position().y();
    m_latestGroundTruth.z = msg.pose().position().z();

    m_latestGroundTruth.vx = msg.twist().linear().x();
    m_latestGroundTruth.vy = msg.twist().linear().y();
    m_latestGroundTruth.vz = msg.twist().linear().z();

    m_latestGroundTruth.valid = true;
}

ImuData ImuHilPlugin::GetLatestImu() const
{
    std::lock_guard<std::mutex> lock(m_imuMutex);
    return m_latestImu;
}

void ImuHilPlugin::SendMotorSpeeds(double m0, double m1, double m2, double m3)
{
    gz::msgs::Actuators msg;

    /*model motor ordering
     *m0 = front right
     *m1 = rear left
     *m2 = front left
     *m3 = rear right
    */
    msg.add_velocity(m0);
    msg.add_velocity(m1);
    msg.add_velocity(m2);
    msg.add_velocity(m3);

    // printf("[ImuHilPlugin] SendMotorSpeeds: %f %f %f %f\n", m0, m1, m2, m3);
    m_motorPublisher.Publish(msg);
}

double ImuHilPlugin::ToMotorSpeed(double normalized)
{
    normalized = std::clamp(normalized, 0.0, 1.0);

    constexpr double minSpeed = 0.0;
    constexpr double maxSpeed = 1200.0;

    return minSpeed + normalized * (maxSpeed - minSpeed);
}

double ImuHilPlugin::NowWallSec()
{
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

void ImuHilPlugin::PrintStatsIfNeeded()
{
    const double nowWallSec = NowWallSec();

    if (m_lastStatsWallSec <= 0.0)
    {
        m_lastStatsWallSec = nowWallSec;
        return;
    }

    const double elapsed = nowWallSec - m_lastStatsWallSec;

    if (elapsed < 1.0)
    {
        return;
    }

    const double hilHz = m_hilTxCount / elapsed;
    const double servoHz = m_mavlinkBridge.m_servoRxCount / elapsed;
    const double manualHz = m_manualTxCount / elapsed;
    // const double debugHz = m_debugRxCount / elapsed;

    printf(
        "[HIL Stats] hil_tx=%.1f Hz servo_rx=%.1f Hz manual_tx=%.1f Hz | "
        "hil_dt=%.2f..%.2f ms servo_dt=%.2f..%.2f ms servo_max_count=%i\n",
        hilHz,
        servoHz,
        manualHz,
        // debugHz,
        m_hilDtMin * 1000.0,
        m_hilDtMax * 1000.0,
        m_mavlinkBridge.m_servoDtMin * 1000.0,
        m_mavlinkBridge.m_servoDtMax * 1000.0,
        m_mavlinkBridge.m_servoMaxCount
    );

    m_hilTxCount = 0;
    m_mavlinkBridge.m_servoRxCount = 0;
    m_manualTxCount = 0;
    // m_debugRxCount = 0;

    m_hilDtMin = 999.0;
    m_hilDtMax = 0.0;
    m_mavlinkBridge.m_servoDtMin = 999.0;
    m_mavlinkBridge.m_servoDtMax = 0.0;

    m_lastStatsWallSec = nowWallSec;
}

bool ImuHilPlugin::HasInputChanged(const ManualControl &a, const ManualControl &b)
{
    constexpr int eps = 5;

    return std::abs(a.throttle - b.throttle) > eps ||
           std::abs(a.roll - b.roll) > eps ||
           std::abs(a.pitch - b.pitch) > eps ||
           std::abs(a.yaw - b.yaw) > eps ||
           a.arm != b.arm ||
           a.acroMode != b.acroMode;
}

bool ImuHilPlugin::ShouldSendManual(const ManualControl &current)
{
    const double now = NowWallSec();

    constexpr double minPeriodSec = 1.0 / 20.0;   // максимум 20 Hz
    constexpr double forcePeriodSec = 1.0 / 10.0; // хоча б 10 Hz refresh

    const bool rateLimitPassed =
        (now - m_lastManualSendSec) >= minPeriodSec;

    const bool forceRefresh =
        (now - m_lastManualSendSec) >= forcePeriodSec;

    const bool changed = HasInputChanged(current, m_lastManualControl);

    return forceRefresh || (changed && rateLimitPassed);
}

NAMESPACE_END

GZ_ADD_PLUGIN(
    hil_plugin::ImuHilPlugin,
    gz::sim::System,
    hil_plugin::ImuHilPlugin::ISystemConfigure,
    hil_plugin::ImuHilPlugin::ISystemPreUpdate,
    hil_plugin::ImuHilPlugin::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    hil_plugin::ImuHilPlugin,
    "hil_plugin::ImuHilPlugin"
)
