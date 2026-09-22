/// @file
/// The micro-ROS side: one node, three best-effort publishers of standard
/// sensor_msgs types, and a 100 Hz timer that drains the sensor mailboxes.
///
/// Connection handling follows micro-ROS's reconnection example: wait for the
/// agent, create entities, and if the agent stops answering tear everything
/// down and go back to waiting — so the node survives the laptop running the
/// agent going to sleep.

#include "config.hpp"
#include "samples.hpp"
#include "sensors.hpp"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <sensor_msgs/msg/range.h>
#include <uros_network_interfaces.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace ping {
namespace {

constexpr const char* kTag = "ros";

constexpr std::int64_t kPublishPeriodMs = 10;
constexpr std::int64_t kAgentCheckPeriodUs = 1'000'000;
constexpr std::int64_t kTimeResyncPeriodUs = 60'000'000;

// TF03 datasheet: 0.5° field of view, 0.1 m blind zone, 100 m reach.
constexpr float kLidarFieldOfViewRad = 0.5F * std::numbers::pi_v<float> / 180.0F;
constexpr float kLidarMinRangeM = 0.1F;
constexpr float kLidarMaxRangeM = 100.0F;

/// Every rcl call returns a status; this logs the failing call and says
/// whether it worked, so setup can bail out and retry cleanly.
bool ok(rcl_ret_t ret, const char* what)
{
    if (ret != RCL_RET_OK) {
        ESP_LOGE(kTag, "%s failed: %d", what, static_cast<int>(ret));
        rcl_reset_error();
        return false;
    }
    return true;
}

/// Points a message's frame_id at a string literal. No allocation, and the
/// literal outlives every message.
void set_frame(rosidl_runtime_c__String& field, const char* frame)
{
    field.data = const_cast<char*>(frame);
    field.size = std::strlen(frame);
    field.capacity = field.size + 1;
}

/// Converts a sample time on the esp_timer clock to ROS time, using the
/// offset micro-ROS learned from the agent.
builtin_interfaces__msg__Time to_ros_time(std::int64_t sample_us)
{
    const std::int64_t age_ns = (esp_timer_get_time() - sample_us) * 1'000;
    const std::int64_t ns = rmw_uros_epoch_nanos() - age_ns;
    return {
        .sec = static_cast<std::int32_t>(ns / 1'000'000'000),
        .nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000),
    };
}

/// Everything that exists only while the agent is reachable.
struct Session {
    rclc_support_t support{};
    rcl_node_t node{};
    rcl_publisher_t imu_pub{};
    rcl_publisher_t range_pub{};
    rcl_publisher_t fix_pub{};
    rcl_timer_t timer{};
    rclc_executor_t executor{};
    /// Whether rclc_support_init got far enough that there is a context to
    /// tear down.
    bool support_up = false;

    sensor_msgs__msg__Imu imu_msg{};
    sensor_msgs__msg__Range range_msg{};
    sensor_msgs__msg__NavSatFix fix_msg{};
};

Session session;

void publish_imu(const ImuSample& s)
{
    auto& m = session.imu_msg;
    m.header.stamp = to_ros_time(s.stamp_us);
    m.orientation = {.x = s.qx, .y = s.qy, .z = s.qz, .w = s.qw};
    m.angular_velocity = {.x = s.gyro_x, .y = s.gyro_y, .z = s.gyro_z};
    m.linear_acceleration = {.x = s.accel_x, .y = s.accel_y, .z = s.accel_z};

    // The hub gives one accuracy figure, for heading. Use it for all three
    // axes: pessimistic for roll and pitch, which gravity pins down far
    // better, but never falsely confident.
    const double variance = static_cast<double>(s.heading_accuracy_rad) * s.heading_accuracy_rad;
    m.orientation_covariance[0] = variance;
    m.orientation_covariance[4] = variance;
    m.orientation_covariance[8] = variance;

    ok(rcl_publish(&session.imu_pub, &m, nullptr), "publish imu");
}

void publish_range(const RangeSample& s)
{
    auto& m = session.range_msg;
    m.header.stamp = to_ros_time(s.stamp_us);
    // REP-117: +Inf means "nothing detected within range".
    m.range = s.valid ? static_cast<float>(s.distance_cm) / 100.0F : std::numeric_limits<float>::infinity();
    ok(rcl_publish(&session.range_pub, &m, nullptr), "publish range");
}

void publish_fix(const FixSample& s)
{
    auto& m = session.fix_msg;
    m.header.stamp = to_ros_time(s.stamp_us);

    const bool has_position = s.fix_ok && s.fix_type >= 2 && s.fix_type <= 4;
    m.status.status = !has_position ? sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX
                      : s.differential ? sensor_msgs__msg__NavSatStatus__STATUS_SBAS_FIX
                                       : sensor_msgs__msg__NavSatStatus__STATUS_FIX;
    // The M9N tracks all four constellations concurrently.
    m.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS |
                       sensor_msgs__msg__NavSatStatus__SERVICE_GLONASS |
                       sensor_msgs__msg__NavSatStatus__SERVICE_COMPASS |
                       sensor_msgs__msg__NavSatStatus__SERVICE_GALILEO;

    m.latitude = s.latitude_deg;
    m.longitude = s.longitude_deg;
    m.altitude = s.altitude_m;

    const double h = s.horizontal_accuracy_m * s.horizontal_accuracy_m;
    const double v = s.vertical_accuracy_m * s.vertical_accuracy_m;
    m.position_covariance[0] = h;
    m.position_covariance[4] = h;
    m.position_covariance[8] = v;
    m.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_DIAGONAL_KNOWN;

    ok(rcl_publish(&session.fix_pub, &m, nullptr), "publish fix");
}

void on_timer(rcl_timer_t*, std::int64_t)
{
    if (auto s = imu_mailbox.take()) {
        publish_imu(*s);
    }
    if (auto s = range_mailbox.take()) {
        publish_range(*s);
    }
    if (auto s = fix_mailbox.take()) {
        publish_fix(*s);
    }
}

void init_messages()
{
    set_frame(session.imu_msg.header.frame_id, config::kImuFrame);
    set_frame(session.range_msg.header.frame_id, config::kLidarFrame);
    set_frame(session.fix_msg.header.frame_id, config::kGnssFrame);

    session.range_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    session.range_msg.field_of_view = kLidarFieldOfViewRad;
    session.range_msg.min_range = kLidarMinRangeM;
    session.range_msg.max_range = kLidarMaxRangeM;

    // Covariance of zero means "unknown" for these two (sensor_msgs/Imu).
    // The BNO085 doesn't report gyro or accelerometer noise.
}

bool create_session()
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    if (!ok(rcl_init_options_init(&options, allocator), "init options")) {
        return false;
    }
    rmw_init_options_t* rmw = rcl_init_options_get_rmw_init_options(&options);
    if (!ok(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw),
            "agent address")) {
        return false;
    }

    auto& s = session;
    const bool created =
        (s.support_up = ok(rclc_support_init_with_options(&s.support, 0, nullptr, &options, &allocator), "support")) &&
        ok(rclc_node_init_default(&s.node, config::kNodeName, "", &s.support), "node") &&
        // Best effort: a dropped IMU sample is replaced 10 ms later, and
        // reliable QoS would stall the publisher waiting on retransmits.
        ok(rclc_publisher_init_best_effort(&s.imu_pub, &s.node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                           config::kImuTopic),
           "imu publisher") &&
        ok(rclc_publisher_init_best_effort(&s.range_pub, &s.node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                                           config::kRangeTopic),
           "range publisher") &&
        ok(rclc_publisher_init_best_effort(&s.fix_pub, &s.node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
                                           config::kFixTopic),
           "fix publisher") &&
        ok(rclc_timer_init_default2(&s.timer, &s.support, RCL_MS_TO_NS(kPublishPeriodMs), on_timer, true),
           "timer") &&
        ok(rclc_executor_init(&s.executor, &s.support.context, 1, &allocator), "executor") &&
        ok(rclc_executor_add_timer(&s.executor, &s.timer), "executor timer");

    rcl_init_options_fini(&options);
    if (!created) {
        return false;
    }

    // Stamps are only meaningful once the MCU clock is tied to the agent's.
    ok(rmw_uros_sync_session(1'000), "time sync");
    return true;
}

void destroy_session()
{
    auto& s = session;
    if (!s.support_up) {
        return;
    }
    rmw_context_t* rmw = rcl_context_get_rmw_context(&s.support.context);
    // The agent is already gone; don't wait on it to acknowledge teardown.
    rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);

    rcl_publisher_fini(&s.imu_pub, &s.node);
    rcl_publisher_fini(&s.range_pub, &s.node);
    rcl_publisher_fini(&s.fix_pub, &s.node);
    rcl_timer_fini(&s.timer);
    rclc_executor_fini(&s.executor);
    rcl_node_fini(&s.node);
    rclc_support_fini(&s.support);

    // Zero every handle so the next create_session starts from scratch; the
    // messages keep their frame ids and fixed fields.
    s.support = {};
    s.node = {};
    s.imu_pub = s.range_pub = s.fix_pub = {};
    s.timer = {};
    s.executor = {};
    s.support_up = false;
}

} // namespace

void ros_task(void*)
{
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    init_messages();

    for (;;) {
        ESP_LOGI(kTag, "waiting for micro-ROS agent at %s:%s", CONFIG_MICRO_ROS_AGENT_IP,
                 CONFIG_MICRO_ROS_AGENT_PORT);
        // Session setup fails fast while the agent is unreachable, so simply
        // retrying it is the wait.
        while (!create_session()) {
            destroy_session();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGI(kTag, "connected; publishing %s, %s, %s", config::kImuTopic, config::kRangeTopic,
                 config::kFixTopic);

        std::int64_t last_check = esp_timer_get_time();
        std::int64_t last_sync = last_check;
        for (;;) {
            rclc_executor_spin_some(&session.executor, RCL_MS_TO_NS(kPublishPeriodMs));

            const std::int64_t now = esp_timer_get_time();
            if (now - last_check > kAgentCheckPeriodUs) {
                last_check = now;
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    ESP_LOGW(kTag, "agent lost");
                    break;
                }
            }
            if (now - last_sync > kTimeResyncPeriodUs) {
                last_sync = now;
                rmw_uros_sync_session(100);
            }
        }

        destroy_session();
    }
}

} // namespace ping
