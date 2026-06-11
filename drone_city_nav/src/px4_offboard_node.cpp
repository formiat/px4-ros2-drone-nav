#include "drone_city_nav/types.hpp"

#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::uint8_t boundedUint8(const std::int64_t value) {
  return static_cast<std::uint8_t>(std::clamp<std::int64_t>(value, 0, 255));
}

[[nodiscard]] std::uint16_t boundedUint16(const std::int64_t value) {
  return static_cast<std::uint16_t>(std::clamp<std::int64_t>(value, 0, 65535));
}

[[nodiscard]] double pathPointX(const nav_msgs::msg::Path &path,
                                const std::size_t index) {
  return path.poses.at(index).pose.position.x;
}

[[nodiscard]] double pathPointY(const nav_msgs::msg::Path &path,
                                const std::size_t index) {
  return path.poses.at(index).pose.position.y;
}

[[nodiscard]] const char *commandName(const std::uint32_t command) noexcept {
  switch (command) {
  case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE:
    return "VEHICLE_CMD_DO_SET_MODE";
  case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM:
    return "VEHICLE_CMD_COMPONENT_ARM_DISARM";
  default:
    return "UNKNOWN";
  }
}

} // namespace

class Px4OffboardNode final : public rclcpp::Node {
public:
  Px4OffboardNode() : Node{"px4_offboard_node"} {
    cruise_altitude_m_ = declare_parameter<double>("cruise_altitude_m", 12.0);
    min_navigation_altitude_m_ =
        std::clamp(declare_parameter<double>("min_navigation_altitude_m", 0.0),
                   0.0, std::abs(cruise_altitude_m_));
    face_target_yaw_ = declare_parameter<bool>("face_target_yaw", false);
    acceptance_radius_m_ =
        declare_parameter<double>("acceptance_radius_m", 1.5);
    max_setpoint_distance_m_ =
        std::clamp(declare_parameter<double>("max_setpoint_distance_m", 2.0),
                   0.5, 50.0);
    max_commanded_target_step_m_ = std::clamp(
        declare_parameter<double>("max_commanded_target_step_m", 0.25), 0.01,
        10.0);
    lookahead_distance_m_ =
        std::clamp(declare_parameter<double>("lookahead_distance_m", 6.0),
                   0.0, 50.0);
    path_switch_hysteresis_m_ =
        std::clamp(declare_parameter<double>("path_switch_hysteresis_m", 3.0),
                   0.0, 100.0);
    path_continuity_reuse_radius_m_ = std::clamp(
        declare_parameter<double>("path_continuity_reuse_radius_m", 6.0), 0.0,
        100.0);
    path_continuity_max_target_distance_m_ =
        std::clamp(declare_parameter<double>(
                       "path_continuity_max_target_distance_m", 20.0),
                   0.0, 500.0);
    warmup_setpoints_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("warmup_setpoints", 20), 1, 100000));
    auto_arm_ = declare_parameter<bool>("auto_arm", true);
    auto_offboard_ = declare_parameter<bool>("auto_offboard", true);
    command_resend_period_s_ =
        declare_parameter<double>("command_resend_period_s", 2.0);
    mission_goal_ = Point2{declare_parameter<double>("goal_x_m", 85.0),
                           declare_parameter<double>("goal_y_m", 0.0)};
    hold_x_m_ = declare_parameter<double>("hold_x_m", 0.0);
    hold_y_m_ = declare_parameter<double>("hold_y_m", 0.0);
    target_system_ =
        boundedUint8(declare_parameter<std::int64_t>("target_system", 1));
    target_component_ =
        boundedUint8(declare_parameter<std::int64_t>("target_component", 1));
    source_system_ =
        boundedUint8(declare_parameter<std::int64_t>("source_system", 1));
    source_component_ =
        boundedUint16(declare_parameter<std::int64_t>("source_component", 1));

    const std::string path_topic =
        declare_parameter<std::string>("path_topic", "/drone_city_nav/path");
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");
    const std::string vehicle_status_topic = declare_parameter<std::string>(
        "px4_vehicle_status_topic", "/fmu/out/vehicle_status");
    const std::string emergency_stop_topic = declare_parameter<std::string>(
        "emergency_stop_topic", "/drone_city_nav/emergency_stop");

    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic, rclcpp::QoS{1}.reliable(),
        [this](const nav_msgs::msg::Path::SharedPtr msg) { onPath(*msg); });
    local_position_sub_ =
        create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            local_position_topic, px4_qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
              onLocalPosition(*msg);
            });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
          onVehicleStatus(*msg);
        });
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        emergency_stop_topic, rclcpp::QoS{1}.reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          onEmergencyStop(*msg);
        });

    offboard_control_mode_pub_ =
        create_publisher<px4_msgs::msg::OffboardControlMode>(
            declare_parameter<std::string>("offboard_control_mode_topic",
                                           "/fmu/in/offboard_control_mode"),
            px4_qos);
    trajectory_setpoint_pub_ =
        create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            declare_parameter<std::string>("trajectory_setpoint_topic",
                                           "/fmu/in/trajectory_setpoint"),
            px4_qos);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        declare_parameter<std::string>("vehicle_command_topic",
                                       "/fmu/in/vehicle_command"),
        px4_qos);

    timer_ = create_wall_timer(std::chrono::milliseconds{100},
                               [this]() { onTimer(); });
    last_command_time_ =
        now() - rclcpp::Duration::from_seconds(command_resend_period_s_);

    RCLCPP_INFO(
        get_logger(),
        "PX4 offboard node ready: altitude=%.1fm acceptance=%.1fm auto_arm=%s "
        "auto_offboard=%s min_navigation_altitude=%.1fm face_target_yaw=%s "
        "max_setpoint_distance=%.1fm commanded_target_step=%.2fm "
        "lookahead=%.1fm "
        "path_switch_hysteresis=%.1fm path_continuity_reuse_radius=%.1fm "
        "path_continuity_max_target_distance=%.1fm mission_goal=(%.1f, %.1f)",
        cruise_altitude_m_, acceptance_radius_m_, auto_arm_ ? "true" : "false",
        auto_offboard_ ? "true" : "false", min_navigation_altitude_m_,
        face_target_yaw_ ? "true" : "false", max_setpoint_distance_m_,
        max_commanded_target_step_m_, lookahead_distance_m_,
        path_switch_hysteresis_m_,
        path_continuity_reuse_radius_m_,
        path_continuity_max_target_distance_m_, mission_goal_.x,
        mission_goal_.y);
    RCLCPP_INFO(get_logger(),
                "PX4 offboard subscriptions: path='%s' local_position='%s' "
                "vehicle_status='%s' emergency_stop='%s'",
                path_topic.c_str(), local_position_topic.c_str(),
                vehicle_status_topic.c_str(), emergency_stop_topic.c_str());
  }

private:
  void onPath(const nav_msgs::msg::Path &path) {
    const bool had_active_target = path_valid_ && local_position_valid_ &&
                                   waypoint_index_ < path_.poses.size();
    const Point2 previous_target =
        had_active_target ? Point2{pathPointX(path_, waypoint_index_),
                                   pathPointY(path_, waypoint_index_)}
                          : Point2{};

    path_ = path;
    path_valid_ = !path_.poses.empty();

    if (!path_valid_) {
      if (last_logged_path_size_ != 0U) {
        if (local_position_valid_) {
          no_path_hold_target_ = current_position_;
          no_path_hold_target_valid_ = true;
          commanded_target_ = no_path_hold_target_;
          commanded_target_valid_ = true;
          waypoint_index_ = 0U;
          RCLCPP_WARN(
              get_logger(),
              "Received empty path; holding fixed target at current position "
              "(%.2f, %.2f) and resetting commanded target",
              no_path_hold_target_.x, no_path_hold_target_.y);
        } else {
          no_path_hold_target_valid_ = false;
          commanded_target_valid_ = false;
          waypoint_index_ = 0U;
          RCLCPP_WARN(get_logger(),
                      "Received empty path before local position; holding "
                      "configured fallback target");
        }
        last_logged_path_size_ = 0U;
      }
      return;
    }

    no_path_hold_target_valid_ = false;
    const std::size_t candidate_index = lookaheadWaypointIndex();
    waypoint_index_ =
        continuityWaypointIndex(previous_target, candidate_index,
                                had_active_target);
    const Point2 first{pathPointX(path_, 0U), pathPointY(path_, 0U)};
    const Point2 last{pathPointX(path_, path_.poses.size() - 1U),
                      pathPointY(path_, path_.poses.size() - 1U)};
    const bool path_changed = path_.poses.size() != last_logged_path_size_ ||
                              squaredDistance(first, last_logged_path_first_) >
                                  0.01 ||
                              squaredDistance(last, last_logged_path_last_) >
                                  0.01;
    if (path_changed) {
      RCLCPP_INFO(get_logger(),
                  "Received path: waypoints=%zu selected=%zu first=(%.2f, %.2f) "
                  "last=(%.2f, %.2f)",
                  path_.poses.size(), waypoint_index_ + 1U, first.x, first.y,
                  last.x, last.y);
      last_logged_path_size_ = path_.poses.size();
      last_logged_path_first_ = first;
      last_logged_path_last_ = last;
    }
  }

  [[nodiscard]] std::size_t closestWaypointIndex() const {
    if (!local_position_valid_ || path_.poses.empty()) {
      return 0U;
    }

    std::size_t closest_index = 0U;
    double closest_distance_sq = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0U; i < path_.poses.size(); ++i) {
      const Point2 waypoint{pathPointX(path_, i), pathPointY(path_, i)};
      const double distance_sq = squaredDistance(current_position_, waypoint);
      if (distance_sq < closest_distance_sq) {
        closest_distance_sq = distance_sq;
        closest_index = i;
      }
    }

    return closest_index;
  }

  [[nodiscard]] std::size_t lookaheadWaypointIndex() const {
    if (!local_position_valid_ || path_.poses.empty()) {
      return 0U;
    }

    const std::size_t closest_index = closestWaypointIndex();
    const double current_goal_distance =
        distance(current_position_, mission_goal_);
    for (std::size_t i = closest_index; i < path_.poses.size(); ++i) {
      const Point2 waypoint{pathPointX(path_, i), pathPointY(path_, i)};
      const bool far_enough =
          distance(current_position_, waypoint) >= lookahead_distance_m_;
      const bool progresses_to_goal =
          distance(waypoint, mission_goal_) + acceptance_radius_m_ <
          current_goal_distance;
      if (far_enough && progresses_to_goal) {
        return i;
      }
    }

    return path_.poses.size() - 1U;
  }

  [[nodiscard]] std::size_t
  continuityWaypointIndex(const Point2 previous_target,
                          const std::size_t candidate_index,
                          const bool had_active_target) const {
    if (!had_active_target || path_switch_hysteresis_m_ <= 0.0 ||
        path_continuity_reuse_radius_m_ <= 0.0 ||
        candidate_index >= path_.poses.size()) {
      return candidate_index;
    }
    if (distance(current_position_, previous_target) >
        path_continuity_max_target_distance_m_) {
      return candidate_index;
    }

    const Point2 candidate{pathPointX(path_, candidate_index),
                           pathPointY(path_, candidate_index)};
    if (distance(previous_target, candidate) <= path_switch_hysteresis_m_) {
      return candidate_index;
    }

    std::size_t closest_index = candidate_index;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0U; i < path_.poses.size(); ++i) {
      const Point2 waypoint{pathPointX(path_, i), pathPointY(path_, i)};
      const double waypoint_distance = distance(previous_target, waypoint);
      if (waypoint_distance < closest_distance) {
        closest_distance = waypoint_distance;
        closest_index = i;
      }
    }

    if (closest_distance <= path_continuity_reuse_radius_m_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Path continuity kept waypoint near previous target: candidate=%zu "
          "kept=%zu previous_target=(%.2f, %.2f) closest_distance=%.2f",
          candidate_index + 1U, closest_index + 1U, previous_target.x,
          previous_target.y, closest_distance);
      return closest_index;
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Path continuity allowed target switch: candidate=%zu "
        "previous_target=(%.2f, %.2f) closest_distance=%.2f",
        candidate_index + 1U, previous_target.x, previous_target.y,
        closest_distance);
    return candidate_index;
  }

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition &msg) {
    if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring invalid PX4 local position: xy_valid=%s x=%.2f y=%.2f",
          msg.xy_valid ? "true" : "false", static_cast<double>(msg.x),
          static_cast<double>(msg.y));
      return;
    }

    current_position_ =
        Point2{static_cast<double>(msg.x), static_cast<double>(msg.y)};
    if (msg.z_valid && std::isfinite(msg.z)) {
      current_altitude_m_ = -static_cast<double>(msg.z);
      altitude_valid_ = true;
      if (!navigation_started_ &&
          current_altitude_m_ >= min_navigation_altitude_m_) {
        navigation_started_ = true;
        RCLCPP_INFO(get_logger(),
                    "Navigation altitude reached: altitude=%.2f required=%.2f",
                    current_altitude_m_, min_navigation_altitude_m_);
      }
    }
    if (std::isfinite(msg.heading)) {
      current_heading_rad_ = static_cast<double>(msg.heading);
    }
    local_position_valid_ = true;
    if (!takeoff_hold_target_valid_) {
      takeoff_hold_target_ = current_position_;
      takeoff_hold_target_valid_ = true;
    }

    if (!local_position_seen_) {
      local_position_seen_ = true;
      RCLCPP_INFO(get_logger(),
                  "First valid PX4 local position: x=%.2f y=%.2f "
                  "altitude=%.2f heading=%.2f",
                  current_position_.x, current_position_.y, current_altitude_m_,
                  current_heading_rad_);
    }
  }

  void onVehicleStatus(const px4_msgs::msg::VehicleStatus &msg) {
    vehicle_status_ = msg;
    vehicle_status_valid_ = true;

    const auto arming_state = static_cast<int>(msg.arming_state);
    const auto nav_state = static_cast<int>(msg.nav_state);
    if (arming_state != last_logged_arming_state_ ||
        nav_state != last_logged_nav_state_) {
      RCLCPP_INFO(get_logger(), "PX4 vehicle status: arming_state=%d nav_state=%d",
                  arming_state, nav_state);
      last_logged_arming_state_ = arming_state;
      last_logged_nav_state_ = nav_state;
    }
  }

  void onEmergencyStop(const std_msgs::msg::Bool &msg) {
    if (!msg.data || emergency_stop_requested_) {
      return;
    }

    emergency_stop_requested_ = true;
    path_valid_ = false;
    RCLCPP_ERROR(get_logger(),
                 "Emergency stop requested; stopping trajectory setpoints and "
                 "sending disarm commands");
  }

  void onTimer() {
    if (emergency_stop_requested_) {
      handleEmergencyStop();
      return;
    }

    publishOffboardControlMode();
    publishTrajectorySetpoint();
    logControlSummary();

    if (setpoint_counter_ < warmup_setpoints_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Offboard warmup: sent %d/%d setpoints",
                           setpoint_counter_, warmup_setpoints_);
      ++setpoint_counter_;
      return;
    }

    const rclcpp::Time current_time = now();
    if ((current_time - last_command_time_).seconds() <
        command_resend_period_s_) {
      return;
    }

    if (auto_offboard_ && !isOffboard()) {
      publishVehicleCommand(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      last_command_time_ = current_time;
      return;
    }

    if (auto_arm_ && !isArmed()) {
      publishVehicleCommand(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
          1.0F);
      last_command_time_ = current_time;
    }
  }

  void handleEmergencyStop() {
    const rclcpp::Time current_time = now();
    if ((current_time - last_command_time_).seconds() <
        command_resend_period_s_) {
      return;
    }

    publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F,
        21196.0F);
    last_command_time_ = current_time;
  }

  void publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = nowMicros();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_control_mode_pub_->publish(msg);
  }

  void publishTrajectorySetpoint() {
    advanceWaypointIfNeeded();

    const Point2 target = smoothedCommandTarget(limitedTarget(currentTarget()));
    const float nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = nowMicros();
    msg.position = std::array<float, 3>{
        static_cast<float>(target.x), static_cast<float>(target.y),
        static_cast<float>(-std::abs(cruise_altitude_m_))};
    msg.velocity = std::array<float, 3>{0.0F, 0.0F, 0.0F};
    msg.acceleration = std::array<float, 3>{nan, nan, nan};
    msg.jerk = std::array<float, 3>{nan, nan, nan};
    msg.yaw =
        static_cast<float>(face_target_yaw_ ? targetYaw(target)
                                            : current_heading_rad_);
    msg.yawspeed = nan;

    trajectory_setpoint_pub_->publish(msg);
  }

  void publishVehicleCommand(const std::uint32_t command,
                             const float param1 = 0.0F,
                             const float param2 = 0.0F) {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = nowMicros();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = target_system_;
    msg.target_component = target_component_;
    msg.source_system = source_system_;
    msg.source_component = source_component_;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Sent PX4 command: %s (%u) param1=%.2f param2=%.2f",
                commandName(command), static_cast<unsigned int>(command),
                static_cast<double>(param1), static_cast<double>(param2));
  }

  void advanceWaypointIfNeeded() {
    if (!path_valid_ || !local_position_valid_) {
      return;
    }

    const std::size_t previous_waypoint_index = waypoint_index_;
    while (waypoint_index_ + 1U < path_.poses.size()) {
      const Point2 waypoint{pathPointX(path_, waypoint_index_),
                            pathPointY(path_, waypoint_index_)};
      if (distance(current_position_, waypoint) > acceptance_radius_m_) {
        break;
      }
      ++waypoint_index_;
    }

    if (waypoint_index_ != previous_waypoint_index) {
      const Point2 target{pathPointX(path_, waypoint_index_),
                          pathPointY(path_, waypoint_index_)};
      RCLCPP_INFO(get_logger(),
                  "Waypoint advanced: index=%zu/%zu current=(%.2f, %.2f) "
                  "target=(%.2f, %.2f)",
                  waypoint_index_ + 1U, path_.poses.size(), current_position_.x,
                  current_position_.y, target.x, target.y);
    }
  }

  [[nodiscard]] Point2 currentTarget() const {
    if (!navigationAllowed()) {
      if (takeoff_hold_target_valid_) {
        return takeoff_hold_target_;
      }
      if (local_position_valid_) {
        return current_position_;
      }
    }

    if (path_valid_ && waypoint_index_ < path_.poses.size()) {
      return Point2{pathPointX(path_, waypoint_index_),
                    pathPointY(path_, waypoint_index_)};
    }

    if (no_path_hold_target_valid_) {
      return no_path_hold_target_;
    }

    if (local_position_valid_) {
      return current_position_;
    }

    return Point2{hold_x_m_, hold_y_m_};
  }

  [[nodiscard]] Point2 limitedTarget(const Point2 target) const {
    if (!local_position_valid_) {
      return target;
    }

    const double dx = target.x - current_position_.x;
    const double dy = target.y - current_position_.y;
    const double target_distance = std::hypot(dx, dy);
    if (target_distance <= max_setpoint_distance_m_ ||
        !(target_distance > 0.0)) {
      return target;
    }

    const double scale = max_setpoint_distance_m_ / target_distance;
    return Point2{current_position_.x + dx * scale,
                  current_position_.y + dy * scale};
  }

  Point2 smoothedCommandTarget(const Point2 desired_target) {
    if (!local_position_valid_) {
      commanded_target_valid_ = false;
      return desired_target;
    }
    if (!commanded_target_valid_) {
      commanded_target_ = limitedTarget(desired_target);
      commanded_target_valid_ = true;
      return commanded_target_;
    }

    const double dx = desired_target.x - commanded_target_.x;
    const double dy = desired_target.y - commanded_target_.y;
    const double target_step = std::hypot(dx, dy);
    if (target_step <= max_commanded_target_step_m_ || !(target_step > 0.0)) {
      commanded_target_ = desired_target;
      return clampCommandedTargetToCurrent();
    }

    const double scale = max_commanded_target_step_m_ / target_step;
    commanded_target_ =
        Point2{commanded_target_.x + dx * scale, commanded_target_.y + dy * scale};
    return clampCommandedTargetToCurrent();
  }

  Point2 clampCommandedTargetToCurrent() {
    if (!local_position_valid_) {
      return commanded_target_;
    }

    const double target_distance = distance(current_position_, commanded_target_);
    if (target_distance <= max_setpoint_distance_m_ || !(target_distance > 0.0)) {
      return commanded_target_;
    }

    const double scale = max_setpoint_distance_m_ / target_distance;
    commanded_target_ = Point2{
        current_position_.x + (commanded_target_.x - current_position_.x) * scale,
        current_position_.y + (commanded_target_.y - current_position_.y) * scale};
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Clamped commanded target to current position radius: distance=%.2f "
        "limit=%.2f target=(%.2f, %.2f)",
        target_distance, max_setpoint_distance_m_, commanded_target_.x,
        commanded_target_.y);
    return commanded_target_;
  }

  [[nodiscard]] double targetYaw(const Point2 target) const {
    if (!local_position_valid_) {
      return current_heading_rad_;
    }

    const double dx = target.x - current_position_.x;
    const double dy = target.y - current_position_.y;
    if (std::hypot(dx, dy) < 0.2) {
      return current_heading_rad_;
    }

    return std::atan2(dy, dx);
  }

  [[nodiscard]] bool navigationAllowed() const {
    if (min_navigation_altitude_m_ <= 0.0) {
      return true;
    }
    return navigation_started_;
  }

  [[nodiscard]] bool isArmed() const {
    return vehicle_status_valid_ &&
           vehicle_status_.arming_state ==
               px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  [[nodiscard]] bool isOffboard() const {
    return vehicle_status_valid_ &&
           vehicle_status_.nav_state ==
               px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  [[nodiscard]] std::uint64_t nowMicros() const {
    return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void logControlSummary() {
    const Point2 target = commanded_target_valid_
                              ? commanded_target_
                              : limitedTarget(currentTarget());
    const double target_distance =
        local_position_valid_ ? distance(current_position_, target)
                              : std::numeric_limits<double>::quiet_NaN();
    const double mission_goal_distance =
        local_position_valid_ ? distance(current_position_, mission_goal_)
                              : std::numeric_limits<double>::quiet_NaN();
    const double path_goal_distance =
        local_position_valid_ && path_valid_
            ? distance(current_position_,
                       Point2{pathPointX(path_, path_.poses.size() - 1U),
                              pathPointY(path_, path_.poses.size() - 1U)})
            : std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Offboard summary: local_position=%s altitude=%.2f nav_allowed=%s "
        "status=%s armed=%s offboard=%s path=%s hold=%s waypoint=%zu/%zu "
        "current=(%.2f, %.2f) target=(%.2f, %.2f) "
        "distance_to_target=%.2f distance_to_path_goal=%.2f "
        "distance_to_mission_goal=%.2f",
        local_position_valid_ ? "true" : "false", current_altitude_m_,
        navigationAllowed() ? "true" : "false",
        vehicle_status_valid_ ? "true" : "false", isArmed() ? "true" : "false",
        isOffboard() ? "true" : "false", path_valid_ ? "true" : "false",
        no_path_hold_target_valid_ ? "true" : "false",
        path_valid_ ? waypoint_index_ + 1U : 0U, path_.poses.size(),
        current_position_.x, current_position_.y, target.x, target.y,
        target_distance, path_goal_distance, mission_goal_distance);
  }

  nav_msgs::msg::Path path_;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  Point2 current_position_{};
  Point2 no_path_hold_target_{};
  Point2 takeoff_hold_target_{};
  Point2 commanded_target_{};
  Point2 mission_goal_{85.0, 0.0};
  double current_heading_rad_{0.0};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double cruise_altitude_m_{12.0};
  double min_navigation_altitude_m_{0.0};
  double acceptance_radius_m_{1.5};
  double max_setpoint_distance_m_{2.0};
  double max_commanded_target_step_m_{0.25};
  double lookahead_distance_m_{6.0};
  double path_switch_hysteresis_m_{3.0};
  double path_continuity_reuse_radius_m_{6.0};
  double path_continuity_max_target_distance_m_{20.0};
  double command_resend_period_s_{2.0};
  double hold_x_m_{0.0};
  double hold_y_m_{0.0};
  std::size_t waypoint_index_{0U};
  int warmup_setpoints_{20};
  int setpoint_counter_{0};
  bool path_valid_{false};
  bool local_position_valid_{false};
  bool vehicle_status_valid_{false};
  bool altitude_valid_{false};
  bool local_position_seen_{false};
  bool auto_arm_{true};
  bool auto_offboard_{true};
  bool emergency_stop_requested_{false};
  bool no_path_hold_target_valid_{false};
  bool takeoff_hold_target_valid_{false};
  bool commanded_target_valid_{false};
  bool face_target_yaw_{false};
  bool navigation_started_{false};
  std::uint8_t target_system_{1U};
  std::uint8_t target_component_{1U};
  std::uint8_t source_system_{1U};
  std::uint16_t source_component_{1U};
  int last_logged_arming_state_{-1};
  int last_logged_nav_state_{-1};
  std::size_t last_logged_path_size_{std::numeric_limits<std::size_t>::max()};
  Point2 last_logged_path_first_{};
  Point2 last_logged_path_last_{};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
      vehicle_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
      vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::Px4OffboardNode>());
  rclcpp::shutdown();
  return 0;
}
