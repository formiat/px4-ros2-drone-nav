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
    acceptance_radius_m_ =
        declare_parameter<double>("acceptance_radius_m", 1.5);
    max_setpoint_distance_m_ =
        std::clamp(declare_parameter<double>("max_setpoint_distance_m", 2.0),
                   0.5, 50.0);
    lookahead_distance_m_ =
        std::clamp(declare_parameter<double>("lookahead_distance_m", 6.0),
                   0.0, 50.0);
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
        "auto_offboard=%s max_setpoint_distance=%.1fm lookahead=%.1fm "
        "mission_goal=(%.1f, %.1f)",
        cruise_altitude_m_, acceptance_radius_m_, auto_arm_ ? "true" : "false",
        auto_offboard_ ? "true" : "false", max_setpoint_distance_m_,
        lookahead_distance_m_, mission_goal_.x, mission_goal_.y);
    RCLCPP_INFO(get_logger(),
                "PX4 offboard subscriptions: path='%s' local_position='%s' "
                "vehicle_status='%s' emergency_stop='%s'",
                path_topic.c_str(), local_position_topic.c_str(),
                vehicle_status_topic.c_str(), emergency_stop_topic.c_str());
  }

private:
  void onPath(const nav_msgs::msg::Path &path) {
    path_ = path;
    path_valid_ = !path_.poses.empty();

    if (!path_valid_) {
      if (last_logged_path_size_ != 0U) {
        RCLCPP_WARN(get_logger(), "Received empty path; holding current target");
        last_logged_path_size_ = 0U;
      }
      return;
    }

    waypoint_index_ = lookaheadWaypointIndex();
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
    if (std::isfinite(msg.heading)) {
      current_heading_rad_ = static_cast<double>(msg.heading);
    }
    local_position_valid_ = true;

    if (!local_position_seen_) {
      local_position_seen_ = true;
      RCLCPP_INFO(get_logger(),
                  "First valid PX4 local position: x=%.2f y=%.2f heading=%.2f",
                  current_position_.x, current_position_.y,
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

    const Point2 target = limitedTarget(currentTarget());
    const float nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = nowMicros();
    msg.position = std::array<float, 3>{
        static_cast<float>(target.x), static_cast<float>(target.y),
        static_cast<float>(-std::abs(cruise_altitude_m_))};
    msg.velocity = std::array<float, 3>{nan, nan, nan};
    msg.acceleration = std::array<float, 3>{nan, nan, nan};
    msg.jerk = std::array<float, 3>{nan, nan, nan};
    msg.yaw = static_cast<float>(targetYaw(target));
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
    if (path_valid_ && waypoint_index_ < path_.poses.size()) {
      return Point2{pathPointX(path_, waypoint_index_),
                    pathPointY(path_, waypoint_index_)};
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
    const Point2 target = limitedTarget(currentTarget());
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
        "Offboard summary: local_position=%s status=%s armed=%s offboard=%s "
        "path=%s waypoint=%zu/%zu current=(%.2f, %.2f) target=(%.2f, %.2f) "
        "distance_to_target=%.2f distance_to_path_goal=%.2f "
        "distance_to_mission_goal=%.2f",
        local_position_valid_ ? "true" : "false",
        vehicle_status_valid_ ? "true" : "false", isArmed() ? "true" : "false",
        isOffboard() ? "true" : "false", path_valid_ ? "true" : "false",
        path_valid_ ? waypoint_index_ + 1U : 0U, path_.poses.size(),
        current_position_.x, current_position_.y, target.x, target.y,
        target_distance, path_goal_distance, mission_goal_distance);
  }

  nav_msgs::msg::Path path_;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  Point2 current_position_{};
  Point2 mission_goal_{85.0, 0.0};
  double current_heading_rad_{0.0};
  double cruise_altitude_m_{12.0};
  double acceptance_radius_m_{1.5};
  double max_setpoint_distance_m_{2.0};
  double lookahead_distance_m_{6.0};
  double command_resend_period_s_{2.0};
  double hold_x_m_{0.0};
  double hold_y_m_{0.0};
  std::size_t waypoint_index_{0U};
  int warmup_setpoints_{20};
  int setpoint_counter_{0};
  bool path_valid_{false};
  bool local_position_valid_{false};
  bool vehicle_status_valid_{false};
  bool local_position_seen_{false};
  bool auto_arm_{true};
  bool auto_offboard_{true};
  bool emergency_stop_requested_{false};
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
