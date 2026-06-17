#include "drone_city_nav/offboard_path_follower.hpp"
#include "drone_city_nav/offboard_speed_controller.hpp"
#include "drone_city_nav/offboard_target_safety.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/types.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
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
#include <numbers>
#include <optional>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr auto kControllerPeriod = std::chrono::milliseconds{100};
constexpr double kControllerPeriodS = 0.1;
constexpr double kTinyDistanceM = 1.0e-6;
constexpr std::int8_t kInflatedOccupancyValue = 80;
constexpr std::int8_t kOccupiedOccupancyValue = 100;

[[nodiscard]] std::uint8_t boundedUint8(const std::int64_t value) {
  return static_cast<std::uint8_t>(std::clamp<std::int64_t>(value, 0, 255));
}

[[nodiscard]] std::uint16_t boundedUint16(const std::int64_t value) {
  return static_cast<std::uint16_t>(std::clamp<std::int64_t>(value, 0, 65535));
}

[[nodiscard]] const char* commandName(const std::uint32_t command) noexcept {
  switch (command) {
    case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE:
      return "VEHICLE_CMD_DO_SET_MODE";
    case px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM:
      return "VEHICLE_CMD_COMPONENT_ARM_DISARM";
    default:
      return "UNKNOWN";
  }
}

[[nodiscard]] double boundedFiniteDouble(const double value, const double fallback,
                                         const double min_value,
                                         const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

} // namespace

class Px4OffboardNode final : public rclcpp::Node {
public:
  Px4OffboardNode()
      : Node{"px4_offboard_node"} {
    cruise_altitude_m_ = declare_parameter<double>("cruise_altitude_m", 12.0);
    min_navigation_altitude_m_ =
        std::clamp(declare_parameter<double>("min_navigation_altitude_m", 0.0), 0.0,
                   std::abs(cruise_altitude_m_));
    takeoff_hover_s_ =
        std::clamp(declare_parameter<double>("takeoff_hover_s", 2.0), 0.0, 30.0);
    face_target_yaw_ = declare_parameter<bool>("face_target_yaw", false);
    acceptance_radius_m_ = declare_parameter<double>("acceptance_radius_m", 1.5);
    max_setpoint_distance_m_ = std::clamp(
        declare_parameter<double>("max_setpoint_distance_m", 2.0), 0.5, 50.0);
    max_commanded_target_step_m_ = std::clamp(
        declare_parameter<double>("max_commanded_target_step_m", 0.25), 0.01, 10.0);
    min_commanded_target_lead_m_ =
        std::clamp(declare_parameter<double>("min_commanded_target_lead_m", 0.0), 0.0,
                   max_setpoint_distance_m_);
    SpeedControllerConfig speed_config{};
    speed_config.max_commanded_target_step_m = max_commanded_target_step_m_;
    speed_config.desired_speed_mps = std::clamp(
        declare_parameter<double>("desired_speed_mps",
                                  max_commanded_target_step_m_ / kControllerPeriodS),
        0.0, 50.0);
    speed_config.max_accel_mps2 =
        std::clamp(declare_parameter<double>("max_accel_mps2", 2.0), 0.1, 50.0);
    speed_config.min_command_speed_mps =
        std::clamp(declare_parameter<double>("min_command_speed_mps", 0.0), 0.0, 50.0);
    speed_config.goal_slowdown_radius_m = std::clamp(
        declare_parameter<double>("goal_slowdown_radius_m", 10.0), 0.0, 200.0);
    speed_config.braking_safety_margin_m = std::clamp(
        declare_parameter<double>("braking_safety_margin_m", acceptance_radius_m_), 0.0,
        50.0);
    speed_config.turn_slowdown_angle_rad =
        std::clamp(declare_parameter<double>("turn_slowdown_angle_rad", 0.7), 0.0,
                   std::numbers::pi);
    speed_config.turn_slowdown_min_speed_mps = std::clamp(
        declare_parameter<double>("turn_slowdown_min_speed_mps", 1.5), 0.0, 50.0);
    speed_config.narrow_clearance_slowdown_radius_m =
        std::clamp(declare_parameter<double>("narrow_clearance_slowdown_radius_m", 7.0),
                   0.0, 100.0);
    speed_config.narrow_clearance_min_speed_mps = std::clamp(
        declare_parameter<double>("narrow_clearance_min_speed_mps", 1.0), 0.0, 50.0);
    speed_config.clearance_braking_margin_m = std::clamp(
        declare_parameter<double>("clearance_braking_margin_m", 1.0), 0.0, 100.0);
    speed_controller_.setConfig(speed_config);
    velocity_feedforward_enabled_ =
        declare_parameter<bool>("velocity_feedforward_enabled", false);
    max_clearance_grid_staleness_ns_ = static_cast<std::int64_t>(
        std::clamp<double>(
            declare_parameter<double>("max_clearance_grid_staleness_s", 1.5), 0.0,
            3600.0) *
        1.0e9);
    max_pose_staleness_ns_ = static_cast<std::int64_t>(
        boundedFiniteDouble(declare_parameter<double>("max_pose_staleness_s", 1.0), 1.0,
                            0.0, 3600.0) *
        1.0e9);
    lookahead_distance_m_ =
        std::clamp(declare_parameter<double>("lookahead_distance_m", 6.0), 0.0, 50.0);
    dynamic_lookahead_enabled_ =
        declare_parameter<bool>("dynamic_lookahead_enabled", true);
    lookahead_time_s_ =
        std::clamp(declare_parameter<double>("lookahead_time_s", 1.2), 0.0, 10.0);
    min_lookahead_distance_m_ = std::clamp(
        declare_parameter<double>("min_lookahead_distance_m", lookahead_distance_m_),
        0.0, 50.0);
    max_lookahead_distance_m_ = std::clamp(
        declare_parameter<double>("max_lookahead_distance_m", lookahead_distance_m_),
        0.0, 100.0);
    if (max_lookahead_distance_m_ < min_lookahead_distance_m_) {
      max_lookahead_distance_m_ = min_lookahead_distance_m_;
    }
    path_switch_hysteresis_m_ = std::clamp(
        declare_parameter<double>("path_switch_hysteresis_m", 3.0), 0.0, 100.0);
    path_continuity_reuse_radius_m_ = std::clamp(
        declare_parameter<double>("path_continuity_reuse_radius_m", 6.0), 0.0, 100.0);
    path_continuity_max_target_distance_m_ = std::clamp(
        declare_parameter<double>("path_continuity_max_target_distance_m", 20.0), 0.0,
        500.0);
    target_segment_safety_check_enabled_ =
        declare_parameter<bool>("target_segment_safety_check_enabled", true);
    clearance_escape_enabled_ =
        declare_parameter<bool>("clearance_escape_enabled", true);
    clearance_escape_step_m_ =
        std::clamp(declare_parameter<double>("clearance_escape_step_m", 0.5), 0.0,
                   max_setpoint_distance_m_);
    clearance_escape_min_improvement_m_ = std::clamp(
        declare_parameter<double>("clearance_escape_min_improvement_m", 0.05), 0.0,
        10.0);
    telemetry_log_period_ns_ = static_cast<std::int64_t>(
        std::clamp(declare_parameter<double>("telemetry_log_period_s", 0.5), 0.1,
                   60.0) *
        1.0e9);
    warmup_setpoints_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("warmup_setpoints", 20), 1, 100000));
    auto_arm_ = declare_parameter<bool>("auto_arm", true);
    auto_offboard_ = declare_parameter<bool>("auto_offboard", true);
    command_resend_period_s_ = boundedFiniteDouble(
        declare_parameter<double>("command_resend_period_s", 2.0), 2.0, 0.05, 60.0);
    px4_local_origin_ = Point2{declare_parameter<double>("px4_local_origin_x_m", 0.0),
                               declare_parameter<double>("px4_local_origin_y_m", 0.0)};
    mission_goal_ = Point2{declare_parameter<double>("goal_x_m", 85.0),
                           declare_parameter<double>("goal_y_m", 0.0)};
    hold_x_m_ = declare_parameter<double>("hold_x_m", 0.0);
    hold_y_m_ = declare_parameter<double>("hold_y_m", 0.0);
    target_system_ = boundedUint8(declare_parameter<std::int64_t>("target_system", 1));
    target_component_ =
        boundedUint8(declare_parameter<std::int64_t>("target_component", 1));
    source_system_ = boundedUint8(declare_parameter<std::int64_t>("source_system", 1));
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
    const std::string occupancy_grid_topic = declare_parameter<std::string>(
        "occupancy_grid_topic", "/drone_city_nav/occupancy_grid");

    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic, rclcpp::QoS{1}.reliable(),
        [this](const nav_msgs::msg::Path::SharedPtr msg) { onPath(*msg); });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
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
        [this](const std_msgs::msg::Bool::SharedPtr msg) { onEmergencyStop(*msg); });
    occupancy_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        occupancy_grid_topic, rclcpp::QoS{1}.transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          onOccupancyGrid(*msg);
        });

    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        declare_parameter<std::string>("offboard_control_mode_topic",
                                       "/fmu/in/offboard_control_mode"),
        px4_qos);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        declare_parameter<std::string>("trajectory_setpoint_topic",
                                       "/fmu/in/trajectory_setpoint"),
        px4_qos);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        declare_parameter<std::string>("vehicle_command_topic",
                                       "/fmu/in/vehicle_command"),
        px4_qos);

    timer_ = create_wall_timer(kControllerPeriod, [this]() { onTimer(); });
    last_command_time_ =
        now() - rclcpp::Duration::from_seconds(command_resend_period_s_);

    RCLCPP_INFO(
        get_logger(),
        "PX4 offboard node ready: altitude=%.1fm acceptance=%.1fm auto_arm=%s "
        "auto_offboard=%s min_navigation_altitude=%.1fm face_target_yaw=%s "
        "takeoff_hover=%.1fs "
        "max_setpoint_distance=%.1fm commanded_target_step=%.2fm "
        "min_commanded_target_lead=%.1fm "
        "desired_speed=%.2fmps max_accel=%.2fmps2 lookahead=%.1fm "
        "dynamic_lookahead=%s lookahead_time=%.2fs "
        "lookahead_range=[%.1f, %.1f] "
        "goal_slowdown_radius=%.1fm turn_slowdown_angle=%.2frad "
        "narrow_clearance_radius=%.1fm clearance_braking_margin=%.1fm "
        "velocity_feedforward=%s "
        "path_switch_hysteresis=%.1fm path_continuity_reuse_radius=%.1fm "
        "path_continuity_max_target_distance=%.1fm target_safety=%s "
        "clearance_escape=%s clearance_escape_step=%.2fm "
        "clearance_escape_min_improvement=%.2fm mission_goal=(%.1f, %.1f) "
        "px4_local_origin=(%.1f, %.1f) telemetry_log_period=%.2fs "
        "max_pose_staleness=%.2fs command_resend_period=%.2fs",
        cruise_altitude_m_, acceptance_radius_m_, auto_arm_ ? "true" : "false",
        auto_offboard_ ? "true" : "false", min_navigation_altitude_m_,
        face_target_yaw_ ? "true" : "false", takeoff_hover_s_, max_setpoint_distance_m_,
        max_commanded_target_step_m_, min_commanded_target_lead_m_,
        speed_controller_.config().desired_speed_mps,
        speed_controller_.config().max_accel_mps2, lookahead_distance_m_,
        dynamic_lookahead_enabled_ ? "true" : "false", lookahead_time_s_,
        min_lookahead_distance_m_, max_lookahead_distance_m_,
        speed_controller_.config().goal_slowdown_radius_m,
        speed_controller_.config().turn_slowdown_angle_rad,
        speed_controller_.config().narrow_clearance_slowdown_radius_m,
        speed_controller_.config().clearance_braking_margin_m,
        velocity_feedforward_enabled_ ? "true" : "false", path_switch_hysteresis_m_,
        path_continuity_reuse_radius_m_, path_continuity_max_target_distance_m_,
        target_segment_safety_check_enabled_ ? "true" : "false",
        clearance_escape_enabled_ ? "true" : "false", clearance_escape_step_m_,
        clearance_escape_min_improvement_m_, mission_goal_.x, mission_goal_.y,
        px4_local_origin_.x, px4_local_origin_.y,
        static_cast<double>(telemetry_log_period_ns_) / 1.0e9,
        static_cast<double>(max_pose_staleness_ns_) / 1.0e9, command_resend_period_s_);
    RCLCPP_INFO(get_logger(),
                "PX4 offboard subscriptions: path='%s' local_position='%s' "
                "vehicle_status='%s' emergency_stop='%s' occupancy_grid='%s'",
                path_topic.c_str(), local_position_topic.c_str(),
                vehicle_status_topic.c_str(), emergency_stop_topic.c_str(),
                occupancy_grid_topic.c_str());
  }

private:
  [[nodiscard]] std::vector<Point2>
  pathPointsFromMessage(const nav_msgs::msg::Path& path) const {
    std::vector<Point2> points;
    points.reserve(path.poses.size());
    for (const auto& pose : path.poses) {
      points.push_back(Point2{pose.pose.position.x, pose.pose.position.y});
    }
    return points;
  }

  [[nodiscard]] OffboardPathFollowerConfig pathFollowerConfig() const {
    return OffboardPathFollowerConfig{acceptance_radius_m_,
                                      lookahead_distance_m_,
                                      lookahead_time_s_,
                                      min_lookahead_distance_m_,
                                      max_lookahead_distance_m_,
                                      path_switch_hysteresis_m_,
                                      path_continuity_reuse_radius_m_,
                                      path_continuity_max_target_distance_m_,
                                      max_setpoint_distance_m_,
                                      dynamic_lookahead_enabled_};
  }

  void onPath(const nav_msgs::msg::Path& path) {
    const bool had_active_target =
        path_valid_ && localPositionFresh() && waypoint_index_ < path_points_.size();
    const Point2 previous_target =
        had_active_target ? path_points_[waypoint_index_] : Point2{};

    path_points_ = pathPointsFromMessage(path);
    path_valid_ = !path_points_.empty();

    if (!path_valid_) {
      if (last_logged_path_size_ != 0U) {
        if (local_position_valid_) {
          no_path_hold_target_ = current_position_;
          no_path_hold_target_valid_ = true;
          commanded_target_ = no_path_hold_target_;
          commanded_target_valid_ = true;
          waypoint_index_ = 0U;
          RCLCPP_WARN(get_logger(),
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
        continuityWaypointIndex(previous_target, candidate_index, had_active_target);
    const Point2 first = path_points_.front();
    const Point2 last = path_points_.back();
    const bool path_changed = path_points_.size() != last_logged_path_size_ ||
                              squaredDistance(first, last_logged_path_first_) > 0.01 ||
                              squaredDistance(last, last_logged_path_last_) > 0.01;
    if (path_changed) {
      resetCommandedTargetToCurrentPath("path_changed");
      const PathMetrics metrics = pointPathMetrics(path_points_);
      RCLCPP_INFO(get_logger(),
                  "Received path: waypoints=%zu segments=%zu straight_segments=%zu "
                  "turns=%zu length=%.2f selected=%zu first=(%.2f, %.2f) "
                  "last=(%.2f, %.2f)",
                  path_points_.size(), metrics.segments, metrics.straight_segments,
                  metrics.turns, metrics.length_m, waypoint_index_ + 1U, first.x,
                  first.y, last.x, last.y);
      last_logged_path_size_ = path_points_.size();
      last_logged_path_first_ = first;
      last_logged_path_last_ = last;
    }
  }

  [[nodiscard]] std::size_t closestWaypointIndex() const {
    if (!localPositionFresh() || path_points_.empty()) {
      return 0U;
    }
    return drone_city_nav::closestWaypointIndex(path_points_, current_position_);
  }

  [[nodiscard]] std::size_t lookaheadWaypointIndex() const {
    if (!localPositionFresh() || path_points_.empty()) {
      return 0U;
    }
    return drone_city_nav::lookaheadWaypointIndex(
        path_points_, current_position_, mission_goal_, pathFollowerConfig(),
        speed_controller_.config().desired_speed_mps);
  }

  [[nodiscard]] double effectiveLookaheadDistanceM() const {
    return drone_city_nav::effectiveLookaheadDistanceM(
        pathFollowerConfig(), speed_controller_.config().desired_speed_mps);
  }

  [[nodiscard]] std::optional<OffboardPathProjection> closestPathProjection() const {
    if (!localPositionFresh() || path_points_.empty()) {
      return std::nullopt;
    }
    return drone_city_nav::closestOffboardPathProjection(path_points_,
                                                         current_position_);
  }

  [[nodiscard]] Point2 lookaheadTargetOnPath() const {
    return drone_city_nav::lookaheadTargetOnPath(
        path_points_, current_position_, waypoint_index_, pathFollowerConfig(),
        speed_controller_.config().desired_speed_mps);
  }

  [[nodiscard]] std::size_t
  continuityWaypointIndex(const Point2 previous_target,
                          const std::size_t candidate_index,
                          const bool had_active_target) const {
    if (!had_active_target || path_switch_hysteresis_m_ <= 0.0 ||
        path_continuity_reuse_radius_m_ <= 0.0 ||
        candidate_index >= path_points_.size()) {
      return candidate_index;
    }
    if (distance(current_position_, previous_target) >
        path_continuity_max_target_distance_m_) {
      return candidate_index;
    }

    const std::size_t closest_index = drone_city_nav::continuityWaypointIndex(
        path_points_, current_position_, previous_target, candidate_index,
        had_active_target, pathFollowerConfig());
    const double closest_distance =
        closest_index < path_points_.size()
            ? distance(previous_target, path_points_[closest_index])
            : std::numeric_limits<double>::infinity();
    if (closest_index != candidate_index) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Path continuity kept waypoint near previous target: candidate=%zu "
          "kept=%zu previous_target=(%.2f, %.2f) closest_distance=%.2f",
          candidate_index + 1U, closest_index + 1U, previous_target.x,
          previous_target.y, closest_distance);
      return closest_index;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "Path continuity allowed target switch: candidate=%zu "
                         "previous_target=(%.2f, %.2f) closest_distance=%.2f",
                         candidate_index + 1U, previous_target.x, previous_target.y,
                         closest_distance);
    return candidate_index;
  }

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
    if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
      if (local_position_valid_) {
        last_local_position_update_ns_ = 0;
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring invalid PX4 local position and holding the last known target: "
          "xy_valid=%s x=%.2f y=%.2f had_previous_position=%s",
          msg.xy_valid ? "true" : "false", static_cast<double>(msg.x),
          static_cast<double>(msg.y), local_position_valid_ ? "true" : "false");
      return;
    }

    last_local_position_update_ns_ = get_clock()->now().nanoseconds();
    current_position_ = Point2{static_cast<double>(msg.x) + px4_local_origin_.x,
                               static_cast<double>(msg.y) + px4_local_origin_.y};
    if (std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
      current_velocity_ =
          Point2{static_cast<double>(msg.vx), static_cast<double>(msg.vy)};
      current_speed_mps_ = std::hypot(current_velocity_.x, current_velocity_.y);
      current_velocity_valid_ = true;
    } else {
      current_velocity_ = Point2{};
      current_speed_mps_ = std::numeric_limits<double>::quiet_NaN();
      current_velocity_valid_ = false;
    }
    if (msg.z_valid && std::isfinite(msg.z)) {
      current_altitude_m_ = -static_cast<double>(msg.z);
      altitude_valid_ = true;
      updateNavigationStartState();
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

  void onVehicleStatus(const px4_msgs::msg::VehicleStatus& msg) {
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

  void onEmergencyStop(const std_msgs::msg::Bool& msg) {
    if (!msg.data || emergency_stop_requested_) {
      return;
    }

    emergency_stop_requested_ = true;
    path_valid_ = false;
    RCLCPP_ERROR(get_logger(),
                 "Emergency stop requested; stopping trajectory setpoints and "
                 "sending disarm commands");
  }

  void onOccupancyGrid(const nav_msgs::msg::OccupancyGrid& msg) {
    if (!(msg.info.resolution > 0.0F) || msg.info.width == 0U ||
        msg.info.height == 0U ||
        msg.info.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        msg.info.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring invalid offboard occupancy grid metadata");
      return;
    }

    const std::size_t expected_size = static_cast<std::size_t>(msg.info.width) *
                                      static_cast<std::size_t>(msg.info.height);
    if (msg.data.size() != expected_size) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring offboard occupancy grid with mismatched data size: expected=%zu "
          "got=%zu",
          expected_size, msg.data.size());
      return;
    }

    occupancy_grid_ = msg;
    occupancy_grid_valid_ = true;
    last_occupancy_grid_update_ns_ = get_clock()->now().nanoseconds();
    if (!occupancy_grid_seen_logged_) {
      occupancy_grid_seen_logged_ = true;
      RCLCPP_INFO(get_logger(),
                  "First offboard occupancy grid: size=%ux%u resolution=%.2f "
                  "origin=(%.2f, %.2f)",
                  msg.info.width, msg.info.height,
                  static_cast<double>(msg.info.resolution), msg.info.origin.position.x,
                  msg.info.origin.position.y);
    }
  }

  void onTimer() {
    if (emergency_stop_requested_) {
      handleEmergencyStop();
      return;
    }

    publishOffboardControlMode();
    publishTrajectorySetpoint();
    logTelemetry();
    logControlSummary();

    if (setpoint_counter_ < warmup_setpoints_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Offboard warmup: sent %d/%d setpoints", setpoint_counter_,
                           warmup_setpoints_);
      ++setpoint_counter_;
      return;
    }

    const rclcpp::Time current_time = now();
    if ((current_time - last_command_time_).seconds() < command_resend_period_s_) {
      return;
    }

    if (auto_offboard_ && !isOffboard()) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                            1.0F, 6.0F);
      last_command_time_ = current_time;
      return;
    }

    if (auto_arm_ && !isArmed()) {
      publishVehicleCommand(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
      last_command_time_ = current_time;
    }
  }

  void handleEmergencyStop() {
    const rclcpp::Time current_time = now();
    if ((current_time - last_command_time_).seconds() < command_resend_period_s_) {
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
    msg.velocity = velocity_feedforward_enabled_;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_control_mode_pub_->publish(msg);
  }

  void publishTrajectorySetpoint() {
    advanceWaypointIfNeeded();

    const Point2 desired_target = limitedTarget(currentTarget());
    const SpeedControllerInput speed_input = makeSpeedControllerInput();
    last_speed_output_ = speed_controller_.update(speed_input);
    const Point2 target = selectSafeCommandTarget(desired_target, speed_input);
    commanded_target_ = target;
    commanded_target_valid_ = local_position_valid_;
    last_published_target_ = target;
    last_published_target_valid_ = true;
    const float nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = nowMicros();
    const Point2 px4_local_target = mapToPx4Local(target);
    msg.position = std::array<float, 3>{
        static_cast<float>(px4_local_target.x), static_cast<float>(px4_local_target.y),
        static_cast<float>(-std::abs(cruise_altitude_m_))};
    msg.velocity = velocityFeedforward(target, last_speed_output_);
    msg.acceleration = std::array<float, 3>{nan, nan, nan};
    msg.jerk = std::array<float, 3>{nan, nan, nan};
    msg.yaw =
        static_cast<float>(face_target_yaw_ ? targetYaw(target) : current_heading_rad_);
    msg.yawspeed = nan;

    trajectory_setpoint_pub_->publish(msg);
  }

  void publishVehicleCommand(const std::uint32_t command, const float param1 = 0.0F,
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
    if (!path_valid_ || !localPositionFresh()) {
      return;
    }

    const std::size_t previous_waypoint_index = waypoint_index_;
    waypoint_index_ = drone_city_nav::advanceWaypointIndex(
        path_points_, current_position_, waypoint_index_, pathFollowerConfig());

    if (waypoint_index_ != previous_waypoint_index) {
      const Point2 target = path_points_[waypoint_index_];
      RCLCPP_INFO(get_logger(),
                  "Waypoint advanced: index=%zu/%zu current=(%.2f, %.2f) "
                  "target=(%.2f, %.2f)",
                  waypoint_index_ + 1U, path_points_.size(), current_position_.x,
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

    if (localPositionFresh() && path_valid_ && waypoint_index_ < path_points_.size()) {
      return lookaheadTargetOnPath();
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
    return drone_city_nav::limitedTarget(
        target, current_position_, local_position_valid_, max_setpoint_distance_m_);
  }

  [[nodiscard]] double occupancyGridResolutionM() const noexcept {
    return static_cast<double>(occupancy_grid_.info.resolution);
  }

  [[nodiscard]] std::optional<GridIndex> occupancyGridCell(const Point2 point) const {
    if (!occupancyGridFresh()) {
      return std::nullopt;
    }

    const double resolution = occupancyGridResolutionM();
    const double origin_x = occupancy_grid_.info.origin.position.x;
    const double origin_y = occupancy_grid_.info.origin.position.y;
    const auto width = static_cast<int>(occupancy_grid_.info.width);
    const auto height = static_cast<int>(occupancy_grid_.info.height);
    const GridIndex cell{
        static_cast<int>(std::floor((point.x - origin_x) / resolution)),
        static_cast<int>(std::floor((point.y - origin_y) / resolution))};
    if (cell.x < 0 || cell.y < 0 || cell.x >= width || cell.y >= height) {
      return std::nullopt;
    }
    return cell;
  }

  [[nodiscard]] std::int8_t occupancyGridValue(const GridIndex cell) const {
    const auto width = static_cast<int>(occupancy_grid_.info.width);
    const std::size_t data_index =
        static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(cell.x);
    return occupancy_grid_.data.at(data_index);
  }

  [[nodiscard]] bool occupancyGridCellBlocked(const GridIndex cell) const {
    return occupancyGridValue(cell) >= kInflatedOccupancyValue;
  }

  [[nodiscard]] bool occupancyGridCellOccupied(const GridIndex cell) const {
    return occupancyGridValue(cell) >= kOccupiedOccupancyValue;
  }

  [[nodiscard]] std::vector<GridIndex>
  occupancyGridCellsOnLine(const GridIndex start, const GridIndex end) const {
    std::vector<GridIndex> cells;

    int x0 = start.x;
    int y0 = start.y;
    const int x1 = end.x;
    const int y1 = end.y;
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;

    const auto width = static_cast<int>(occupancy_grid_.info.width);
    const auto height = static_cast<int>(occupancy_grid_.info.height);
    while (true) {
      if (x0 >= 0 && y0 >= 0 && x0 < width && y0 < height) {
        cells.push_back(GridIndex{x0, y0});
      }
      if (x0 == x1 && y0 == y1) {
        break;
      }

      const int doubled_error = 2 * error;
      if (doubled_error >= dy) {
        error += dy;
        x0 += sx;
      }
      if (doubled_error <= dx) {
        error += dx;
        y0 += sy;
      }
    }

    return cells;
  }

  [[nodiscard]] TargetSegmentSafety
  evaluateTargetSegmentSafety(const Point2 start, const Point2 end,
                              const bool allow_escape) const {
    TargetSegmentSafetyInput input{};
    input.safety_check_enabled = target_segment_safety_check_enabled_;
    input.grid_available = occupancyGridFresh();
    input.allow_escape = allow_escape;
    input.clearance_stop_requested = allow_escape;
    input.min_clearance_improvement_m = clearance_escape_min_improvement_m_;
    const auto start_cell = occupancyGridCell(start);
    const auto end_cell = occupancyGridCell(end);
    input.start_cell_valid = start_cell.has_value();
    input.end_cell_valid = end_cell.has_value();

    if (!input.safety_check_enabled || !input.grid_available ||
        !input.start_cell_valid || !input.end_cell_valid) {
      return evaluateTargetSegmentSafetyPolicy(input);
    }

    input.start_blocked = occupancyGridCellBlocked(*start_cell);
    input.start_occupied = occupancyGridCellOccupied(*start_cell);
    input.end_blocked = occupancyGridCellBlocked(*end_cell);

    const std::vector<GridIndex> segment_cells =
        occupancyGridCellsOnLine(*start_cell, *end_cell);
    for (const GridIndex cell : segment_cells) {
      if (occupancyGridCellOccupied(cell)) {
        ++input.occupied_cells;
      }
      if (occupancyGridCellBlocked(cell)) {
        ++input.blocked_cells;
      }
    }

    input.start_clearance_m = estimateLocalClearanceM(start);
    input.end_clearance_m = estimateLocalClearanceM(end);
    return evaluateTargetSegmentSafetyPolicy(input);
  }

  [[nodiscard]] bool clearanceEscapeRequested(const SpeedControllerInput& input) const {
    if (!clearance_escape_enabled_ || input.hold_position ||
        !(clearance_escape_step_m_ > 0.0) ||
        last_speed_output_.requested_speed_mps > 0.0) {
      return false;
    }

    const bool clearance_stopped =
        last_speed_output_.limit_reason == SpeedLimitReason::kClearance ||
        (last_speed_output_.limit_reason == SpeedLimitReason::kTrackingOverspeed &&
         last_speed_output_.limits.clearance_limit_mps <= kTinyDistanceM);
    return clearance_stopped;
  }

  [[nodiscard]] Point2 pathTargetAtDistance(const double path_distance_m,
                                            const Point2 fallback_target) const {
    if (!path_valid_ || path_points_.empty() || !localPositionFresh()) {
      return fallback_target;
    }
    return targetOnPathAtDistance(path_points_, current_position_, path_distance_m);
  }

  [[nodiscard]] Point2 selectSafePathTarget(const double requested_lead_m,
                                            const Point2 fallback_target,
                                            const bool allow_escape) {
    const double bounded_lead_m =
        std::clamp(requested_lead_m, 0.0, max_setpoint_distance_m_);
    if (!(bounded_lead_m > 0.0) || !localPositionFresh()) {
      return current_position_;
    }

    const double resolution =
        occupancyGridFresh() ? std::max(occupancyGridResolutionM(), 0.25) : 0.5;
    const double min_step_m = std::min(std::max(resolution, 0.25), bounded_lead_m);
    const int candidate_count = std::max(
        1, static_cast<int>(std::ceil(bounded_lead_m / std::max(min_step_m, 0.1))));

    for (int step = candidate_count; step >= 1; --step) {
      const double distance_m = bounded_lead_m * static_cast<double>(step) /
                                static_cast<double>(candidate_count);
      const Point2 candidate = pathTargetAtDistance(distance_m, fallback_target);
      if (distance(current_position_, candidate) <= kTinyDistanceM) {
        continue;
      }

      const TargetSegmentSafety safety =
          evaluateTargetSegmentSafety(current_position_, candidate, allow_escape);
      if (safety.allowed) {
        if (step != candidate_count || safety.escape) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "Target segment safety adjusted command: requested_lead=%.2f "
              "selected_lead=%.2f escape=%s blocked_cells=%zu occupied_cells=%zu "
              "reason=%s clearance_start=%.2f clearance_end=%.2f "
              "target=(%.2f, %.2f)",
              bounded_lead_m, distance_m, safety.escape ? "true" : "false",
              safety.blocked_cells, safety.occupied_cells,
              targetSegmentSafetyReasonName(safety.reason), safety.start_clearance_m,
              safety.end_clearance_m, candidate.x, candidate.y);
        }
        return candidate;
      }
    }

    const TargetSegmentSafety desired_safety =
        evaluateTargetSegmentSafety(current_position_, fallback_target, allow_escape);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Target segment safety hold: requested_lead=%.2f allow_escape=%s "
        "blocked_cells=%zu occupied_cells=%zu start_blocked=%s end_blocked=%s "
        "reason=%s clearance_start=%.2f clearance_end=%.2f "
        "fallback_target=(%.2f, %.2f)",
        bounded_lead_m, allow_escape ? "true" : "false", desired_safety.blocked_cells,
        desired_safety.occupied_cells, desired_safety.start_blocked ? "true" : "false",
        desired_safety.end_blocked ? "true" : "false",
        targetSegmentSafetyReasonName(desired_safety.reason),
        desired_safety.start_clearance_m, desired_safety.end_clearance_m,
        fallback_target.x, fallback_target.y);
    return current_position_;
  }

  [[nodiscard]] Point2
  selectSafeCommandTarget(const Point2 desired_target,
                          const SpeedControllerInput& speed_input) {
    if (!local_position_valid_) {
      return desired_target;
    }
    if (!localPositionFresh()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Holding last known position because PX4 local position is stale: "
          "age_s=%.2f max_age_s=%.2f target=(%.2f, %.2f)",
          localPositionAgeSeconds(),
          static_cast<double>(max_pose_staleness_ns_) / 1.0e9, current_position_.x,
          current_position_.y);
      return current_position_;
    }
    if (speed_input.hold_position) {
      return current_position_;
    }

    const bool escape_requested = clearanceEscapeRequested(speed_input);
    double requested_lead_m = 0.0;
    if (last_speed_output_.requested_speed_mps > 0.0) {
      requested_lead_m =
          std::max(last_speed_output_.target_step_m,
                   effectiveMinimumTargetLeadM(last_speed_output_.requested_speed_mps));
    } else if (escape_requested) {
      requested_lead_m = clearance_escape_step_m_;
    }

    if (!(requested_lead_m > 0.0)) {
      return current_position_;
    }
    return selectSafePathTarget(requested_lead_m, desired_target, escape_requested);
  }

  Point2 smoothedCommandTarget(const Point2 desired_target, const double target_step_m,
                               const bool snap_to_desired_target) {
    CommandTargetState state{commanded_target_valid_, commanded_target_};
    const Point2 target = drone_city_nav::smoothedCommandTarget(
        desired_target, target_step_m, snap_to_desired_target, current_position_,
        local_position_valid_, max_setpoint_distance_m_, state);
    commanded_target_valid_ = state.valid;
    commanded_target_ = state.target;
    return target;
  }

  void resetCommandedTargetToCurrentPath(const char* reason) {
    if (!local_position_valid_ || !path_valid_ ||
        waypoint_index_ >= path_points_.size()) {
      commanded_target_valid_ = false;
      last_published_target_valid_ = false;
      return;
    }

    commanded_target_ = current_position_;
    commanded_target_valid_ = true;
    last_published_target_ = commanded_target_;
    last_published_target_valid_ = true;
    RCLCPP_INFO(get_logger(),
                "Reset commanded target to current position after path update: "
                "reason=%s target=(%.2f, %.2f) current=(%.2f, %.2f) "
                "waypoint=%zu/%zu",
                reason, commanded_target_.x, commanded_target_.y, current_position_.x,
                current_position_.y, waypoint_index_ + 1U, path_points_.size());
  }

  [[nodiscard]] bool shouldHoldPosition() const {
    return !localPositionFresh() || !navigationAllowed() || !path_valid_ ||
           waypoint_index_ >= path_points_.size();
  }

  [[nodiscard]] SpeedControllerInput makeSpeedControllerInput() const {
    SpeedControllerInput input{};
    input.hold_position = shouldHoldPosition();
    const bool pose_fresh = localPositionFresh();
    input.controller_dt_s = kControllerPeriodS;
    input.distance_to_goal_m = pose_fresh ? distance(current_position_, mission_goal_)
                                          : std::numeric_limits<double>::quiet_NaN();
    input.turn_angle_rad = pathTurnAngleAtWaypoint(waypoint_index_);
    input.local_clearance_m = pose_fresh ? estimateLocalClearanceM(current_position_)
                                         : std::numeric_limits<double>::quiet_NaN();
    input.actual_speed_mps = current_speed_mps_;
    return input;
  }

  [[nodiscard]] double pathTurnAngleAtWaypoint(const std::size_t index) const {
    if (!path_valid_ || !localPositionFresh()) {
      return 0.0;
    }
    return drone_city_nav::pathTurnAngleAtWaypoint(
        path_points_, index, current_position_, true, pathFollowerConfig(),
        speed_controller_.config().desired_speed_mps);
  }

  [[nodiscard]] const char* pathSegmentTypeName(const double turn_angle_rad) const {
    if (!path_valid_) {
      return "no_path";
    }
    if (turn_angle_rad < 0.15) {
      return "straight";
    }
    if (turn_angle_rad < speed_controller_.config().turn_slowdown_angle_rad) {
      return "gentle_turn";
    }
    if (turn_angle_rad < 1.2) {
      return "turn";
    }
    return "sharp_turn";
  }

  [[nodiscard]] const char* motionPhaseName(const SpeedLimitReason reason,
                                            const bool hold_position) const noexcept {
    if (hold_position) {
      return no_path_hold_target_valid_ ? "hold_no_path" : "hold";
    }

    switch (reason) {
      case SpeedLimitReason::kHold:
        return "hold";
      case SpeedLimitReason::kInvalidInput:
        return "invalid_input";
      case SpeedLimitReason::kCruise:
        return "cruise";
      case SpeedLimitReason::kAcceleration:
        return "accelerating";
      case SpeedLimitReason::kGoal:
        return "goal_braking";
      case SpeedLimitReason::kTurn:
        return "turn_slowdown";
      case SpeedLimitReason::kClearance:
        return "clearance_slowdown";
      case SpeedLimitReason::kHardStepCap:
        return "step_limited";
      case SpeedLimitReason::kTrackingOverspeed:
        return "tracking_recovery";
    }
    return "unknown";
  }

  [[nodiscard]] bool occupancyGridFresh() const {
    if (!occupancy_grid_valid_ || last_occupancy_grid_update_ns_ <= 0) {
      return false;
    }
    if (max_clearance_grid_staleness_ns_ <= 0) {
      return true;
    }
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    return now_ns >= last_occupancy_grid_update_ns_ &&
           now_ns - last_occupancy_grid_update_ns_ <= max_clearance_grid_staleness_ns_;
  }

  [[nodiscard]] bool localPositionFresh() const {
    if (!local_position_valid_ || last_local_position_update_ns_ <= 0) {
      return false;
    }
    if (max_pose_staleness_ns_ <= 0) {
      return true;
    }
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    return now_ns >= last_local_position_update_ns_ &&
           now_ns - last_local_position_update_ns_ <= max_pose_staleness_ns_;
  }

  [[nodiscard]] double localPositionAgeSeconds() const {
    if (last_local_position_update_ns_ <= 0) {
      return std::numeric_limits<double>::infinity();
    }
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    if (now_ns <= last_local_position_update_ns_) {
      return 0.0;
    }
    return static_cast<double>(now_ns - last_local_position_update_ns_) / 1.0e9;
  }

  [[nodiscard]] double estimateLocalClearanceM(const Point2 point) const {
    if (!occupancyGridFresh()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const double resolution = static_cast<double>(occupancy_grid_.info.resolution);
    const double origin_x = occupancy_grid_.info.origin.position.x;
    const double origin_y = occupancy_grid_.info.origin.position.y;
    const auto width = static_cast<int>(occupancy_grid_.info.width);
    const auto height = static_cast<int>(occupancy_grid_.info.height);
    const GridIndex center{
        static_cast<int>(std::floor((point.x - origin_x) / resolution)),
        static_cast<int>(std::floor((point.y - origin_y) / resolution))};
    if (center.x < 0 || center.y < 0 || center.x >= width || center.y >= height) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const int radius_cells = static_cast<int>(std::ceil(
        speed_controller_.config().narrow_clearance_slowdown_radius_m / resolution));
    const int min_x = std::max(center.x - radius_cells, 0);
    const int max_x = std::min(center.x + radius_cells, width - 1);
    const int min_y = std::max(center.y - radius_cells, 0);
    const int max_y = std::min(center.y + radius_cells, height - 1);

    double nearest_clearance_m = std::numeric_limits<double>::infinity();
    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {
        const std::size_t data_index =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x);
        if (occupancy_grid_.data[data_index] < kInflatedOccupancyValue) {
          continue;
        }
        const Point2 cell_center{origin_x + (static_cast<double>(x) + 0.5) * resolution,
                                 origin_y +
                                     (static_cast<double>(y) + 0.5) * resolution};
        nearest_clearance_m =
            std::min(nearest_clearance_m, distance(point, cell_center));
      }
    }

    return nearest_clearance_m;
  }

  [[nodiscard]] std::array<float, 3>
  velocityFeedforward(const Point2 target,
                      const SpeedControllerOutput& speed_output) const {
    if (!velocity_feedforward_enabled_ || !localPositionFresh() ||
        !(speed_output.requested_speed_mps > 0.0)) {
      return std::array<float, 3>{0.0F, 0.0F, 0.0F};
    }

    const double target_distance = distance(current_position_, target);
    if (target_distance <= kTinyDistanceM) {
      return std::array<float, 3>{0.0F, 0.0F, 0.0F};
    }

    const double scale = speed_output.requested_speed_mps / target_distance;
    return std::array<float, 3>{
        static_cast<float>((target.x - current_position_.x) * scale),
        static_cast<float>((target.y - current_position_.y) * scale), 0.0F};
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
    if (!localPositionFresh()) {
      return current_heading_rad_;
    }

    const double dx = target.x - current_position_.x;
    const double dy = target.y - current_position_.y;
    if (std::hypot(dx, dy) < 0.2) {
      return current_heading_rad_;
    }

    return std::atan2(dy, dx);
  }

  [[nodiscard]] Point2 mapToPx4Local(const Point2 point) const noexcept {
    return Point2{point.x - px4_local_origin_.x, point.y - px4_local_origin_.y};
  }

  [[nodiscard]] double
  effectiveMinimumTargetLeadM(const double requested_speed_mps) const noexcept {
    const double desired_speed_mps = speed_controller_.config().desired_speed_mps;
    if (!std::isfinite(requested_speed_mps) || requested_speed_mps <= 0.0 ||
        !std::isfinite(desired_speed_mps) || desired_speed_mps <= 0.0) {
      return 0.0;
    }

    const double speed_ratio =
        std::clamp(requested_speed_mps / desired_speed_mps, 0.0, 1.0);
    return min_commanded_target_lead_m_ * speed_ratio;
  }

  void updateNavigationStartState() {
    if (navigation_started_ || min_navigation_altitude_m_ <= 0.0 || !altitude_valid_) {
      return;
    }
    if (current_altitude_m_ < min_navigation_altitude_m_) {
      return;
    }

    const rclcpp::Time now_time = get_clock()->now();
    if (!navigation_altitude_reached_) {
      navigation_altitude_reached_ = true;
      navigation_altitude_reached_time_ = now_time;
      commanded_target_valid_ = false;
      last_published_target_valid_ = false;
      speed_controller_.reset();
      RCLCPP_INFO(get_logger(),
                  "Navigation altitude reached; holding before horizontal flight: "
                  "altitude=%.2f required=%.2f hover_s=%.2f",
                  current_altitude_m_, min_navigation_altitude_m_, takeoff_hover_s_);
    }

    const double hover_elapsed_s =
        (now_time - navigation_altitude_reached_time_).seconds();
    if (hover_elapsed_s + kTinyDistanceM < takeoff_hover_s_) {
      return;
    }

    navigation_started_ = true;
    commanded_target_valid_ = false;
    last_published_target_valid_ = false;
    speed_controller_.reset();
    RCLCPP_INFO(get_logger(),
                "Takeoff hover complete; horizontal navigation enabled: "
                "altitude=%.2f hover_elapsed=%.2f required_hover=%.2f",
                current_altitude_m_, hover_elapsed_s, takeoff_hover_s_);
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
    const Point2 target = loggedTarget();
    const double target_distance = local_position_valid_
                                       ? distance(current_position_, target)
                                       : std::numeric_limits<double>::quiet_NaN();
    const double mission_goal_distance =
        local_position_valid_ ? distance(current_position_, mission_goal_)
                              : std::numeric_limits<double>::quiet_NaN();
    const double path_goal_distance =
        local_position_valid_ && path_valid_
            ? distance(current_position_, path_points_.back())
            : std::numeric_limits<double>::quiet_NaN();
    const double local_clearance_m = local_position_valid_
                                         ? estimateLocalClearanceM(current_position_)
                                         : std::numeric_limits<double>::quiet_NaN();
    const bool hold_position = shouldHoldPosition();
    const bool pose_fresh = localPositionFresh();
    const double pose_age_s = localPositionAgeSeconds();
    const double turn_angle_rad = pathTurnAngleAtWaypoint(waypoint_index_);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Offboard summary: local_position=%s pose_fresh=%s pose_age_s=%.2f "
        "altitude=%.2f nav_allowed=%s "
        "status=%s armed=%s offboard=%s path=%s hold=%s waypoint=%zu/%zu "
        "motion_phase=%s path_segment=%s "
        "current=(%.2f, %.2f) target=(%.2f, %.2f) "
        "distance_to_target=%.2f distance_to_path_goal=%.2f "
        "distance_to_mission_goal=%.2f requested_speed=%.2f actual_speed=%.2f "
        "speed_limit_reason=%s allowed_speed=%.2f braking_distance=%.2f "
        "target_step=%.2f effective_lookahead=%.2f turn_angle=%.2f "
        "local_clearance=%.2f effective_min_lead=%.2f "
        "speed_limits[goal=%.2f turn=%.2f clearance=%.2f step=%.2f]",
        local_position_valid_ ? "true" : "false", pose_fresh ? "true" : "false",
        pose_age_s, current_altitude_m_, navigationAllowed() ? "true" : "false",
        vehicle_status_valid_ ? "true" : "false", isArmed() ? "true" : "false",
        isOffboard() ? "true" : "false", path_valid_ ? "true" : "false",
        no_path_hold_target_valid_ ? "true" : "false",
        path_valid_ ? waypoint_index_ + 1U : 0U, path_points_.size(),
        motionPhaseName(last_speed_output_.limit_reason, hold_position),
        pathSegmentTypeName(turn_angle_rad), current_position_.x, current_position_.y,
        target.x, target.y, target_distance, path_goal_distance, mission_goal_distance,
        last_speed_output_.requested_speed_mps, current_speed_mps_,
        speedLimitReasonName(last_speed_output_.limit_reason),
        last_speed_output_.allowed_speed_mps, last_speed_output_.braking_distance_m,
        last_speed_output_.target_step_m, effectiveLookaheadDistanceM(), turn_angle_rad,
        local_clearance_m,
        effectiveMinimumTargetLeadM(last_speed_output_.requested_speed_mps),
        last_speed_output_.limits.goal_limit_mps,
        last_speed_output_.limits.turn_limit_mps,
        last_speed_output_.limits.clearance_limit_mps,
        last_speed_output_.limits.step_cap_limit_mps);
  }

  void logTelemetry() {
    if (!local_position_valid_) {
      return;
    }

    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    if (last_telemetry_log_ns_ > 0 &&
        now_ns - last_telemetry_log_ns_ < telemetry_log_period_ns_) {
      return;
    }
    last_telemetry_log_ns_ = now_ns;

    const Point2 target = loggedTarget();
    const double target_distance = distance(current_position_, target);
    const double mission_goal_distance = distance(current_position_, mission_goal_);
    const double path_goal_distance =
        path_valid_ ? distance(current_position_, path_points_.back())
                    : std::numeric_limits<double>::quiet_NaN();
    const double local_clearance_m = estimateLocalClearanceM(current_position_);
    const bool hold_position = shouldHoldPosition();
    const bool pose_fresh = localPositionFresh();
    const double pose_age_s = localPositionAgeSeconds();
    const double turn_angle_rad = pathTurnAngleAtWaypoint(waypoint_index_);

    RCLCPP_INFO(
        get_logger(),
        "Drone telemetry: current=(%.2f, %.2f) pose_fresh=%s pose_age_s=%.2f "
        "altitude=%.2f "
        "velocity=(%.2f, %.2f) velocity_valid=%s actual_speed=%.2f "
        "requested_speed=%.2f allowed_speed=%.2f target=(%.2f, %.2f) "
        "distance_to_target=%.2f distance_to_path_goal=%.2f "
        "distance_to_mission_goal=%.2f waypoint=%zu/%zu motion_phase=%s "
        "path_segment=%s speed_limit_reason=%s local_clearance=%.2f "
        "effective_min_lead=%.2f "
        "speed_limits[goal=%.2f turn=%.2f clearance=%.2f step=%.2f]",
        current_position_.x, current_position_.y, pose_fresh ? "true" : "false",
        pose_age_s, current_altitude_m_, current_velocity_.x, current_velocity_.y,
        current_velocity_valid_ ? "true" : "false", current_speed_mps_,
        last_speed_output_.requested_speed_mps, last_speed_output_.allowed_speed_mps,
        target.x, target.y, target_distance, path_goal_distance, mission_goal_distance,
        path_valid_ ? waypoint_index_ + 1U : 0U, path_points_.size(),
        motionPhaseName(last_speed_output_.limit_reason, hold_position),
        pathSegmentTypeName(turn_angle_rad),
        speedLimitReasonName(last_speed_output_.limit_reason), local_clearance_m,
        effectiveMinimumTargetLeadM(last_speed_output_.requested_speed_mps),
        last_speed_output_.limits.goal_limit_mps,
        last_speed_output_.limits.turn_limit_mps,
        last_speed_output_.limits.clearance_limit_mps,
        last_speed_output_.limits.step_cap_limit_mps);
  }

  [[nodiscard]] Point2 loggedTarget() const {
    if (last_published_target_valid_) {
      return last_published_target_;
    }
    if (commanded_target_valid_) {
      return commanded_target_;
    }
    return limitedTarget(currentTarget());
  }

  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  Point2 current_position_{};
  Point2 current_velocity_{};
  Point2 no_path_hold_target_{};
  Point2 takeoff_hold_target_{};
  Point2 commanded_target_{};
  Point2 last_published_target_{};
  Point2 mission_goal_{85.0, 0.0};
  Point2 px4_local_origin_{};
  double current_heading_rad_{0.0};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double cruise_altitude_m_{12.0};
  double min_navigation_altitude_m_{0.0};
  double takeoff_hover_s_{2.0};
  double acceptance_radius_m_{1.5};
  double max_setpoint_distance_m_{2.0};
  double max_commanded_target_step_m_{0.25};
  double min_commanded_target_lead_m_{0.0};
  double lookahead_distance_m_{6.0};
  double lookahead_time_s_{1.2};
  double min_lookahead_distance_m_{6.0};
  double max_lookahead_distance_m_{6.0};
  double path_switch_hysteresis_m_{3.0};
  double path_continuity_reuse_radius_m_{6.0};
  double path_continuity_max_target_distance_m_{20.0};
  double clearance_escape_step_m_{0.5};
  double clearance_escape_min_improvement_m_{0.05};
  double command_resend_period_s_{2.0};
  double hold_x_m_{0.0};
  double hold_y_m_{0.0};
  double current_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  std::int64_t max_clearance_grid_staleness_ns_{1'500'000'000};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t telemetry_log_period_ns_{500'000'000};
  std::int64_t last_occupancy_grid_update_ns_{0};
  std::int64_t last_local_position_update_ns_{0};
  std::int64_t last_telemetry_log_ns_{0};
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
  bool occupancy_grid_valid_{false};
  bool occupancy_grid_seen_logged_{false};
  bool current_velocity_valid_{false};
  bool no_path_hold_target_valid_{false};
  bool takeoff_hold_target_valid_{false};
  bool commanded_target_valid_{false};
  bool last_published_target_valid_{false};
  bool face_target_yaw_{false};
  bool navigation_altitude_reached_{false};
  bool navigation_started_{false};
  bool target_segment_safety_check_enabled_{true};
  bool clearance_escape_enabled_{true};
  bool velocity_feedforward_enabled_{false};
  bool dynamic_lookahead_enabled_{true};
  std::uint8_t target_system_{1U};
  std::uint8_t target_component_{1U};
  std::uint8_t source_system_{1U};
  std::uint16_t source_component_{1U};
  int last_logged_arming_state_{-1};
  int last_logged_nav_state_{-1};
  std::size_t last_logged_path_size_{std::numeric_limits<std::size_t>::max()};
  Point2 last_logged_path_first_{};
  Point2 last_logged_path_last_{};
  OffboardSpeedController speed_controller_;
  SpeedControllerOutput last_speed_output_{};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time navigation_altitude_reached_time_{0, 0, RCL_ROS_TIME};
  std::vector<Point2> path_points_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::Px4OffboardNode>());
  rclcpp::shutdown();
  return 0;
}
