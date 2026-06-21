#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/offboard_path_follower.hpp"
#include "drone_city_nav/offboard_target_continuity.hpp"
#include "drone_city_nav/offboard_velocity_follower.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/types.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <algorithm>
#include <array>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr auto kControllerPeriod = std::chrono::milliseconds{100};
constexpr double kTinyDistanceM = 1.0e-6;
constexpr double kLocalClearanceDiagnosticRadiusM = 12.0;
constexpr std::int8_t kInflatedOccupancyValue = 80;

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

[[nodiscard]] double radiansToDegrees(const double radians) noexcept {
  return radians * 180.0 / std::numbers::pi;
}

[[nodiscard]] double radiansFromDegrees(const double degrees) noexcept {
  return degrees * std::numbers::pi / 180.0;
}

[[nodiscard]] double normalizeAngle(const double angle_rad) noexcept {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

[[nodiscard]] double angleBetweenVectorsRad(const Point2 first,
                                            const Point2 second) noexcept {
  const double first_length = std::hypot(first.x, first.y);
  const double second_length = std::hypot(second.x, second.y);
  if (first_length <= kTinyDistanceM || second_length <= kTinyDistanceM) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double cosine = std::clamp((first.x * second.x + first.y * second.y) /
                                       (first_length * second_length),
                                   -1.0, 1.0);
  return std::acos(cosine);
}

[[nodiscard]] std::uint64_t
stampNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000U;
  return static_cast<std::uint64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::uint64_t>(stamp.nanosec);
}

void writeJsonBool(std::ostream& stream, const bool value) {
  stream << (value ? "true" : "false");
}

void writeJsonNumberOrNull(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
    return;
  }
  stream << "null";
}

struct PathTrackingDiagnostics {
  bool valid{false};
  std::size_t segment_start_index{0U};
  double segment_t{0.0};
  double cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double signed_cross_track_error_m{std::numeric_limits<double>::quiet_NaN()};
  double path_heading_rad{std::numeric_limits<double>::quiet_NaN()};
  double heading_error_rad{std::numeric_limits<double>::quiet_NaN()};
  Point2 projection{};
};

struct NearestObstacleDiagnostic {
  bool valid{false};
  double clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double bearing_map_rad{std::numeric_limits<double>::quiet_NaN()};
  double bearing_body_rad{std::numeric_limits<double>::quiet_NaN()};
  double bearing_body_deg{std::numeric_limits<double>::quiet_NaN()};
  Point2 point{};
};

enum class OffboardSetpointMode : std::uint8_t {
  kPositionHold,
  kVelocityCruise,
};

[[nodiscard]] const char*
offboardSetpointModeName(const OffboardSetpointMode mode) noexcept {
  switch (mode) {
    case OffboardSetpointMode::kPositionHold:
      return "position_hold";
    case OffboardSetpointMode::kVelocityCruise:
      return "velocity_cruise";
  }
  return "unknown";
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
    turn_preview_distance_m_ = std::clamp(
        declare_parameter<double>("turn_preview_distance_m", 32.0), 0.0, 500.0);
    sharp_turn_hold_angle_rad_ =
        std::clamp(radiansFromDegrees(
                       declare_parameter<double>("sharp_turn_hold_angle_deg", 60.0)),
                   0.0, std::numbers::pi);
    sharp_turn_hold_s_ =
        std::clamp(declare_parameter<double>("sharp_turn_hold_s", 2.0), 0.0, 30.0);
    max_clearance_grid_staleness_ns_ = static_cast<std::int64_t>(
        std::clamp<double>(
            declare_parameter<double>("max_clearance_grid_staleness_s", 1.5), 0.0,
            3600.0) *
        1.0e9);
    max_pose_staleness_ns_ = static_cast<std::int64_t>(
        boundedFiniteDouble(declare_parameter<double>("max_pose_staleness_s", 1.0), 1.0,
                            0.0, 3600.0) *
        1.0e9);
    path_switch_hysteresis_m_ = std::clamp(
        declare_parameter<double>("path_switch_hysteresis_m", 3.0), 0.0, 100.0);
    path_continuity_reuse_radius_m_ = std::clamp(
        declare_parameter<double>("path_continuity_reuse_radius_m", 6.0), 0.0, 100.0);
    path_continuity_max_target_distance_m_ = std::clamp(
        declare_parameter<double>("path_continuity_max_target_distance_m", 20.0), 0.0,
        500.0);
    commanded_target_hysteresis_m_ = std::clamp(
        declare_parameter<double>("commanded_target_hysteresis_m", 0.5), 0.0, 10.0);
    target_switch_hold_s_ =
        std::clamp(declare_parameter<double>("target_switch_hold_s", 2.0), 0.0, 30.0);
    target_switch_hold_delta_m_ = std::clamp(
        declare_parameter<double>("target_switch_hold_delta_m", 5.0), 0.0, 500.0);
    target_switch_hold_angle_rad_ =
        std::clamp(radiansFromDegrees(
                       declare_parameter<double>("target_switch_hold_angle_deg", 45.0)),
                   0.0, std::numbers::pi);
    cruise_velocity_control_enabled_ =
        declare_parameter<bool>("cruise_velocity_control_enabled", true);
    velocity_follower_config_.cruise_speed_mps =
        std::clamp(declare_parameter<double>("cruise_speed_mps", 12.0), 0.0, 100.0);
    velocity_follower_config_.min_turn_speed_mps =
        std::clamp(declare_parameter<double>("min_turn_speed_mps", 2.0), 0.0,
                   velocity_follower_config_.cruise_speed_mps);
    velocity_follower_config_.max_accel_mps2 =
        std::clamp(declare_parameter<double>("max_accel_mps2", 3.0), 0.0, 100.0);
    velocity_follower_config_.max_decel_mps2 =
        std::clamp(declare_parameter<double>("max_decel_mps2", 4.0), 0.0, 100.0);
    velocity_follower_config_.max_lateral_accel_mps2 = std::clamp(
        declare_parameter<double>("max_lateral_accel_mps2", 3.0), 0.0, 100.0);
    velocity_follower_config_.turn_preview_distance_m = turn_preview_distance_m_;
    velocity_follower_config_.turn_slowdown_min_angle_rad =
        std::clamp(radiansFromDegrees(
                       declare_parameter<double>("turn_slowdown_min_angle_deg", 15.0)),
                   0.0, std::numbers::pi);
    velocity_follower_config_.sharp_turn_angle_rad = std::clamp(
        radiansFromDegrees(declare_parameter<double>("sharp_turn_angle_deg", 90.0)),
        velocity_follower_config_.turn_slowdown_min_angle_rad, std::numbers::pi);
    velocity_follower_config_.braking_margin_m =
        std::clamp(declare_parameter<double>("braking_margin_m", 2.0), 0.0, 100.0);
    velocity_follower_config_.cross_track_gain =
        std::clamp(declare_parameter<double>("cross_track_gain", 0.25), 0.0, 10.0);
    velocity_follower_config_.max_cross_track_correction_angle_rad =
        std::clamp(radiansFromDegrees(declare_parameter<double>(
                       "max_cross_track_correction_angle_deg", 20.0)),
                   0.0, std::numbers::pi / 2.0);
    velocity_follower_config_.final_acceptance_radius_m = acceptance_radius_m_;
    altitude_hold_kp_ =
        std::clamp(declare_parameter<double>("altitude_hold_kp", 0.5), 0.0, 10.0);
    max_vertical_speed_mps_ =
        std::clamp(declare_parameter<double>("max_vertical_speed_mps", 2.0), 0.0, 20.0);
    telemetry_log_period_ns_ = static_cast<std::int64_t>(
        std::clamp(declare_parameter<double>("telemetry_log_period_s", 0.5), 0.1,
                   60.0) *
        1.0e9);
    flight_blackbox_enabled_ = declare_parameter<bool>("flight_blackbox_enabled", true);
    flight_blackbox_path_ = declare_parameter<std::string>(
        "flight_blackbox_path", "log/offboard_blackbox.jsonl");
    warmup_setpoints_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("warmup_setpoints", 20), 1, 100000));
    auto_arm_ = declare_parameter<bool>("auto_arm", true);
    auto_offboard_ = declare_parameter<bool>("auto_offboard", true);
    const double requested_command_resend_period_s =
        declare_parameter<double>("command_resend_period_s", 2.0);
    command_resend_period_s_ =
        boundedFiniteDouble(requested_command_resend_period_s, 2.0, 0.05, 60.0);
    if (!std::isfinite(requested_command_resend_period_s) ||
        requested_command_resend_period_s != command_resend_period_s_) {
      RCLCPP_WARN(get_logger(),
                  "Sanitized command_resend_period_s: requested=%.3f final=%.3f "
                  "allowed_range=[0.050, 60.000]",
                  requested_command_resend_period_s, command_resend_period_s_);
    }
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
    const std::string path_id_topic =
        declare_parameter<std::string>("path_id_topic", "/drone_city_nav/path_id");
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");
    const std::string attitude_topic = declare_parameter<std::string>(
        "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");
    const std::string vehicle_status_topic = declare_parameter<std::string>(
        "px4_vehicle_status_topic", "/fmu/out/vehicle_status");
    const std::string emergency_stop_topic = declare_parameter<std::string>(
        "emergency_stop_topic", "/drone_city_nav/emergency_stop");
    const std::string prohibited_grid_topic = declare_parameter<std::string>(
        "prohibited_grid_topic", "/drone_city_nav/prohibited_grid");

    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
    const auto emergency_stop_qos = rclcpp::QoS{1}.reliable().durability_volatile();
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic, rclcpp::QoS{1}.reliable(),
        [this](const nav_msgs::msg::Path::SharedPtr msg) { onPath(*msg); });
    path_id_sub_ = create_subscription<std_msgs::msg::UInt64>(
        path_id_topic, rclcpp::QoS{1}.reliable(),
        [this](const std_msgs::msg::UInt64::SharedPtr msg) { onPathId(*msg); });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          onLocalPosition(*msg);
        });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
          onAttitude(*msg);
        });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        vehicle_status_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
          onVehicleStatus(*msg);
        });
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        emergency_stop_topic, emergency_stop_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { onEmergencyStop(*msg); });
    prohibited_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        prohibited_grid_topic, rclcpp::QoS{1}.transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          onProhibitedGrid(*msg);
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
    openFlightBlackbox();

    RCLCPP_INFO(
        get_logger(),
        "PX4 offboard node ready: altitude=%.1fm acceptance=%.1fm auto_arm=%s "
        "auto_offboard=%s min_navigation_altitude=%.1fm face_target_yaw=%s "
        "takeoff_hover=%.1fs "
        "turn_preview_distance=%.1fm sharp_turn_hold_angle=%.1fdeg "
        "sharp_turn_hold=%.2fs "
        "target_switch_hold=%.2fs target_switch_delta=%.1fm "
        "target_switch_angle=%.1fdeg "
        "velocity_cruise=%s cruise_speed=%.2fmps min_turn_speed=%.2fmps "
        "max_accel=%.2fmps2 max_decel=%.2fmps2 max_lateral_accel=%.2fmps2 "
        "turn_slowdown_min_angle=%.1fdeg sharp_turn_angle=%.1fdeg "
        "braking_margin=%.2fm cross_track_gain=%.2f "
        "max_cross_track_correction_angle=%.1fdeg altitude_hold_kp=%.2f "
        "max_vertical_speed=%.2fmps "
        "path_switch_hysteresis=%.1fm path_continuity_reuse_radius=%.1fm "
        "path_continuity_max_target_distance=%.1fm "
        "commanded_target_hysteresis=%.2fm mission_goal=(%.1f, %.1f) "
        "px4_local_origin=(%.1f, %.1f) telemetry_log_period=%.2fs "
        "flight_blackbox=%s flight_blackbox_path='%s' "
        "max_pose_staleness=%.2fs command_resend_period=%.2fs",
        cruise_altitude_m_, acceptance_radius_m_, auto_arm_ ? "true" : "false",
        auto_offboard_ ? "true" : "false", min_navigation_altitude_m_,
        face_target_yaw_ ? "true" : "false", takeoff_hover_s_, turn_preview_distance_m_,
        radiansToDegrees(sharp_turn_hold_angle_rad_), sharp_turn_hold_s_,
        target_switch_hold_s_, target_switch_hold_delta_m_,
        radiansToDegrees(target_switch_hold_angle_rad_),
        cruise_velocity_control_enabled_ ? "true" : "false",
        velocity_follower_config_.cruise_speed_mps,
        velocity_follower_config_.min_turn_speed_mps,
        velocity_follower_config_.max_accel_mps2,
        velocity_follower_config_.max_decel_mps2,
        velocity_follower_config_.max_lateral_accel_mps2,
        radiansToDegrees(velocity_follower_config_.turn_slowdown_min_angle_rad),
        radiansToDegrees(velocity_follower_config_.sharp_turn_angle_rad),
        velocity_follower_config_.braking_margin_m,
        velocity_follower_config_.cross_track_gain,
        radiansToDegrees(
            velocity_follower_config_.max_cross_track_correction_angle_rad),
        altitude_hold_kp_, max_vertical_speed_mps_, path_switch_hysteresis_m_,
        path_continuity_reuse_radius_m_, path_continuity_max_target_distance_m_,
        commanded_target_hysteresis_m_, mission_goal_.x, mission_goal_.y,
        px4_local_origin_.x, px4_local_origin_.y,
        static_cast<double>(telemetry_log_period_ns_) / 1.0e9,
        flight_blackbox_enabled_ ? "true" : "false", flight_blackbox_path_.c_str(),
        static_cast<double>(max_pose_staleness_ns_) / 1.0e9, command_resend_period_s_);
    RCLCPP_INFO(
        get_logger(),
        "PX4 offboard subscriptions: path='%s' path_id='%s' local_position='%s' "
        "attitude='%s' vehicle_status='%s' emergency_stop='%s' "
        "prohibited_grid='%s'",
        path_topic.c_str(), path_id_topic.c_str(), local_position_topic.c_str(),
        attitude_topic.c_str(), vehicle_status_topic.c_str(),
        emergency_stop_topic.c_str(), prohibited_grid_topic.c_str());
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
    return OffboardPathFollowerConfig{
        acceptance_radius_m_, turn_preview_distance_m_, path_switch_hysteresis_m_,
        path_continuity_reuse_radius_m_, path_continuity_max_target_distance_m_};
  }

  [[nodiscard]] Point2 activePathTargetForContinuity() const {
    if (last_published_target_valid_) {
      return last_published_target_;
    }
    if (commanded_target_valid_) {
      return commanded_target_;
    }
    if (path_valid_ && waypoint_index_ < path_points_.size()) {
      return path_points_[waypoint_index_];
    }
    return current_position_;
  }

  void onPath(const nav_msgs::msg::Path& path) {
    ++received_path_update_id_;
    last_received_path_stamp_ns_ = stampNanoseconds(path.header.stamp);
    const bool had_active_target =
        path_valid_ && localPositionFresh() && waypoint_index_ < path_points_.size();
    const Point2 previous_target =
        had_active_target ? activePathTargetForContinuity() : Point2{};

    path_points_ = pathPointsFromMessage(path);
    path_valid_ = !path_points_.empty();
    path_update_target_hysteresis_pending_ = false;
    sharp_turn_hold_active_ = false;
    target_switch_hold_active_ = false;
    sharp_turn_hold_completed_index_.reset();

    if (!path_valid_) {
      if (last_logged_path_size_ != 0U) {
        if (local_position_valid_) {
          no_path_hold_target_ = current_position_;
          no_path_hold_target_valid_ = true;
          commanded_target_ = no_path_hold_target_;
          commanded_target_valid_ = true;
          waypoint_index_ = 0U;
          path_update_raw_candidate_index_ = 0U;
          RCLCPP_WARN(get_logger(),
                      "Received empty path: local_path_update_id=%" PRIu64
                      " planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                      " holding fixed target at current position (%.2f, %.2f) "
                      "and resetting commanded target",
                      received_path_update_id_, latest_planner_path_id_,
                      last_received_path_stamp_ns_, no_path_hold_target_.x,
                      no_path_hold_target_.y);
        } else {
          no_path_hold_target_valid_ = false;
          commanded_target_valid_ = false;
          waypoint_index_ = 0U;
          path_update_raw_candidate_index_ = 0U;
          RCLCPP_WARN(get_logger(),
                      "Received empty path: local_path_update_id=%" PRIu64
                      " planner_path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                      " before local position; holding configured fallback target",
                      received_path_update_id_, latest_planner_path_id_,
                      last_received_path_stamp_ns_);
        }
        last_logged_path_size_ = 0U;
      }
      return;
    }

    no_path_hold_target_valid_ = false;
    const std::size_t candidate_index =
        localPositionFresh()
            ? drone_city_nav::advanceWaypointIndex(path_points_, current_position_, 0U,
                                                   pathFollowerConfig())
            : 0U;
    path_update_raw_candidate_index_ = candidate_index;
    waypoint_index_ =
        continuityWaypointIndex(previous_target, candidate_index, had_active_target);
    path_update_target_hysteresis_pending_ =
        had_active_target && commanded_target_hysteresis_m_ > 0.0;
    const Point2 first = path_points_.front();
    const Point2 last = path_points_.back();
    const bool path_changed = path_points_.size() != last_logged_path_size_ ||
                              squaredDistance(first, last_logged_path_first_) > 0.01 ||
                              squaredDistance(last, last_logged_path_last_) > 0.01;
    if (path_changed) {
      const PathMetrics metrics = pointPathMetrics(path_points_);
      RCLCPP_INFO(
          get_logger(),
          "Received path: local_path_update_id=%" PRIu64 " planner_path_id=%" PRIu64
          " path_stamp_ns=%" PRIu64 " waypoints=%zu segments=%zu straight_segments=%zu "
          "turns=%zu length=%.2f selected=%zu first=(%.2f, %.2f) "
          "segment_lengths[min=%.2f mean=%.2f max=%.2f lt2=%zu lt5=%zu lt10=%zu] "
          "last=(%.2f, %.2f) continuity_target=(%.2f, %.2f) "
          "had_active_target=%s",
          received_path_update_id_, latest_planner_path_id_,
          last_received_path_stamp_ns_, path_points_.size(), metrics.segments,
          metrics.straight_segments, metrics.turns, metrics.length_m,
          waypoint_index_ + 1U, first.x, first.y, metrics.min_segment_length_m,
          metrics.mean_segment_length_m, metrics.max_segment_length_m,
          metrics.segments_shorter_than_2m, metrics.segments_shorter_than_5m,
          metrics.segments_shorter_than_10m, last.x, last.y, previous_target.x,
          previous_target.y, had_active_target ? "true" : "false");
      last_logged_path_size_ = path_points_.size();
      last_logged_path_first_ = first;
      last_logged_path_last_ = last;
    }
  }

  void onPathId(const std_msgs::msg::UInt64& msg) {
    latest_planner_path_id_ = msg.data;
    latest_planner_path_id_seen_ = true;
  }

  void openFlightBlackbox() {
    if (!flight_blackbox_enabled_) {
      return;
    }

    const std::filesystem::path path{flight_blackbox_path_};
    const std::filesystem::path parent = path.parent_path();
    std::error_code error;
    if (!parent.empty()) {
      std::filesystem::create_directories(parent, error);
      if (error) {
        RCLCPP_WARN(get_logger(), "Failed to create flight blackbox directory '%s': %s",
                    parent.string().c_str(), error.message().c_str());
        flight_blackbox_enabled_ = false;
        return;
      }
    }

    flight_blackbox_stream_.open(path, std::ios::out | std::ios::trunc);
    if (!flight_blackbox_stream_.is_open()) {
      RCLCPP_WARN(get_logger(), "Failed to open flight blackbox '%s'",
                  flight_blackbox_path_.c_str());
      flight_blackbox_enabled_ = false;
      return;
    }

    flight_blackbox_stream_ << std::setprecision(6);
    RCLCPP_INFO(get_logger(), "Writing flight blackbox telemetry to '%s'",
                flight_blackbox_path_.c_str());
  }

  [[nodiscard]] std::optional<OffboardPathProjection> closestPathProjection() const {
    if (!localPositionFresh() || path_points_.empty()) {
      return std::nullopt;
    }
    return drone_city_nav::closestOffboardPathProjection(path_points_,
                                                         current_position_);
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

  void onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
    last_attitude_update_ns_ = get_clock()->now().nanoseconds();
    const auto euler = quaternionToEuler(msg.q);
    if (!euler.has_value()) {
      attitude_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring invalid PX4 attitude quaternion");
      return;
    }

    current_attitude_ = *euler;
    attitude_valid_ = true;
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
    target_switch_hold_active_ = false;
    RCLCPP_ERROR(get_logger(),
                 "Emergency stop requested; stopping trajectory setpoints and "
                 "sending disarm commands");
  }

  void onProhibitedGrid(const nav_msgs::msg::OccupancyGrid& msg) {
    if (!(msg.info.resolution > 0.0F) || msg.info.width == 0U ||
        msg.info.height == 0U ||
        msg.info.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        msg.info.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring invalid offboard prohibited grid metadata");
      return;
    }

    const std::size_t expected_size = static_cast<std::size_t>(msg.info.width) *
                                      static_cast<std::size_t>(msg.info.height);
    if (msg.data.size() != expected_size) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring offboard prohibited grid with mismatched data size: expected=%zu "
          "got=%zu",
          expected_size, msg.data.size());
      return;
    }

    prohibited_grid_ = msg;
    prohibited_grid_valid_ = true;
    last_prohibited_grid_update_ns_ = get_clock()->now().nanoseconds();
    if (!prohibited_grid_seen_logged_) {
      prohibited_grid_seen_logged_ = true;
      RCLCPP_INFO(get_logger(),
                  "First offboard prohibited grid: size=%ux%u resolution=%.2f "
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
    const OffboardSetpointMode mode = currentSetpointMode();
    msg.position = mode == OffboardSetpointMode::kPositionHold;
    msg.velocity = mode == OffboardSetpointMode::kVelocityCruise;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_control_mode_pub_->publish(msg);
  }

  void publishTrajectorySetpoint() {
    advanceWaypointIfNeeded();

    if (currentSetpointMode() == OffboardSetpointMode::kVelocityCruise) {
      if (publishVelocityTrajectorySetpoint()) {
        return;
      }
    }

    const bool had_previous_target = last_published_target_valid_;
    const Point2 previous_target = last_published_target_;
    const Point2 desired_target = currentTarget();
    const Point2 proposed_target =
        selectCommandTarget(desired_target, shouldHoldPosition());
    const Point2 target = applyPathUpdateTargetHysteresis(
        proposed_target, previous_target, had_previous_target);
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
    msg.velocity = std::array<float, 3>{nan, nan, nan};
    msg.acceleration = std::array<float, 3>{nan, nan, nan};
    msg.jerk = std::array<float, 3>{nan, nan, nan};
    msg.yaw =
        static_cast<float>(face_target_yaw_ ? targetYaw(target) : current_heading_rad_);
    msg.yawspeed = nan;
    updateCommandDiagnostics(target, previous_target, had_previous_target,
                             static_cast<double>(msg.yaw));
    resetVelocityDiagnostics();

    trajectory_setpoint_pub_->publish(msg);
  }

  [[nodiscard]] OffboardSetpointMode currentSetpointMode() const {
    return velocityCruiseReady() ? OffboardSetpointMode::kVelocityCruise
                                 : OffboardSetpointMode::kPositionHold;
  }

  [[nodiscard]] bool velocityCruiseReady() const {
    return cruise_velocity_control_enabled_ && localPositionFresh() &&
           navigationAllowed() && path_valid_ &&
           waypoint_index_ < path_points_.size() && !finalPathGoalReached() &&
           !no_path_hold_target_valid_ && !sharp_turn_hold_active_ &&
           !target_switch_hold_active_;
  }

  [[nodiscard]] bool finalPathGoalReached() const {
    return localPositionFresh() && path_valid_ && !path_points_.empty() &&
           distance(current_position_, path_points_.back()) <= acceptance_radius_m_;
  }

  [[nodiscard]] double consumeVelocityPlanDtS() {
    const rclcpp::Time current_time = get_clock()->now();
    double dt_s = static_cast<double>(kControllerPeriod.count()) / 1000.0;
    if (last_velocity_plan_time_.nanoseconds() > 0 &&
        current_time > last_velocity_plan_time_) {
      dt_s =
          std::clamp((current_time - last_velocity_plan_time_).seconds(), 0.001, 1.0);
    }
    last_velocity_plan_time_ = current_time;
    return dt_s;
  }

  [[nodiscard]] double verticalVelocitySetpointNed() {
    if (!altitude_valid_ || !(max_vertical_speed_mps_ > 0.0)) {
      last_altitude_error_m_ = std::numeric_limits<double>::quiet_NaN();
      return 0.0;
    }

    last_altitude_error_m_ = cruise_altitude_m_ - current_altitude_m_;
    return -std::clamp(last_altitude_error_m_ * altitude_hold_kp_,
                       -max_vertical_speed_mps_, max_vertical_speed_mps_);
  }

  [[nodiscard]] double velocityYaw(const Point2 velocity_xy) const {
    if (!face_target_yaw_ || std::hypot(velocity_xy.x, velocity_xy.y) < 0.2) {
      return current_heading_rad_;
    }
    return std::atan2(velocity_xy.y, velocity_xy.x);
  }

  bool publishVelocityTrajectorySetpoint() {
    const bool had_previous_target = last_published_target_valid_;
    const Point2 previous_target = last_published_target_;
    const Point2 target = currentTarget();
    const double dt_s = consumeVelocityPlanDtS();
    const VelocitySetpointPlan plan = planVelocitySetpoint(
        path_points_, current_position_, current_velocity_, current_velocity_valid_,
        waypoint_index_, dt_s, velocity_follower_state_, velocity_follower_config_);
    last_velocity_plan_ = plan;
    last_velocity_plan_valid_ = plan.valid;
    if (!plan.valid || plan.final_goal_reached) {
      resetVelocityDiagnostics();
      return false;
    }

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const double vz_ned = verticalVelocitySetpointNed();
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = nowMicros();
    msg.position = std::array<float, 3>{nan, nan, nan};
    msg.velocity = std::array<float, 3>{static_cast<float>(plan.velocity_xy.x),
                                        static_cast<float>(plan.velocity_xy.y),
                                        static_cast<float>(vz_ned)};
    msg.acceleration = std::array<float, 3>{nan, nan, nan};
    msg.jerk = std::array<float, 3>{nan, nan, nan};
    msg.yaw = static_cast<float>(velocityYaw(plan.velocity_xy));
    msg.yawspeed = nan;

    velocity_follower_state_.previous_velocity_setpoint = plan.velocity_xy;
    velocity_follower_state_.previous_velocity_setpoint_valid = true;
    last_velocity_setpoint_ = plan.velocity_xy;
    last_vertical_velocity_setpoint_mps_ = vz_ned;
    last_velocity_setpoint_speed_mps_ = plan.speed_mps;
    last_offboard_setpoint_mode_ = OffboardSetpointMode::kVelocityCruise;
    commanded_target_ = target;
    commanded_target_valid_ = true;
    last_published_target_ = target;
    last_published_target_valid_ = true;
    updateCommandDiagnostics(target, previous_target, had_previous_target,
                             static_cast<double>(msg.yaw));
    trajectory_setpoint_pub_->publish(msg);
    return true;
  }

  void resetVelocityDiagnostics() {
    velocity_follower_state_ = VelocityFollowerState{};
    last_velocity_plan_valid_ = false;
    last_velocity_plan_ = VelocitySetpointPlan{};
    last_velocity_plan_.reason = VelocitySetpointReason::kHold;
    last_velocity_setpoint_ = Point2{};
    last_velocity_setpoint_speed_mps_ = 0.0;
    last_vertical_velocity_setpoint_mps_ = 0.0;
    last_altitude_error_m_ = std::numeric_limits<double>::quiet_NaN();
    last_offboard_setpoint_mode_ = OffboardSetpointMode::kPositionHold;
    last_velocity_plan_time_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
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
    if (!cruise_velocity_control_enabled_) {
      if (updateSharpTurnHold()) {
        return;
      }
      if (updateTargetSwitchHold()) {
        return;
      }
    }

    const std::size_t previous_waypoint_index = waypoint_index_;
    const std::size_t next_waypoint_index = drone_city_nav::advanceWaypointIndex(
        path_points_, current_position_, waypoint_index_, pathFollowerConfig());

    if (next_waypoint_index == previous_waypoint_index) {
      return;
    }

    if (!cruise_velocity_control_enabled_ &&
        shouldStartSharpTurnHold(previous_waypoint_index, next_waypoint_index)) {
      startSharpTurnHold(previous_waypoint_index);
      return;
    }

    waypoint_index_ = next_waypoint_index;
    if (waypoint_index_ != previous_waypoint_index) {
      const Point2 target = path_points_[waypoint_index_];
      RCLCPP_INFO(get_logger(),
                  "Waypoint advanced: index=%zu/%zu current=(%.2f, %.2f) "
                  "target=(%.2f, %.2f)",
                  waypoint_index_ + 1U, path_points_.size(), current_position_.x,
                  current_position_.y, target.x, target.y);
    }
  }

  bool updateSharpTurnHold() {
    if (!sharp_turn_hold_active_) {
      return false;
    }

    const double elapsed_s =
        (get_clock()->now() - sharp_turn_hold_started_time_).seconds();
    if (elapsed_s + kTinyDistanceM < sharp_turn_hold_s_) {
      return true;
    }

    sharp_turn_hold_active_ = false;
    sharp_turn_hold_completed_index_ = sharp_turn_hold_waypoint_index_;
    RCLCPP_INFO(get_logger(),
                "Sharp-turn hold complete: waypoint=%zu/%zu elapsed=%.2fs "
                "required=%.2fs target=(%.2f, %.2f)",
                sharp_turn_hold_waypoint_index_ + 1U, path_points_.size(), elapsed_s,
                sharp_turn_hold_s_, sharp_turn_hold_target_.x,
                sharp_turn_hold_target_.y);
    return false;
  }

  bool updateTargetSwitchHold() {
    if (!target_switch_hold_active_) {
      return false;
    }

    const double elapsed_s =
        (get_clock()->now() - target_switch_hold_started_time_).seconds();
    if (elapsed_s + kTinyDistanceM < target_switch_hold_s_) {
      return true;
    }

    target_switch_hold_active_ = false;
    RCLCPP_INFO(get_logger(),
                "Target-switch hold complete: elapsed=%.2fs required=%.2fs "
                "pending_target=(%.2f, %.2f)",
                elapsed_s, target_switch_hold_s_, target_switch_pending_target_.x,
                target_switch_pending_target_.y);
    return false;
  }

  [[nodiscard]] bool
  shouldStartSharpTurnHold(const std::size_t previous_waypoint_index,
                           const std::size_t next_waypoint_index) const {
    if (!(sharp_turn_hold_s_ > 0.0) || next_waypoint_index <= previous_waypoint_index ||
        previous_waypoint_index == 0U ||
        previous_waypoint_index + 1U >= path_points_.size()) {
      return false;
    }
    if (sharp_turn_hold_completed_index_.has_value() &&
        *sharp_turn_hold_completed_index_ == previous_waypoint_index) {
      return false;
    }

    return pathCornerAngleRad(previous_waypoint_index) >= sharp_turn_hold_angle_rad_;
  }

  void startSharpTurnHold(const std::size_t waypoint_index) {
    sharp_turn_hold_active_ = true;
    sharp_turn_hold_waypoint_index_ = waypoint_index;
    sharp_turn_hold_target_ = path_points_[waypoint_index];
    sharp_turn_hold_started_time_ = get_clock()->now();
    const double angle_rad = pathCornerAngleRad(waypoint_index);
    RCLCPP_INFO(get_logger(),
                "Sharp-turn hold started: waypoint=%zu/%zu angle=%.1fdeg "
                "threshold=%.1fdeg hold_s=%.2f current=(%.2f, %.2f) "
                "target=(%.2f, %.2f)",
                waypoint_index + 1U, path_points_.size(), radiansToDegrees(angle_rad),
                radiansToDegrees(sharp_turn_hold_angle_rad_), sharp_turn_hold_s_,
                current_position_.x, current_position_.y, sharp_turn_hold_target_.x,
                sharp_turn_hold_target_.y);
  }

  [[nodiscard]] double targetSwitchAngleRad(const Point2 previous_target,
                                            const Point2 selected_target) const {
    const Point2 previous_vector{previous_target.x - current_position_.x,
                                 previous_target.y - current_position_.y};
    const Point2 selected_vector{selected_target.x - current_position_.x,
                                 selected_target.y - current_position_.y};
    return angleBetweenVectorsRad(previous_vector, selected_vector);
  }

  [[nodiscard]] bool shouldStartTargetSwitchHold(const Point2 selected_target,
                                                 const bool had_previous_target,
                                                 const double target_delta_m,
                                                 const double target_angle_rad) const {
    if (!(target_switch_hold_s_ > 0.0) || !had_previous_target ||
        !localPositionFresh() || !navigationAllowed() || sharp_turn_hold_active_ ||
        cruise_velocity_control_enabled_) {
      return false;
    }
    if (target_delta_m <= kTinyDistanceM) {
      return false;
    }
    if (distance(current_position_, selected_target) <= acceptance_radius_m_) {
      return false;
    }

    const bool distance_trigger = target_switch_hold_delta_m_ > 0.0 &&
                                  target_delta_m > target_switch_hold_delta_m_;
    const bool angle_trigger = target_switch_hold_angle_rad_ > 0.0 &&
                               std::isfinite(target_angle_rad) &&
                               target_angle_rad > target_switch_hold_angle_rad_;
    return distance_trigger || angle_trigger;
  }

  void startTargetSwitchHold(const Point2 previous_target, const Point2 selected_target,
                             const double target_delta_m,
                             const double target_angle_rad) {
    target_switch_hold_active_ = true;
    target_switch_hold_target_ = current_position_;
    target_switch_pending_target_ = selected_target;
    target_switch_hold_started_time_ = get_clock()->now();
    last_target_switch_hold_delta_m_ = target_delta_m;
    last_target_switch_hold_angle_rad_ = target_angle_rad;
    RCLCPP_WARN(
        get_logger(),
        "Target-switch hold started: hold_s=%.2f delta=%.2fm "
        "angle=%.1fdeg thresholds[delta=%.2fm angle=%.1fdeg] "
        "current=(%.2f, %.2f) previous=(%.2f, %.2f) "
        "selected=(%.2f, %.2f)",
        target_switch_hold_s_, target_delta_m, radiansToDegrees(target_angle_rad),
        target_switch_hold_delta_m_, radiansToDegrees(target_switch_hold_angle_rad_),
        target_switch_hold_target_.x, target_switch_hold_target_.y, previous_target.x,
        previous_target.y, selected_target.x, selected_target.y);
  }

  [[nodiscard]] double pathCornerAngleRad(const std::size_t waypoint_index) const {
    if (waypoint_index == 0U || waypoint_index + 1U >= path_points_.size()) {
      return 0.0;
    }

    const Point2 previous = path_points_[waypoint_index - 1U];
    const Point2 current = path_points_[waypoint_index];
    const Point2 next = path_points_[waypoint_index + 1U];
    const Point2 incoming{current.x - previous.x, current.y - previous.y};
    const Point2 outgoing{next.x - current.x, next.y - current.y};
    const double incoming_length = std::hypot(incoming.x, incoming.y);
    const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
    if (incoming_length <= kTinyDistanceM || outgoing_length <= kTinyDistanceM) {
      return 0.0;
    }

    const double cosine =
        std::clamp((incoming.x * outgoing.x + incoming.y * outgoing.y) /
                       (incoming_length * outgoing_length),
                   -1.0, 1.0);
    return std::acos(cosine);
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

    if (sharp_turn_hold_active_) {
      return sharp_turn_hold_target_;
    }

    if (localPositionFresh() && path_valid_ && waypoint_index_ < path_points_.size()) {
      return path_points_[waypoint_index_];
    }

    if (no_path_hold_target_valid_) {
      return no_path_hold_target_;
    }

    if (local_position_valid_) {
      return current_position_;
    }

    return Point2{hold_x_m_, hold_y_m_};
  }

  [[nodiscard]] Point2 selectCommandTarget(const Point2 desired_target,
                                           const bool hold_position) {
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
    if (sharp_turn_hold_active_) {
      return desired_target;
    }
    if (target_switch_hold_active_) {
      return target_switch_hold_target_;
    }
    if (hold_position) {
      return current_position_;
    }

    return desired_target;
  }

  [[nodiscard]] Point2 applyPathUpdateTargetHysteresis(const Point2 proposed_target,
                                                       const Point2 previous_target,
                                                       const bool had_previous_target) {
    last_commanded_target_hysteresis_used_ = false;
    last_commanded_target_hysteresis_delta_m_ =
        std::numeric_limits<double>::quiet_NaN();
    last_commanded_target_hysteresis_path_error_m_ =
        std::numeric_limits<double>::quiet_NaN();

    if (!path_update_target_hysteresis_pending_) {
      return proposed_target;
    }
    path_update_target_hysteresis_pending_ = false;

    if (!localPositionFresh()) {
      last_target_continuity_grid_available_ = false;
      last_target_continuity_decision_ = TargetContinuityDecision{};
      last_target_continuity_decision_.target = proposed_target;
      last_commanded_target_hysteresis_delta_m_ =
          had_previous_target ? distance(previous_target, proposed_target)
                              : std::numeric_limits<double>::quiet_NaN();
      RCLCPP_INFO(
          get_logger(),
          "Commanded target continuity switched to proposed target because local "
          "position is stale: reason=%s previous=(%.2f, %.2f) proposed=(%.2f, %.2f)",
          targetContinuityDecisionReasonName(last_target_continuity_decision_.reason),
          previous_target.x, previous_target.y, proposed_target.x, proposed_target.y);
      return proposed_target;
    }

    const std::optional<OccupancyGrid2D> prohibited_grid =
        prohibitedGridForTargetContinuity();
    last_target_continuity_grid_available_ = prohibited_grid.has_value();
    last_target_continuity_decision_ = decideTargetAfterReplan(
        path_points_, current_position_, previous_target, proposed_target,
        had_previous_target, path_update_raw_candidate_index_,
        path_update_raw_candidate_index_, commanded_target_hysteresis_m_,
        prohibited_grid.has_value() ? &*prohibited_grid : nullptr);
    if (last_target_continuity_decision_.target_waypoint_index_valid &&
        last_target_continuity_decision_.target_waypoint_index < path_points_.size()) {
      waypoint_index_ = last_target_continuity_decision_.target_waypoint_index;
    }
    last_commanded_target_hysteresis_delta_m_ =
        last_target_continuity_decision_.target_delta_m;
    last_commanded_target_hysteresis_path_error_m_ =
        last_target_continuity_decision_.path_error_m;
    last_commanded_target_hysteresis_used_ =
        last_target_continuity_decision_.reason ==
        TargetContinuityDecisionReason::kKeptPreviousTarget;

    RCLCPP_INFO(
        get_logger(),
        "Commanded target continuity decision after path update: reason=%s "
        "kept_previous=%s prohibited_grid_available=%s previous_safe=%s "
        "delta=%.2fm path_error=%.2fm tolerance=%.2fm previous=(%.2f, %.2f) "
        "proposed=(%.2f, %.2f) selected=(%.2f, %.2f) "
        "raw_candidate=%zu selected_waypoint=%zu selected_traversable=%s",
        targetContinuityDecisionReasonName(last_target_continuity_decision_.reason),
        last_commanded_target_hysteresis_used_ ? "true" : "false",
        last_target_continuity_grid_available_ ? "true" : "false",
        last_target_continuity_decision_.previous_target_safe ? "true" : "false",
        last_target_continuity_decision_.target_delta_m,
        last_target_continuity_decision_.path_error_m, commanded_target_hysteresis_m_,
        previous_target.x, previous_target.y, proposed_target.x, proposed_target.y,
        last_target_continuity_decision_.target.x,
        last_target_continuity_decision_.target.y,
        path_update_raw_candidate_index_ + 1U,
        last_target_continuity_decision_.target_waypoint_index_valid
            ? last_target_continuity_decision_.target_waypoint_index + 1U
            : 0U,
        last_target_continuity_decision_.selected_target_traversable ? "true"
                                                                     : "false");
    const Point2 selected_target = last_target_continuity_decision_.target;
    const double selected_delta_m = had_previous_target
                                        ? distance(previous_target, selected_target)
                                        : std::numeric_limits<double>::quiet_NaN();
    const double selected_angle_rad =
        targetSwitchAngleRad(previous_target, selected_target);
    if (shouldStartTargetSwitchHold(selected_target, had_previous_target,
                                    selected_delta_m, selected_angle_rad)) {
      startTargetSwitchHold(previous_target, selected_target, selected_delta_m,
                            selected_angle_rad);
      return target_switch_hold_target_;
    }
    return selected_target;
  }

  [[nodiscard]] bool shouldHoldPosition() const {
    return sharp_turn_hold_active_ || target_switch_hold_active_ ||
           !localPositionFresh() || !navigationAllowed() || !path_valid_ ||
           waypoint_index_ >= path_points_.size() || finalPathGoalReached();
  }

  [[nodiscard]] UpcomingTurn upcomingTurnAtWaypoint(const std::size_t index) const {
    if (!path_valid_ || !localPositionFresh()) {
      return UpcomingTurn{};
    }
    return drone_city_nav::upcomingTurnAtWaypoint(
        path_points_, index, current_position_, true, pathFollowerConfig());
  }

  [[nodiscard]] const char* pathSegmentTypeName(const double turn_angle_rad) const {
    if (!path_valid_) {
      return "no_path";
    }
    if (turn_angle_rad < 0.15) {
      return "straight";
    }
    if (turn_angle_rad < sharp_turn_hold_angle_rad_) {
      return "gentle_turn";
    }
    return "sharp_turn";
  }

  [[nodiscard]] const char* motionPhaseName(const bool hold_position) const noexcept {
    if (sharp_turn_hold_active_) {
      return "sharp_turn_hold";
    }
    if (target_switch_hold_active_) {
      return "target_switch_hold";
    }
    if (no_path_hold_target_valid_) {
      return "hold_no_path";
    }
    if (hold_position) {
      return "hold";
    }
    return "path_following";
  }

  [[nodiscard]] bool prohibitedGridFresh() const {
    if (!prohibited_grid_valid_ || last_prohibited_grid_update_ns_ <= 0) {
      return false;
    }
    if (max_clearance_grid_staleness_ns_ <= 0) {
      return true;
    }
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    return now_ns >= last_prohibited_grid_update_ns_ &&
           now_ns - last_prohibited_grid_update_ns_ <= max_clearance_grid_staleness_ns_;
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

  [[nodiscard]] std::optional<OccupancyGrid2D>
  prohibitedGridForTargetContinuity() const {
    if (!prohibitedGridFresh() || !(prohibited_grid_.info.resolution > 0.0F) ||
        prohibited_grid_.info.width == 0U || prohibited_grid_.info.height == 0U ||
        prohibited_grid_.info.width >
            static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        prohibited_grid_.info.height >
            static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
      return std::nullopt;
    }

    const auto width = static_cast<int>(prohibited_grid_.info.width);
    const auto height = static_cast<int>(prohibited_grid_.info.height);
    const std::size_t expected_data_size =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (prohibited_grid_.data.size() != expected_data_size) {
      return std::nullopt;
    }

    OccupancyGrid2D grid{GridBounds{
        prohibited_grid_.info.origin.position.x,
        prohibited_grid_.info.origin.position.y,
        static_cast<double>(prohibited_grid_.info.resolution), width, height}};
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const GridIndex cell{x, y};
        const std::size_t index =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x);
        if (prohibited_grid_.data[index] >= kInflatedOccupancyValue) {
          grid.setOccupied(cell);
        } else if (prohibited_grid_.data[index] == 0) {
          grid.setFree(cell);
        }
      }
    }
    return grid;
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

  [[nodiscard]] double sharpTurnHoldElapsedS() const {
    if (!sharp_turn_hold_active_) {
      return 0.0;
    }
    return std::max(0.0,
                    (get_clock()->now() - sharp_turn_hold_started_time_).seconds());
  }

  [[nodiscard]] double targetSwitchHoldElapsedS() const {
    if (!target_switch_hold_active_) {
      return 0.0;
    }
    return std::max(0.0,
                    (get_clock()->now() - target_switch_hold_started_time_).seconds());
  }

  [[nodiscard]] double attitudeAgeSeconds() const {
    if (last_attitude_update_ns_ <= 0) {
      return std::numeric_limits<double>::infinity();
    }
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    if (now_ns <= last_attitude_update_ns_) {
      return 0.0;
    }
    return static_cast<double>(now_ns - last_attitude_update_ns_) / 1.0e9;
  }

  [[nodiscard]] PathTrackingDiagnostics pathTrackingDiagnostics() const {
    PathTrackingDiagnostics diagnostics{};
    const auto projection = closestPathProjection();
    if (!projection.has_value() || path_points_.size() < 2U) {
      return diagnostics;
    }

    const std::size_t segment_index =
        std::min(projection->segment_start_index, path_points_.size() - 2U);
    const Point2 segment_start = path_points_[segment_index];
    const Point2 segment_end = path_points_[segment_index + 1U];
    const Point2 segment{segment_end.x - segment_start.x,
                         segment_end.y - segment_start.y};
    const double segment_length_m = std::hypot(segment.x, segment.y);
    if (!(segment_length_m > kTinyDistanceM)) {
      return diagnostics;
    }

    const Point2 relative{current_position_.x - segment_start.x,
                          current_position_.y - segment_start.y};
    const double signed_error_m =
        (segment.x * relative.y - segment.y * relative.x) / segment_length_m;
    const double path_heading_rad = std::atan2(segment.y, segment.x);

    diagnostics.valid = true;
    diagnostics.segment_start_index = segment_index;
    diagnostics.segment_t = projection->segment_t;
    diagnostics.cross_track_error_m = std::sqrt(projection->distance_sq);
    diagnostics.signed_cross_track_error_m = signed_error_m;
    diagnostics.path_heading_rad = path_heading_rad;
    diagnostics.heading_error_rad =
        normalizeAngle(current_heading_rad_ - path_heading_rad);
    diagnostics.projection = projection->point;
    return diagnostics;
  }

  [[nodiscard]] double estimateLocalClearanceM(const Point2 point) const {
    return estimateGridClearanceM(point, kInflatedOccupancyValue);
  }

  [[nodiscard]] NearestObstacleDiagnostic
  nearestObstacleDiagnostic(const Point2 point,
                            const std::int8_t min_occupancy_value) const {
    NearestObstacleDiagnostic diagnostic{};
    if (!prohibitedGridFresh() || !(prohibited_grid_.info.resolution > 0.0F) ||
        prohibited_grid_.info.width == 0U || prohibited_grid_.info.height == 0U) {
      return diagnostic;
    }

    const auto width = static_cast<int>(prohibited_grid_.info.width);
    const auto height = static_cast<int>(prohibited_grid_.info.height);
    const std::size_t expected_data_size =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (prohibited_grid_.data.size() != expected_data_size) {
      return diagnostic;
    }

    const double resolution = static_cast<double>(prohibited_grid_.info.resolution);
    const double origin_x = prohibited_grid_.info.origin.position.x;
    const double origin_y = prohibited_grid_.info.origin.position.y;
    const GridIndex center{
        static_cast<int>(std::floor((point.x - origin_x) / resolution)),
        static_cast<int>(std::floor((point.y - origin_y) / resolution))};
    if (center.x < 0 || center.y < 0 || center.x >= width || center.y >= height) {
      return diagnostic;
    }

    const int radius_cells =
        static_cast<int>(std::ceil(kLocalClearanceDiagnosticRadiusM / resolution));
    const int min_x = std::max(center.x - radius_cells, 0);
    const int max_x = std::min(center.x + radius_cells, width - 1);
    const int min_y = std::max(center.y - radius_cells, 0);
    const int max_y = std::min(center.y + radius_cells, height - 1);

    double nearest_distance_m = std::numeric_limits<double>::infinity();
    Point2 nearest_point{};
    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {
        const std::size_t data_index =
            static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(x);
        if (prohibited_grid_.data[data_index] < min_occupancy_value) {
          continue;
        }
        const Point2 cell_center{origin_x + (static_cast<double>(x) + 0.5) * resolution,
                                 origin_y +
                                     (static_cast<double>(y) + 0.5) * resolution};
        const double candidate_distance_m = distance(point, cell_center);
        if (candidate_distance_m < nearest_distance_m) {
          nearest_distance_m = candidate_distance_m;
          nearest_point = cell_center;
        }
      }
    }

    if (!std::isfinite(nearest_distance_m)) {
      return diagnostic;
    }

    const double bearing_map_rad =
        std::atan2(nearest_point.y - point.y, nearest_point.x - point.x);
    diagnostic.valid = true;
    diagnostic.clearance_m = nearest_distance_m;
    diagnostic.bearing_map_rad = bearing_map_rad;
    diagnostic.bearing_body_rad =
        normalizeAngle(bearing_map_rad - current_heading_rad_);
    diagnostic.bearing_body_deg = radiansToDegrees(diagnostic.bearing_body_rad);
    diagnostic.point = nearest_point;
    return diagnostic;
  }

  [[nodiscard]] double
  estimateGridClearanceM(const Point2 point,
                         const std::int8_t min_occupancy_value) const {
    if (!prohibitedGridFresh()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    return occupancyGridClearanceM(
        prohibited_grid_, point, kLocalClearanceDiagnosticRadiusM, min_occupancy_value);
  }

  void updateCommandDiagnostics(const Point2 target, const Point2 previous_target,
                                const bool had_previous_target,
                                const double commanded_yaw_rad) {
    last_commanded_target_distance_m_ = local_position_valid_
                                            ? distance(current_position_, target)
                                            : std::numeric_limits<double>::quiet_NaN();
    last_commanded_target_delta_m_ = had_previous_target
                                         ? distance(previous_target, target)
                                         : std::numeric_limits<double>::quiet_NaN();
    last_commanded_yaw_rad_ = commanded_yaw_rad;
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
    const UpcomingTurn upcoming_turn = upcomingTurnAtWaypoint(waypoint_index_);
    const double turn_angle_rad = upcoming_turn.angle_rad;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Offboard summary: local_position=%s pose_fresh=%s pose_age_s=%.2f "
        "altitude=%.2f nav_allowed=%s "
        "status=%s armed=%s offboard=%s path=%s hold=%s waypoint=%zu/%zu "
        "motion_phase=%s control_mode=%s path_segment=%s "
        "current=(%.2f, %.2f) target=(%.2f, %.2f) "
        "distance_to_target=%.2f distance_to_path_goal=%.2f "
        "distance_to_mission_goal=%.2f actual_speed=%.2f "
        "velocity_setpoint=(%.2f, %.2f, %.2f) velocity_setpoint_speed=%.2f "
        "speed_limit_reason=%s raw_speed_limit=%.2f accel_limited_speed=%.2f "
        "turn_target_speed=%.2f braking_distance=%.2f "
        "velocity_delta=%.2f cross_track_correction=%.2f altitude_error=%.2f "
        "turn[valid=%s index=%zu distance=%.2f angle=%.2f] "
        "sharp_turn_hold[active=%s waypoint=%zu elapsed=%.2f required=%.2f "
        "angle_threshold_rad=%.2f] "
        "target_switch_hold[active=%s elapsed=%.2f required=%.2f "
        "delta=%.2f angle=%.2f] "
        "local_clearance=%.2f "
        "target_continuity[reason=%s grid=%s previous_safe=%s]",
        local_position_valid_ ? "true" : "false", pose_fresh ? "true" : "false",
        pose_age_s, current_altitude_m_, navigationAllowed() ? "true" : "false",
        vehicle_status_valid_ ? "true" : "false", isArmed() ? "true" : "false",
        isOffboard() ? "true" : "false", path_valid_ ? "true" : "false",
        hold_position ? "true" : "false", path_valid_ ? waypoint_index_ + 1U : 0U,
        path_points_.size(), motionPhaseName(hold_position),
        offboardSetpointModeName(last_offboard_setpoint_mode_),
        pathSegmentTypeName(turn_angle_rad), current_position_.x, current_position_.y,
        target.x, target.y, target_distance, path_goal_distance, mission_goal_distance,
        current_speed_mps_, last_velocity_setpoint_.x, last_velocity_setpoint_.y,
        last_vertical_velocity_setpoint_mps_, last_velocity_setpoint_speed_mps_,
        velocitySetpointReasonName(last_velocity_plan_.reason),
        last_velocity_plan_.raw_speed_limit_mps,
        last_velocity_plan_.accel_limited_speed_mps,
        last_velocity_plan_.turn.target_turn_speed_mps,
        last_velocity_plan_.turn.braking_distance_m,
        last_velocity_plan_.velocity_delta_mps,
        last_velocity_plan_.cross_track_correction_mps, last_altitude_error_m_,
        upcoming_turn.valid ? "true" : "false",
        upcoming_turn.valid ? upcoming_turn.waypoint_index + 1U : 0U,
        upcoming_turn.distance_to_turn_m, turn_angle_rad,
        sharp_turn_hold_active_ ? "true" : "false",
        sharp_turn_hold_active_ ? sharp_turn_hold_waypoint_index_ + 1U : 0U,
        sharpTurnHoldElapsedS(), sharp_turn_hold_s_, sharp_turn_hold_angle_rad_,
        target_switch_hold_active_ ? "true" : "false", targetSwitchHoldElapsedS(),
        target_switch_hold_s_, last_target_switch_hold_delta_m_,
        last_target_switch_hold_angle_rad_, local_clearance_m,
        targetContinuityDecisionReasonName(last_target_continuity_decision_.reason),
        last_target_continuity_grid_available_ ? "true" : "false",
        last_target_continuity_decision_.previous_target_safe ? "true" : "false");
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
    const NearestObstacleDiagnostic nearest_obstacle =
        nearestObstacleDiagnostic(current_position_, kInflatedOccupancyValue);
    const double local_clearance_m = nearest_obstacle.valid
                                         ? nearest_obstacle.clearance_m
                                         : estimateLocalClearanceM(current_position_);
    const bool hold_position = shouldHoldPosition();
    const bool pose_fresh = localPositionFresh();
    const double pose_age_s = localPositionAgeSeconds();
    const double attitude_age_s = attitudeAgeSeconds();
    const double roll_deg = radiansToDegrees(current_attitude_.roll_rad);
    const double pitch_deg = radiansToDegrees(current_attitude_.pitch_rad);
    const double attitude_yaw_deg = radiansToDegrees(current_attitude_.yaw_rad);
    const double tilt_deg = radiansToDegrees(
        std::hypot(current_attitude_.roll_rad, current_attitude_.pitch_rad));
    const UpcomingTurn upcoming_turn = upcomingTurnAtWaypoint(waypoint_index_);
    const double turn_angle_rad = upcoming_turn.angle_rad;
    const PathTrackingDiagnostics path_tracking = pathTrackingDiagnostics();

    RCLCPP_INFO(
        get_logger(),
        "Drone telemetry: current=(%.2f, %.2f) pose_fresh=%s pose_age_s=%.2f "
        "altitude=%.2f heading=%.3f "
        "attitude[valid=%s age_s=%.2f roll=%.3frad pitch=%.3frad yaw=%.3frad "
        "roll_deg=%.1f pitch_deg=%.1f yaw_deg=%.1f tilt_deg=%.1f] "
        "velocity=(%.2f, %.2f) velocity_valid=%s actual_speed=%.2f "
        "target=(%.2f, %.2f) "
        "distance_to_target=%.2f distance_to_path_goal=%.2f "
        "distance_to_mission_goal=%.2f waypoint=%zu/%zu motion_phase=%s "
        "path_segment=%s local_clearance=%.2f "
        "turn[valid=%s index=%zu distance=%.2f angle=%.3f] "
        "sharp_turn_hold[active=%s waypoint=%zu elapsed=%.2f required=%.2f "
        "angle_threshold_rad=%.3f] "
        "target_switch_hold[active=%s elapsed=%.2f required=%.2f "
        "delta=%.2f angle=%.3f]",
        current_position_.x, current_position_.y, pose_fresh ? "true" : "false",
        pose_age_s, current_altitude_m_, current_heading_rad_,
        attitude_valid_ ? "true" : "false", attitude_age_s, current_attitude_.roll_rad,
        current_attitude_.pitch_rad, current_attitude_.yaw_rad, roll_deg, pitch_deg,
        attitude_yaw_deg, tilt_deg, current_velocity_.x, current_velocity_.y,
        current_velocity_valid_ ? "true" : "false", current_speed_mps_, target.x,
        target.y, target_distance, path_goal_distance, mission_goal_distance,
        path_valid_ ? waypoint_index_ + 1U : 0U, path_points_.size(),
        motionPhaseName(hold_position), pathSegmentTypeName(turn_angle_rad),
        local_clearance_m, upcoming_turn.valid ? "true" : "false",
        upcoming_turn.valid ? upcoming_turn.waypoint_index + 1U : 0U,
        upcoming_turn.distance_to_turn_m, turn_angle_rad,
        sharp_turn_hold_active_ ? "true" : "false",
        sharp_turn_hold_active_ ? sharp_turn_hold_waypoint_index_ + 1U : 0U,
        sharpTurnHoldElapsedS(), sharp_turn_hold_s_, sharp_turn_hold_angle_rad_,
        target_switch_hold_active_ ? "true" : "false", targetSwitchHoldElapsedS(),
        target_switch_hold_s_, last_target_switch_hold_delta_m_,
        last_target_switch_hold_angle_rad_);
    RCLCPP_INFO(get_logger(),
                "Drone path diagnostics: path_id[local_update=%" PRIu64
                " planner=%" PRIu64 " planner_seen=%s stamp_ns=%" PRIu64
                "] tracking[valid=%s cross_track=%.2f signed_cross_track=%.2f "
                "heading_error=%.3f path_heading=%.3f segment=%zu t=%.2f "
                "projection=(%.2f, %.2f)]",
                received_path_update_id_, latest_planner_path_id_,
                latest_planner_path_id_seen_ ? "true" : "false",
                last_received_path_stamp_ns_, path_tracking.valid ? "true" : "false",
                path_tracking.cross_track_error_m,
                path_tracking.signed_cross_track_error_m,
                path_tracking.heading_error_rad, path_tracking.path_heading_rad,
                path_tracking.segment_start_index, path_tracking.segment_t,
                path_tracking.projection.x, path_tracking.projection.y);
    RCLCPP_INFO(
        get_logger(),
        "Drone command diagnostics: command[target_delta=%.2f "
        "target_distance=%.2f yaw=%.3f target_hysteresis_used=%s "
        "target_hysteresis_delta=%.2f target_hysteresis_path_error=%.2f "
        "target_continuity_reason=%s target_continuity_grid=%s "
        "target_continuity_previous_safe=%s "
        "target_switch_hold_active=%s]",
        last_commanded_target_delta_m_, last_commanded_target_distance_m_,
        last_commanded_yaw_rad_,
        last_commanded_target_hysteresis_used_ ? "true" : "false",
        last_commanded_target_hysteresis_delta_m_,
        last_commanded_target_hysteresis_path_error_m_,
        targetContinuityDecisionReasonName(last_target_continuity_decision_.reason),
        last_target_continuity_grid_available_ ? "true" : "false",
        last_target_continuity_decision_.previous_target_safe ? "true" : "false",
        target_switch_hold_active_ ? "true" : "false");
    RCLCPP_INFO(get_logger(),
                "Drone velocity command diagnostics: control_mode=%s "
                "velocity_setpoint=(%.2f, %.2f, %.2f) velocity_setpoint_speed=%.2f "
                "speed_limit_reason=%s raw_speed_limit=%.2f accel_limited_speed=%.2f "
                "turn_target_speed=%.2f braking_distance=%.2f distance_to_turn=%.2f "
                "turn_angle=%.3f velocity_delta=%.2f cross_track_correction=%.2f "
                "altitude_error=%.2f tangent=(%.2f, %.2f) projection=(%.2f, %.2f)",
                offboardSetpointModeName(last_offboard_setpoint_mode_),
                last_velocity_setpoint_.x, last_velocity_setpoint_.y,
                last_vertical_velocity_setpoint_mps_, last_velocity_setpoint_speed_mps_,
                velocitySetpointReasonName(last_velocity_plan_.reason),
                last_velocity_plan_.raw_speed_limit_mps,
                last_velocity_plan_.accel_limited_speed_mps,
                last_velocity_plan_.turn.target_turn_speed_mps,
                last_velocity_plan_.turn.braking_distance_m,
                last_velocity_plan_.turn.distance_to_turn_m,
                last_velocity_plan_.turn.angle_rad,
                last_velocity_plan_.velocity_delta_mps,
                last_velocity_plan_.cross_track_correction_mps, last_altitude_error_m_,
                last_velocity_plan_.path_tangent.x, last_velocity_plan_.path_tangent.y,
                last_velocity_plan_.projection.x, last_velocity_plan_.projection.y);
    RCLCPP_INFO(get_logger(),
                "Drone obstacle diagnostics: nearest_obstacle[valid=%s clearance=%.2f "
                "bearing_map=%.3f bearing_body=%.3f bearing_body_deg=%.1f "
                "point=(%.2f, %.2f)]",
                nearest_obstacle.valid ? "true" : "false", nearest_obstacle.clearance_m,
                nearest_obstacle.bearing_map_rad, nearest_obstacle.bearing_body_rad,
                nearest_obstacle.bearing_body_deg, nearest_obstacle.point.x,
                nearest_obstacle.point.y);
    writeFlightBlackbox(now_ns, target, target_distance, path_goal_distance,
                        mission_goal_distance, local_clearance_m, pose_fresh,
                        pose_age_s, attitude_age_s, upcoming_turn, hold_position,
                        path_tracking, nearest_obstacle);
  }

  void writeFlightBlackbox(const std::int64_t now_ns, const Point2 target,
                           const double target_distance_m,
                           const double path_goal_distance_m,
                           const double mission_goal_distance_m,
                           const double local_clearance_m, const bool pose_fresh,
                           const double pose_age_s, const double attitude_age_s,
                           const UpcomingTurn& upcoming_turn, const bool hold_position,
                           const PathTrackingDiagnostics& path_tracking,
                           const NearestObstacleDiagnostic& nearest_obstacle) {
    if (!flight_blackbox_enabled_ || !flight_blackbox_stream_.is_open()) {
      return;
    }

    flight_blackbox_stream_ << "{\"time_ns\":" << now_ns;
    flight_blackbox_stream_ << ",\"path_id\":{\"local_update\":"
                            << received_path_update_id_
                            << ",\"planner\":" << latest_planner_path_id_
                            << ",\"planner_seen\":";
    writeJsonBool(flight_blackbox_stream_, latest_planner_path_id_seen_);
    flight_blackbox_stream_ << ",\"stamp_ns\":" << last_received_path_stamp_ns_ << "}";
    flight_blackbox_stream_ << ",\"pose\":{\"fresh\":";
    writeJsonBool(flight_blackbox_stream_, pose_fresh);
    flight_blackbox_stream_ << ",\"age_s\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, pose_age_s);
    flight_blackbox_stream_ << ",\"x\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, current_position_.x);
    flight_blackbox_stream_ << ",\"y\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, current_position_.y);
    flight_blackbox_stream_ << ",\"altitude_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, current_altitude_m_);
    flight_blackbox_stream_ << ",\"heading_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, current_heading_rad_);
    flight_blackbox_stream_ << "}";
    flight_blackbox_stream_ << ",\"attitude\":{\"valid\":";
    writeJsonBool(flight_blackbox_stream_, attitude_valid_);
    flight_blackbox_stream_ << ",\"age_s\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, attitude_age_s);
    flight_blackbox_stream_ << ",\"roll_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, current_attitude_.roll_rad);
    flight_blackbox_stream_ << ",\"pitch_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, current_attitude_.pitch_rad);
    flight_blackbox_stream_ << ",\"yaw_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, current_attitude_.yaw_rad);
    flight_blackbox_stream_ << ",\"tilt_deg\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          radiansToDegrees(std::hypot(current_attitude_.roll_rad,
                                                      current_attitude_.pitch_rad)));
    flight_blackbox_stream_ << "}";
    flight_blackbox_stream_ << ",\"velocity\":{\"valid\":";
    writeJsonBool(flight_blackbox_stream_, current_velocity_valid_);
    flight_blackbox_stream_ << ",\"x\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, current_velocity_.x);
    flight_blackbox_stream_ << ",\"y\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, current_velocity_.y);
    flight_blackbox_stream_ << ",\"speed_mps\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, current_speed_mps_);
    flight_blackbox_stream_ << "}";
    flight_blackbox_stream_ << ",\"target\":{\"x\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, target.x);
    flight_blackbox_stream_ << ",\"y\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, target.y);
    flight_blackbox_stream_ << ",\"distance_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, target_distance_m);
    flight_blackbox_stream_ << ",\"delta_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_commanded_target_delta_m_);
    flight_blackbox_stream_ << "}";
    flight_blackbox_stream_ << ",\"command\":{\"yaw_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_commanded_yaw_rad_);
    flight_blackbox_stream_ << ",\"target_hysteresis_used\":";
    writeJsonBool(flight_blackbox_stream_, last_commanded_target_hysteresis_used_);
    flight_blackbox_stream_ << ",\"target_hysteresis_delta_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          last_commanded_target_hysteresis_delta_m_);
    flight_blackbox_stream_ << ",\"target_hysteresis_path_error_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          last_commanded_target_hysteresis_path_error_m_);
    flight_blackbox_stream_ << ",\"target_continuity_reason\":\""
                            << targetContinuityDecisionReasonName(
                                   last_target_continuity_decision_.reason)
                            << "\",\"target_continuity_grid_available\":";
    writeJsonBool(flight_blackbox_stream_, last_target_continuity_grid_available_);
    flight_blackbox_stream_ << ",\"target_continuity_previous_safe\":";
    writeJsonBool(flight_blackbox_stream_,
                  last_target_continuity_decision_.previous_target_safe);
    flight_blackbox_stream_ << "}";
    flight_blackbox_stream_ << ",\"velocity_command\":{\"control_mode\":\""
                            << offboardSetpointModeName(last_offboard_setpoint_mode_)
                            << "\",\"setpoint_x\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_setpoint_.x);
    flight_blackbox_stream_ << ",\"setpoint_y\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_setpoint_.y);
    flight_blackbox_stream_ << ",\"setpoint_z\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          last_vertical_velocity_setpoint_mps_);
    flight_blackbox_stream_ << ",\"setpoint_speed_mps\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_setpoint_speed_mps_);
    flight_blackbox_stream_ << ",\"speed_limit_reason\":\""
                            << velocitySetpointReasonName(last_velocity_plan_.reason)
                            << "\",\"raw_speed_limit_mps\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          last_velocity_plan_.raw_speed_limit_mps);
    flight_blackbox_stream_ << ",\"accel_limited_speed_mps\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          last_velocity_plan_.accel_limited_speed_mps);
    flight_blackbox_stream_ << ",\"turn_target_speed_mps\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          last_velocity_plan_.turn.target_turn_speed_mps);
    flight_blackbox_stream_ << ",\"braking_distance_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          last_velocity_plan_.turn.braking_distance_m);
    flight_blackbox_stream_ << ",\"distance_to_turn_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          last_velocity_plan_.turn.distance_to_turn_m);
    flight_blackbox_stream_ << ",\"turn_angle_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.turn.angle_rad);
    flight_blackbox_stream_ << ",\"velocity_delta_mps\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          last_velocity_plan_.velocity_delta_mps);
    flight_blackbox_stream_ << ",\"cross_track_correction_mps\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          last_velocity_plan_.cross_track_correction_mps);
    flight_blackbox_stream_ << ",\"altitude_error_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_altitude_error_m_);
    flight_blackbox_stream_ << ",\"path_tangent_x\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.path_tangent.x);
    flight_blackbox_stream_ << ",\"path_tangent_y\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.path_tangent.y);
    flight_blackbox_stream_ << ",\"projection_x\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.projection.x);
    flight_blackbox_stream_ << ",\"projection_y\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_velocity_plan_.projection.y);
    flight_blackbox_stream_ << "}";
    flight_blackbox_stream_ << ",\"path\":{\"valid\":";
    writeJsonBool(flight_blackbox_stream_, path_valid_);
    flight_blackbox_stream_ << ",\"waypoint_index\":"
                            << (path_valid_ ? waypoint_index_ + 1U : 0U)
                            << ",\"waypoint_count\":" << path_points_.size()
                            << ",\"path_goal_distance_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, path_goal_distance_m);
    flight_blackbox_stream_ << ",\"mission_goal_distance_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, mission_goal_distance_m);
    flight_blackbox_stream_ << ",\"turn_angle_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, upcoming_turn.angle_rad);
    flight_blackbox_stream_ << ",\"turn_valid\":";
    writeJsonBool(flight_blackbox_stream_, upcoming_turn.valid);
    flight_blackbox_stream_ << ",\"turn_waypoint_index\":"
                            << (upcoming_turn.valid ? upcoming_turn.waypoint_index + 1U
                                                    : 0U);
    flight_blackbox_stream_ << ",\"turn_distance_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, upcoming_turn.distance_to_turn_m);
    flight_blackbox_stream_ << ",\"turn_point_x\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, upcoming_turn.turn_point.x);
    flight_blackbox_stream_ << ",\"turn_point_y\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, upcoming_turn.turn_point.y);
    flight_blackbox_stream_ << ",\"segment_type\":\""
                            << pathSegmentTypeName(upcoming_turn.angle_rad) << "\"";
    flight_blackbox_stream_ << ",\"tracking\":{\"valid\":";
    writeJsonBool(flight_blackbox_stream_, path_tracking.valid);
    flight_blackbox_stream_ << ",\"cross_track_error_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.cross_track_error_m);
    flight_blackbox_stream_ << ",\"signed_cross_track_error_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_,
                          path_tracking.signed_cross_track_error_m);
    flight_blackbox_stream_ << ",\"heading_error_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.heading_error_rad);
    flight_blackbox_stream_ << ",\"path_heading_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.path_heading_rad);
    flight_blackbox_stream_ << ",\"segment_start_index\":"
                            << path_tracking.segment_start_index << ",\"segment_t\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.segment_t);
    flight_blackbox_stream_ << ",\"projection_x\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.projection.x);
    flight_blackbox_stream_ << ",\"projection_y\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, path_tracking.projection.y);
    flight_blackbox_stream_ << "}}";
    flight_blackbox_stream_ << ",\"control\":{\"motion_phase\":\""
                            << motionPhaseName(hold_position)
                            << "\",\"sharp_turn_hold_active\":";
    writeJsonBool(flight_blackbox_stream_, sharp_turn_hold_active_);
    flight_blackbox_stream_ << ",\"sharp_turn_hold_waypoint_index\":"
                            << (sharp_turn_hold_active_
                                    ? sharp_turn_hold_waypoint_index_ + 1U
                                    : 0U);
    flight_blackbox_stream_ << ",\"sharp_turn_hold_elapsed_s\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, sharpTurnHoldElapsedS());
    flight_blackbox_stream_ << ",\"sharp_turn_hold_required_s\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, sharp_turn_hold_s_);
    flight_blackbox_stream_ << ",\"sharp_turn_hold_angle_threshold_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, sharp_turn_hold_angle_rad_);
    flight_blackbox_stream_ << ",\"target_switch_hold_active\":";
    writeJsonBool(flight_blackbox_stream_, target_switch_hold_active_);
    flight_blackbox_stream_ << ",\"target_switch_hold_elapsed_s\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, targetSwitchHoldElapsedS());
    flight_blackbox_stream_ << ",\"target_switch_hold_required_s\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, target_switch_hold_s_);
    flight_blackbox_stream_ << ",\"target_switch_hold_delta_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_target_switch_hold_delta_m_);
    flight_blackbox_stream_ << ",\"target_switch_hold_angle_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, last_target_switch_hold_angle_rad_);
    flight_blackbox_stream_ << ",\"target_switch_hold_delta_threshold_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, target_switch_hold_delta_m_);
    flight_blackbox_stream_ << ",\"target_switch_hold_angle_threshold_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, target_switch_hold_angle_rad_);
    flight_blackbox_stream_ << "}";
    flight_blackbox_stream_ << ",\"obstacle\":{\"local_clearance_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, local_clearance_m);
    flight_blackbox_stream_ << ",\"nearest_valid\":";
    writeJsonBool(flight_blackbox_stream_, nearest_obstacle.valid);
    flight_blackbox_stream_ << ",\"nearest_clearance_m\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, nearest_obstacle.clearance_m);
    flight_blackbox_stream_ << ",\"bearing_map_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, nearest_obstacle.bearing_map_rad);
    flight_blackbox_stream_ << ",\"bearing_body_rad\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, nearest_obstacle.bearing_body_rad);
    flight_blackbox_stream_ << ",\"bearing_body_deg\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, nearest_obstacle.bearing_body_deg);
    flight_blackbox_stream_ << ",\"point_x\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, nearest_obstacle.point.x);
    flight_blackbox_stream_ << ",\"point_y\":";
    writeJsonNumberOrNull(flight_blackbox_stream_, nearest_obstacle.point.y);
    flight_blackbox_stream_ << "}}\n";
  }

  [[nodiscard]] Point2 loggedTarget() const {
    if (last_published_target_valid_) {
      return last_published_target_;
    }
    if (commanded_target_valid_) {
      return commanded_target_;
    }
    return currentTarget();
  }

  nav_msgs::msg::OccupancyGrid prohibited_grid_;
  px4_msgs::msg::VehicleStatus vehicle_status_;
  AttitudeEuler current_attitude_{};
  Point2 current_position_{};
  Point2 current_velocity_{};
  Point2 no_path_hold_target_{};
  Point2 sharp_turn_hold_target_{};
  Point2 target_switch_hold_target_{};
  Point2 target_switch_pending_target_{};
  Point2 takeoff_hold_target_{};
  Point2 commanded_target_{};
  Point2 last_published_target_{};
  Point2 last_velocity_setpoint_{};
  Point2 mission_goal_{85.0, 0.0};
  Point2 px4_local_origin_{};
  double current_heading_rad_{0.0};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double cruise_altitude_m_{12.0};
  double min_navigation_altitude_m_{0.0};
  double takeoff_hover_s_{2.0};
  double acceptance_radius_m_{1.5};
  double turn_preview_distance_m_{32.0};
  double sharp_turn_hold_angle_rad_{std::numbers::pi / 3.0};
  double sharp_turn_hold_s_{2.0};
  double target_switch_hold_s_{2.0};
  double target_switch_hold_delta_m_{5.0};
  double target_switch_hold_angle_rad_{std::numbers::pi / 4.0};
  double altitude_hold_kp_{0.5};
  double max_vertical_speed_mps_{2.0};
  double path_switch_hysteresis_m_{3.0};
  double path_continuity_reuse_radius_m_{6.0};
  double path_continuity_max_target_distance_m_{20.0};
  double commanded_target_hysteresis_m_{0.5};
  double command_resend_period_s_{2.0};
  double hold_x_m_{0.0};
  double hold_y_m_{0.0};
  double current_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_commanded_target_delta_m_{std::numeric_limits<double>::quiet_NaN()};
  double last_commanded_target_distance_m_{std::numeric_limits<double>::quiet_NaN()};
  double last_commanded_yaw_rad_{std::numeric_limits<double>::quiet_NaN()};
  double last_commanded_target_hysteresis_delta_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double last_commanded_target_hysteresis_path_error_m_{
      std::numeric_limits<double>::quiet_NaN()};
  double last_target_switch_hold_delta_m_{std::numeric_limits<double>::quiet_NaN()};
  double last_target_switch_hold_angle_rad_{std::numeric_limits<double>::quiet_NaN()};
  double last_velocity_setpoint_speed_mps_{0.0};
  double last_vertical_velocity_setpoint_mps_{0.0};
  double last_altitude_error_m_{std::numeric_limits<double>::quiet_NaN()};
  std::int64_t max_clearance_grid_staleness_ns_{1'500'000'000};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t telemetry_log_period_ns_{500'000'000};
  std::int64_t last_prohibited_grid_update_ns_{0};
  std::int64_t last_attitude_update_ns_{0};
  std::int64_t last_local_position_update_ns_{0};
  std::int64_t last_telemetry_log_ns_{0};
  std::uint64_t latest_planner_path_id_{0U};
  std::uint64_t received_path_update_id_{0U};
  std::uint64_t last_received_path_stamp_ns_{0U};
  std::size_t waypoint_index_{0U};
  std::size_t sharp_turn_hold_waypoint_index_{0U};
  std::size_t path_update_raw_candidate_index_{0U};
  std::optional<std::size_t> sharp_turn_hold_completed_index_;
  int warmup_setpoints_{20};
  int setpoint_counter_{0};
  bool path_valid_{false};
  bool local_position_valid_{false};
  bool vehicle_status_valid_{false};
  bool attitude_valid_{false};
  bool altitude_valid_{false};
  bool local_position_seen_{false};
  bool auto_arm_{true};
  bool auto_offboard_{true};
  bool emergency_stop_requested_{false};
  bool prohibited_grid_valid_{false};
  bool prohibited_grid_seen_logged_{false};
  bool current_velocity_valid_{false};
  bool no_path_hold_target_valid_{false};
  bool sharp_turn_hold_active_{false};
  bool target_switch_hold_active_{false};
  bool cruise_velocity_control_enabled_{true};
  bool takeoff_hold_target_valid_{false};
  bool commanded_target_valid_{false};
  bool last_published_target_valid_{false};
  bool face_target_yaw_{false};
  bool navigation_altitude_reached_{false};
  bool navigation_started_{false};
  bool latest_planner_path_id_seen_{false};
  bool flight_blackbox_enabled_{true};
  bool path_update_target_hysteresis_pending_{false};
  bool last_commanded_target_hysteresis_used_{false};
  bool last_target_continuity_grid_available_{false};
  std::uint8_t target_system_{1U};
  std::uint8_t target_component_{1U};
  std::uint8_t source_system_{1U};
  std::uint16_t source_component_{1U};
  int last_logged_arming_state_{-1};
  int last_logged_nav_state_{-1};
  std::size_t last_logged_path_size_{std::numeric_limits<std::size_t>::max()};
  Point2 last_logged_path_first_{};
  Point2 last_logged_path_last_{};
  TargetContinuityDecision last_target_continuity_decision_{};
  VelocityFollowerConfig velocity_follower_config_{};
  VelocityFollowerState velocity_follower_state_{};
  VelocitySetpointPlan last_velocity_plan_{};
  bool last_velocity_plan_valid_{false};
  OffboardSetpointMode last_offboard_setpoint_mode_{
      OffboardSetpointMode::kPositionHold};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time navigation_altitude_reached_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time sharp_turn_hold_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_switch_hold_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_plan_time_{0, 0, RCL_ROS_TIME};
  std::string flight_blackbox_path_{"log/offboard_blackbox.jsonl"};
  std::ofstream flight_blackbox_stream_;
  std::vector<Point2> path_points_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr path_id_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr prohibited_grid_sub_;
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
