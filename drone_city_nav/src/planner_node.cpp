#include "drone_city_nav/current_lidar_overlay.hpp"
#include "drone_city_nav/grid_overlay.hpp"
#include "drone_city_nav/lidar_motion_compensation.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/path_smoothing.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/planner_node_config.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/static_map_debug.hpp"
#include "drone_city_nav/static_map_source.hpp"
#include "drone_city_nav/trajectory_diagnostics_io.hpp"
#include "drone_city_nav/trajectory_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] double radiansToDegrees(const double radians) noexcept {
  return radians * 180.0 / std::numbers::pi;
}

constexpr double kPublishedPathCollinearityToleranceM = 0.05;
constexpr double kGroundDebugZ = 0.05;

[[nodiscard]] std::uint64_t
stampNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000U;
  return static_cast<std::uint64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::uint64_t>(stamp.nanosec);
}

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

struct PublishedPathSafetySummary {
  std::size_t segments{0U};
  std::size_t non_traversable_segments{0U};
  std::size_t escape_segments{0U};
  std::size_t first_non_traversable_segment{0U};
  Point2 first_non_traversable_start{};
  Point2 first_non_traversable_end{};
  bool has_non_traversable_segment{false};
};

[[nodiscard]] std::vector<Point2>
trajectorySamplePoints(const std::span<const TrajectoryPointSample> samples) {
  std::vector<Point2> points;
  points.reserve(samples.size());
  for (const TrajectoryPointSample& sample : samples) {
    points.push_back(sample.point);
  }
  return points;
}

enum class PathPublicationReason : std::uint8_t {
  kComputedPath,
  kHoldNoPose,
  kHoldNoPlanningGrid,
  kHoldInvalidPath,
  kHoldAfterPlanningFailure,
};

[[nodiscard]] const char*
pathPublicationReasonName(const PathPublicationReason reason) noexcept {
  switch (reason) {
    case PathPublicationReason::kComputedPath:
      return "computed_path";
    case PathPublicationReason::kHoldNoPose:
      return "hold_no_pose";
    case PathPublicationReason::kHoldNoPlanningGrid:
      return "hold_no_planning_grid";
    case PathPublicationReason::kHoldInvalidPath:
      return "hold_invalid_path";
    case PathPublicationReason::kHoldAfterPlanningFailure:
      return "hold_after_planning_failure";
  }

  return "unknown";
}

} // namespace

class PlannerNode final : public rclcpp::Node {
public:
  PlannerNode()
      : Node{"planner_node"} {
    const PlannerNodeConfig config = loadPlannerNodeConfig(*this);
    applyConfig(config);
    if (config.initial_pose.use_until_px4) {
      current_pose_ =
          Pose2{config.initial_pose.position, config.initial_pose.heading_rad};
      pose_valid_ = true;
      last_pose_update_ns_ = get_clock()->now().nanoseconds();
    }

    const auto sensor_qos = rclcpp::SensorDataQoS{};
    memory_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        config.topics.obstacle_memory_grid, rclcpp::QoS{1}.transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          onMemoryGrid(*msg);
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        config.topics.lidar, sensor_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { onScan(*msg); });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        config.topics.local_position, sensor_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          onLocalPosition(*msg);
        });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        config.topics.attitude, sensor_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
          onAttitude(*msg);
        });

    prohibited_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        config.topics.prohibited_grid, rclcpp::QoS{1}.transient_local());
    static_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        config.topics.static_map_grid, rclcpp::QoS{1}.transient_local());
    static_map_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        config.topics.static_map_points, rclcpp::QoS{1}.transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(config.topics.path,
                                                      rclcpp::QoS{1}.reliable());
    path_id_pub_ = create_publisher<std_msgs::msg::UInt64>(config.topics.path_id,
                                                           rclcpp::QoS{1}.reliable());
    trajectory_diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
        config.topics.trajectory_diagnostics, rclcpp::QoS{1}.reliable());
    waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        config.topics.current_waypoint, rclcpp::QoS{1}.reliable());

    loadConfiguredStaticMap();
    if (static_map_debug_publish_period_s_ > 0.0) {
      static_map_debug_timer_ = create_wall_timer(
          std::chrono::duration<double>{static_map_debug_publish_period_s_},
          [this]() { republishStaticMapDebug(); });
    }
    timer_ = create_wall_timer(
        std::chrono::duration<double>{
            std::max(0.05, config.timing.path_prohibited_intersection_check_period_s)},
        [this]() { checkCurrentPathAndPublish(); });

    RCLCPP_INFO(get_logger(),
                "Planner ready: start=(%.1f, %.1f) goal=(%.1f, %.1f) "
                "inflation=%.2fm",
                start_.x, start_.y, goal_.x, goal_.y, inflation_radius_m_);
    RCLCPP_INFO(get_logger(),
                "Planner subscriptions: obstacle_memory_grid='%s' local_position='%s' "
                "attitude='%s'",
                config.topics.obstacle_memory_grid.c_str(),
                config.topics.local_position.c_str(), config.topics.attitude.c_str());
    RCLCPP_INFO(get_logger(), "Planner publications: path='%s' path_id='%s'",
                config.topics.path.c_str(), config.topics.path_id.c_str());
    RCLCPP_INFO(get_logger(),
                "Planning grid contract: raw_sources=[static,memory,current_lidar] "
                "inflation_owner=planner prohibited_output='%s'",
                config.topics.prohibited_grid.c_str());
    RCLCPP_INFO(
        get_logger(),
        "Planner trajectory pipeline: output_path=final_racing_trajectory "
        "rough_astar_scope=internal_seed "
        "speed[cruise=%.2fmps min_turn=%.2fmps max_accel=%.2fmps2 "
        "max_decel=%.2fmps2 max_lateral=%.2fmps2 profile_decel=%.2fmps2 "
        "sample_step=%.2fm] "
        "corridor[max_radius=%.2fm sample_step=%.2fm safety_margin=%.2fm "
        "center_recovery_max=%.2fm "
        "lateral_window=%.2fm lateral_ratio=%.2f lateral_margin=%.2fm] "
        "racing_line[iterations=%zu optimizer_sample_step=%.2fm offset_step=%.2fm "
        "min_step=%.2fm weights(length=%.3f time=%.2f "
        "curvature=%.2f curvature_change=%.2f offset_change=%.2f "
        "offset_second=%.2f center=%.3f edge=%.2f edge_margin=%.2fm "
        "max_length_ratio=%.2f)] "
        "turn_smoothing[trigger_heading=%.1fdeg trigger_radius=%.2fm "
        "entry=%.2fm exit=%.2fm sample_step=%.2fm outer_bias=%.2f "
        "outer_shift=[%.2f, %.2f] corridor_margin=%.2fm max_length_ratio=%.2f "
        "max_passes=%zu]",
        trajectory_planner_config_.speed_profile.cruise_speed_mps,
        trajectory_planner_config_.speed_profile.min_turn_speed_mps,
        trajectory_planner_config_.speed_profile.max_accel_mps2,
        trajectory_planner_config_.speed_profile.max_decel_mps2,
        trajectory_planner_config_.speed_profile.max_lateral_accel_mps2,
        trajectory_planner_config_.speed_profile.speed_profile_decel_mps2,
        trajectory_planner_config_.speed_profile.speed_profile_sample_step_m,
        trajectory_planner_config_.corridor.max_radius_m,
        trajectory_planner_config_.corridor.sample_step_m,
        trajectory_planner_config_.corridor.safety_margin_m,
        trajectory_planner_config_.corridor.center_recovery_max_m,
        trajectory_planner_config_.corridor.lateral_limit_window_m,
        trajectory_planner_config_.corridor.lateral_limit_ratio,
        trajectory_planner_config_.corridor.lateral_limit_margin_m,
        trajectory_planner_config_.racing_line.max_iterations,
        trajectory_planner_config_.racing_line.optimizer_sample_step_m,
        trajectory_planner_config_.racing_line.initial_offset_step_m,
        trajectory_planner_config_.racing_line.min_offset_step_m,
        trajectory_planner_config_.racing_line.weight_length,
        trajectory_planner_config_.racing_line.weight_time,
        trajectory_planner_config_.racing_line.weight_curvature,
        trajectory_planner_config_.racing_line.weight_curvature_change,
        trajectory_planner_config_.racing_line.weight_offset_change,
        trajectory_planner_config_.racing_line.weight_offset_second_change,
        trajectory_planner_config_.racing_line.weight_center_bias,
        trajectory_planner_config_.racing_line.weight_edge_margin,
        trajectory_planner_config_.racing_line.desired_edge_margin_m,
        trajectory_planner_config_.racing_line.max_length_ratio,
        radiansToDegrees(
            trajectory_planner_config_.turn_smoothing.trigger_heading_delta_rad),
        trajectory_planner_config_.turn_smoothing.trigger_min_radius_m,
        trajectory_planner_config_.turn_smoothing.entry_distance_m,
        trajectory_planner_config_.turn_smoothing.exit_distance_m,
        trajectory_planner_config_.turn_smoothing.sample_step_m,
        trajectory_planner_config_.turn_smoothing.outer_bias_ratio,
        trajectory_planner_config_.turn_smoothing.min_outer_shift_m,
        trajectory_planner_config_.turn_smoothing.max_outer_shift_m,
        trajectory_planner_config_.turn_smoothing.min_corridor_margin_m,
        trajectory_planner_config_.turn_smoothing.max_length_ratio,
        trajectory_planner_config_.turn_smoothing.max_passes);
    RCLCPP_INFO(
        get_logger(),
        "Planner obstacle sources: static=%s memory=%s current_lidar=%s "
        "static_path='%s' fallback_grid=%dx%d@%.2fm origin=(%.2f, %.2f)",
        use_static_map_ ? "true" : "false", use_obstacle_memory_ ? "true" : "false",
        use_current_lidar_obstacles_ ? "true" : "false",
        static_map_resolved_path_.string().c_str(), fallback_grid_bounds_.width_cells,
        fallback_grid_bounds_.height_cells, fallback_grid_bounds_.resolution_m,
        fallback_grid_bounds_.origin_x, fallback_grid_bounds_.origin_y);
    RCLCPP_INFO(get_logger(),
                "Planner lidar overlay: enabled=%s topic='%s' max_range=%.2f "
                "max_staleness=%.2fs yaw_source=%s compensate_attitude=%s "
                "motion_compensation=%s pose_latency=%.3fs "
                "lidar_z_offset=%.2f "
                "projected_altitude_range=[%.2f, %.2f] "
                "lidar_mount_rpy=(%.3f, %.3f, %.3f)",
                use_current_lidar_obstacles_ ? "true" : "false",
                config.topics.lidar.c_str(), max_lidar_range_m_,
                static_cast<double>(max_current_lidar_staleness_ns_) / 1.0e9,
                use_px4_heading_for_scan_ ? "px4_heading" : "initial_map_aligned",
                compensate_lidar_attitude_ ? "true" : "false",
                motion_compensate_lidar_pose_ ? "true" : "false", lidar_pose_latency_s_,
                lidar_z_offset_m_, min_projected_lidar_altitude_m_,
                max_projected_lidar_altitude_m_, lidar_mount_roll_rad_,
                lidar_mount_pitch_rad_, lidar_mount_yaw_rad_);
    RCLCPP_INFO(get_logger(),
                "Planner path policy: stable_path_reuse=%s "
                "stable_goal_tolerance=%.2fm",
                stable_path_reuse_enabled_ ? "true" : "false",
                stable_path_goal_tolerance_m_);
    RCLCPP_INFO(get_logger(),
                "Planner path preference: astar_heuristic_weight=%.2f "
                "astar_turn_weight=%.2f "
                "evasive_maneuvering=%s evasive_straight_weight=%.2f "
                "initial_heading_bias=%s initial_heading_min_speed=%.2fm/s "
                "initial_heading_weight=%.2f",
                astar_config_.heuristic_weight, astar_config_.turn_cost_weight,
                astar_config_.evasive_maneuvering_enabled ? "true" : "false",
                astar_config_.evasive_maneuvering_straight_cost_weight,
                astar_config_.initial_heading_bias_enabled ? "true" : "false",
                astar_config_.initial_heading_bias_min_speed_mps,
                astar_config_.initial_heading_bias_weight);
  }

private:
  void applyConfig(const PlannerNodeConfig& config) {
    frame_id_ = config.frame_id;
    start_ = config.start;
    goal_ = config.goal;
    inflation_radius_m_ = config.inflation_radius_m;
    max_pose_staleness_ns_ = config.timing.max_pose_staleness_ns;
    stable_path_reuse_enabled_ = config.fallback.stable_path_reuse_enabled;
    stable_path_goal_tolerance_m_ = config.planner_core.stable_path_goal_tolerance_m;
    memory_occupied_value_ = config.memory_grid.occupied_value;
    memory_free_value_ = config.memory_grid.free_value;
    use_static_map_ = config.static_map.enabled;
    use_obstacle_memory_ = config.planning_grid_builder.use_obstacle_memory;
    static_map_path_param_ = config.static_map.configured_path.string();
    static_map_min_blocking_height_m_ = config.static_map.min_blocking_height_m;
    fallback_grid_bounds_ = config.planning_grid_builder.fallback_bounds;
    use_current_lidar_obstacles_ =
        config.planning_grid_builder.use_current_lidar_obstacles;
    max_current_lidar_staleness_ns_ = config.timing.max_current_lidar_staleness_ns;
    max_lidar_range_m_ = config.lidar_projection.max_lidar_range_m;
    range_hit_epsilon_m_ = config.lidar_projection.range_hit_epsilon_m;
    scan_yaw_offset_rad_ = config.lidar_projection.scan_yaw_offset_rad;
    use_px4_heading_for_scan_ = config.current_lidar.use_px4_heading_for_scan;
    motion_compensate_lidar_pose_ = config.current_lidar.motion_compensate_lidar_pose;
    lidar_pose_latency_s_ = config.current_lidar.lidar_pose_latency_s;
    compensate_lidar_attitude_ = config.lidar_projection.compensate_attitude;
    lidar_mount_roll_rad_ = config.lidar_projection.lidar_mount_roll_rad;
    lidar_mount_pitch_rad_ = config.lidar_projection.lidar_mount_pitch_rad;
    lidar_mount_yaw_rad_ = config.lidar_projection.lidar_mount_yaw_rad;
    lidar_z_offset_m_ = config.lidar_projection.lidar_z_offset_m;
    min_projected_lidar_altitude_m_ = config.lidar_projection.min_projected_altitude_m;
    max_projected_lidar_altitude_m_ = config.lidar_projection.max_projected_altitude_m;
    astar_config_ = config.planner_core.astar;
    trajectory_planner_config_ = config.trajectory_planner;
    initial_heading_rad_ = config.initial_pose.heading_rad;
    px4_local_origin_ = config.initial_pose.px4_local_origin;
    static_map_debug_publish_period_s_ =
        config.timing.static_map_debug_publish_period_s;
    planner_core_.setConfig(config.planner_core);
  }

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
    if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
      invalidateCurrentPose();
      current_velocity_ = Point2{};
      current_speed_mps_ = std::numeric_limits<double>::quiet_NaN();
      current_velocity_valid_ = false;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planner invalidated cached pose after invalid PX4 local position: "
          "xy_valid=%s x=%.2f y=%.2f",
          msg.xy_valid ? "true" : "false", static_cast<double>(msg.x),
          static_cast<double>(msg.y));
      return;
    }

    current_pose_.position = Point2{static_cast<double>(msg.x) + px4_local_origin_.x,
                                    static_cast<double>(msg.y) + px4_local_origin_.y};
    if (msg.heading_good_for_control && std::isfinite(msg.heading)) {
      current_pose_.yaw_rad = static_cast<double>(msg.heading);
    }
    if (msg.z_valid && std::isfinite(msg.z)) {
      current_altitude_m_ = -static_cast<double>(msg.z);
      altitude_valid_ = true;
    } else {
      altitude_valid_ = false;
    }
    if (msg.v_xy_valid && std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
      current_velocity_ =
          Point2{static_cast<double>(msg.vx), static_cast<double>(msg.vy)};
      current_speed_mps_ = std::hypot(current_velocity_.x, current_velocity_.y);
      current_velocity_valid_ = true;
    } else {
      current_velocity_ = Point2{};
      current_speed_mps_ = std::numeric_limits<double>::quiet_NaN();
      current_velocity_valid_ = false;
    }
    pose_valid_ = true;
    last_pose_update_ns_ = get_clock()->now().nanoseconds();

    if (!local_position_seen_) {
      local_position_seen_ = true;
      const double altitude_m = (msg.z_valid && std::isfinite(msg.z))
                                    ? -static_cast<double>(msg.z)
                                    : std::numeric_limits<double>::quiet_NaN();
      RCLCPP_INFO(get_logger(),
                  "First valid PX4 local position: x=%.2f y=%.2f z=%.2f "
                  "altitude=%.2f yaw=%.2f distance_to_start=%.2f "
                  "distance_to_goal=%.2f",
                  current_pose_.position.x, current_pose_.position.y,
                  static_cast<double>(msg.z), altitude_m, current_pose_.yaw_rad,
                  distance(current_pose_.position, start_),
                  distance(current_pose_.position, goal_));
    }
  }

  void onMemoryGrid(const nav_msgs::msg::OccupancyGrid& msg) {
    RawOccupancyGridFromRosResult converted = rawOccupancyGridFromRos(
        msg, RawOccupancyGridFromRosConfig{memory_occupied_value_, memory_free_value_});
    if (!converted.grid.has_value()) {
      if (converted.error == OccupancyGridFromRosError::kMismatchedDataSize) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Ignoring obstacle memory grid with mismatched data size: expected=%zu "
            "got=%zu",
            converted.expected_data_size, converted.actual_data_size);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Ignoring invalid obstacle memory grid metadata");
      }
      return;
    }

    memory_grid_ = std::move(*converted.grid);
    if (converted.intermediate_value_cells > 0U) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignored intermediate values while reading raw obstacle memory grid: "
          "intermediate_cells=%zu. Raw memory topics must use only occupied=%d, "
          "free=%d, or unknown=-1 values.",
          converted.intermediate_value_cells, memory_occupied_value_,
          memory_free_value_);
    }
    if (!memory_grid_seen_) {
      memory_grid_seen_ = true;
      RCLCPP_INFO(
          get_logger(),
          "First obstacle memory grid: size=%dx%d resolution=%.2f origin=(%.2f, "
          "%.2f)",
          memory_grid_->width(), memory_grid_->height(), memory_grid_->resolution(),
          memory_grid_->originX(), memory_grid_->originY());
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan& msg) {
    last_scan_ = msg;
    scan_seen_ = true;
    last_scan_update_ns_ = get_clock()->now().nanoseconds();
    last_scan_projection_pose_valid_ = pose_valid_;
    if (last_scan_projection_pose_valid_) {
      last_scan_projection_pose_ = currentLidarProjectionPose();
      const LidarPoseMotionCompensationResult motion_compensation =
          compensateLidarPoseForLatency(
              last_scan_projection_pose_.position, current_velocity_,
              motion_compensate_lidar_pose_, current_velocity_valid_,
              currentLidarPoseReceiveLagSeconds(last_scan_update_ns_,
                                                last_pose_update_ns_),
              lidar_pose_latency_s_);
      last_scan_projection_pose_.position = motion_compensation.position;
      last_scan_pose_lag_s_ = motion_compensation.pose_lag_s;
      last_scan_pose_latency_s_ = motion_compensation.latency_s;
      last_scan_motion_shift_ = motion_compensation.applied_shift;
      last_scan_motion_shift_m_ = motion_compensation.applied_shift_m;
    } else {
      last_scan_pose_lag_s_ = 0.0;
      last_scan_pose_latency_s_ = 0.0;
      last_scan_motion_shift_ = Point2{};
      last_scan_motion_shift_m_ = 0.0;
    }
    if (!scan_seen_logged_) {
      scan_seen_logged_ = true;
      RCLCPP_INFO(get_logger(),
                  "First planner lidar scan: beams=%zu range=[%.2f, %.2f] "
                  "angle=[%.2f, %.2f] projection_pose=%s pose_lag=%.3fs "
                  "pose_latency=%.3fs motion_shift=(%.2f, %.2f) "
                  "motion_shift_m=%.2f",
                  last_scan_.ranges.size(), static_cast<double>(last_scan_.range_min),
                  static_cast<double>(last_scan_.range_max),
                  static_cast<double>(last_scan_.angle_min),
                  static_cast<double>(last_scan_.angle_max),
                  last_scan_projection_pose_valid_ ? "true" : "false",
                  last_scan_pose_lag_s_, last_scan_pose_latency_s_,
                  last_scan_motion_shift_.x, last_scan_motion_shift_.y,
                  last_scan_motion_shift_m_);
    }
  }

  void onAttitude(const px4_msgs::msg::VehicleAttitude& msg) {
    const auto euler = quaternionToEuler(msg.q);
    if (!euler.has_value()) {
      attitude_valid_ = false;
      return;
    }

    current_attitude_ = *euler;
    attitude_valid_ = true;
  }

  [[nodiscard]] std::filesystem::path staticMapPackageShareDirectory() const {
    try {
      return std::filesystem::path{
          ament_index_cpp::get_package_share_directory("drone_city_nav")};
    } catch (const std::exception&) {
      return {};
    }
  }

  void loadConfiguredStaticMap() {
    StaticMapSourceResult result = loadStaticMapSource(StaticMapSourceConfig{
        use_static_map_, static_map_path_param_, staticMapPackageShareDirectory(),
        frame_id_, static_map_min_blocking_height_m_});
    static_map_resolved_path_ = result.resolved_path;

    if (result.status == StaticMapSourceStatus::kDisabled) {
      RCLCPP_INFO(get_logger(), "Static city map source is disabled");
      return;
    }

    if (result.status == StaticMapSourceStatus::kLoadFailed ||
        !result.grid.has_value()) {
      static_grid_.reset();
      static_map_rectangles_ = 0U;
      static_map_occupied_cells_ = 0U;
      RCLCPP_ERROR(get_logger(), "Failed to load static city map: path='%s' error='%s'",
                   static_map_resolved_path_.string().c_str(),
                   result.error_message.c_str());
      return;
    }

    if (!result.frame_matches) {
      RCLCPP_WARN(get_logger(),
                  "Static city map frame differs from planner frame: map='%s' "
                  "planner='%s'",
                  result.map_frame_id.c_str(), frame_id_.c_str());
    }
    static_grid_ = std::move(result.grid);
    static_map_rectangles_ = result.rectangles;
    static_map_occupied_cells_ = result.occupied_cells;
    RCLCPP_INFO(get_logger(),
                "Static city map loaded: path='%s' frame='%s' rectangles=%zu "
                "occupied_cells=%zu grid=%dx%d@%.2fm origin=(%.2f, %.2f) "
                "min_blocking_height=%.2f",
                static_map_resolved_path_.string().c_str(), result.map_frame_id.c_str(),
                static_map_rectangles_, static_map_occupied_cells_,
                static_grid_->width(), static_grid_->height(),
                static_grid_->resolution(), static_grid_->originX(),
                static_grid_->originY(), static_map_min_blocking_height_m_);
    publishStaticMapDebug(*static_grid_, true);
  }

  [[nodiscard]] PlanningGridBuilderConfig planningGridBuilderConfig() const {
    PlanningGridBuilderConfig config{};
    config.use_static_map = use_static_map_;
    config.use_obstacle_memory = use_obstacle_memory_;
    config.use_current_lidar_obstacles = use_current_lidar_obstacles_;
    config.fallback_bounds = fallback_grid_bounds_;
    config.inflation_radius_m = inflation_radius_m_;
    return config;
  }

  [[nodiscard]] std::optional<PlanningGridBuildResult>
  buildPlanningGrid(const std::int64_t now_ns) {
    const PlanningGridBuilderConfig config = planningGridBuilderConfig();
    PlanningGridSources sources{};
    sources.static_grid = static_grid_ ? &*static_grid_ : nullptr;
    sources.static_rectangles = static_map_rectangles_;
    sources.static_occupied_cells = static_map_occupied_cells_;
    sources.static_map_path = static_map_resolved_path_.string();
    sources.memory_grid = memory_grid_ ? &*memory_grid_ : nullptr;

    std::optional<OccupancyGrid2D> current_lidar_grid;
    if (const std::optional<GridBounds> bounds =
            selectPlanningGridBounds(config, sources);
        bounds.has_value()) {
      current_lidar_grid.emplace(*bounds);
      sources.current_lidar = overlayCurrentLidarHits(*current_lidar_grid, now_ns);
      sources.current_lidar_grid = &*current_lidar_grid;
    }

    PlanningGridBuildResult result = drone_city_nav::buildPlanningGrid(config, sources);
    if (result.memory.enabled && !result.memory.seen) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Obstacle memory source is enabled but no grid has been "
                           "received yet");
    } else if (result.memory.seen && memory_grid_.has_value() &&
               !result.memory.geometry_matches) {
      const OccupancyGrid2D& memory_grid = *memory_grid_;
      std::optional<OccupancyGrid2D> diagnostic_grid;
      const OccupancyGrid2D* planning_grid = result.grid ? &*result.grid : nullptr;
      if (planning_grid == nullptr) {
        if (const std::optional<GridBounds> bounds =
                selectPlanningGridBounds(config, sources);
            bounds.has_value()) {
          diagnostic_grid.emplace(*bounds);
          planning_grid = &*diagnostic_grid;
        }
      }
      if (planning_grid != nullptr) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Skipping obstacle memory overlay due to grid geometry mismatch: "
            "planning=%dx%d@%.2f origin=(%.2f, %.2f) memory=%dx%d@%.2f "
            "origin=(%.2f, %.2f)",
            planning_grid->width(), planning_grid->height(),
            planning_grid->resolution(), planning_grid->originX(),
            planning_grid->originY(), memory_grid.width(), memory_grid.height(),
            memory_grid.resolution(), memory_grid.originX(), memory_grid.originY());
      }
    }

    if (result.status == PlanningGridStatus::kNoEnabledSources) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planner has no enabled obstacle sources; skipping path check");
      return std::nullopt;
    }
    if (result.status == PlanningGridStatus::kStaticMapEnabledButMissing) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner static map source is enabled but not loaded; "
                           "skipping path check");
      return std::nullopt;
    }
    if (result.status == PlanningGridStatus::kNoReadySourceData ||
        !result.grid.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner has no ready obstacle source data; skipping path "
                           "check status=%s",
                           planningGridStatusName(result.status));
      return std::nullopt;
    }

    return result;
  }

  void checkCurrentPathAndPublish() {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const bool pose_fresh =
        timestampIsFresh(last_pose_update_ns_, now_ns, max_pose_staleness_ns_);
    const double pose_age_s = poseAgeSeconds(now_ns);
    if (pose_valid_ && !pose_fresh) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planner skipped path check because PX4 local position is stale; keeping the "
          "last published path: pose_age_s=%.2f",
          pose_age_s);
      return;
    }
    if (!pose_valid_ || !finite2D(current_pose_.position)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner is waiting for a valid PX4 local position; "
                           "keeping the last published path");
      return;
    }
    const auto planning_grid_started_at = std::chrono::steady_clock::now();
    auto planning_result = buildPlanningGrid(now_ns);
    const double planning_grid_duration_ms =
        elapsedMilliseconds(planning_grid_started_at);
    if (!planning_result.has_value()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planner skipped path publication because the planning grid is "
          "not ready; keeping the last published path");
      return;
    }
    if (!planning_result->grid.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner grid builder returned no grid despite a ready "
                           "result; keeping the last published path");
      return;
    }
    OccupancyGrid2D planning_grid = std::move(*planning_result->grid);
    publishProhibitedGrid(planning_grid);
    if (keepCurrentPathIfStillClear(planning_grid)) {
      return;
    }

    const AStarConfig planning_astar_config = astarConfigForCurrentVelocity();
    auto path_result =
        computePathOnGrid(planning_grid, "combined", planning_astar_config);
    if (!path_result.has_value()) {
      publishPlanningFailureHold();
      return;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planning summary: pose=(%.2f, %.2f) distance_to_start=%.2f "
        "distance_to_goal=%.2f raw_static=%zu raw_memory=%zu "
        "raw_current_lidar=%zu prohibited=%zu occupied=%zu inflated=%zu free=%zu "
        "unknown=%zu inflation_owner=planner inflation_radius_m=%.2f "
        "static[enabled=%s loaded=%s used=%s rectangles=%zu occupied_cells=%zu "
        "path='%s'] "
        "memory[enabled=%s seen=%s used=%s geometry_matches=%s occupied=%zu free=%zu "
        "unknown=%zu overlay_occupied=%zu overlay_free=%zu] "
        "current_lidar[enabled=%s used=%s fresh=%s processed=%zu hits=%zu "
        "altitude_rejected=%zu occupied_cells=%zu overlay_applied=%zu "
        "overlay_preserved=%zu outside=%zu] "
        "source=combined astar_status=%s heuristic_weight=%.2f expanded=%zu "
        "cost=%.2f raw_path=%zu smoothed_path=%zu "
        "initial_heading_bias[enabled=%s active=%s speed=%.2f min_speed=%.2f "
        "weight=%.2f velocity=(%.2f, %.2f)] "
        "path_metrics[raw_segments=%zu raw_straight_segments=%zu raw_turns=%zu "
        "raw_length=%.2f smoothed_segments=%zu smoothed_straight_segments=%zu "
        "smoothed_turns=%zu smoothed_length=%.2f] "
        "path_clearance[raw=%.2f smoothed=%.2f] "
        "timing[grid=%.1f path_total=%.1f astar=%.1f smoothing=%.1f "
        "core_breakdown[grid_stats=%.1f raw_metrics=%.1f smoothed_metrics=%.1f "
        "raw_clearance=%.1f smoothed_clearance=%.1f]]",
        current_pose_.position.x, current_pose_.position.y,
        distance(current_pose_.position, start_),
        distance(current_pose_.position, goal_),
        planning_result->static_source.occupied_cells,
        planning_result->memory.source_counts.occupied_cells,
        planning_result->current_lidar.occupied_cells,
        path_result->grid_stats.occupied_cells + path_result->grid_stats.inflated_cells,
        path_result->grid_stats.occupied_cells, path_result->grid_stats.inflated_cells,
        path_result->grid_stats.free_cells, path_result->grid_stats.unknown_cells,
        inflation_radius_m_, planning_result->static_source.enabled ? "true" : "false",
        planning_result->static_source.loaded ? "true" : "false",
        planning_result->static_source.used ? "true" : "false",
        planning_result->static_source.rectangles,
        planning_result->static_source.occupied_cells,
        planning_result->static_source.path.c_str(),
        planning_result->memory.enabled ? "true" : "false",
        planning_result->memory.seen ? "true" : "false",
        planning_result->memory.used ? "true" : "false",
        planning_result->memory.geometry_matches ? "true" : "false",
        planning_result->memory.source_counts.occupied_cells,
        planning_result->memory.source_counts.free_cells,
        planning_result->memory.source_counts.unknown_cells,
        planning_result->memory.overlay.occupied_cells_applied,
        planning_result->memory.overlay.free_cells_applied,
        planning_result->current_lidar.enabled ? "true" : "false",
        planning_result->current_lidar.used ? "true" : "false",
        planning_result->current_lidar.fresh ? "true" : "false",
        planning_result->current_lidar.processed_beams,
        planning_result->current_lidar.hit_beams,
        planning_result->current_lidar.altitude_rejected_beams,
        planning_result->current_lidar.occupied_cells,
        planning_result->current_lidar.overlay_occupied_cells_applied,
        planning_result->current_lidar.overlay_occupied_cells_preserved,
        planning_result->current_lidar.outside_hits,
        astarStatusName(path_result->astar.status),
        planning_astar_config.heuristic_weight, path_result->astar.expanded_cells,
        path_result->astar.total_cost, path_result->raw_path_metrics.points,
        path_result->smoothed_path_metrics.points,
        planning_astar_config.initial_heading_bias_enabled ? "true" : "false",
        initialHeadingBiasActive(planning_astar_config) ? "true" : "false",
        current_speed_mps_, planning_astar_config.initial_heading_bias_min_speed_mps,
        planning_astar_config.initial_heading_bias_weight,
        planning_astar_config.initial_heading_bias_velocity_x_mps,
        planning_astar_config.initial_heading_bias_velocity_y_mps,
        path_result->raw_path_metrics.segments,
        path_result->raw_path_metrics.straight_segments,
        path_result->raw_path_metrics.turns, path_result->raw_path_metrics.length_m,
        path_result->smoothed_path_metrics.segments,
        path_result->smoothed_path_metrics.straight_segments,
        path_result->smoothed_path_metrics.turns,
        path_result->smoothed_path_metrics.length_m, path_result->raw_path_clearance_m,
        path_result->smoothed_path_clearance_m, planning_grid_duration_ms,
        path_result->total_duration_ms, path_result->astar_duration_ms,
        path_result->smoothing_duration_ms, path_result->grid_stats_duration_ms,
        path_result->raw_path_metrics_duration_ms,
        path_result->smoothed_path_metrics_duration_ms,
        path_result->raw_path_clearance_duration_ms,
        path_result->smoothed_path_clearance_duration_ms);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Path smoothing diagnostics: input_points=%zu output_points=%zu "
        "checks=%zu accepted=%zu shortcuts=%zu forced_adjacent=%zu rejected=%zu "
        "rejected_prohibited=%zu rejected_outside_grid=%zu "
        "rejected_prohibited_cells=%zu "
        "raw_segment_lengths[min=%.2f mean=%.2f max=%.2f lt2=%zu lt5=%zu "
        "lt10=%zu] "
        "smoothed_segment_lengths[min=%.2f mean=%.2f max=%.2f lt2=%zu lt5=%zu "
        "lt10=%zu]",
        path_result->smoothing_stats.input_points,
        path_result->smoothing_stats.output_points,
        path_result->smoothing_stats.line_of_sight_checks,
        path_result->smoothing_stats.accepted_segments,
        path_result->smoothing_stats.shortcut_segments,
        path_result->smoothing_stats.forced_adjacent_segments,
        path_result->smoothing_stats.rejected_segments,
        path_result->smoothing_stats.rejected_prohibited,
        path_result->smoothing_stats.rejected_outside_grid,
        path_result->smoothing_stats.rejected_prohibited_cells,
        path_result->raw_path_metrics.min_segment_length_m,
        path_result->raw_path_metrics.mean_segment_length_m,
        path_result->raw_path_metrics.max_segment_length_m,
        path_result->raw_path_metrics.segments_shorter_than_2m,
        path_result->raw_path_metrics.segments_shorter_than_5m,
        path_result->raw_path_metrics.segments_shorter_than_10m,
        path_result->smoothed_path_metrics.min_segment_length_m,
        path_result->smoothed_path_metrics.mean_segment_length_m,
        path_result->smoothed_path_metrics.max_segment_length_m,
        path_result->smoothed_path_metrics.segments_shorter_than_2m,
        path_result->smoothed_path_metrics.segments_shorter_than_5m,
        path_result->smoothed_path_metrics.segments_shorter_than_10m);
    if (path_result->smoothing_returned_empty_path) {
      RCLCPP_WARN(get_logger(),
                  "Path smoothing returned an empty path; falling back to raw A* path: "
                  "raw_points=%zu",
                  path_result->astar.path.size());
    }
    publishPathFromPathCells(planning_grid, path_result->astar.path,
                             path_result->smoothed_cells, "combined");
  }

  [[nodiscard]] AStarConfig astarConfigForCurrentVelocity() const {
    AStarConfig config = astar_config_;
    if (current_velocity_valid_ && std::isfinite(current_speed_mps_) &&
        current_speed_mps_ >= config.initial_heading_bias_min_speed_mps) {
      config.initial_heading_bias_velocity_x_mps = current_velocity_.x;
      config.initial_heading_bias_velocity_y_mps = current_velocity_.y;
    }
    return config;
  }

  [[nodiscard]] static bool
  initialHeadingBiasActive(const AStarConfig& config) noexcept {
    const double speed_mps = std::hypot(config.initial_heading_bias_velocity_x_mps,
                                        config.initial_heading_bias_velocity_y_mps);
    return config.initial_heading_bias_enabled &&
           config.initial_heading_bias_weight > 0.0 && std::isfinite(speed_mps) &&
           speed_mps >= config.initial_heading_bias_min_speed_mps;
  }

  [[nodiscard]] std::optional<PathComputationResult>
  computePathOnGrid(const OccupancyGrid2D& grid, const char* source_label,
                    const AStarConfig& astar_config) {
    const auto start_cell = grid.worldToCell(current_pose_.position);
    const auto goal_cell = grid.worldToCell(goal_);
    if (!start_cell.has_value() || !goal_cell.has_value()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Start or goal is outside the %s planning grid: "
                            "start=(%.2f, %.2f) goal=(%.2f, %.2f)",
                            source_label, current_pose_.position.x,
                            current_pose_.position.y, goal_.x, goal_.y);
      return std::nullopt;
    }
    const bool start_prohibited = grid.isProhibited(*start_cell);
    const bool goal_prohibited = grid.isProhibited(*goal_cell);
    if (goal_prohibited || grid.isOccupied(*start_cell)) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Start or goal is not plannable on %s grid; refusing to move planning "
          "endpoints: start_cell=(%d,%d) occupied=%s inflated=%s prohibited=%s "
          "goal_cell=(%d,%d) occupied=%s inflated=%s prohibited=%s",
          source_label, start_cell->x, start_cell->y,
          grid.isOccupied(*start_cell) ? "true" : "false",
          grid.isInflated(*start_cell) ? "true" : "false",
          start_prohibited ? "true" : "false", goal_cell->x, goal_cell->y,
          grid.isOccupied(*goal_cell) ? "true" : "false",
          grid.isInflated(*goal_cell) ? "true" : "false",
          goal_prohibited ? "true" : "false");
      return std::nullopt;
    }
    if (start_prohibited) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Start is inside inflated prohibited space on %s grid; attempting "
          "escape-start planning: start_cell=(%d,%d)",
          source_label, start_cell->x, start_cell->y);
    }

    const auto path_compute_started_at = std::chrono::steady_clock::now();
    auto result =
        planner_core_.computePath(grid, current_pose_.position, goal_, astar_config);
    const double path_compute_duration_ms =
        elapsedMilliseconds(path_compute_started_at);
    ++astar_runs_;
    if (!result.has_value()) {
      ++astar_failures_;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "A* did not find a path on %s grid: start=(%d,%d) goal=(%d,%d) "
          "duration_ms=%.1f",
          source_label, start_cell->x, start_cell->y, goal_cell->x, goal_cell->y,
          path_compute_duration_ms);
      return std::nullopt;
    }

    ++astar_successes_;
    if (result->start_escape_used && result->requested_start_cell.has_value() &&
        result->start_cell.has_value()) {
      RCLCPP_WARN(
          get_logger(),
          "A* recovered from inflated start on %s grid: requested_start=(%d,%d) "
          "escape_start=(%d,%d) escape_distance=%.2fm",
          source_label, result->requested_start_cell->x,
          result->requested_start_cell->y, result->start_cell->x, result->start_cell->y,
          result->start_escape_distance_m);
    }
    return result;
  }

  bool publishPathFromPathCells(const OccupancyGrid2D& grid,
                                const std::vector<GridIndex>& raw_cells,
                                const std::vector<GridIndex>& smoothed_cells,
                                const char* source_label) {
    struct CandidatePath {
      std::vector<Point2> points;
      const char* source_kind{""};
      std::size_t input_cells{0U};
      std::size_t pre_collapse_points{0U};
      std::size_t collapsed_points{0U};
      bool collapse_reverted{false};
    };

    const auto build_candidate =
        [&](const std::vector<GridIndex>& cells,
            const char* source_kind) -> std::optional<CandidatePath> {
      if (cells.empty()) {
        return std::nullopt;
      }

      std::vector<Point2> path_points = cellsToPoints(grid, cells);
      if (!connectRouteToCurrentPose(grid, path_points, source_label)) {
        return std::nullopt;
      }

      const std::vector<Point2> pre_collapse_path_points = path_points;
      path_points = collapseCollinearPath(pre_collapse_path_points,
                                          kPublishedPathCollinearityToleranceM);
      CandidatePath candidate{
          std::move(path_points),          source_kind, cells.size(),
          pre_collapse_path_points.size(), 0U,          false};
      candidate.collapsed_points = candidate.points.size();

      if (!pathIsTraversable(grid, candidate.points)) {
        if (pathIsTraversable(grid, pre_collapse_path_points)) {
          candidate.points = pre_collapse_path_points;
          candidate.collapse_reverted = true;
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 3000,
              "%s path restored pre-collapse waypoints because collinear collapse "
              "would create a non-traversable segment: source=%s before=%zu "
              "collapsed=%zu",
              source_label, source_kind, pre_collapse_path_points.size(),
              candidate.collapsed_points);
        } else {
          logRejectedUnsafeRoute(
              grid, pre_collapse_path_points, source_label,
              "pre-collapse path contains a non-traversable segment");
          return std::nullopt;
        }
      }

      return candidate;
    };

    const std::vector<GridIndex>* selected_cells = &smoothed_cells;
    const char* selected_source_kind = "smoothed";
    bool used_raw_fallback = false;
    if (selected_cells->empty()) {
      RCLCPP_WARN(get_logger(),
                  "%s path has empty smoothed cells; falling back to raw A* cells: "
                  "raw_cells=%zu",
                  source_label, raw_cells.size());
      selected_cells = &raw_cells;
      selected_source_kind = "raw";
      used_raw_fallback = true;
    }

    std::optional<CandidatePath> candidate =
        build_candidate(*selected_cells, selected_source_kind);
    if (!candidate.has_value() && selected_cells != &raw_cells && !raw_cells.empty()) {
      RCLCPP_WARN(get_logger(),
                  "%s path postprocess rejected smoothed cells; falling back to raw "
                  "A* cells: smoothed_cells=%zu raw_cells=%zu",
                  source_label, smoothed_cells.size(), raw_cells.size());
      selected_cells = &raw_cells;
      selected_source_kind = "raw";
      used_raw_fallback = true;
      candidate = build_candidate(*selected_cells, selected_source_kind);
    }
    if (!candidate.has_value()) {
      RCLCPP_WARN(get_logger(),
                  "%s path postprocess could not build a traversable route seed: "
                  "smoothed_cells=%zu raw_cells=%zu",
                  source_label, smoothed_cells.size(), raw_cells.size());
      publishPath({}, PathPublicationReason::kHoldInvalidPath);
      return false;
    }

    const std::vector<Point2> route_points = std::move(candidate->points);
    RCLCPP_INFO(get_logger(),
                "%s path postprocess: selected_source=%s raw_cells=%zu "
                "smoothed_cells=%zu selected_cells=%zu pre_collapse_points=%zu "
                "collapsed_points=%zu route_points=%zu route_segments=%zu "
                "raw_fallback=%s collapse_reverted=%s",
                source_label, candidate->source_kind, raw_cells.size(),
                smoothed_cells.size(), candidate->input_cells,
                candidate->pre_collapse_points, candidate->collapsed_points,
                route_points.size(),
                !route_points.empty() ? route_points.size() - 1U : 0U,
                used_raw_fallback ? "true" : "false",
                candidate->collapse_reverted ? "true" : "false");
    if (route_points.size() != candidate->pre_collapse_points) {
      RCLCPP_INFO(get_logger(),
                  "%s path collinear waypoints collapsed: before=%zu after=%zu "
                  "tolerance=%.2fm",
                  source_label, candidate->pre_collapse_points, route_points.size(),
                  kPublishedPathCollinearityToleranceM);
    }

    const auto started_at = std::chrono::steady_clock::now();
    TrajectoryPlannerResult trajectory_result = planRacingTrajectory(
        TrajectoryPlannerInput{
            std::span<const Point2>{route_points.data(), route_points.size()}, &grid},
        trajectory_planner_config_);
    const double duration_ms =
        static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now() - started_at)
                                .count()) /
        1000.0;
    writeCorridorSamplesDump(trajectory_result, source_label, next_path_id_);
    if (!trajectory_result.valid) {
      RCLCPP_WARN(
          get_logger(),
          "%s racing trajectory build failed; rough A* route will not be published as "
          "runtime path: status=%.*s route_points=%zu duration_ms=%.1f "
          "timing[total=%.1f corridor=%.1f racing_line=%.1f "
          "turn_smoothing=%.1f speed_profile=%.1f] "
          "corridor[samples=%zu width_min=%.2f width_mean=%.2f] "
          "racing_line[iterations=%zu evals=%zu collision_rejections=%zu]",
          source_label,
          static_cast<int>(
              trajectoryPlannerStatusName(trajectory_result.stats.status).size()),
          trajectoryPlannerStatusName(trajectory_result.stats.status).data(),
          route_points.size(), duration_ms, trajectory_result.stats.total_duration_ms,
          trajectory_result.stats.corridor_duration_ms,
          trajectory_result.stats.racing_line_duration_ms,
          trajectory_result.stats.turn_smoothing_duration_ms,
          trajectory_result.stats.speed_profile_duration_ms,
          trajectory_result.stats.corridor.samples,
          trajectory_result.stats.corridor.min_width_m,
          trajectory_result.stats.corridor.mean_width_m,
          trajectory_result.stats.racing_line.iterations,
          trajectory_result.stats.racing_line.candidate_evaluations,
          trajectory_result.stats.racing_line.collision_rejections);
      publishPath({}, PathPublicationReason::kHoldInvalidPath);
      return false;
    }

    std::vector<Point2> trajectory_points =
        trajectorySamplePoints(trajectory_result.samples);
    if (trajectory_points.size() < 2U || !pathIsTraversable(grid, trajectory_points)) {
      RCLCPP_WARN(
          get_logger(),
          "%s racing trajectory build produced a non-traversable runtime trajectory; "
          "holding instead of publishing rough A* route: route_points=%zu "
          "trajectory_points=%zu duration_ms=%.1f status=%.*s "
          "timing[total=%.1f corridor=%.1f racing_line=%.1f "
          "turn_smoothing=%.1f speed_profile=%.1f]",
          source_label, route_points.size(), trajectory_points.size(), duration_ms,
          static_cast<int>(
              trajectoryPlannerStatusName(trajectory_result.stats.status).size()),
          trajectoryPlannerStatusName(trajectory_result.stats.status).data(),
          trajectory_result.stats.total_duration_ms,
          trajectory_result.stats.corridor_duration_ms,
          trajectory_result.stats.racing_line_duration_ms,
          trajectory_result.stats.turn_smoothing_duration_ms,
          trajectory_result.stats.speed_profile_duration_ms);
      publishPath({}, PathPublicationReason::kHoldInvalidPath);
      return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "%s final racing trajectory: route_points=%zu trajectory_points=%zu "
        "duration_ms=%.1f status=%.*s "
        "timing[total=%.1f corridor=%.1f racing_line=%.1f "
        "turn_smoothing=%.1f speed_profile=%.1f] "
        "length=%.2f samples=%zu "
        "corridor[samples=%zu width_min=%.2f width_mean=%.2f width_max=%.2f "
        "lateral_limited=%zu] "
        "racing_line[iterations=%zu evals=%zu skipped_noop=%zu "
        "eval_time=%.1fms score_time=%.1fms cost_initial=%.3f cost_final=%.3f "
        "length_initial=%.2f length_final=%.2f length_ratio=%.3f "
        "max_offset=%.2f edge_margin_min=%.2f time_final=%.2f "
        "time_centerline=%.2f time_gain=%.2f speed_limit_min=%.2f "
        "speed_limit_max=%.2f curvature_limited=%zu] "
        "turn_smoothing[detected=%zu attempted=%zu candidate_attempts=%zu "
        "smoothed=%zu "
        "rejected(prohibited=%zu corridor=%zu length=%zu not_improved=%zu) "
        "heading_before=%.1fdeg heading_after=%.1fdeg "
        "curvature_jump_before=%.3f curvature_jump_after=%.3f "
        "min_inner_margin=%.2f max_outer_shift=%.2f "
        "accepted(entry=%.2fm exit=%.2fm shift_scale=%.2f)] "
        "speed_profile[min=%.2f mean=%.2f max=%.2f curvature_limited=%zu]",
        source_label, route_points.size(), trajectory_points.size(), duration_ms,
        static_cast<int>(
            trajectoryPlannerStatusName(trajectory_result.stats.status).size()),
        trajectoryPlannerStatusName(trajectory_result.stats.status).data(),
        trajectory_result.stats.total_duration_ms,
        trajectory_result.stats.corridor_duration_ms,
        trajectory_result.stats.racing_line_duration_ms,
        trajectory_result.stats.turn_smoothing_duration_ms,
        trajectory_result.stats.speed_profile_duration_ms,
        trajectory_result.stats.length_m, trajectory_result.stats.samples,
        trajectory_result.stats.corridor.samples,
        trajectory_result.stats.corridor.min_width_m,
        trajectory_result.stats.corridor.mean_width_m,
        trajectory_result.stats.corridor.max_width_m,
        trajectory_result.stats.corridor.lateral_limited_samples,
        trajectory_result.stats.racing_line.iterations,
        trajectory_result.stats.racing_line.candidate_evaluations,
        trajectory_result.stats.racing_line.skipped_noop_candidates,
        trajectory_result.stats.racing_line.candidate_path_evaluation_duration_ms,
        trajectory_result.stats.racing_line.candidate_score_duration_ms,
        trajectory_result.stats.racing_line.initial_cost,
        trajectory_result.stats.racing_line.final_cost,
        trajectory_result.stats.racing_line.centerline_length_m,
        trajectory_result.stats.racing_line.final_length_m,
        trajectory_result.stats.racing_line.final_length_ratio,
        trajectory_result.stats.racing_line.max_abs_offset_m,
        trajectory_result.stats.racing_line.min_edge_margin_m,
        trajectory_result.stats.racing_line.estimated_time_s,
        trajectory_result.stats.racing_line.centerline_estimated_time_s,
        trajectory_result.stats.racing_line.time_gain_s,
        trajectory_result.stats.racing_line.min_speed_limit_mps,
        trajectory_result.stats.racing_line.max_speed_limit_mps,
        trajectory_result.stats.racing_line.curvature_limited_samples,
        trajectory_result.stats.turn_smoothing.detected_corners,
        trajectory_result.stats.turn_smoothing.attempted_corners,
        trajectory_result.stats.turn_smoothing.candidate_attempts,
        trajectory_result.stats.turn_smoothing.smoothed_corners,
        trajectory_result.stats.turn_smoothing.rejected_prohibited,
        trajectory_result.stats.turn_smoothing.rejected_corridor,
        trajectory_result.stats.turn_smoothing.rejected_length,
        trajectory_result.stats.turn_smoothing.rejected_not_improved,
        radiansToDegrees(
            trajectory_result.stats.turn_smoothing.max_heading_delta_before_rad),
        radiansToDegrees(
            trajectory_result.stats.turn_smoothing.max_heading_delta_after_rad),
        trajectory_result.stats.turn_smoothing.max_curvature_jump_before_1pm,
        trajectory_result.stats.turn_smoothing.max_curvature_jump_after_1pm,
        trajectory_result.stats.turn_smoothing.min_inner_margin_m,
        trajectory_result.stats.turn_smoothing.max_applied_outer_shift_m,
        trajectory_result.stats.turn_smoothing.accepted_entry_distance_m,
        trajectory_result.stats.turn_smoothing.accepted_exit_distance_m,
        trajectory_result.stats.turn_smoothing.accepted_shift_scale,
        trajectory_result.stats.speed_profile_min_mps,
        trajectory_result.stats.speed_profile_mean_mps,
        trajectory_result.stats.speed_profile_max_mps,
        trajectory_result.stats.speed_profile_curvature_limited_samples);

    last_valid_path_points_ = trajectory_points;
    logPublishedPathSafety(grid, trajectory_points, "final_trajectory");
    publishPath(trajectory_points, PathPublicationReason::kComputedPath,
                &trajectory_result.stats);
    return true;
  }

  [[nodiscard]] PublishedPathSafetySummary
  summarizePublishedPathSafety(const OccupancyGrid2D& grid,
                               const std::span<const Point2> path_points) const {
    PublishedPathSafetySummary summary{};
    if (path_points.size() < 2U) {
      return summary;
    }

    summary.segments = path_points.size() - 1U;
    for (std::size_t index = 1U; index < path_points.size(); ++index) {
      const Point2 segment_start = path_points[index - 1U];
      const Point2 segment_end = path_points[index];
      if (pathSegmentIsTraversable(grid, segment_start, segment_end)) {
        if (!pathSegmentIsAllowed(grid, segment_start, segment_end)) {
          ++summary.escape_segments;
        }
      } else {
        ++summary.non_traversable_segments;
        if (!summary.has_non_traversable_segment) {
          summary.first_non_traversable_segment = index - 1U;
          summary.first_non_traversable_start = segment_start;
          summary.first_non_traversable_end = segment_end;
          summary.has_non_traversable_segment = true;
        }
      }
    }

    return summary;
  }

  void logPublishedPathSafety(const OccupancyGrid2D& grid,
                              const std::span<const Point2> path_points,
                              const char* source_label) const {
    const PublishedPathSafetySummary summary =
        summarizePublishedPathSafety(grid, path_points);
    const bool unsafe_path = summary.non_traversable_segments > 0U;
    if (unsafe_path) {
      RCLCPP_WARN(
          get_logger(),
          "%s published path traversability: segments=%zu "
          "non_traversable_segments=%zu escape_segments=%zu traversable=false "
          "first_non_traversable_segment=%zu "
          "segment_start=(%.2f, %.2f) segment_end=(%.2f, %.2f)",
          source_label, summary.segments, summary.non_traversable_segments,
          summary.escape_segments, summary.first_non_traversable_segment,
          summary.first_non_traversable_start.x, summary.first_non_traversable_start.y,
          summary.first_non_traversable_end.x, summary.first_non_traversable_end.y);
      return;
    }

    if (summary.escape_segments > 0U) {
      RCLCPP_WARN(get_logger(),
                  "%s published path traversability: segments=%zu "
                  "non_traversable_segments=0 escape_segments=%zu traversable=true",
                  source_label, summary.segments, summary.escape_segments);
      return;
    }

    RCLCPP_INFO(get_logger(),
                "%s published path traversability: segments=%zu "
                "non_traversable_segments=0 escape_segments=0 traversable=true",
                source_label, summary.segments);
  }

  [[nodiscard]] bool connectRouteToCurrentPose(const OccupancyGrid2D& grid,
                                               std::vector<Point2>& path_points,
                                               const char* source_label) const {
    if (path_points.empty()) {
      return false;
    }

    const Point2 current_position = current_pose_.position;
    const Point2 first_path_point = path_points.front();
    const double distance_to_first_m = distance(current_position, first_path_point);
    if (distance_to_first_m < 1.0e-6) {
      path_points.front() = current_position;
      return true;
    }

    if (distance_to_first_m < grid.resolution() && path_points.size() >= 2U &&
        pathSegmentIsTraversable(grid, current_position, path_points[1])) {
      path_points.front() = current_position;
      return true;
    }

    if (pathSegmentIsTraversable(grid, current_position, first_path_point)) {
      path_points.insert(path_points.begin(), current_position);
      return true;
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "%s route candidate rejected because current pose cannot connect to the "
        "planned "
        "start with a traversable segment: current=(%.2f, %.2f) first=(%.2f, %.2f) "
        "distance=%.2fm",
        source_label, current_position.x, current_position.y, first_path_point.x,
        first_path_point.y, distance_to_first_m);
    return false;
  }

  void logRejectedUnsafeRoute(const OccupancyGrid2D& grid,
                              const std::span<const Point2> path_points,
                              const char* source_label, const char* reason) const {
    const PublishedPathSafetySummary summary =
        summarizePublishedPathSafety(grid, path_points);
    RCLCPP_WARN(
        get_logger(),
        "%s route rejected before racing trajectory build: reason='%s' segments=%zu "
        "non_traversable_segments=%zu escape_segments=%zu "
        "first_non_traversable_segment=%zu "
        "segment_start=(%.2f, %.2f) "
        "segment_end=(%.2f, %.2f)",
        source_label, reason, summary.segments, summary.non_traversable_segments,
        summary.escape_segments, summary.first_non_traversable_segment,
        summary.first_non_traversable_start.x, summary.first_non_traversable_start.y,
        summary.first_non_traversable_end.x, summary.first_non_traversable_end.y);
  }

  [[nodiscard]] double currentLidarRangeMax() const {
    return std::min(static_cast<double>(last_scan_.range_max), max_lidar_range_m_);
  }

  [[nodiscard]] double
  currentLidarPoseReceiveLagSeconds(const std::int64_t scan_receive_ns,
                                    const std::int64_t pose_receive_ns) const {
    if (scan_receive_ns > 0 && pose_receive_ns > 0 &&
        scan_receive_ns > pose_receive_ns) {
      return static_cast<double>(scan_receive_ns - pose_receive_ns) / 1.0e9;
    }
    return 0.0;
  }

  [[nodiscard]] LidarProjectionPose currentLidarProjectionPose() const {
    return LidarProjectionPose{current_pose_.position,
                               current_altitude_m_,
                               use_px4_heading_for_scan_ ? current_pose_.yaw_rad
                                                         : initial_heading_rad_,
                               current_attitude_.roll_rad,
                               current_attitude_.pitch_rad,
                               altitude_valid_,
                               attitude_valid_};
  }

  [[nodiscard]] LidarProjectionConfig currentLidarProjectionConfig() const {
    return LidarProjectionConfig{max_lidar_range_m_,
                                 range_hit_epsilon_m_,
                                 scan_yaw_offset_rad_,
                                 lidar_z_offset_m_,
                                 min_projected_lidar_altitude_m_,
                                 max_projected_lidar_altitude_m_,
                                 compensate_lidar_attitude_,
                                 lidar_mount_roll_rad_,
                                 lidar_mount_pitch_rad_,
                                 lidar_mount_yaw_rad_};
  }

  CurrentLidarOverlayStats overlayCurrentLidarHits(OccupancyGrid2D& grid,
                                                   const std::int64_t now_ns) const {
    CurrentLidarOverlayStats stats{};
    stats.enabled = use_current_lidar_obstacles_;
    if (!use_current_lidar_obstacles_) {
      return stats;
    }

    stats.fresh =
        timestampIsFresh(last_scan_update_ns_, now_ns, max_current_lidar_staleness_ns_);
    if (!scan_seen_ || !stats.fresh) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planner current lidar overlay is waiting for a fresh scan: seen=%s "
          "fresh=%s age_s=%.2f",
          scan_seen_ ? "true" : "false", stats.fresh ? "true" : "false",
          scanAgeSeconds(now_ns));
      return stats;
    }
    if (!last_scan_projection_pose_valid_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planner current lidar overlay is waiting for a valid scan projection pose");
      return stats;
    }

    const double scan_range_max = currentLidarRangeMax();
    if (!(scan_range_max > 0.0) || last_scan_.angle_increment == 0.0F) {
      return stats;
    }

    const CurrentLidarOverlayStats overlay_stats =
        drone_city_nav::overlayCurrentLidarHits(
            grid,
            LidarScanView{std::span<const float>{last_scan_.ranges.data(),
                                                 last_scan_.ranges.size()},
                          static_cast<double>(last_scan_.range_min), scan_range_max,
                          static_cast<double>(last_scan_.angle_min),
                          static_cast<double>(last_scan_.angle_increment)},
            last_scan_projection_pose_, currentLidarProjectionConfig());
    stats.used = overlay_stats.used;
    stats.processed_beams = overlay_stats.processed_beams;
    stats.hit_beams = overlay_stats.hit_beams;
    stats.altitude_rejected_beams = overlay_stats.altitude_rejected_beams;
    stats.occupied_cells = overlay_stats.occupied_cells;
    stats.outside_hits = overlay_stats.outside_hits;
    return stats;
  }

  [[nodiscard]] std_msgs::msg::Header makePlannerHeader() const {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;
    return header;
  }

  void publishStaticMapDebug(const OccupancyGrid2D& grid, const bool log_publication) {
    const StaticMapDebugConfig config{makePlannerHeader(),
                                      static_cast<float>(kGroundDebugZ)};
    static_map_pub_->publish(staticMapGridMessage(grid, config));
    static_map_points_pub_->publish(staticMapPointCloud(grid, config));
    if (log_publication) {
      RCLCPP_INFO(get_logger(),
                  "Published static map grid: cells=%zu occupied=%zu "
                  "points_topic='%s' republish_period=%.2fs",
                  grid.cellCount(), static_map_occupied_cells_,
                  static_map_points_pub_->get_topic_name(),
                  static_map_debug_publish_period_s_);
    }
  }

  void republishStaticMapDebug() {
    if (!static_grid_.has_value()) {
      return;
    }

    publishStaticMapDebug(*static_grid_, false);
  }

  void publishProhibitedGrid(const OccupancyGrid2D& grid) {
    prohibited_grid_pub_->publish(
        prohibitedGridToRos(grid, ProhibitedGridToRosConfig{makePlannerHeader()}));
  }

  void publishPath(const std::vector<Point2>& points,
                   const PathPublicationReason reason,
                   const TrajectoryPlannerStats* trajectory_stats = nullptr) {
    recordPathPublication(reason, points.empty());
    const std::uint64_t path_id = next_path_id_++;

    if (points.empty()) {
      last_valid_path_points_.clear();
    }

    const PathMetrics metrics = pointPathMetrics(points);
    const std_msgs::msg::Header header = makePlannerHeader();
    const std::uint64_t path_stamp_ns = stampNanoseconds(header.stamp);
    const nav_msgs::msg::Path path = pathToRos(
        std::span<const Point2>{points.data(), points.size()}, header, kGroundDebugZ);

    std_msgs::msg::UInt64 path_id_msg;
    path_id_msg.data = path_id;
    path_id_pub_->publish(path_id_msg);
    if (trajectory_stats != nullptr && !points.empty()) {
      publishTrajectoryDiagnostics(path_id, path_stamp_ns, *trajectory_stats);
    }
    path_pub_->publish(path);
    if (!path.poses.empty()) {
      waypoint_pub_->publish(path.poses.front());
    }

    logPathUpdate(path, metrics, reason, path_id);
    logPlannerCountersThrottled();
  }

  void publishTrajectoryDiagnostics(const std::uint64_t path_id,
                                    const std::uint64_t path_stamp_ns,
                                    const TrajectoryPlannerStats& stats) const {
    if (!trajectory_diagnostics_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = trajectoryPlannerDiagnosticsJson(path_id, path_stamp_ns, stats);
    trajectory_diagnostics_pub_->publish(msg);
  }

  [[nodiscard]] static std::filesystem::path corridorSamplesDirectory() {
    return std::filesystem::path{"log"} / "corridor_samples";
  }

  static void writeCsvNumberOrEmpty(std::ostream& stream, const double value) {
    if (std::isfinite(value)) {
      stream << value;
    }
  }

  bool writeCorridorSamplesCsvFile(const std::filesystem::path& path,
                                   const TrajectoryPlannerResult& result,
                                   const char* source_label,
                                   const std::uint64_t candidate_path_id) const {
    std::ofstream stream{path, std::ios::out | std::ios::trunc};
    if (!stream.is_open()) {
      return false;
    }

    stream << std::setprecision(9);
    stream << "# source=" << source_label << " candidate_path_id=" << candidate_path_id
           << " status=" << trajectoryPlannerStatusName(result.stats.status)
           << " valid=" << (result.valid ? "true" : "false") << "\n";
    stream << "sample_index,s_m,route_x,route_y,center_x,center_y,tangent_x,"
              "tangent_y,normal_x,normal_y,left_bound_m,right_bound_m,width_m,"
              "clearance_m,center_recovery_m\n";
    for (std::size_t i = 0U; i < result.corridor_samples.size(); ++i) {
      const CorridorSample& sample = result.corridor_samples[i];
      stream << i << ",";
      writeCsvNumberOrEmpty(stream, sample.s_m);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.route_center.x);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.route_center.y);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.center.x);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.center.y);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.tangent.x);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.tangent.y);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.normal.x);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.normal.y);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.left_bound_m);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.right_bound_m);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.left_bound_m + sample.right_bound_m);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.clearance_m);
      stream << ",";
      writeCsvNumberOrEmpty(stream, sample.center_recovery_m);
      stream << "\n";
    }
    return stream.good();
  }

  void writeCorridorSamplesDump(const TrajectoryPlannerResult& result,
                                const char* source_label,
                                const std::uint64_t candidate_path_id) const {
    if (result.corridor_samples.empty()) {
      return;
    }

    const std::filesystem::path directory = corridorSamplesDirectory();
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error) {
      RCLCPP_WARN(get_logger(), "Failed to create corridor samples directory '%s': %s",
                  directory.string().c_str(), error.message().c_str());
      return;
    }

    const std::int64_t stamp_ns = get_clock()->now().nanoseconds();
    const std::filesystem::path latest_path = directory / "latest.csv";
    const std::filesystem::path history_path =
        directory / ("path_" + std::to_string(candidate_path_id) + "_" +
                     std::to_string(stamp_ns) + ".csv");
    const bool wrote_latest = writeCorridorSamplesCsvFile(
        latest_path, result, source_label, candidate_path_id);
    const bool wrote_history = writeCorridorSamplesCsvFile(
        history_path, result, source_label, candidate_path_id);
    if (!wrote_latest || !wrote_history) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Failed to write corridor samples dump: latest='%s' history='%s'",
          latest_path.string().c_str(), history_path.string().c_str());
    }
  }

  void publishPlanningFailureHold() {
    if (!last_valid_path_points_.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Clearing path after replanning failure; holding position instead of reusing "
          "stale waypoints");
    }
    publishPath({}, PathPublicationReason::kHoldAfterPlanningFailure);
  }

  void invalidateCurrentPose() {
    current_pose_ = Pose2{};
    current_altitude_m_ = std::numeric_limits<double>::quiet_NaN();
    pose_valid_ = false;
    altitude_valid_ = false;
    last_scan_projection_pose_valid_ = false;
    last_pose_update_ns_ = 0;
  }

  [[nodiscard]] double poseAgeSeconds(const std::int64_t now_ns) const {
    if (last_pose_update_ns_ <= 0 || now_ns <= last_pose_update_ns_) {
      return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(now_ns - last_pose_update_ns_) / 1.0e9;
  }

  [[nodiscard]] double scanAgeSeconds(const std::int64_t now_ns) const {
    if (last_scan_update_ns_ <= 0 || now_ns <= last_scan_update_ns_) {
      return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(now_ns - last_scan_update_ns_) / 1.0e9;
  }

  bool keepCurrentPathIfStillClear(const OccupancyGrid2D& grid) {
    if (!stable_path_reuse_enabled_ || last_valid_path_points_.size() < 2U) {
      return false;
    }

    const StablePathDecision decision = planner_core_.evaluateStablePath(
        grid, last_valid_path_points_, current_pose_.position, goal_);
    if (decision.reason == StablePathDecisionReason::kGoalMismatch ||
        decision.reason == StablePathDecisionReason::kProjectionUnavailable ||
        decision.reason == StablePathDecisionReason::kNoPreviousPath ||
        decision.reason == StablePathDecisionReason::kDisabled) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Stable path reuse rejected; running A*: reason=%s "
          "previous_waypoints=%zu endpoint_goal_distance=%.2fm "
          "goal_tolerance=%.2fm deviation=%.2fm "
          "current=(%.2f, %.2f) goal=(%.2f, %.2f)",
          stablePathDecisionReasonName(decision.reason), last_valid_path_points_.size(),
          decision.endpoint_goal_distance_m, stable_path_goal_tolerance_m_,
          decision.deviation_m, current_pose_.position.x, current_pose_.position.y,
          goal_.x, goal_.y);
      return false;
    }

    if (decision.reason == StablePathDecisionReason::kProhibitedConfirmed) {
      ++prohibited_replans_;
      const Point2 prohibited_start =
          decision.prohibited_segment_index < decision.remaining_path.size()
              ? decision.remaining_path[decision.prohibited_segment_index]
              : Point2{};
      const Point2 prohibited_end =
          decision.prohibited_segment_index + 1U < decision.remaining_path.size()
              ? decision.remaining_path[decision.prohibited_segment_index + 1U]
              : Point2{};
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Current path intersects newly available prohibited obstacle data; "
          "running A* from current pose: reason=%s "
          "remaining_waypoints=%zu deviation=%.2fm prohibited_segment=%zu "
          "segment_start=(%.2f, %.2f) segment_end=(%.2f, %.2f)",
          stablePathDecisionReasonName(decision.reason), decision.remaining_path.size(),
          decision.deviation_m, decision.prohibited_segment_index, prohibited_start.x,
          prohibited_start.y, prohibited_end.x, prohibited_end.y);
      return false;
    }

    last_valid_path_points_ = decision.remaining_path;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Keeping current path because the remaining path is still clear: "
        "reason=%s remaining_waypoints=%zu deviation=%.2fm distance_to_goal=%.2f",
        stablePathDecisionReasonName(decision.reason), last_valid_path_points_.size(),
        decision.deviation_m, distance(current_pose_.position, goal_));
    return true;
  }

  void logPathUpdate(const nav_msgs::msg::Path& path, const PathMetrics& metrics,
                     const PathPublicationReason reason, const std::uint64_t path_id) {
    const std::size_t path_size = path.poses.size();
    const bool path_changed = path_size != last_logged_path_size_;
    const std::string counters = plannerCountersSummary();
    const std::uint64_t path_stamp_ns = stampNanoseconds(path.header.stamp);
    if (path_size == 0U) {
      if (path_changed) {
        RCLCPP_WARN(get_logger(),
                    "Published empty path: path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                    " reason=%s counters[%s]",
                    path_id, path_stamp_ns, pathPublicationReasonName(reason),
                    counters.c_str());
        last_logged_path_size_ = path_size;
      }
      return;
    }

    const Point2 first{path.poses.front().pose.position.x,
                       path.poses.front().pose.position.y};
    const Point2 last{path.poses.back().pose.position.x,
                      path.poses.back().pose.position.y};
    const bool endpoint_changed =
        squaredDistance(first, last_logged_path_first_) > 0.01 ||
        squaredDistance(last, last_logged_path_last_) > 0.01;
    if (path_changed || endpoint_changed) {
      std::ostringstream preview;
      const std::size_t preview_count = std::min<std::size_t>(path_size, 6U);
      for (std::size_t i = 0U; i < preview_count; ++i) {
        if (i != 0U) {
          preview << " -> ";
        }
        preview << "(" << path.poses[i].pose.position.x << ", "
                << path.poses[i].pose.position.y << ")";
      }
      RCLCPP_INFO(get_logger(),
                  "Published path: path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                  " reason=%s waypoints=%zu segments=%zu "
                  "straight_segments=%zu turns=%zu length=%.2f first=(%.2f, %.2f) "
                  "segment_lengths[min=%.2f mean=%.2f max=%.2f lt2=%zu lt5=%zu "
                  "lt10=%zu] "
                  "last=(%.2f, %.2f) counters[%s] preview=%s",
                  path_id, path_stamp_ns, pathPublicationReasonName(reason), path_size,
                  metrics.segments, metrics.straight_segments, metrics.turns,
                  metrics.length_m, first.x, first.y, metrics.min_segment_length_m,
                  metrics.mean_segment_length_m, metrics.max_segment_length_m,
                  metrics.segments_shorter_than_2m, metrics.segments_shorter_than_5m,
                  metrics.segments_shorter_than_10m, last.x, last.y, counters.c_str(),
                  preview.str().c_str());
      last_logged_path_size_ = path_size;
      last_logged_path_first_ = first;
      last_logged_path_last_ = last;
    }
  }

  void recordPathPublication(const PathPublicationReason reason,
                             const bool empty_path) {
    ++path_publications_;
    if (empty_path) {
      ++hold_path_publications_;
    } else {
      ++non_empty_path_publications_;
    }

    switch (reason) {
      case PathPublicationReason::kComputedPath:
        ++computed_path_publications_;
        break;
      case PathPublicationReason::kHoldNoPose:
      case PathPublicationReason::kHoldNoPlanningGrid:
      case PathPublicationReason::kHoldInvalidPath:
      case PathPublicationReason::kHoldAfterPlanningFailure:
        break;
    }
  }

  [[nodiscard]] std::string plannerCountersSummary() const {
    std::ostringstream summary;
    summary << "astar_runs=" << astar_runs_ << " astar_successes=" << astar_successes_
            << " astar_failures=" << astar_failures_
            << " prohibited_replans=" << prohibited_replans_
            << " path_publications=" << path_publications_
            << " non_empty_path_publications=" << non_empty_path_publications_
            << " hold_path_publications=" << hold_path_publications_
            << " computed_path_publications=" << computed_path_publications_;
    return summary.str();
  }

  void logPlannerCountersThrottled() {
    const std::string counters = plannerCountersSummary();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Planner counters: %s",
                         counters.c_str());
  }

  std::optional<OccupancyGrid2D> memory_grid_;
  std::optional<OccupancyGrid2D> static_grid_;
  PlannerCore planner_core_;
  AStarConfig astar_config_{};
  TrajectoryPlannerConfig trajectory_planner_config_{};

  Pose2 current_pose_{};
  Point2 current_velocity_{};
  AttitudeEuler current_attitude_{};
  LidarProjectionPose last_scan_projection_pose_{};
  Point2 start_{};
  Point2 goal_{};
  Point2 px4_local_origin_{};
  sensor_msgs::msg::LaserScan last_scan_;
  bool pose_valid_{false};
  bool local_position_seen_{false};
  bool memory_grid_seen_{false};
  bool scan_seen_{false};
  bool scan_seen_logged_{false};
  bool stable_path_reuse_enabled_{true};
  bool use_static_map_{true};
  bool use_obstacle_memory_{true};
  bool use_current_lidar_obstacles_{true};
  bool use_px4_heading_for_scan_{false};
  bool motion_compensate_lidar_pose_{true};
  bool compensate_lidar_attitude_{false};
  bool altitude_valid_{false};
  bool attitude_valid_{false};
  bool current_velocity_valid_{false};
  bool last_scan_projection_pose_valid_{false};
  std::string frame_id_{"map"};
  std::string static_map_path_param_{"worlds/generated_city.map2d"};
  std::filesystem::path static_map_resolved_path_;
  GridBounds fallback_grid_bounds_{-10.0, -10.0, 0.5, 230, 350};
  double inflation_radius_m_{2.5};
  double static_map_min_blocking_height_m_{0.0};
  double stable_path_goal_tolerance_m_{3.0};
  double max_lidar_range_m_{35.0};
  double range_hit_epsilon_m_{0.05};
  double scan_yaw_offset_rad_{0.0};
  double initial_heading_rad_{0.0};
  double static_map_debug_publish_period_s_{1.0};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double current_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_scan_pose_lag_s_{0.0};
  double last_scan_pose_latency_s_{0.0};
  double last_scan_motion_shift_m_{0.0};
  double lidar_pose_latency_s_{0.05};
  Point2 last_scan_motion_shift_{};
  double lidar_z_offset_m_{0.0};
  double lidar_mount_roll_rad_{0.0};
  double lidar_mount_pitch_rad_{0.0};
  double lidar_mount_yaw_rad_{0.0};
  double min_projected_lidar_altitude_m_{0.0};
  double max_projected_lidar_altitude_m_{100000.0};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t max_current_lidar_staleness_ns_{750'000'000};
  std::int64_t last_pose_update_ns_{0};
  std::int64_t last_scan_update_ns_{0};
  int memory_occupied_value_{100};
  int memory_free_value_{0};
  std::size_t last_logged_path_size_{std::numeric_limits<std::size_t>::max()};
  std::size_t static_map_rectangles_{0U};
  std::size_t static_map_occupied_cells_{0U};
  std::uint64_t astar_runs_{0U};
  std::uint64_t astar_successes_{0U};
  std::uint64_t astar_failures_{0U};
  std::uint64_t prohibited_replans_{0U};
  std::uint64_t path_publications_{0U};
  std::uint64_t non_empty_path_publications_{0U};
  std::uint64_t hold_path_publications_{0U};
  std::uint64_t computed_path_publications_{0U};
  std::uint64_t next_path_id_{1U};
  Point2 last_logged_path_first_{};
  Point2 last_logged_path_last_{};
  std::vector<Point2> last_valid_path_points_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr memory_grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr prohibited_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_map_points_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr path_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_diagnostics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::TimerBase::SharedPtr static_map_debug_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
