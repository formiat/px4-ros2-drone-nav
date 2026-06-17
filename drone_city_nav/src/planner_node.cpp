#include "drone_city_nav/current_lidar_overlay.hpp"
#include "drone_city_nav/grid_overlay.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/path_smoothing.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/planner_node_config.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/static_map_debug.hpp"
#include "drone_city_nav/static_map_source.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
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

constexpr double kPublishedPathCollinearityToleranceM = 0.05;

[[nodiscard]] double
pathDistanceToSegmentStart(const std::span<const Point2> path_points,
                           const std::size_t segment_start_index) noexcept {
  if (path_points.size() < 2U) {
    return 0.0;
  }

  const std::size_t bounded_segment_start =
      std::min(segment_start_index, path_points.size() - 1U);
  double distance_m = 0.0;
  for (std::size_t index = 1U; index <= bounded_segment_start; ++index) {
    distance_m += distance(path_points[index - 1U], path_points[index]);
  }
  return distance_m;
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

    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        config.topics.occupancy_grid, rclcpp::QoS{1}.transient_local());
    static_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        config.topics.static_map_grid, rclcpp::QoS{1}.transient_local());
    static_map_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        config.topics.static_map_points, rclcpp::QoS{1}.transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(config.topics.path,
                                                      rclcpp::QoS{1}.reliable());
    waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        config.topics.current_waypoint, rclcpp::QoS{1}.reliable());

    loadConfiguredStaticMap();
    if (static_map_debug_publish_period_s_ > 0.0) {
      static_map_debug_timer_ = create_wall_timer(
          std::chrono::duration<double>{static_map_debug_publish_period_s_},
          [this]() { republishStaticMapDebug(); });
    }
    timer_ = create_wall_timer(
        std::chrono::duration<double>{std::max(0.05, config.timing.replan_period_s)},
        [this]() { replanAndPublish(); });

    RCLCPP_INFO(get_logger(),
                "Planner ready: start=(%.1f, %.1f) goal=(%.1f, %.1f) "
                "inflation=%.2fm",
                start_.x, start_.y, goal_.x, goal_.y, inflation_radius_m_);
    RCLCPP_INFO(get_logger(),
                "Planner subscriptions: obstacle_memory_grid='%s' local_position='%s' "
                "attitude='%s'",
                config.topics.obstacle_memory_grid.c_str(),
                config.topics.local_position.c_str(), config.topics.attitude.c_str());
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
                "max_staleness=%.2fs depth=%.2f swap_lidar_xy=%s yaw_source=%s "
                "compensate_attitude=%s lidar_z_offset=%.2f "
                "projected_altitude_range=[%.2f, %.2f] "
                "lidar_mount_rpy=(%.3f, %.3f, %.3f)",
                use_current_lidar_obstacles_ ? "true" : "false",
                config.topics.lidar.c_str(), max_lidar_range_m_,
                static_cast<double>(max_current_lidar_staleness_ns_) / 1.0e9,
                current_lidar_obstacle_depth_m_,
                swap_lidar_xy_to_local_frame_ ? "true" : "false",
                use_px4_heading_for_scan_ ? "px4_heading" : "initial_map_aligned",
                compensate_lidar_attitude_ ? "true" : "false", lidar_z_offset_m_,
                min_projected_lidar_altitude_m_, max_projected_lidar_altitude_m_,
                lidar_mount_roll_rad_, lidar_mount_pitch_rad_, lidar_mount_yaw_rad_);
    if (compensate_lidar_attitude_ && swap_lidar_xy_to_local_frame_) {
      RCLCPP_WARN(
          get_logger(),
          "Planner current lidar overlay is using legacy swap_lidar_xy_to_local_frame "
          "with attitude compensation. Prefer lidar_mount_* parameters for physical "
          "3D projection.");
    }
    RCLCPP_INFO(
        get_logger(),
        "Planner fallback policy: direct_path_fallback=%s "
        "reuse_last_valid_path_on_failure=%s "
        "max_initial_lateral_deviation=%.2fm "
        "stable_path_reuse=%s stable_max_deviation=%.2fm "
        "stable_goal_tolerance=%.2fm stable_blocking_blocked_length=%.2fm "
        "stable_blocking_replan_horizon=%.2fm "
        "stable_blocked_confirmations=%d",
        direct_path_fallback_ ? "true" : "false",
        reuse_last_valid_path_on_failure_ ? "true" : "false",
        max_initial_lateral_deviation_m_, stable_path_reuse_enabled_ ? "true" : "false",
        stable_path_reuse_max_deviation_m_, stable_path_goal_tolerance_m_,
        stable_path_blocking_blocked_length_m_, stable_path_blocking_replan_horizon_m_,
        stable_path_blocked_confirmations_required_);
    RCLCPP_INFO(get_logger(),
                "Planner obstacle clearance preference: astar_radius=%.2fm "
                "astar_weight=%.2f astar_turn_weight=%.2f evasive_maneuvering=%s "
                "evasive_straight_weight=%.2f smoothing_min_clearance=%.2fm "
                "comfort_max_detour_ratio=%.2f",
                astar_config_.obstacle_clearance_cost_radius_m,
                astar_config_.obstacle_clearance_cost_weight,
                astar_config_.turn_cost_weight,
                astar_config_.evasive_maneuvering_enabled ? "true" : "false",
                astar_config_.evasive_maneuvering_straight_cost_weight,
                path_smoothing_config_.minimum_obstacle_clearance_m,
                planner_core_.config().comfort_path_max_detour_ratio);
  }

private:
  void applyConfig(const PlannerNodeConfig& config) {
    frame_id_ = config.frame_id;
    start_ = config.start;
    goal_ = config.goal;
    cruise_altitude_m_ = config.cruise_altitude_m;
    inflation_radius_m_ = config.inflation_radius_m;
    max_pose_staleness_ns_ = config.timing.max_pose_staleness_ns;
    direct_path_fallback_ = config.fallback.direct_path_fallback;
    reuse_last_valid_path_on_failure_ =
        config.fallback.reuse_last_valid_path_on_failure;
    stable_path_reuse_enabled_ = config.fallback.stable_path_reuse_enabled;
    stable_path_reuse_max_deviation_m_ =
        config.planner_core.stable_path_reuse_max_deviation_m;
    stable_path_goal_tolerance_m_ = config.planner_core.stable_path_goal_tolerance_m;
    stable_path_blocking_blocked_length_m_ =
        config.planner_core.stable_path_blocking_blocked_length_m;
    stable_path_blocking_replan_horizon_m_ =
        config.planner_core.stable_path_blocking_replan_horizon_m;
    stable_path_blocked_confirmations_required_ =
        config.planner_core.stable_path_blocked_confirmations_required;
    max_initial_lateral_deviation_m_ = config.fallback.max_initial_lateral_deviation_m;
    nearest_free_radius_cells_ = config.planner_core.nearest_free_radius_cells;
    occupied_threshold_ = config.memory_grid.occupied_threshold;
    free_threshold_ = config.memory_grid.free_threshold;
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
    current_lidar_obstacle_depth_m_ = config.current_lidar.obstacle_depth_m;
    scan_yaw_offset_rad_ = config.lidar_projection.scan_yaw_offset_rad;
    use_px4_heading_for_scan_ = config.current_lidar.use_px4_heading_for_scan;
    swap_lidar_xy_to_local_frame_ =
        config.lidar_projection.swap_lidar_xy_to_local_frame;
    compensate_lidar_attitude_ = config.lidar_projection.compensate_attitude;
    lidar_mount_roll_rad_ = config.lidar_projection.lidar_mount_roll_rad;
    lidar_mount_pitch_rad_ = config.lidar_projection.lidar_mount_pitch_rad;
    lidar_mount_yaw_rad_ = config.lidar_projection.lidar_mount_yaw_rad;
    lidar_z_offset_m_ = config.lidar_projection.lidar_z_offset_m;
    min_projected_lidar_altitude_m_ = config.lidar_projection.min_projected_altitude_m;
    max_projected_lidar_altitude_m_ = config.lidar_projection.max_projected_altitude_m;
    astar_config_ = config.planner_core.astar;
    path_smoothing_config_ = config.path_smoothing;
    initial_heading_rad_ = config.initial_pose.heading_rad;
    px4_local_origin_ = config.initial_pose.px4_local_origin;
    static_map_debug_publish_period_s_ =
        config.timing.static_map_debug_publish_period_s;
    planner_core_.setConfig(config.planner_core);
  }

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
    if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
      invalidateCurrentPose();
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
    OccupancyGridFromRosResult converted = occupancyGridFromRos(
        msg, OccupancyGridFromRosConfig{occupied_threshold_, free_threshold_});
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
    if (!scan_seen_logged_) {
      scan_seen_logged_ = true;
      RCLCPP_INFO(get_logger(),
                  "First planner lidar scan: beams=%zu range=[%.2f, %.2f] "
                  "angle=[%.2f, %.2f]",
                  last_scan_.ranges.size(), static_cast<double>(last_scan_.range_min),
                  static_cast<double>(last_scan_.range_max),
                  static_cast<double>(last_scan_.angle_min),
                  static_cast<double>(last_scan_.angle_max));
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
          "Planner has no enabled obstacle sources; publishing hold path");
      return std::nullopt;
    }
    if (result.status == PlanningGridStatus::kStaticMapEnabledButMissing) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner static map source is enabled but not loaded; "
                           "publishing hold path");
      return std::nullopt;
    }
    if (result.status == PlanningGridStatus::kNoReadySourceData ||
        !result.grid.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner has no ready obstacle source data; publishing hold "
                           "path status=%s",
                           planningGridStatusName(result.status));
      return std::nullopt;
    }

    return result;
  }

  void replanAndPublish() {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const bool pose_fresh =
        timestampIsFresh(last_pose_update_ns_, now_ns, max_pose_staleness_ns_);
    const double pose_age_s = poseAgeSeconds(now_ns);
    if (pose_valid_ && !pose_fresh) {
      invalidateCurrentPose();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planner invalidated stale PX4 local position before replanning: "
          "pose_age_s=%.2f",
          pose_age_s);
    }
    if (!pose_valid_ || !finite2D(current_pose_.position)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner is waiting for a valid PX4 local position; "
                           "publishing hold path");
      publishPath({});
      return;
    }
    auto planning_result = buildPlanningGrid(now_ns);
    if (!planning_result.has_value()) {
      publishPath({});
      return;
    }
    if (!planning_result->grid.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner grid builder returned no grid despite a ready "
                           "result; publishing hold path");
      publishPath({});
      return;
    }
    OccupancyGrid2D planning_grid = std::move(*planning_result->grid);
    publishOccupancyGrid(planning_grid);
    if (keepCurrentPathIfStillClear(planning_grid)) {
      return;
    }

    auto path_result = computePathOnGrid(planning_grid, "combined");
    if (!path_result.has_value()) {
      if (!publishStaticOnlyFallbackPath("combined planning failure")) {
        publishFallbackPath();
      }
      return;
    }
    if (shouldRejectLowClearancePath(path_result->smoothed_path_clearance_m)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Rejecting low-clearance combined path; trying fallback/hold: "
          "raw_clearance=%.2f smoothed_clearance=%.2f threshold=%.2f",
          path_result->raw_path_clearance_m, path_result->smoothed_path_clearance_m,
          lowClearanceFallbackThresholdM());
      if (!publishStaticOnlyFallbackPath("combined low-clearance path")) {
        publishFallbackPath();
      }
      return;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planning summary: pose=(%.2f, %.2f) distance_to_start=%.2f "
        "distance_to_goal=%.2f occupied=%zu inflated=%zu free=%zu unknown=%zu "
        "static[enabled=%s loaded=%s used=%s rectangles=%zu occupied_cells=%zu "
        "path='%s'] "
        "memory[enabled=%s seen=%s used=%s geometry_matches=%s occupied=%zu free=%zu "
        "unknown=%zu overlay_occupied=%zu overlay_free=%zu] "
        "current_lidar[enabled=%s used=%s fresh=%s processed=%zu hits=%zu "
        "altitude_rejected=%zu occupied_cells=%zu outside=%zu] "
        "source=combined astar_status=%s expanded=%zu cost=%.2f raw_path=%zu "
        "smoothed_path=%zu "
        "path_metrics[raw_segments=%zu raw_straight_segments=%zu raw_turns=%zu "
        "raw_length=%.2f smoothed_segments=%zu smoothed_straight_segments=%zu "
        "smoothed_turns=%zu smoothed_length=%.2f] "
        "path_clearance[raw=%.2f smoothed=%.2f] "
        "comfort_detour[enabled=%s selected=%s shortest_length=%.2f "
        "comfort_length=%.2f length_limit=%.2f]",
        current_pose_.position.x, current_pose_.position.y,
        distance(current_pose_.position, start_),
        distance(current_pose_.position, goal_), path_result->grid_stats.occupied_cells,
        path_result->grid_stats.inflated_cells, path_result->grid_stats.free_cells,
        path_result->grid_stats.unknown_cells,
        planning_result->static_source.enabled ? "true" : "false",
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
        planning_result->current_lidar.outside_hits,
        astarStatusName(path_result->astar.status), path_result->astar.expanded_cells,
        path_result->astar.total_cost, path_result->raw_path_metrics.points,
        path_result->smoothed_path_metrics.points,
        path_result->raw_path_metrics.segments,
        path_result->raw_path_metrics.straight_segments,
        path_result->raw_path_metrics.turns, path_result->raw_path_metrics.length_m,
        path_result->smoothed_path_metrics.segments,
        path_result->smoothed_path_metrics.straight_segments,
        path_result->smoothed_path_metrics.turns,
        path_result->smoothed_path_metrics.length_m, path_result->raw_path_clearance_m,
        path_result->smoothed_path_clearance_m,
        path_result->comfort_path_detour_limited ? "true" : "false",
        path_result->comfort_path_selected ? "true" : "false",
        path_result->shortest_path_length_m, path_result->comfort_path_length_m,
        path_result->comfort_path_length_limit_m);
    publishPathFromSmoothedCells(planning_grid, path_result->smoothed_cells,
                                 "combined");
  }

  [[nodiscard]] std::optional<PathComputationResult>
  computePathOnGrid(const OccupancyGrid2D& grid, const char* source_label) {
    const auto start_cell = grid.worldToCell(current_pose_.position);
    const auto goal_cell = grid.worldToCell(goal_);
    if (!start_cell.has_value() || !goal_cell.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Start or goal is outside the %s planning grid",
                           source_label);
      return std::nullopt;
    }

    const auto unblocked_start = grid.nearestUnblocked(
        *start_cell, planner_core_.config().nearest_free_radius_cells);
    const auto unblocked_goal = grid.nearestUnblocked(
        *goal_cell, planner_core_.config().nearest_free_radius_cells);
    if (!unblocked_start.has_value() || !unblocked_goal.has_value()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No unblocked start or goal cell is available on %s grid after inflation",
          source_label);
      return std::nullopt;
    }

    if (*unblocked_start != *start_cell || *unblocked_goal != *goal_cell) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planning endpoints adjusted on %s grid after inflation: "
                           "start=(%d,%d)->(%d,%d) goal=(%d,%d)->(%d,%d)",
                           source_label, start_cell->x, start_cell->y,
                           unblocked_start->x, unblocked_start->y, goal_cell->x,
                           goal_cell->y, unblocked_goal->x, unblocked_goal->y);
    }

    auto result = planner_core_.computePath(grid, current_pose_.position, goal_);
    if (!result.has_value()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "A* did not find a path on %s grid: start=(%d,%d) goal=(%d,%d)", source_label,
          unblocked_start->x, unblocked_start->y, unblocked_goal->x, unblocked_goal->y);
      return std::nullopt;
    }

    return result;
  }

  bool publishPathFromSmoothedCells(const OccupancyGrid2D& grid,
                                    const std::vector<GridIndex>& smoothed_cells,
                                    const char* source_label) {
    std::vector<Point2> path_points = cellsToPoints(grid, smoothed_cells);
    if (path_points.empty()) {
      publishPath({});
      return false;
    }

    if (distance(current_pose_.position, path_points.front()) < grid.resolution()) {
      path_points.front() = current_pose_.position;
    } else {
      path_points.insert(path_points.begin(), current_pose_.position);
    }

    const std::size_t dense_path_points = path_points.size();
    path_points =
        collapseCollinearPath(path_points, kPublishedPathCollinearityToleranceM);
    if (path_points.size() != dense_path_points) {
      RCLCPP_INFO(get_logger(),
                  "%s path collinear waypoints collapsed: before=%zu after=%zu "
                  "tolerance=%.2fm",
                  source_label, dense_path_points, path_points.size(),
                  kPublishedPathCollinearityToleranceM);
    }

    if (!pathIsPublishableAfterFinalEdits(grid, path_points, source_label)) {
      publishPath({});
      return false;
    }
    if (hasExcessiveInitialLateralDeviation(path_points)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "%s path has excessive initial lateral deviation; using "
                           "direct fallback path",
                           source_label);
      publishDirectGoalPath();
      return true;
    }

    last_valid_path_points_ = path_points;
    stable_path_blocked_confirmations_ = 0;
    publishPath(path_points);
    return true;
  }

  [[nodiscard]] bool escapeSegmentLeavesInflation(const OccupancyGrid2D& grid,
                                                  const Point2 start,
                                                  const Point2 end) const {
    const auto start_cell = grid.worldToCell(start);
    const auto end_cell = grid.worldToCell(end);
    if (!start_cell.has_value() || !end_cell.has_value()) {
      return false;
    }
    if (!grid.isBlocked(*start_cell) || grid.isOccupied(*start_cell) ||
        grid.isBlocked(*end_cell)) {
      return false;
    }

    const double occupied_length_m = pathSegmentOccupiedLengthM(grid, start, end);
    return std::isfinite(occupied_length_m) && occupied_length_m <= 0.0;
  }

  [[nodiscard]] bool
  pathIsPublishableAfterFinalEdits(const OccupancyGrid2D& grid,
                                   std::span<const Point2> path_points,
                                   const char* source_label) {
    if (path_points.size() < 2U) {
      return true;
    }

    for (std::size_t index = 1U; index < path_points.size(); ++index) {
      const Point2 segment_start = path_points[index - 1U];
      const Point2 segment_end = path_points[index];
      if (pathSegmentIsUnblocked(grid, segment_start, segment_end)) {
        continue;
      }

      const std::size_t segment_index = index - 1U;
      if (segment_index == 0U &&
          escapeSegmentLeavesInflation(grid, segment_start, segment_end)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "%s path allows first segment as an inflated-zone escape: "
                             "segment=0 start=(%.2f, %.2f) end=(%.2f, %.2f)",
                             source_label, segment_start.x, segment_start.y,
                             segment_end.x, segment_end.y);
        continue;
      }

      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "%s path segment intersects blocked cells after final path edits: "
          "segment=%zu start=(%.2f, %.2f) end=(%.2f, %.2f)",
          source_label, segment_index, segment_start.x, segment_start.y, segment_end.x,
          segment_end.y);
      return false;
    }

    return true;
  }

  [[nodiscard]] double currentLidarRangeMax() const {
    return std::min(static_cast<double>(last_scan_.range_max), max_lidar_range_m_);
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
                                 swap_lidar_xy_to_local_frame_,
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
            currentLidarProjectionPose(), currentLidarProjectionConfig(),
            current_lidar_obstacle_depth_m_);
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
    const StaticMapDebugConfig config{makePlannerHeader(), 0.05F};
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

  void publishOccupancyGrid(const OccupancyGrid2D& grid) {
    occupancy_pub_->publish(
        occupancyGridToRos(grid, OccupancyGridToRosConfig{makePlannerHeader(), true}));
  }

  void publishPath(const std::vector<Point2>& points) {
    if (points.empty()) {
      last_valid_path_points_.clear();
      stable_path_blocked_confirmations_ = 0;
    }

    const PathMetrics metrics = pointPathMetrics(points);
    const nav_msgs::msg::Path path =
        pathToRos(std::span<const Point2>{points.data(), points.size()},
                  makePlannerHeader(), cruise_altitude_m_);

    path_pub_->publish(path);
    if (!path.poses.empty()) {
      waypoint_pub_->publish(path.poses.front());
    }

    logPathUpdate(path, metrics);
  }

  void publishLastValidPathOrEmpty() {
    if (reuse_last_valid_path_on_failure_ && !last_valid_path_points_.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Reusing last valid path after replanning failure: waypoints=%zu",
          last_valid_path_points_.size());
      publishPath(last_valid_path_points_);
      return;
    }

    if (!last_valid_path_points_.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Clearing path after replanning failure; holding position instead of reusing "
          "stale waypoints");
    }
    publishPath({});
  }

  void publishFallbackPath() {
    if (direct_path_fallback_) {
      publishDirectGoalPath();
      return;
    }

    publishLastValidPathOrEmpty();
  }

  bool publishStaticOnlyFallbackPath(const char* reason) {
    if (!use_static_map_ || !static_grid_.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Static-only fallback is unavailable after %s: "
                           "use_static_map=%s loaded=%s",
                           reason, use_static_map_ ? "true" : "false",
                           static_grid_.has_value() ? "true" : "false");
      return false;
    }

    OccupancyGrid2D static_only_grid{static_grid_->bounds()};
    const GridOverlayStats static_overlay =
        overlayOccupiedCells(static_only_grid, *static_grid_);
    static_only_grid.rebuildInflation(inflation_radius_m_);

    auto path_result = computePathOnGrid(static_only_grid, "static-only fallback");
    if (!path_result.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Static-only fallback could not produce a path after %s: "
                           "static_occupied_cells=%zu applied=%zu",
                           reason, static_overlay.source_occupied_cells,
                           static_overlay.occupied_cells_applied);
      return false;
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Using static-only fallback path after %s: static_occupied_cells=%zu "
        "astar_status=%s expanded=%zu cost=%.2f raw_path=%zu smoothed_path=%zu "
        "path_metrics[raw_segments=%zu raw_straight_segments=%zu raw_turns=%zu "
        "raw_length=%.2f smoothed_segments=%zu smoothed_straight_segments=%zu "
        "smoothed_turns=%zu smoothed_length=%.2f] "
        "path_clearance[raw=%.2f smoothed=%.2f]",
        reason, static_overlay.source_occupied_cells,
        astarStatusName(path_result->astar.status), path_result->astar.expanded_cells,
        path_result->astar.total_cost, path_result->raw_path_metrics.points,
        path_result->smoothed_path_metrics.points,
        path_result->raw_path_metrics.segments,
        path_result->raw_path_metrics.straight_segments,
        path_result->raw_path_metrics.turns, path_result->raw_path_metrics.length_m,
        path_result->smoothed_path_metrics.segments,
        path_result->smoothed_path_metrics.straight_segments,
        path_result->smoothed_path_metrics.turns,
        path_result->smoothed_path_metrics.length_m, path_result->raw_path_clearance_m,
        path_result->smoothed_path_clearance_m);
    publishOccupancyGrid(static_only_grid);
    return publishPathFromSmoothedCells(static_only_grid, path_result->smoothed_cells,
                                        "static-only fallback");
  }

  void publishDirectGoalPath() {
    std::vector<Point2> path_points{current_pose_.position, goal_};
    last_valid_path_points_ = path_points;
    stable_path_blocked_confirmations_ = 0;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Publishing direct fallback path: start=(%.2f, %.2f) goal=(%.2f, %.2f)",
        current_pose_.position.x, current_pose_.position.y, goal_.x, goal_.y);
    publishPath(path_points);
  }

  [[nodiscard]] double lowClearanceFallbackThresholdM() const noexcept {
    if (!(path_smoothing_config_.minimum_obstacle_clearance_m > 0.0)) {
      return 0.0;
    }
    return 0.75 * path_smoothing_config_.minimum_obstacle_clearance_m;
  }

  [[nodiscard]] bool
  shouldRejectLowClearancePath(const double smoothed_path_clearance_m) const noexcept {
    return use_static_map_ && static_grid_.has_value() &&
           lowClearanceFallbackThresholdM() > 0.0 &&
           std::isfinite(smoothed_path_clearance_m) &&
           smoothed_path_clearance_m < lowClearanceFallbackThresholdM();
  }

  void invalidateCurrentPose() {
    current_pose_ = Pose2{};
    current_altitude_m_ = std::numeric_limits<double>::quiet_NaN();
    pose_valid_ = false;
    altitude_valid_ = false;
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

  [[nodiscard]] bool
  hasExcessiveInitialLateralDeviation(const std::vector<Point2>& path_points) const {
    if (!direct_path_fallback_ || path_points.size() < 3U) {
      return false;
    }

    const Point2 origin = path_points.front();
    const double lookahead_x = origin.x + 20.0;
    for (const Point2 point : path_points) {
      if (point.x > lookahead_x) {
        break;
      }
      if (std::abs(point.y - origin.y) > max_initial_lateral_deviation_m_) {
        return true;
      }
    }

    return false;
  }

  bool keepCurrentPathIfStillClear(const OccupancyGrid2D& grid) {
    if (!stable_path_reuse_enabled_ || last_valid_path_points_.size() < 2U) {
      return false;
    }

    const StablePathDecision decision = planner_core_.evaluateStablePath(
        grid, last_valid_path_points_, current_pose_.position, goal_,
        stable_path_blocked_confirmations_);
    if (decision.reason == StablePathDecisionReason::kDeviationTooLarge) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Current path cannot be reused because the vehicle is too far from it: "
          "deviation=%.2fm limit=%.2fm",
          decision.deviation_m, stable_path_reuse_max_deviation_m_);
    }
    if (decision.reason == StablePathDecisionReason::kGoalMismatch ||
        decision.reason == StablePathDecisionReason::kProjectionUnavailable ||
        decision.reason == StablePathDecisionReason::kDeviationTooLarge ||
        decision.reason == StablePathDecisionReason::kNoPreviousPath ||
        decision.reason == StablePathDecisionReason::kDisabled) {
      return false;
    }

    if ((decision.reason == StablePathDecisionReason::kBlockedUnconfirmed ||
         decision.reason == StablePathDecisionReason::kBlockedConfirmed) &&
        keepInflationEscapePathIfApplicable(grid, decision)) {
      return true;
    }
    if ((decision.reason == StablePathDecisionReason::kBlockedUnconfirmed ||
         decision.reason == StablePathDecisionReason::kBlockedConfirmed) &&
        deferDistantStablePathBlockIfApplicable(decision)) {
      return true;
    }

    if (decision.reason == StablePathDecisionReason::kBlockedUnconfirmed) {
      stable_path_blocked_confirmations_ = decision.blocked_confirmations;
      last_valid_path_points_ = decision.remaining_path;
      const Point2 blocking_start =
          decision.blocking_segment_index < last_valid_path_points_.size()
              ? last_valid_path_points_[decision.blocking_segment_index]
              : Point2{};
      const Point2 blocking_end =
          decision.blocking_segment_index + 1U < last_valid_path_points_.size()
              ? last_valid_path_points_[decision.blocking_segment_index + 1U]
              : Point2{};
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Current path has an unconfirmed blocked intersection; keeping current "
          "path until it is confirmed: reason=%s confirmations=%d/%d "
          "remaining_waypoints=%zu deviation=%.2fm blocked_segment=%zu "
          "blocked_length=%.2fm segment_start=(%.2f, %.2f) "
          "segment_end=(%.2f, %.2f)",
          stablePathDecisionReasonName(decision.reason),
          stable_path_blocked_confirmations_,
          stable_path_blocked_confirmations_required_, last_valid_path_points_.size(),
          decision.deviation_m, decision.blocking_segment_index,
          decision.blocking_blocked_length_m, blocking_start.x, blocking_start.y,
          blocking_end.x, blocking_end.y);
      return true;
    }

    if (decision.reason == StablePathDecisionReason::kBlockedConfirmed) {
      stable_path_blocked_confirmations_ = decision.blocked_confirmations;
      const Point2 blocking_start =
          decision.blocking_segment_index < decision.remaining_path.size()
              ? decision.remaining_path[decision.blocking_segment_index]
              : Point2{};
      const Point2 blocking_end =
          decision.blocking_segment_index + 1U < decision.remaining_path.size()
              ? decision.remaining_path[decision.blocking_segment_index + 1U]
              : Point2{};
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Current path intersects confirmed newly available occupied obstacle data; "
          "running A* from current pose: reason=%s confirmations=%d/%d "
          "remaining_waypoints=%zu deviation=%.2fm blocked_segment=%zu "
          "blocked_length=%.2fm segment_start=(%.2f, %.2f) "
          "segment_end=(%.2f, %.2f)",
          stablePathDecisionReasonName(decision.reason),
          stable_path_blocked_confirmations_,
          stable_path_blocked_confirmations_required_, decision.remaining_path.size(),
          decision.deviation_m, decision.blocking_segment_index,
          decision.blocking_blocked_length_m, blocking_start.x, blocking_start.y,
          blocking_end.x, blocking_end.y);
      return false;
    }

    stable_path_blocked_confirmations_ = 0;
    last_valid_path_points_ = decision.remaining_path;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Keeping current path because the remaining path is still clear: "
        "reason=%s remaining_waypoints=%zu deviation=%.2fm distance_to_goal=%.2f",
        stablePathDecisionReasonName(decision.reason), last_valid_path_points_.size(),
        decision.deviation_m, distance(current_pose_.position, goal_));
    return true;
  }

  bool deferDistantStablePathBlockIfApplicable(const StablePathDecision& decision) {
    if (!(stable_path_blocking_replan_horizon_m_ > 0.0)) {
      return false;
    }
    const double distance_to_blocked_segment_m = pathDistanceToSegmentStart(
        decision.remaining_path, decision.blocking_segment_index);
    if (!std::isfinite(distance_to_blocked_segment_m) ||
        distance_to_blocked_segment_m <= stable_path_blocking_replan_horizon_m_) {
      return false;
    }

    stable_path_blocked_confirmations_ = 0;
    last_valid_path_points_ = decision.remaining_path;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Deferring current-path replan because the blocked segment is beyond the "
        "near planning horizon: reason=%s remaining_waypoints=%zu deviation=%.2fm "
        "blocked_segment=%zu blocked_length=%.2fm "
        "distance_to_blocked_segment=%.2fm horizon=%.2fm",
        stablePathDecisionReasonName(decision.reason), decision.remaining_path.size(),
        decision.deviation_m, decision.blocking_segment_index,
        decision.blocking_blocked_length_m, distance_to_blocked_segment_m,
        stable_path_blocking_replan_horizon_m_);
    return true;
  }

  bool keepInflationEscapePathIfApplicable(const OccupancyGrid2D& grid,
                                           const StablePathDecision& decision) {
    if (decision.blocking_segment_index != 0U || decision.remaining_path.size() < 2U) {
      return false;
    }
    const Point2 segment_start = decision.remaining_path[0];
    const Point2 segment_end = decision.remaining_path[1];
    if (!escapeSegmentLeavesInflation(grid, segment_start, segment_end)) {
      return false;
    }

    stable_path_blocked_confirmations_ = 0;
    last_valid_path_points_ = decision.remaining_path;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Keeping current path because the first blocked segment is an "
        "inflated-zone escape: reason=%s remaining_waypoints=%zu deviation=%.2fm "
        "blocked_length=%.2fm segment_start=(%.2f, %.2f) "
        "segment_end=(%.2f, %.2f)",
        stablePathDecisionReasonName(decision.reason), decision.remaining_path.size(),
        decision.deviation_m, decision.blocking_blocked_length_m, segment_start.x,
        segment_start.y, segment_end.x, segment_end.y);
    return true;
  }

  void logPathUpdate(const nav_msgs::msg::Path& path, const PathMetrics& metrics) {
    const std::size_t path_size = path.poses.size();
    const bool path_changed = path_size != last_logged_path_size_;
    if (path_size == 0U) {
      if (path_changed) {
        RCLCPP_WARN(get_logger(), "Published empty path");
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
                  "Published path: waypoints=%zu segments=%zu "
                  "straight_segments=%zu turns=%zu length=%.2f first=(%.2f, %.2f) "
                  "last=(%.2f, %.2f) preview=%s",
                  path_size, metrics.segments, metrics.straight_segments, metrics.turns,
                  metrics.length_m, first.x, first.y, last.x, last.y,
                  preview.str().c_str());
      last_logged_path_size_ = path_size;
      last_logged_path_first_ = first;
      last_logged_path_last_ = last;
    }
  }

  std::optional<OccupancyGrid2D> memory_grid_;
  std::optional<OccupancyGrid2D> static_grid_;
  PlannerCore planner_core_;
  AStarConfig astar_config_{};
  PathSmoothingConfig path_smoothing_config_{};

  Pose2 current_pose_{};
  AttitudeEuler current_attitude_{};
  Point2 start_{};
  Point2 goal_{};
  Point2 px4_local_origin_{};
  sensor_msgs::msg::LaserScan last_scan_;
  bool pose_valid_{false};
  bool local_position_seen_{false};
  bool memory_grid_seen_{false};
  bool scan_seen_{false};
  bool scan_seen_logged_{false};
  bool direct_path_fallback_{false};
  bool reuse_last_valid_path_on_failure_{false};
  bool stable_path_reuse_enabled_{true};
  bool use_static_map_{true};
  bool use_obstacle_memory_{true};
  bool use_current_lidar_obstacles_{true};
  bool use_px4_heading_for_scan_{false};
  bool swap_lidar_xy_to_local_frame_{false};
  bool compensate_lidar_attitude_{false};
  bool altitude_valid_{false};
  bool attitude_valid_{false};
  std::string frame_id_{"map"};
  std::string static_map_path_param_{"worlds/generated_city.map2d"};
  std::filesystem::path static_map_resolved_path_;
  GridBounds fallback_grid_bounds_{-10.0, -10.0, 0.5, 230, 350};
  double inflation_radius_m_{2.5};
  double cruise_altitude_m_{12.0};
  double static_map_min_blocking_height_m_{0.0};
  double max_initial_lateral_deviation_m_{8.0};
  double stable_path_reuse_max_deviation_m_{12.0};
  double stable_path_goal_tolerance_m_{3.0};
  double stable_path_blocking_blocked_length_m_{2.0};
  double stable_path_blocking_replan_horizon_m_{25.0};
  double max_lidar_range_m_{35.0};
  double range_hit_epsilon_m_{0.05};
  double current_lidar_obstacle_depth_m_{0.0};
  double scan_yaw_offset_rad_{0.0};
  double initial_heading_rad_{0.0};
  double static_map_debug_publish_period_s_{1.0};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
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
  int nearest_free_radius_cells_{10};
  int occupied_threshold_{65};
  int free_threshold_{0};
  int stable_path_blocked_confirmations_required_{2};
  int stable_path_blocked_confirmations_{0};
  std::size_t last_logged_path_size_{std::numeric_limits<std::size_t>::max()};
  std::size_t static_map_rectangles_{0U};
  std::size_t static_map_occupied_cells_{0U};
  Point2 last_logged_path_first_{};
  Point2 last_logged_path_last_{};
  std::vector<Point2> last_valid_path_points_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr memory_grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_map_points_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
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
