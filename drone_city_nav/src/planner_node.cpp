#include "drone_city_nav/astar_planner.hpp"
#include "drone_city_nav/grid_overlay.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/path_smoothing.hpp"
#include "drone_city_nav/static_city_map.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

struct GridStats {
  std::size_t unknown_cells{0U};
  std::size_t free_cells{0U};
  std::size_t occupied_cells{0U};
  std::size_t inflated_cells{0U};
};

struct CurrentLidarStats {
  bool enabled{false};
  bool used{false};
  bool fresh{false};
  std::size_t processed_beams{0U};
  std::size_t hit_beams{0U};
  std::size_t altitude_rejected_beams{0U};
  std::size_t occupied_cells{0U};
  std::size_t outside_hits{0U};
};

struct StaticSourceStats {
  bool enabled{false};
  bool loaded{false};
  bool used{false};
  std::size_t rectangles{0U};
  std::size_t occupied_cells{0U};
  std::string path;
};

struct MemorySourceStats {
  bool enabled{false};
  bool seen{false};
  bool used{false};
  bool geometry_matches{false};
  GridStats source_counts{};
  GridOverlayStats overlay{};
};

struct PlanningGridBuildResult {
  OccupancyGrid2D grid;
  StaticSourceStats static_source{};
  MemorySourceStats memory{};
  CurrentLidarStats current_lidar{};
};

struct PathComputationResult {
  AStarResult astar{};
  std::vector<GridIndex> smoothed_cells;
  GridStats grid_stats{};
  double raw_path_clearance_m{std::numeric_limits<double>::infinity()};
  double smoothed_path_clearance_m{std::numeric_limits<double>::infinity()};
};

[[nodiscard]] int positiveCellCount(const double length_m, const double resolution_m) {
  return std::max(1, static_cast<int>(std::ceil(length_m / resolution_m)));
}

} // namespace

class PlannerNode final : public rclcpp::Node {
public:
  PlannerNode()
      : Node{"planner_node"} {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    start_ = Point2{declare_parameter<double>("start_x_m", 0.0),
                    declare_parameter<double>("start_y_m", 0.0)};
    goal_ = Point2{declare_parameter<double>("goal_x_m", 85.0),
                   declare_parameter<double>("goal_y_m", 0.0)};
    cruise_altitude_m_ = declare_parameter<double>("cruise_altitude_m", 12.0);
    inflation_radius_m_ = declare_parameter<double>("inflation_radius_m", 2.5);
    max_pose_staleness_ns_ = static_cast<std::int64_t>(
        std::clamp<double>(declare_parameter<double>("max_pose_staleness_s", 1.0), 0.0,
                           3600.0) *
        1.0e9);
    direct_path_fallback_ = declare_parameter<bool>("direct_path_fallback", false);
    reuse_last_valid_path_on_failure_ =
        declare_parameter<bool>("reuse_last_valid_path_on_failure", false);
    max_initial_goal_regression_m_ = std::clamp(
        declare_parameter<double>("max_initial_goal_regression_m", 8.0), 0.0, 1000.0);
    initial_progress_probe_distance_m_ =
        std::clamp(declare_parameter<double>("initial_progress_probe_distance_m", 2.0),
                   0.1, 100.0);
    max_initial_lateral_deviation_m_ =
        declare_parameter<double>("max_initial_lateral_deviation_m", 8.0);
    nearest_free_radius_cells_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("nearest_free_radius_cells", 10), 0, 100000));
    occupied_threshold_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("memory_occupied_threshold", 65), 1, 100));
    free_threshold_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("memory_free_threshold", 0), 0, 100));
    use_static_map_ = declare_parameter<bool>("use_static_map", true);
    use_obstacle_memory_ = declare_parameter<bool>("use_obstacle_memory", true);
    static_map_path_param_ = declare_parameter<std::string>(
        "static_map_path", "worlds/generated_city.map2d");
    static_map_min_blocking_height_m_ =
        std::clamp(declare_parameter<double>("static_map_min_blocking_height_m", 0.0),
                   0.0, 100000.0);
    const double planning_grid_resolution_m =
        std::max(0.01, declare_parameter<double>("planning_grid_resolution_m", 0.5));
    fallback_grid_bounds_ = GridBounds{
        declare_parameter<double>("planning_grid_origin_x", -10.0),
        declare_parameter<double>("planning_grid_origin_y", -10.0),
        planning_grid_resolution_m,
        positiveCellCount(declare_parameter<double>("planning_grid_width_m", 115.0),
                          planning_grid_resolution_m),
        positiveCellCount(declare_parameter<double>("planning_grid_height_m", 175.0),
                          planning_grid_resolution_m)};
    use_current_lidar_obstacles_ =
        declare_parameter<bool>("use_current_lidar_obstacles", true);
    max_current_lidar_staleness_ns_ = static_cast<std::int64_t>(
        std::clamp<double>(
            declare_parameter<double>("max_current_lidar_staleness_s", 0.75), 0.0,
            3600.0) *
        1.0e9);
    max_lidar_range_m_ = declare_parameter<double>("max_lidar_range_m", 35.0);
    range_hit_epsilon_m_ = declare_parameter<double>("range_hit_epsilon_m", 0.05);
    current_lidar_obstacle_depth_m_ = std::clamp(
        declare_parameter<double>("current_lidar_obstacle_depth_m", 0.0), 0.0, 100.0);
    scan_yaw_offset_rad_ = declare_parameter<double>("scan_yaw_offset_rad", 0.0);
    use_px4_heading_for_scan_ =
        declare_parameter<bool>("use_px4_heading_for_scan", false);
    swap_lidar_xy_to_local_frame_ =
        declare_parameter<bool>("swap_lidar_xy_to_local_frame", false);
    compensate_lidar_attitude_ =
        declare_parameter<bool>("compensate_lidar_attitude", false);
    lidar_mount_roll_rad_ = declare_parameter<double>("lidar_mount_roll_rad", 0.0);
    lidar_mount_pitch_rad_ = declare_parameter<double>("lidar_mount_pitch_rad", 0.0);
    lidar_mount_yaw_rad_ = declare_parameter<double>("lidar_mount_yaw_rad", 0.0);
    lidar_z_offset_m_ = declare_parameter<double>("lidar_z_offset_m", 0.0);
    min_projected_lidar_altitude_m_ =
        declare_parameter<double>("min_projected_lidar_altitude_m", 0.0);
    max_projected_lidar_altitude_m_ =
        declare_parameter<double>("max_projected_lidar_altitude_m", 100000.0);
    astar_config_.max_expansions = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("astar_max_expansions", 100000), 1,
        std::numeric_limits<int>::max()));
    astar_config_.obstacle_clearance_cost_radius_m = std::clamp(
        declare_parameter<double>("astar_obstacle_clearance_cost_radius_m", 0.0), 0.0,
        100.0);
    astar_config_.obstacle_clearance_cost_weight = std::clamp(
        declare_parameter<double>("astar_obstacle_clearance_cost_weight", 0.0), 0.0,
        1000.0);
    astar_config_.turn_cost_weight = std::clamp(
        declare_parameter<double>("astar_turn_cost_weight", 0.0), 0.0, 1000.0);
    path_smoothing_config_.minimum_obstacle_clearance_m = std::clamp(
        declare_parameter<double>("path_smoothing_min_obstacle_clearance_m", 0.0), 0.0,
        100.0);

    const bool use_initial_pose =
        declare_parameter<bool>("use_initial_pose_until_px4", true);
    initial_heading_rad_ = declare_parameter<double>("initial_heading_rad", 0.0);
    if (use_initial_pose) {
      current_pose_ = Pose2{Point2{declare_parameter<double>("initial_x_m", 0.0),
                                   declare_parameter<double>("initial_y_m", 0.0)},
                            initial_heading_rad_};
      pose_valid_ = true;
      last_pose_update_ns_ = get_clock()->now().nanoseconds();
    }

    const std::string memory_grid_topic = declare_parameter<std::string>(
        "obstacle_memory_grid_topic", "/drone_city_nav/obstacle_memory_grid");
    const std::string lidar_topic =
        declare_parameter<std::string>("lidar_topic", "/scan");
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");
    const std::string attitude_topic = declare_parameter<std::string>(
        "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");

    const auto sensor_qos = rclcpp::SensorDataQoS{};
    memory_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        memory_grid_topic, rclcpp::QoS{1}.transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          onMemoryGrid(*msg);
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, sensor_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { onScan(*msg); });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          onLocalPosition(*msg);
        });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
          onAttitude(*msg);
        });

    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("occupancy_grid_topic",
                                       "/drone_city_nav/occupancy_grid"),
        rclcpp::QoS{1}.transient_local());
    static_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("static_map_grid_topic",
                                       "/drone_city_nav/static_map_grid"),
        rclcpp::QoS{1}.transient_local());
    static_map_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("static_map_points_topic",
                                       "/drone_city_nav/static_map_points"),
        rclcpp::QoS{1}.transient_local());
    static_map_debug_publish_period_s_ = std::clamp(
        declare_parameter<double>("static_map_debug_publish_period_s", 1.0), 0.0, 60.0);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        declare_parameter<std::string>("path_topic", "/drone_city_nav/path"),
        rclcpp::QoS{1}.reliable());
    waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("current_waypoint_topic",
                                       "/drone_city_nav/current_waypoint"),
        rclcpp::QoS{1}.reliable());

    const double replan_period_s = declare_parameter<double>("replan_period_s", 0.5);
    loadConfiguredStaticMap();
    if (static_map_debug_publish_period_s_ > 0.0) {
      static_map_debug_timer_ = create_wall_timer(
          std::chrono::duration<double>{static_map_debug_publish_period_s_},
          [this]() { republishStaticMapDebug(); });
    }
    timer_ = create_wall_timer(
        std::chrono::duration<double>{std::max(0.05, replan_period_s)},
        [this]() { replanAndPublish(); });

    RCLCPP_INFO(get_logger(),
                "Planner ready: start=(%.1f, %.1f) goal=(%.1f, %.1f) "
                "inflation=%.2fm",
                start_.x, start_.y, goal_.x, goal_.y, inflation_radius_m_);
    RCLCPP_INFO(get_logger(),
                "Planner subscriptions: obstacle_memory_grid='%s' local_position='%s' "
                "attitude='%s'",
                memory_grid_topic.c_str(), local_position_topic.c_str(),
                attitude_topic.c_str());
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
                use_current_lidar_obstacles_ ? "true" : "false", lidar_topic.c_str(),
                max_lidar_range_m_,
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
    RCLCPP_INFO(get_logger(),
                "Planner fallback policy: direct_path_fallback=%s "
                "reuse_last_valid_path_on_failure=%s max_initial_goal_regression=%.2fm "
                "initial_progress_probe=%.2fm max_initial_lateral_deviation=%.2fm",
                direct_path_fallback_ ? "true" : "false",
                reuse_last_valid_path_on_failure_ ? "true" : "false",
                max_initial_goal_regression_m_, initial_progress_probe_distance_m_,
                max_initial_lateral_deviation_m_);
    RCLCPP_INFO(get_logger(),
                "Planner obstacle clearance preference: astar_radius=%.2fm "
                "astar_weight=%.2f astar_turn_weight=%.2f "
                "smoothing_min_clearance=%.2fm",
                astar_config_.obstacle_clearance_cost_radius_m,
                astar_config_.obstacle_clearance_cost_weight,
                astar_config_.turn_cost_weight,
                path_smoothing_config_.minimum_obstacle_clearance_m);
  }

private:
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

    current_pose_.position =
        Point2{static_cast<double>(msg.x), static_cast<double>(msg.y)};
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
    auto converted = occupancyGridFromMessage(msg);
    if (!converted.has_value()) {
      return;
    }

    memory_grid_ = std::move(*converted);
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

  [[nodiscard]] std::optional<OccupancyGrid2D>
  occupancyGridFromMessage(const nav_msgs::msg::OccupancyGrid& msg) {
    if (!(msg.info.resolution > 0.0F) || msg.info.width == 0U ||
        msg.info.height == 0U ||
        msg.info.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        msg.info.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring invalid obstacle memory grid metadata");
      return std::nullopt;
    }

    const auto width = static_cast<int>(msg.info.width);
    const auto height = static_cast<int>(msg.info.height);
    const std::size_t expected_size =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if (msg.data.size() != expected_size) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring obstacle memory grid with mismatched data size: "
                           "expected=%zu got=%zu",
                           expected_size, msg.data.size());
      return std::nullopt;
    }

    OccupancyGrid2D grid{
        GridBounds{msg.info.origin.position.x, msg.info.origin.position.y,
                   static_cast<double>(msg.info.resolution), width, height}};
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const GridIndex cell{x, y};
        const std::size_t index = grid.linearIndex(cell);
        const auto raw_value = msg.data[index];
        if (raw_value < 0) {
          continue;
        }
        const int value = static_cast<int>(static_cast<unsigned char>(raw_value));
        if (value >= occupied_threshold_) {
          grid.setOccupied(cell);
        } else if (value >= free_threshold_) {
          grid.setFree(cell);
        }
      }
    }

    return grid;
  }

  [[nodiscard]] std::filesystem::path
  resolveStaticMapPath(const std::string& configured_path) const {
    std::filesystem::path path =
        configured_path.empty() ? std::filesystem::path{"worlds/generated_city.map2d"}
                                : std::filesystem::path{configured_path};
    if (path.is_absolute()) {
      return path;
    }
    if (std::filesystem::exists(path)) {
      return std::filesystem::absolute(path);
    }

    try {
      const auto package_share = std::filesystem::path{
          ament_index_cpp::get_package_share_directory("drone_city_nav")};
      std::filesystem::path package_candidate = package_share / path;
      if (std::filesystem::exists(package_candidate)) {
        return package_candidate;
      }
      std::filesystem::path worlds_candidate =
          package_share / "worlds" / path.filename();
      if (std::filesystem::exists(worlds_candidate)) {
        return worlds_candidate;
      }
    } catch (const std::exception&) {
      return path;
    }

    return path;
  }

  void loadConfiguredStaticMap() {
    static_map_resolved_path_ = resolveStaticMapPath(static_map_path_param_);
    if (!use_static_map_) {
      RCLCPP_INFO(get_logger(), "Static city map source is disabled");
      return;
    }

    try {
      const StaticCityMap static_map = loadStaticCityMap(static_map_resolved_path_);
      if (static_map.frame_id != frame_id_) {
        RCLCPP_WARN(get_logger(),
                    "Static city map frame differs from planner frame: map='%s' "
                    "planner='%s'",
                    static_map.frame_id.c_str(), frame_id_.c_str());
      }
      static_map_rectangles_ = static_map.rectangles.size();
      static_grid_ =
          rasterizeStaticCityMap(static_map, static_map_min_blocking_height_m_);
      const GridStats stats = collectGridStats(*static_grid_);
      static_map_occupied_cells_ = stats.occupied_cells;
      RCLCPP_INFO(
          get_logger(),
          "Static city map loaded: path='%s' frame='%s' rectangles=%zu "
          "occupied_cells=%zu grid=%dx%d@%.2fm origin=(%.2f, %.2f) "
          "min_blocking_height=%.2f",
          static_map_resolved_path_.string().c_str(), static_map.frame_id.c_str(),
          static_map_rectangles_, static_map_occupied_cells_, static_grid_->width(),
          static_grid_->height(), static_grid_->resolution(), static_grid_->originX(),
          static_grid_->originY(), static_map_min_blocking_height_m_);
      publishStaticMapDebug(*static_grid_, true);
    } catch (const std::exception& error) {
      static_grid_.reset();
      static_map_rectangles_ = 0U;
      static_map_occupied_cells_ = 0U;
      RCLCPP_ERROR(get_logger(), "Failed to load static city map: path='%s' error='%s'",
                   static_map_resolved_path_.string().c_str(), error.what());
    }
  }

  [[nodiscard]] OccupancyGrid2D makeBasePlanningGrid() const {
    if (use_static_map_ && static_grid_.has_value()) {
      return OccupancyGrid2D{static_grid_->bounds()};
    }
    if (use_obstacle_memory_ && memory_grid_.has_value()) {
      return OccupancyGrid2D{memory_grid_->bounds()};
    }
    return OccupancyGrid2D{fallback_grid_bounds_};
  }

  [[nodiscard]] std::optional<PlanningGridBuildResult>
  buildPlanningGrid(const std::int64_t now_ns) {
    StaticSourceStats static_stats{};
    static_stats.enabled = use_static_map_;
    static_stats.loaded = static_grid_.has_value();
    static_stats.rectangles = static_map_rectangles_;
    static_stats.occupied_cells = static_map_occupied_cells_;
    static_stats.path = static_map_resolved_path_.string();

    MemorySourceStats memory_stats{};
    memory_stats.enabled = use_obstacle_memory_;
    memory_stats.seen = memory_grid_.has_value();

    if (!use_static_map_ && !use_obstacle_memory_ && !use_current_lidar_obstacles_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planner has no enabled obstacle sources; publishing hold path");
      return std::nullopt;
    }
    if (use_static_map_ && !static_grid_.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner static map source is enabled but not loaded; "
                           "publishing hold path");
      return std::nullopt;
    }

    OccupancyGrid2D planning_grid = makeBasePlanningGrid();
    if (use_static_map_ && static_grid_.has_value()) {
      const GridOverlayStats static_overlay =
          overlayOccupiedCells(planning_grid, *static_grid_);
      static_stats.occupied_cells = static_overlay.source_occupied_cells;
      static_stats.used = true;
    }

    if (use_obstacle_memory_) {
      if (memory_grid_.has_value()) {
        memory_stats.source_counts = collectGridStats(*memory_grid_);
        memory_stats.geometry_matches =
            haveSameGridGeometry(planning_grid, *memory_grid_);
        if (memory_stats.geometry_matches) {
          memory_stats.overlay = overlayKnownMemoryCells(planning_grid, *memory_grid_);
          memory_stats.used = true;
        } else {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Skipping obstacle memory overlay due to grid geometry mismatch: "
              "planning=%dx%d@%.2f origin=(%.2f, %.2f) memory=%dx%d@%.2f "
              "origin=(%.2f, %.2f)",
              planning_grid.width(), planning_grid.height(), planning_grid.resolution(),
              planning_grid.originX(), planning_grid.originY(), memory_grid_->width(),
              memory_grid_->height(), memory_grid_->resolution(),
              memory_grid_->originX(), memory_grid_->originY());
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Obstacle memory source is enabled but no grid has been "
                             "received yet");
      }
    }

    const CurrentLidarStats current_lidar_stats =
        overlayCurrentLidarHits(planning_grid, now_ns);
    const bool any_source_ready =
        static_stats.used || memory_stats.used ||
        (current_lidar_stats.enabled && current_lidar_stats.used &&
         current_lidar_stats.fresh);
    if (!any_source_ready) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner has no ready obstacle source data; publishing hold "
                           "path");
      return std::nullopt;
    }

    planning_grid.rebuildInflation(inflation_radius_m_);
    return PlanningGridBuildResult{std::move(planning_grid), std::move(static_stats),
                                   memory_stats, current_lidar_stats};
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
    OccupancyGrid2D planning_grid = std::move(planning_result->grid);
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
          "Rejecting low-clearance combined path; trying static-only fallback: "
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
        "source=combined expanded=%zu raw_path=%zu smoothed_path=%zu "
        "path_clearance[raw=%.2f smoothed=%.2f]",
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
        planning_result->current_lidar.outside_hits, path_result->astar.expanded_cells,
        path_result->astar.path.size(), path_result->smoothed_cells.size(),
        path_result->raw_path_clearance_m, path_result->smoothed_path_clearance_m);
    publishOccupancyGrid(planning_grid);
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

    const auto unblocked_start =
        grid.nearestUnblocked(*start_cell, nearest_free_radius_cells_);
    const auto unblocked_goal =
        grid.nearestUnblocked(*goal_cell, nearest_free_radius_cells_);
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

    PathComputationResult result{};
    result.astar =
        planner_.plan(grid, *unblocked_start, *unblocked_goal, astar_config_);
    if (!result.astar.success) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "A* did not find a path on %s grid; expanded=%zu start=(%d,%d) "
          "goal=(%d,%d)",
          source_label, result.astar.expanded_cells, unblocked_start->x,
          unblocked_start->y, unblocked_goal->x, unblocked_goal->y);
      return std::nullopt;
    }

    result.smoothed_cells = smoothPath(grid, result.astar.path, path_smoothing_config_);
    result.grid_stats = collectGridStats(grid);
    result.raw_path_clearance_m = pathMinimumBlockedClearanceM(grid, result.astar.path);
    result.smoothed_path_clearance_m =
        pathMinimumBlockedClearanceM(grid, result.smoothed_cells);
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
    pruneBackwardInitialWaypoints(path_points, grid);
    if (!pathIsUnblocked(grid, path_points, source_label)) {
      publishPath({});
      return false;
    }
    if (publishLastValidPathInsteadOfRegressiveSwitch(grid, path_points,
                                                      source_label)) {
      return true;
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
    publishPath(path_points);
    return true;
  }

  [[nodiscard]] GridStats collectGridStats(const OccupancyGrid2D& grid) const {
    GridStats stats{};
    for (int y = 0; y < grid.height(); ++y) {
      for (int x = 0; x < grid.width(); ++x) {
        const GridIndex cell{x, y};
        if (grid.isInflated(cell)) {
          ++stats.inflated_cells;
        }
        switch (grid.state(cell)) {
          case CellState::kUnknown:
            ++stats.unknown_cells;
            break;
          case CellState::kFree:
            ++stats.free_cells;
            break;
          case CellState::kOccupied:
            ++stats.occupied_cells;
            break;
        }
      }
    }
    return stats;
  }

  [[nodiscard]] double clearanceDiagnosticRadiusM() const noexcept {
    const double configured_clearance_m =
        std::max(astar_config_.obstacle_clearance_cost_radius_m,
                 path_smoothing_config_.minimum_obstacle_clearance_m);
    return std::max(10.0, configured_clearance_m);
  }

  [[nodiscard]] double nearestBlockedDistanceM(const OccupancyGrid2D& grid,
                                               const GridIndex cell) const {
    if (!(grid.resolution() > 0.0)) {
      return std::numeric_limits<double>::infinity();
    }

    const int radius_cells =
        static_cast<int>(std::ceil(clearanceDiagnosticRadiusM() / grid.resolution()));
    double nearest_distance_m = std::numeric_limits<double>::infinity();
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const GridIndex candidate{cell.x + dx, cell.y + dy};
        if (!grid.contains(candidate) || !grid.isBlocked(candidate)) {
          continue;
        }
        nearest_distance_m =
            std::min(nearest_distance_m,
                     distance(grid.cellCenter(cell), grid.cellCenter(candidate)));
      }
    }

    return nearest_distance_m;
  }

  [[nodiscard]] double
  pathMinimumBlockedClearanceM(const OccupancyGrid2D& grid,
                               const std::vector<GridIndex>& path) const {
    double minimum_clearance_m = std::numeric_limits<double>::infinity();
    for (const GridIndex cell : path) {
      if (!grid.contains(cell)) {
        continue;
      }
      minimum_clearance_m =
          std::min(minimum_clearance_m, nearestBlockedDistanceM(grid, cell));
    }
    return minimum_clearance_m;
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

  std::size_t markCurrentLidarObstacle(OccupancyGrid2D& grid, const Point2 endpoint,
                                       const Point2 depth_endpoint) const {
    const auto endpoint_cell = grid.worldToCell(endpoint);
    if (!endpoint_cell.has_value()) {
      return 0U;
    }

    std::size_t occupied_cells = 0U;
    if (current_lidar_obstacle_depth_m_ <= 0.0) {
      grid.setOccupied(*endpoint_cell);
      return 1U;
    }

    const auto depth_cell = grid.worldToCell(depth_endpoint);
    if (!depth_cell.has_value()) {
      grid.setOccupied(*endpoint_cell);
      return 1U;
    }

    for (const GridIndex cell : grid.cellsOnLine(*endpoint_cell, *depth_cell)) {
      grid.setOccupied(cell);
      ++occupied_cells;
    }
    return occupied_cells;
  }

  CurrentLidarStats overlayCurrentLidarHits(OccupancyGrid2D& grid,
                                            const std::int64_t now_ns) const {
    CurrentLidarStats stats{};
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
    stats.used = true;

    const double scan_range_max = currentLidarRangeMax();
    if (!(scan_range_max > 0.0) || last_scan_.angle_increment == 0.0F) {
      return stats;
    }

    OccupancyGrid2D current_lidar_grid{grid.bounds()};
    const LidarProjectionPose projection_pose = currentLidarProjectionPose();
    const LidarProjectionConfig projection_config = currentLidarProjectionConfig();
    for (std::size_t i = 0U; i < last_scan_.ranges.size(); ++i) {
      const float raw_range = last_scan_.ranges[i];
      if (!lidarRawRangeUsable(raw_range, static_cast<double>(last_scan_.range_min))) {
        continue;
      }
      ++stats.processed_beams;

      const LidarBeamProjection projection = projectLidarBeam(
          projection_pose, projection_config, static_cast<double>(last_scan_.range_min),
          scan_range_max, static_cast<double>(last_scan_.angle_min),
          static_cast<double>(last_scan_.angle_increment), i, raw_range,
          current_lidar_obstacle_depth_m_);
      if (projection.status == LidarBeamProjectionStatus::kAltitudeRejected) {
        ++stats.altitude_rejected_beams;
        continue;
      }
      if (projection.status != LidarBeamProjectionStatus::kAccepted ||
          !projection.hit) {
        continue;
      }

      ++stats.hit_beams;
      const std::size_t occupied_cells = markCurrentLidarObstacle(
          current_lidar_grid, projection.endpoint, projection.depth_endpoint);
      if (occupied_cells == 0U) {
        ++stats.outside_hits;
      } else {
        stats.occupied_cells += occupied_cells;
      }
    }
    const GridOverlayStats overlay_stats =
        overlayCurrentLidarCells(grid, current_lidar_grid);
    stats.occupied_cells = overlay_stats.source_occupied_cells;

    return stats;
  }

  [[nodiscard]] nav_msgs::msg::OccupancyGrid
  makeOccupancyGridMessage(const OccupancyGrid2D& grid, const bool include_inflation) {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.info.map_load_time = msg.header.stamp;
    msg.info.resolution = static_cast<float>(grid.resolution());
    msg.info.width = static_cast<std::uint32_t>(grid.width());
    msg.info.height = static_cast<std::uint32_t>(grid.height());
    msg.info.origin.position.x = grid.originX();
    msg.info.origin.position.y = grid.originY();
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(grid.cellCount(), static_cast<std::int8_t>(-1));

    for (int y = 0; y < grid.height(); ++y) {
      for (int x = 0; x < grid.width(); ++x) {
        const GridIndex cell{x, y};
        const std::size_t index = grid.linearIndex(cell);
        if (grid.isOccupied(cell)) {
          msg.data[index] = static_cast<std::int8_t>(100);
        } else if (include_inflation && grid.isInflated(cell)) {
          msg.data[index] = static_cast<std::int8_t>(80);
        } else if (grid.state(cell) == CellState::kFree) {
          msg.data[index] = static_cast<std::int8_t>(0);
        }
      }
    }

    return msg;
  }

  void publishStaticMapDebug(const OccupancyGrid2D& grid, const bool log_publication) {
    static_map_pub_->publish(makeOccupancyGridMessage(grid, false));
    publishStaticMapPoints(grid);
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

  void publishStaticMapPoints(const OccupancyGrid2D& grid) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = frame_id_;
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(static_map_occupied_cells_);
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 12U;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(3U);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0U;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1U;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4U;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1U;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8U;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1U;
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));

    std::size_t point_index = 0U;
    for (int y = 0; y < grid.height(); ++y) {
      for (int x = 0; x < grid.width(); ++x) {
        const GridIndex cell{x, y};
        if (!grid.isOccupied(cell)) {
          continue;
        }

        const Point2 center = grid.cellCenter(cell);
        const float point_x = static_cast<float>(center.x);
        const float point_y = static_cast<float>(center.y);
        const float point_z = 0.05F;
        const std::size_t offset =
            point_index * static_cast<std::size_t>(cloud.point_step);
        std::memcpy(&cloud.data[offset], &point_x, sizeof(float));
        std::memcpy(&cloud.data[offset + 4U], &point_y, sizeof(float));
        std::memcpy(&cloud.data[offset + 8U], &point_z, sizeof(float));
        ++point_index;
      }
    }

    if (point_index != static_cast<std::size_t>(cloud.width)) {
      cloud.width = static_cast<std::uint32_t>(point_index);
      cloud.row_step = cloud.point_step * cloud.width;
      cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
    }

    static_map_points_pub_->publish(cloud);
  }

  void publishOccupancyGrid(const OccupancyGrid2D& grid) {
    occupancy_pub_->publish(makeOccupancyGridMessage(grid, true));
  }

  void publishPath(const std::vector<Point2>& points) {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    path.poses.reserve(points.size());

    for (const Point2 point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = cruise_altitude_m_;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    path_pub_->publish(path);
    if (!path.poses.empty()) {
      waypoint_pub_->publish(path.poses.front());
    }

    logPathUpdate(path);
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
        "expanded=%zu raw_path=%zu smoothed_path=%zu "
        "path_clearance[raw=%.2f smoothed=%.2f]",
        reason, static_overlay.source_occupied_cells, path_result->astar.expanded_cells,
        path_result->astar.path.size(), path_result->smoothed_cells.size(),
        path_result->raw_path_clearance_m, path_result->smoothed_path_clearance_m);
    publishOccupancyGrid(static_only_grid);
    return publishPathFromSmoothedCells(static_only_grid, path_result->smoothed_cells,
                                        "static-only fallback");
  }

  void publishDirectGoalPath() {
    std::vector<Point2> path_points{current_pose_.position, goal_};
    last_valid_path_points_ = path_points;
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

  [[nodiscard]] double
  initialGoalRegressionM(const std::vector<Point2>& path_points) const {
    if (path_points.size() < 2U || !finite2D(current_pose_.position)) {
      return 0.0;
    }

    const double current_goal_distance = distance(current_pose_.position, goal_);
    const double probe_distance_m = std::max(initial_progress_probe_distance_m_,
                                             2.0 * fallback_grid_bounds_.resolution_m);
    for (auto point = path_points.begin() + 1; point != path_points.end(); ++point) {
      if (distance(current_pose_.position, *point) < probe_distance_m) {
        continue;
      }
      return distance(*point, goal_) - current_goal_distance;
    }

    return distance(path_points.back(), goal_) - current_goal_distance;
  }

  bool publishLastValidPathInsteadOfRegressiveSwitch(
      const OccupancyGrid2D& grid, const std::vector<Point2>& candidate_path_points,
      const char* source_label) {
    if (!(max_initial_goal_regression_m_ > 0.0) ||
        last_valid_path_points_.size() < 2U || candidate_path_points.size() < 2U) {
      return false;
    }

    const double candidate_regression_m = initialGoalRegressionM(candidate_path_points);
    if (!(candidate_regression_m > max_initial_goal_regression_m_)) {
      return false;
    }

    const double previous_regression_m =
        initialGoalRegressionM(last_valid_path_points_);
    if (previous_regression_m > max_initial_goal_regression_m_) {
      return false;
    }
    if (!pathIsUnblocked(grid, last_valid_path_points_, "previous stable path")) {
      return false;
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Keeping previous path instead of regressive %s switch: "
        "candidate_initial_goal_regression=%.2fm "
        "previous_initial_goal_regression=%.2fm "
        "threshold=%.2fm previous_waypoints=%zu candidate_waypoints=%zu",
        source_label, candidate_regression_m, previous_regression_m,
        max_initial_goal_regression_m_, last_valid_path_points_.size(),
        candidate_path_points.size());
    publishPath(last_valid_path_points_);
    return true;
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

  [[nodiscard]] bool pathSegmentIsUnblocked(const OccupancyGrid2D& grid,
                                            const Point2 start,
                                            const Point2 end) const {
    const auto start_cell = grid.worldToCell(start);
    const auto end_cell = grid.worldToCell(end);
    if (!start_cell.has_value() || !end_cell.has_value()) {
      return false;
    }

    return hasLineOfSight(grid, *start_cell, *end_cell);
  }

  [[nodiscard]] bool pathIsUnblocked(const OccupancyGrid2D& grid,
                                     const std::vector<Point2>& path_points,
                                     const char* source_label) const {
    if (path_points.size() < 2U) {
      return true;
    }

    for (std::size_t index = 1U; index < path_points.size(); ++index) {
      if (pathSegmentIsUnblocked(grid, path_points[index - 1U], path_points[index])) {
        continue;
      }

      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "%s path segment intersects blocked cells after final path "
                           "edits: segment=%zu "
                           "start=(%.2f, %.2f) end=(%.2f, %.2f)",
                           source_label, index - 1U, path_points[index - 1U].x,
                           path_points[index - 1U].y, path_points[index].x,
                           path_points[index].y);
      return false;
    }
    return true;
  }

  void pruneBackwardInitialWaypoints(std::vector<Point2>& path_points,
                                     const OccupancyGrid2D& grid) const {
    if (path_points.size() < 3U) {
      return;
    }

    const Point2 origin = path_points.front();
    const Point2 goal_vector{goal_.x - origin.x, goal_.y - origin.y};
    const double goal_distance_sq = squaredDistance(origin, goal_);
    if (!(goal_distance_sq > 0.0)) {
      return;
    }

    auto first_forward = path_points.begin() + 1;
    while (first_forward + 1 != path_points.end()) {
      const Point2 candidate_vector{first_forward->x - origin.x,
                                    first_forward->y - origin.y};
      const double projection =
          candidate_vector.x * goal_vector.x + candidate_vector.y * goal_vector.y;
      if (projection >= grid.resolution() * grid.resolution()) {
        break;
      }
      ++first_forward;
    }

    if (first_forward != path_points.begin() + 1) {
      if (!pathSegmentIsUnblocked(grid, origin, *first_forward)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Skipped pruning backward initial waypoints because it would create an "
            "unsafe first path segment: removed_candidate=%zu start=(%.2f, %.2f) "
            "next=(%.2f, %.2f)",
            static_cast<std::size_t>(first_forward - (path_points.begin() + 1)),
            origin.x, origin.y, first_forward->x, first_forward->y);
        return;
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Pruned backward initial waypoints: removed=%zu",
          static_cast<std::size_t>(first_forward - (path_points.begin() + 1)));
      path_points.erase(path_points.begin() + 1, first_forward);
    }
  }

  void logPathUpdate(const nav_msgs::msg::Path& path) {
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
                  "Published path: waypoints=%zu first=(%.2f, %.2f) "
                  "last=(%.2f, %.2f) preview=%s",
                  path_size, first.x, first.y, last.x, last.y, preview.str().c_str());
      last_logged_path_size_ = path_size;
      last_logged_path_first_ = first;
      last_logged_path_last_ = last;
    }
  }

  std::optional<OccupancyGrid2D> memory_grid_;
  std::optional<OccupancyGrid2D> static_grid_;
  AStarPlanner planner_;
  AStarConfig astar_config_{};
  PathSmoothingConfig path_smoothing_config_{};

  Pose2 current_pose_{};
  AttitudeEuler current_attitude_{};
  Point2 start_{};
  Point2 goal_{};
  sensor_msgs::msg::LaserScan last_scan_;
  bool pose_valid_{false};
  bool local_position_seen_{false};
  bool memory_grid_seen_{false};
  bool scan_seen_{false};
  bool scan_seen_logged_{false};
  bool direct_path_fallback_{false};
  bool reuse_last_valid_path_on_failure_{false};
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
  double max_initial_goal_regression_m_{8.0};
  double initial_progress_probe_distance_m_{2.0};
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
