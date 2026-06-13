#include "drone_city_nav/astar_planner.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/path_smoothing.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
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
  bool used{false};
  bool fresh{false};
  std::size_t processed_beams{0U};
  std::size_t hit_beams{0U};
  std::size_t occupied_cells{0U};
  std::size_t outside_hits{0U};
};

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
    max_initial_lateral_deviation_m_ =
        declare_parameter<double>("max_initial_lateral_deviation_m", 8.0);
    nearest_free_radius_cells_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("nearest_free_radius_cells", 10), 0, 100000));
    occupied_threshold_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("memory_occupied_threshold", 65), 1, 100));
    free_threshold_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("memory_free_threshold", 0), 0, 100));
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
    astar_config_.max_expansions = static_cast<std::size_t>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("astar_max_expansions", 100000), 1,
        std::numeric_limits<int>::max()));
    astar_config_.obstacle_clearance_cost_radius_m = std::clamp(
        declare_parameter<double>("astar_obstacle_clearance_cost_radius_m", 0.0), 0.0,
        100.0);
    astar_config_.obstacle_clearance_cost_weight = std::clamp(
        declare_parameter<double>("astar_obstacle_clearance_cost_weight", 0.0), 0.0,
        1000.0);
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

    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("occupancy_grid_topic",
                                       "/drone_city_nav/occupancy_grid"),
        rclcpp::QoS{1}.transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        declare_parameter<std::string>("path_topic", "/drone_city_nav/path"),
        rclcpp::QoS{1}.reliable());
    waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("current_waypoint_topic",
                                       "/drone_city_nav/current_waypoint"),
        rclcpp::QoS{1}.reliable());

    const double replan_period_s = declare_parameter<double>("replan_period_s", 0.5);
    timer_ = create_wall_timer(
        std::chrono::duration<double>{std::max(0.05, replan_period_s)},
        [this]() { replanAndPublish(); });

    RCLCPP_INFO(get_logger(),
                "Planner ready: start=(%.1f, %.1f) goal=(%.1f, %.1f) "
                "inflation=%.2fm",
                start_.x, start_.y, goal_.x, goal_.y, inflation_radius_m_);
    RCLCPP_INFO(get_logger(),
                "Planner subscriptions: obstacle_memory_grid='%s' local_position='%s'",
                memory_grid_topic.c_str(), local_position_topic.c_str());
    RCLCPP_INFO(get_logger(),
                "Planner lidar overlay: enabled=%s topic='%s' max_range=%.2f "
                "max_staleness=%.2fs depth=%.2f swap_lidar_xy=%s yaw_source=%s",
                use_current_lidar_obstacles_ ? "true" : "false", lidar_topic.c_str(),
                max_lidar_range_m_,
                static_cast<double>(max_current_lidar_staleness_ns_) / 1.0e9,
                current_lidar_obstacle_depth_m_,
                swap_lidar_xy_to_local_frame_ ? "true" : "false",
                use_px4_heading_for_scan_ ? "px4_heading" : "initial_map_aligned");
    RCLCPP_INFO(
        get_logger(),
        "Planner fallback policy: direct_path_fallback=%s "
        "reuse_last_valid_path_on_failure=%s max_initial_lateral_deviation=%.2fm",
        direct_path_fallback_ ? "true" : "false",
        reuse_last_valid_path_on_failure_ ? "true" : "false",
        max_initial_lateral_deviation_m_);
    RCLCPP_INFO(get_logger(),
                "Planner obstacle clearance preference: astar_radius=%.2fm "
                "astar_weight=%.2f smoothing_min_clearance=%.2fm",
                astar_config_.obstacle_clearance_cost_radius_m,
                astar_config_.obstacle_clearance_cost_weight,
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
    if (!memory_grid_.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner is waiting for obstacle memory grid; publishing "
                           "hold path");
      publishPath({});
      return;
    }

    OccupancyGrid2D planning_grid = *memory_grid_;
    const CurrentLidarStats current_lidar_stats =
        overlayCurrentLidarHits(planning_grid, now_ns);
    planning_grid.rebuildInflation(inflation_radius_m_);
    publishOccupancyGrid(planning_grid);

    const auto start_cell = planning_grid.worldToCell(current_pose_.position);
    const auto goal_cell = planning_grid.worldToCell(goal_);
    if (!start_cell.has_value() || !goal_cell.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Start or goal is outside the obstacle memory grid");
      publishFallbackPath();
      return;
    }

    const auto unblocked_start =
        planning_grid.nearestUnblocked(*start_cell, nearest_free_radius_cells_);
    const auto unblocked_goal =
        planning_grid.nearestUnblocked(*goal_cell, nearest_free_radius_cells_);
    if (!unblocked_start.has_value() || !unblocked_goal.has_value()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No unblocked start or goal cell is available after inflation");
      publishFallbackPath();
      return;
    }

    if (*unblocked_start != *start_cell || *unblocked_goal != *goal_cell) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planning endpoints adjusted after inflation: start=(%d,%d)->(%d,%d) "
          "goal=(%d,%d)->(%d,%d)",
          start_cell->x, start_cell->y, unblocked_start->x, unblocked_start->y,
          goal_cell->x, goal_cell->y, unblocked_goal->x, unblocked_goal->y);
    }

    const AStarResult astar_result =
        planner_.plan(planning_grid, *unblocked_start, *unblocked_goal, astar_config_);
    if (!astar_result.success) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "A* did not find a path on obstacle memory; expanded=%zu start=(%d,%d) "
          "goal=(%d,%d)",
          astar_result.expanded_cells, unblocked_start->x, unblocked_start->y,
          unblocked_goal->x, unblocked_goal->y);
      publishFallbackPath();
      return;
    }

    const std::vector<GridIndex> smoothed_cells =
        smoothPath(planning_grid, astar_result.path, path_smoothing_config_);
    const GridStats stats = collectGridStats(planning_grid);
    const double raw_path_clearance_m =
        pathMinimumBlockedClearanceM(planning_grid, astar_result.path);
    const double smoothed_path_clearance_m =
        pathMinimumBlockedClearanceM(planning_grid, smoothed_cells);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planning summary: pose=(%.2f, %.2f) distance_to_start=%.2f "
        "distance_to_goal=%.2f occupied=%zu inflated=%zu free=%zu unknown=%zu "
        "current_lidar[used=%s fresh=%s hits=%zu occupied_cells=%zu outside=%zu] "
        "expanded=%zu raw_path=%zu smoothed_path=%zu "
        "path_clearance[raw=%.2f smoothed=%.2f]",
        current_pose_.position.x, current_pose_.position.y,
        distance(current_pose_.position, start_),
        distance(current_pose_.position, goal_), stats.occupied_cells,
        stats.inflated_cells, stats.free_cells, stats.unknown_cells,
        current_lidar_stats.used ? "true" : "false",
        current_lidar_stats.fresh ? "true" : "false", current_lidar_stats.hit_beams,
        current_lidar_stats.occupied_cells, current_lidar_stats.outside_hits,
        astar_result.expanded_cells, astar_result.path.size(), smoothed_cells.size(),
        raw_path_clearance_m, smoothed_path_clearance_m);

    std::vector<Point2> path_points = cellsToPoints(planning_grid, smoothed_cells);
    if (path_points.empty()) {
      publishPath({});
      return;
    }

    if (distance(current_pose_.position, path_points.front()) <
        planning_grid.resolution()) {
      path_points.front() = current_pose_.position;
    } else {
      path_points.insert(path_points.begin(), current_pose_.position);
    }
    pruneBackwardInitialWaypoints(path_points, planning_grid.resolution());
    if (hasExcessiveInitialLateralDeviation(path_points)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "A* path has excessive initial lateral deviation; using "
                           "direct fallback path");
      publishDirectGoalPath();
      return;
    }

    last_valid_path_points_ = path_points;
    publishPath(path_points);
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

  [[nodiscard]] bool currentLidarHit(const float raw_range,
                                     const double scan_range_max) const {
    return std::isfinite(raw_range) && raw_range >= last_scan_.range_min &&
           static_cast<double>(raw_range) < scan_range_max - range_hit_epsilon_m_;
  }

  [[nodiscard]] Point2 lidarDirection(const double angle_rad) const {
    const double scan_x = std::cos(angle_rad);
    const double scan_y = std::sin(angle_rad);
    if (swap_lidar_xy_to_local_frame_) {
      return Point2{scan_y, scan_x};
    }
    return Point2{scan_x, scan_y};
  }

  [[nodiscard]] Point2 currentLidarEndpoint(const std::size_t beam_index,
                                            const double range_m) const {
    const double yaw_rad =
        (use_px4_heading_for_scan_ ? current_pose_.yaw_rad : initial_heading_rad_) +
        scan_yaw_offset_rad_;
    const double angle_rad = yaw_rad + static_cast<double>(last_scan_.angle_min) +
                             static_cast<double>(beam_index) *
                                 static_cast<double>(last_scan_.angle_increment);
    const Point2 direction = lidarDirection(angle_rad);
    return Point2{current_pose_.position.x + range_m * direction.x,
                  current_pose_.position.y + range_m * direction.y};
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
    if (!use_current_lidar_obstacles_) {
      return stats;
    }

    stats.used = scan_seen_;
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

    for (std::size_t i = 0U; i < last_scan_.ranges.size(); ++i) {
      const float raw_range = last_scan_.ranges[i];
      const bool hit = currentLidarHit(raw_range, scan_range_max);
      if (std::isfinite(raw_range) &&
          raw_range >= static_cast<float>(last_scan_.range_min)) {
        ++stats.processed_beams;
      }
      if (!hit) {
        continue;
      }
      const double range_m = static_cast<double>(raw_range);
      if (!(range_m >= static_cast<double>(last_scan_.range_min))) {
        continue;
      }

      ++stats.hit_beams;
      const Point2 endpoint = currentLidarEndpoint(i, range_m);
      const Point2 depth_endpoint =
          currentLidarEndpoint(i, range_m + current_lidar_obstacle_depth_m_);
      const std::size_t occupied_cells =
          markCurrentLidarObstacle(grid, endpoint, depth_endpoint);
      if (occupied_cells == 0U) {
        ++stats.outside_hits;
      } else {
        stats.occupied_cells += occupied_cells;
      }
    }

    return stats;
  }

  void publishOccupancyGrid(const OccupancyGrid2D& grid) {
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
        } else if (grid.isInflated(cell)) {
          msg.data[index] = static_cast<std::int8_t>(80);
        } else if (grid.state(cell) == CellState::kFree) {
          msg.data[index] = static_cast<std::int8_t>(0);
        }
      }
    }

    occupancy_pub_->publish(msg);
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

  void publishDirectGoalPath() {
    std::vector<Point2> path_points{current_pose_.position, goal_};
    last_valid_path_points_ = path_points;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Publishing direct fallback path: start=(%.2f, %.2f) goal=(%.2f, %.2f)",
        current_pose_.position.x, current_pose_.position.y, goal_.x, goal_.y);
    publishPath(path_points);
  }

  void invalidateCurrentPose() {
    current_pose_ = Pose2{};
    pose_valid_ = false;
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

  void pruneBackwardInitialWaypoints(std::vector<Point2>& path_points,
                                     const double grid_resolution_m) const {
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
      if (projection >= grid_resolution_m * grid_resolution_m) {
        break;
      }
      ++first_forward;
    }

    if (first_forward != path_points.begin() + 1) {
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
  AStarPlanner planner_;
  AStarConfig astar_config_{};
  PathSmoothingConfig path_smoothing_config_{};

  Pose2 current_pose_{};
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
  bool use_current_lidar_obstacles_{true};
  bool use_px4_heading_for_scan_{false};
  bool swap_lidar_xy_to_local_frame_{false};
  std::string frame_id_{"map"};
  double inflation_radius_m_{2.5};
  double cruise_altitude_m_{12.0};
  double max_initial_lateral_deviation_m_{8.0};
  double max_lidar_range_m_{35.0};
  double range_hit_epsilon_m_{0.05};
  double current_lidar_obstacle_depth_m_{0.0};
  double scan_yaw_offset_rad_{0.0};
  double initial_heading_rad_{0.0};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t max_current_lidar_staleness_ns_{750'000'000};
  std::int64_t last_pose_update_ns_{0};
  std::int64_t last_scan_update_ns_{0};
  int nearest_free_radius_cells_{10};
  int occupied_threshold_{65};
  int free_threshold_{0};
  std::size_t last_logged_path_size_{std::numeric_limits<std::size_t>::max()};
  Point2 last_logged_path_first_{};
  Point2 last_logged_path_last_{};
  std::vector<Point2> last_valid_path_points_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr memory_grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
