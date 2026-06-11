#include "drone_city_nav/astar_planner.hpp"
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
#include <memory>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] int positiveCellCount(const double length_m,
                                    const double resolution_m) {
  return std::max(1, static_cast<int>(std::ceil(length_m / resolution_m)));
}

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

struct GridStats {
  std::size_t unknown_cells{0U};
  std::size_t free_cells{0U};
  std::size_t occupied_cells{0U};
  std::size_t inflated_cells{0U};
};

} // namespace

class PlannerNode final : public rclcpp::Node {
public:
  PlannerNode() : Node{"planner_node"} {
    const double resolution_m =
        declare_parameter<double>("grid_resolution_m", 0.5);
    const double width_m = declare_parameter<double>("grid_width_m", 120.0);
    const double height_m = declare_parameter<double>("grid_height_m", 80.0);
    const double origin_x = declare_parameter<double>("grid_origin_x", -20.0);
    const double origin_y = declare_parameter<double>("grid_origin_y", -40.0);

    inflation_radius_m_ = declare_parameter<double>("inflation_radius_m", 2.5);
    goal_ = Point2{declare_parameter<double>("goal_x_m", 85.0),
                   declare_parameter<double>("goal_y_m", 0.0)};
    cruise_altitude_m_ = declare_parameter<double>("cruise_altitude_m", 12.0);
    scan_yaw_offset_rad_ =
        declare_parameter<double>("scan_yaw_offset_rad", 0.0);
    max_lidar_range_m_ = declare_parameter<double>("max_lidar_range_m", 35.0);
    range_hit_epsilon_m_ =
        declare_parameter<double>("range_hit_epsilon_m", 0.05);
    nearest_free_radius_cells_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("nearest_free_radius_cells", 10), 0,
        100000));
    scan_stride_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("scan_stride", 1), 1, 100000));
    astar_config_.max_expansions =
        static_cast<std::size_t>(std::clamp<std::int64_t>(
            declare_parameter<std::int64_t>("astar_max_expansions", 100000), 1,
            std::numeric_limits<int>::max()));

    const bool use_initial_pose =
        declare_parameter<bool>("use_initial_pose_until_px4", true);
    if (use_initial_pose) {
      current_pose_ =
          Pose2{Point2{declare_parameter<double>("initial_x_m", 0.0),
                       declare_parameter<double>("initial_y_m", 0.0)},
                declare_parameter<double>("initial_heading_rad", 0.0)};
      pose_valid_ = true;
    }

    grid_ = std::make_unique<OccupancyGrid2D>(
        GridBounds{origin_x, origin_y, resolution_m,
                   positiveCellCount(width_m, resolution_m),
                   positiveCellCount(height_m, resolution_m)});

    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const std::string lidar_topic =
        declare_parameter<std::string>("lidar_topic", "/scan");
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");

    const auto sensor_qos = rclcpp::SensorDataQoS{};
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, sensor_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          onScan(*msg);
        });
    local_position_sub_ =
        create_subscription<px4_msgs::msg::VehicleLocalPosition>(
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

    const double replan_period_s =
        declare_parameter<double>("replan_period_s", 0.5);
    timer_ = create_wall_timer(
        std::chrono::duration<double>{std::max(0.05, replan_period_s)},
        [this]() { replanAndPublish(); });

    RCLCPP_INFO(get_logger(),
                "Planner ready: grid=%dx%d resolution=%.2fm goal=(%.1f, %.1f)",
                grid_->width(), grid_->height(), grid_->resolution(), goal_.x,
                goal_.y);
    RCLCPP_INFO(get_logger(),
                "Planner subscriptions: lidar='%s' local_position='%s'",
                lidar_topic.c_str(), local_position_topic.c_str());
  }

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition &msg) {
    if (!msg.xy_valid || !std::isfinite(msg.x) || !std::isfinite(msg.y)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring invalid PX4 local position: xy_valid=%s x=%.2f y=%.2f",
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

    if (!local_position_seen_) {
      local_position_seen_ = true;
      RCLCPP_INFO(get_logger(),
                  "First valid PX4 local position: x=%.2f y=%.2f yaw=%.2f",
                  current_pose_.position.x, current_pose_.position.y,
                  current_pose_.yaw_rad);
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan &scan) {
    if (!pose_valid_ || grid_ == nullptr || !finite2D(current_pose_.position) ||
        !std::isfinite(scan.angle_increment) || scan.angle_increment == 0.0F) {
      return;
    }

    const double scan_range_max =
        std::min(static_cast<double>(scan.range_max), max_lidar_range_m_);
    if (!(scan_range_max > 0.0)) {
      return;
    }

    const auto stride = static_cast<std::size_t>(scan_stride_);
    std::size_t processed_beams = 0U;
    std::size_t hit_beams = 0U;
    for (std::size_t i = 0; i < scan.ranges.size(); i += stride) {
      const float raw_range = scan.ranges[i];
      const bool finite_range = std::isfinite(raw_range);
      const bool hit = finite_range && raw_range >= scan.range_min &&
                       static_cast<double>(raw_range) <
                           scan_range_max - range_hit_epsilon_m_;
      const double range_m =
          hit ? static_cast<double>(raw_range) : scan_range_max;
      if (!(range_m >= static_cast<double>(scan.range_min))) {
        continue;
      }
      ++processed_beams;
      if (hit) {
        ++hit_beams;
      }

      const double angle_rad =
          current_pose_.yaw_rad + scan_yaw_offset_rad_ +
          static_cast<double>(scan.angle_min) +
          static_cast<double>(i) * static_cast<double>(scan.angle_increment);
      const Point2 end{current_pose_.position.x + range_m * std::cos(angle_rad),
                       current_pose_.position.y +
                           range_m * std::sin(angle_rad)};
      grid_->markRay(current_pose_.position, end, hit);
    }

    if (!scan_seen_) {
      scan_seen_ = true;
      RCLCPP_INFO(get_logger(),
                  "First lidar scan: beams=%zu processed=%zu range=[%.2f, %.2f] "
                  "angle=[%.2f, %.2f]",
                  scan.ranges.size(), processed_beams,
                  static_cast<double>(scan.range_min),
                  static_cast<double>(scan.range_max),
                  static_cast<double>(scan.angle_min),
                  static_cast<double>(scan.angle_max));
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Lidar update: pose=(%.2f, %.2f, yaw=%.2f) processed=%zu hits=%zu "
        "range_max=%.2f",
        current_pose_.position.x, current_pose_.position.y,
        current_pose_.yaw_rad, processed_beams, hit_beams, scan_range_max);
  }

  void replanAndPublish() {
    if (!pose_valid_ || grid_ == nullptr) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Planner is waiting for a valid PX4 local position");
      return;
    }

    grid_->rebuildInflation(inflation_radius_m_);
    publishOccupancyGrid();

    const auto start_cell = grid_->worldToCell(current_pose_.position);
    const auto goal_cell = grid_->worldToCell(goal_);
    if (!start_cell.has_value() || !goal_cell.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Start or goal is outside the occupancy grid");
      publishPath({});
      return;
    }

    const auto unblocked_start =
        grid_->nearestUnblocked(*start_cell, nearest_free_radius_cells_);
    const auto unblocked_goal =
        grid_->nearestUnblocked(*goal_cell, nearest_free_radius_cells_);
    if (!unblocked_start.has_value() || !unblocked_goal.has_value()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No unblocked start or goal cell is available after inflation");
      publishPath({});
      return;
    }

    if (*unblocked_start != *start_cell || *unblocked_goal != *goal_cell) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Planning endpoints adjusted after inflation: start=(%d,%d)->(%d,%d) "
          "goal=(%d,%d)->(%d,%d)",
          start_cell->x, start_cell->y, unblocked_start->x,
          unblocked_start->y, goal_cell->x, goal_cell->y, unblocked_goal->x,
          unblocked_goal->y);
    }

    const AStarResult astar_result =
        planner_.plan(*grid_, *unblocked_start, *unblocked_goal, astar_config_);
    if (!astar_result.success) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "A* did not find a path; expanded=%zu start=(%d,%d) goal=(%d,%d)",
          astar_result.expanded_cells, unblocked_start->x, unblocked_start->y,
          unblocked_goal->x, unblocked_goal->y);
      publishPath({});
      return;
    }

    const std::vector<GridIndex> smoothed_cells =
        smoothPath(*grid_, astar_result.path);
    const GridStats stats = collectGridStats();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planning summary: pose=(%.2f, %.2f) occupied=%zu inflated=%zu free=%zu "
        "unknown=%zu expanded=%zu raw_path=%zu smoothed_path=%zu",
        current_pose_.position.x, current_pose_.position.y,
        stats.occupied_cells, stats.inflated_cells, stats.free_cells,
        stats.unknown_cells, astar_result.expanded_cells,
        astar_result.path.size(), smoothed_cells.size());
    publishPath(cellsToPoints(*grid_, smoothed_cells));
  }

  [[nodiscard]] GridStats collectGridStats() const {
    GridStats stats{};
    if (grid_ == nullptr) {
      return stats;
    }

    for (int y = 0; y < grid_->height(); ++y) {
      for (int x = 0; x < grid_->width(); ++x) {
        const GridIndex cell{x, y};
        if (grid_->isInflated(cell)) {
          ++stats.inflated_cells;
        }

        switch (grid_->state(cell)) {
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

  void publishOccupancyGrid() {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.info.map_load_time = msg.header.stamp;
    msg.info.resolution = static_cast<float>(grid_->resolution());
    msg.info.width = static_cast<std::uint32_t>(grid_->width());
    msg.info.height = static_cast<std::uint32_t>(grid_->height());
    msg.info.origin.position.x = grid_->originX();
    msg.info.origin.position.y = grid_->originY();
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(grid_->cellCount(), -1);

    for (int y = 0; y < grid_->height(); ++y) {
      for (int x = 0; x < grid_->width(); ++x) {
        const GridIndex cell{x, y};
        const std::size_t index = grid_->linearIndex(cell);
        if (grid_->isOccupied(cell)) {
          msg.data[index] = 100;
        } else if (grid_->isInflated(cell)) {
          msg.data[index] = 80;
        } else if (grid_->state(cell) == CellState::kFree) {
          msg.data[index] = 0;
        }
      }
    }

    occupancy_pub_->publish(msg);
  }

  void publishPath(const std::vector<Point2> &points) {
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

  void logPathUpdate(const nav_msgs::msg::Path &path) {
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
      RCLCPP_INFO(get_logger(),
                  "Published path: waypoints=%zu first=(%.2f, %.2f) "
                  "last=(%.2f, %.2f)",
                  path_size, first.x, first.y, last.x, last.y);
      last_logged_path_size_ = path_size;
      last_logged_path_first_ = first;
      last_logged_path_last_ = last;
    }
  }

  std::unique_ptr<OccupancyGrid2D> grid_;
  AStarPlanner planner_;
  AStarConfig astar_config_{};

  Pose2 current_pose_{};
  Point2 goal_{};
  bool pose_valid_{false};
  bool local_position_seen_{false};
  bool scan_seen_{false};
  std::string frame_id_{"map"};
  double inflation_radius_m_{2.5};
  double cruise_altitude_m_{12.0};
  double scan_yaw_offset_rad_{0.0};
  double max_lidar_range_m_{35.0};
  double range_hit_epsilon_m_{0.05};
  int nearest_free_radius_cells_{10};
  int scan_stride_{1};
  std::size_t last_logged_path_size_{std::numeric_limits<std::size_t>::max()};
  Point2 last_logged_path_first_{};
  Point2 last_logged_path_last_{};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
