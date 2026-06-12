#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>

namespace drone_city_nav {
namespace {

[[nodiscard]] int positiveCellCount(const double length_m, const double resolution_m) {
  return std::max(1, static_cast<int>(std::ceil(length_m / resolution_m)));
}

[[nodiscard]] std::int64_t toNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] std::int8_t occupancyValue(const OccupancyGrid2D& grid,
                                         const GridIndex cell,
                                         const bool include_inflation) {
  if (grid.isOccupied(cell)) {
    return static_cast<std::int8_t>(100);
  }
  if (include_inflation && grid.isInflated(cell)) {
    return static_cast<std::int8_t>(80);
  }
  if (grid.state(cell) == CellState::kFree) {
    return static_cast<std::int8_t>(0);
  }
  return static_cast<std::int8_t>(-1);
}

} // namespace

class ObstacleMemoryNode final : public rclcpp::Node {
public:
  ObstacleMemoryNode()
      : Node{"obstacle_memory_node"} {
    const double resolution_m = declare_parameter<double>("grid_resolution_m", 0.5);
    const double width_m = declare_parameter<double>("grid_width_m", 120.0);
    const double height_m = declare_parameter<double>("grid_height_m", 80.0);
    const double origin_x = declare_parameter<double>("grid_origin_x", -20.0);
    const double origin_y = declare_parameter<double>("grid_origin_y", -40.0);

    memory_ = std::make_unique<ObstacleMemoryGrid>(GridBounds{
        origin_x, origin_y, resolution_m, positiveCellCount(width_m, resolution_m),
        positiveCellCount(height_m, resolution_m)});

    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    pose_source_ = declare_parameter<std::string>("pose_source", "px4_local_position");
    use_px4_heading_for_scan_ =
        declare_parameter<bool>("use_px4_heading_for_scan", true);
    min_mapping_altitude_m_ = declare_parameter<double>("min_mapping_altitude_m", 0.0);
    max_pose_staleness_ns_ = static_cast<std::int64_t>(
        std::clamp<double>(declare_parameter<double>("max_pose_staleness_s", 1.0), 0.0,
                           3600.0) *
        1.0e9);
    inflation_radius_m_ = declare_parameter<double>("inflation_radius_m", 2.5);
    local_grid_radius_m_ = declare_parameter<double>("local_grid_radius_m", 45.0);
    memory_config_.max_lidar_range_m =
        declare_parameter<double>("max_lidar_range_m", 35.0);
    memory_config_.range_hit_epsilon_m =
        declare_parameter<double>("range_hit_epsilon_m", 0.05);
    memory_config_.hit_obstacle_depth_m =
        std::clamp(declare_parameter<double>("hit_obstacle_depth_m", 0.0), 0.0, 100.0);
    memory_config_.scan_stride = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("scan_stride", 1), 1, 100000));
    memory_config_.hit_weight = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("hit_weight", 4), 1, 100000));
    memory_config_.miss_weight = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("miss_weight", 1), 1, 100000));
    memory_config_.min_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("min_score", -8), -100000, 0));
    memory_config_.max_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("max_score", 12), 1, 100000));
    memory_config_.occupied_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("occupied_score", 3), 1, 100000));
    memory_config_.free_score = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("free_score", -1), -100000, -1));
    scan_yaw_offset_rad_ = declare_parameter<double>("scan_yaw_offset_rad", 0.0);
    swap_lidar_xy_to_local_frame_ =
        declare_parameter<bool>("swap_lidar_xy_to_local_frame", false);

    gps_config_.max_gps_staleness_ns = static_cast<std::int64_t>(
        std::clamp<double>(declare_parameter<double>("max_gps_staleness_s", 1.0), 0.0,
                           3600.0) *
        1.0e9);
    max_compass_staleness_ns_ = static_cast<std::int64_t>(
        std::clamp<double>(declare_parameter<double>("max_compass_staleness_s", 1.0),
                           0.0, 3600.0) *
        1.0e9);
    gps_config_.max_gps_horizontal_variance_m2 =
        declare_parameter<double>("max_gps_horizontal_variance_m2", 25.0);
    gps_config_.require_known_gps_covariance =
        declare_parameter<bool>("require_known_gps_covariance", false);
    gps_config_.auto_initialize_origin =
        declare_parameter<bool>("auto_initialize_origin", true);
    gps_config_.origin_latitude_deg =
        declare_parameter<double>("origin_latitude_deg", 0.0);
    gps_config_.origin_longitude_deg =
        declare_parameter<double>("origin_longitude_deg", 0.0);
    gps_config_.origin_altitude_m = declare_parameter<double>("origin_altitude_m", 0.0);
    gps_config_.yaw_offset_rad = declare_parameter<double>("yaw_offset_rad", 0.0);
    gps_config_.magnetic_declination_rad =
        declare_parameter<double>("magnetic_declination_rad", 0.0);
    gps_config_.compass_to_body_yaw_offset_rad =
        declare_parameter<double>("compass_to_body_yaw_offset_rad", 0.0);
    if (!gps_config_.auto_initialize_origin) {
      gps_origin_ = GeoReference{gps_config_.origin_latitude_deg,
                                 gps_config_.origin_longitude_deg,
                                 gps_config_.origin_altitude_m, true};
    }

    const bool use_initial_pose =
        declare_parameter<bool>("use_initial_pose_until_px4", true);
    initial_heading_rad_ = declare_parameter<double>("initial_heading_rad", 0.0);
    current_pose_.pose.yaw_rad = initial_heading_rad_;
    current_pose_.yaw_valid = !use_px4_heading_for_scan_;
    if (use_initial_pose) {
      current_pose_.pose.position =
          Point2{declare_parameter<double>("initial_x_m", 0.0),
                 declare_parameter<double>("initial_y_m", 0.0)};
      current_pose_.position_valid = true;
      last_pose_update_ns_ = get_clock()->now().nanoseconds();
    }

    const std::string lidar_topic =
        declare_parameter<std::string>("lidar_topic", "/scan");
    const std::string local_position_topic = declare_parameter<std::string>(
        "px4_local_position_topic", "/fmu/out/vehicle_local_position");
    const std::string gps_topic = declare_parameter<std::string>("gps_topic", "/fix");
    const std::string compass_imu_topic =
        declare_parameter<std::string>("compass_imu_topic", "/imu");

    raw_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("obstacle_memory_grid_topic",
                                       "/drone_city_nav/obstacle_memory_grid"),
        rclcpp::QoS{1}.transient_local());
    inflated_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("obstacle_memory_inflated_grid_topic",
                                       "/drone_city_nav/obstacle_memory_inflated_grid"),
        rclcpp::QoS{1}.transient_local());
    local_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("obstacle_memory_local_grid_topic",
                                       "/drone_city_nav/obstacle_memory_local_grid"),
        rclcpp::QoS{1}.transient_local());

    const auto sensor_qos = rclcpp::SensorDataQoS{};
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, sensor_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { onScan(*msg); });

    if (pose_source_ == "px4_local_position") {
      local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          local_position_topic, sensor_qos,
          [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            onLocalPosition(*msg);
          });
    } else if (pose_source_ == "gps_compass") {
      gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          gps_topic, sensor_qos,
          [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { onGps(*msg); });
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          compass_imu_topic, sensor_qos,
          [this](const sensor_msgs::msg::Imu::SharedPtr msg) { onImu(*msg); });
    } else {
      RCLCPP_ERROR(get_logger(),
                   "Unsupported pose_source='%s'; scans will be skipped until a "
                   "valid pose source is configured",
                   pose_source_.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "Obstacle memory ready: pose_source=%s grid=%dx%d resolution=%.2fm "
                "origin=(%.1f, %.1f) lidar='%s'",
                pose_source_.c_str(), memory_->rawGrid().width(),
                memory_->rawGrid().height(), memory_->rawGrid().resolution(), origin_x,
                origin_y, lidar_topic.c_str());
    RCLCPP_INFO(get_logger(),
                "Obstacle memory config: max_range=%.2f hit_depth=%.2f stride=%d "
                "score[min=%d max=%d free<=%d occupied>=%d] swap_lidar_xy=%s "
                "yaw_source=%s",
                memory_config_.max_lidar_range_m, memory_config_.hit_obstacle_depth_m,
                memory_config_.scan_stride, memory_config_.min_score,
                memory_config_.max_score, memory_config_.free_score,
                memory_config_.occupied_score,
                swap_lidar_xy_to_local_frame_ ? "true" : "false",
                use_px4_heading_for_scan_ ? "px4_heading" : "initial_map_aligned");
  }

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg) {
    const auto sample =
        Px4LocalPositionSample{static_cast<double>(msg.x),
                               static_cast<double>(msg.y),
                               static_cast<double>(msg.z),
                               static_cast<double>(msg.heading),
                               static_cast<std::int64_t>(msg.timestamp) * 1000LL,
                               msg.xy_valid,
                               msg.z_valid,
                               msg.heading_good_for_control};
    const Px4LocalPoseUpdateStatus status = updateNavigationPoseFromPx4LocalPosition(
        sample, Px4LocalPoseConfig{use_px4_heading_for_scan_, initial_heading_rad_},
        current_pose_);
    if (status == Px4LocalPoseUpdateStatus::kInvalidPosition) {
      last_pose_update_ns_ = 0;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Obstacle memory invalidated cached pose after invalid PX4 local position: "
          "xy_valid=%s x=%.2f y=%.2f",
          msg.xy_valid ? "true" : "false", static_cast<double>(msg.x),
          static_cast<double>(msg.y));
      return;
    }

    if (status == Px4LocalPoseUpdateStatus::kInvalidYaw) {
      last_pose_update_ns_ = 0;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Obstacle memory invalidated cached pose after PX4 local position without "
          "valid heading: heading_good_for_control=%s heading=%.3f",
          msg.heading_good_for_control ? "true" : "false",
          static_cast<double>(msg.heading));
      return;
    }

    last_pose_update_ns_ = get_clock()->now().nanoseconds();
    logFirstPose("px4_local_position");
  }

  void onGps(const sensor_msgs::msg::NavSatFix& msg) {
    const bool covariance_known = msg.position_covariance_type !=
                                  sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    const double horizontal_variance =
        std::max(msg.position_covariance[0], msg.position_covariance[4]);
    last_gps_ = GpsFixSample{msg.latitude,
                             msg.longitude,
                             msg.altitude,
                             toNanoseconds(msg.header.stamp),
                             static_cast<int>(msg.status.status),
                             covariance_known,
                             horizontal_variance};
    updateGpsCompassPose();
  }

  void onImu(const sensor_msgs::msg::Imu& msg) {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const CompassYawUpdateStatus status = updateCompassYawFromQuaternion(
        QuaternionSample{msg.orientation.w, msg.orientation.x, msg.orientation.y,
                         msg.orientation.z},
        msg.orientation_covariance[0] >= 0.0, now_ns, compass_yaw_);
    if (status == CompassYawUpdateStatus::kUnavailable) {
      invalidateCurrentPose();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Compass IMU orientation is unavailable; invalidated gps_compass pose");
      return;
    }

    if (status == CompassYawUpdateStatus::kInvalidYaw) {
      invalidateCurrentPose();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Compass IMU quaternion did not produce a valid yaw; invalidated "
          "gps_compass pose");
      return;
    }

    updateGpsCompassPose();
  }

  void updateGpsCompassPose() {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const bool origin_was_initialized = gps_origin_.initialized;
    const GpsCompassPoseUpdateStatus status = updateNavigationPoseFromGpsCompassState(
        last_gps_, compass_yaw_, now_ns, max_compass_staleness_ns_, gps_config_,
        gps_origin_, current_pose_);

    if (status == GpsCompassPoseUpdateStatus::kWaitingForGps ||
        status == GpsCompassPoseUpdateStatus::kMissingCompassYaw) {
      last_pose_update_ns_ = 0;
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for gps_compass pose: gps=%s compass=%s compass_fresh=%s "
          "compass_age_s=%.2f",
          last_gps_.has_value() ? "ready" : "missing",
          compass_yaw_.valid ? "ready" : "missing",
          compassYawReady(compass_yaw_, now_ns, max_compass_staleness_ns_) ? "true"
                                                                           : "false",
          compassAgeSeconds(now_ns));
      return;
    }

    if (status == GpsCompassPoseUpdateStatus::kStaleCompassYaw) {
      last_pose_update_ns_ = 0;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "gps_compass pose rejected because compass yaw is stale: compass_age_s=%.2f",
          compassAgeSeconds(now_ns));
      return;
    }

    if (status == GpsCompassPoseUpdateStatus::kRejectedPose) {
      last_pose_update_ns_ = 0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "gps_compass pose rejected: status=%d covariance_known=%s "
                           "horizontal_variance=%.3f",
                           last_gps_->status,
                           last_gps_->horizontal_variance_known ? "true" : "false",
                           last_gps_->horizontal_variance_m2);
      return;
    }

    if (!origin_was_initialized && gps_origin_.initialized) {
      RCLCPP_INFO(
          get_logger(), "GPS origin initialized: lat=%.8f lon=%.8f altitude=%.2f",
          gps_origin_.latitude_deg, gps_origin_.longitude_deg, gps_origin_.altitude_m);
    }
    last_pose_update_ns_ = now_ns;
    logFirstPose("gps_compass");
  }

  void onScan(const sensor_msgs::msg::LaserScan& scan) {
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const bool pose_fresh =
        timestampIsFresh(last_pose_update_ns_, now_ns, max_pose_staleness_ns_);
    const double pose_age_s = poseAgeSeconds(now_ns);
    if (memory_ == nullptr ||
        !navigationPoseReadyForScan(current_pose_, last_pose_update_ns_, now_ns,
                                    max_pose_staleness_ns_)) {
      if (!pose_fresh) {
        invalidateCurrentPose();
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping obstacle memory scan without valid navigation pose: "
          "position_valid=%s yaw_valid=%s pose_fresh=%s pose_age_s=%.2f",
          current_pose_.position_valid ? "true" : "false",
          current_pose_.yaw_valid ? "true" : "false", pose_fresh ? "true" : "false",
          pose_age_s);
      return;
    }
    if (min_mapping_altitude_m_ > 0.0 &&
        (!current_pose_.altitude_valid ||
         current_pose_.altitude_m < min_mapping_altitude_m_)) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping obstacle memory scan below mapping altitude: altitude=%.2f "
          "valid=%s required=%.2f",
          current_pose_.altitude_m, current_pose_.altitude_valid ? "true" : "false",
          min_mapping_altitude_m_);
      return;
    }

    LaserScan2DView scan_view{
        std::span<const float>{scan.ranges.data(), scan.ranges.size()},
        static_cast<double>(scan.angle_min),
        static_cast<double>(scan.angle_increment),
        static_cast<double>(scan.range_min),
        static_cast<double>(scan.range_max),
        scan_yaw_offset_rad_,
        swap_lidar_xy_to_local_frame_};
    const ObstacleMemoryStats stats =
        memory_->integrateScan(current_pose_.pose, scan_view, memory_config_);
    memory_->rebuildInflation(inflation_radius_m_);

    if (!scan_seen_) {
      scan_seen_ = true;
      RCLCPP_INFO(
          get_logger(),
          "First lidar scan: beams=%zu processed=%zu hits=%zu range=[%.2f, %.2f] "
          "angle=[%.2f, %.2f]",
          scan.ranges.size(), stats.processed_beams, stats.hit_beams,
          static_cast<double>(scan.range_min), static_cast<double>(scan.range_max),
          static_cast<double>(scan.angle_min), static_cast<double>(scan.angle_max));
    }

    publishMemoryGrids();

    const GridCellCounts raw_counts = memory_->countRawCells();
    const GridCellCounts inflated_counts = memory_->countInflatedCells();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Obstacle memory update: pose=(%.2f, %.2f, altitude=%.2f, yaw=%.2f) "
        "processed=%zu hits=%zu invalid=%zu clipped=%zu outside_hits=%zu "
        "free_updates=%zu occupied_updates=%zu depth_cells=%zu raw[occupied=%zu "
        "free=%zu unknown=%zu] inflated=%zu",
        current_pose_.pose.position.x, current_pose_.pose.position.y,
        current_pose_.altitude_m, current_pose_.pose.yaw_rad, stats.processed_beams,
        stats.hit_beams, stats.invalid_ranges, stats.clipped_rays,
        stats.outside_hit_endpoints, stats.free_cells_updated,
        stats.occupied_cells_updated, stats.obstacle_depth_cells,
        raw_counts.occupied_cells, raw_counts.free_cells, raw_counts.unknown_cells,
        inflated_counts.inflated_cells);
  }

  void logFirstPose(const char* source_name) {
    if (pose_seen_) {
      return;
    }
    pose_seen_ = true;
    RCLCPP_INFO(get_logger(),
                "First valid navigation pose: source=%s x=%.2f y=%.2f altitude=%.2f "
                "altitude_valid=%s yaw=%.2f",
                source_name, current_pose_.pose.position.x,
                current_pose_.pose.position.y, current_pose_.altitude_m,
                current_pose_.altitude_valid ? "true" : "false",
                current_pose_.pose.yaw_rad);
  }

  void invalidateCurrentPose() {
    invalidateNavigationPose(current_pose_);
    last_pose_update_ns_ = 0;
  }

  [[nodiscard]] double poseAgeSeconds(const std::int64_t now_ns) const {
    if (last_pose_update_ns_ <= 0 || now_ns <= last_pose_update_ns_) {
      return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(now_ns - last_pose_update_ns_) / 1.0e9;
  }

  [[nodiscard]] double compassAgeSeconds(const std::int64_t now_ns) const {
    if (compass_yaw_.last_update_ns <= 0 || now_ns <= compass_yaw_.last_update_ns) {
      return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(now_ns - compass_yaw_.last_update_ns) / 1.0e9;
  }

  void publishMemoryGrids() {
    const rclcpp::Time stamp = now();
    raw_grid_pub_->publish(makeOccupancyGridMessage(memory_->rawGrid(), false, stamp));
    inflated_grid_pub_->publish(
        makeOccupancyGridMessage(memory_->inflatedGrid(), true, stamp));
    local_grid_pub_->publish(makeLocalGridMessage(stamp));
  }

  [[nodiscard]] nav_msgs::msg::OccupancyGrid
  makeOccupancyGridMessage(const OccupancyGrid2D& grid, const bool include_inflation,
                           const rclcpp::Time& stamp) const {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.info.map_load_time = stamp;
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
        msg.data[grid.linearIndex(cell)] =
            occupancyValue(grid, cell, include_inflation);
      }
    }
    return msg;
  }

  [[nodiscard]] nav_msgs::msg::OccupancyGrid
  makeLocalGridMessage(const rclcpp::Time& stamp) {
    const OccupancyGrid2D& source_grid = memory_->inflatedGrid();
    const double radius = std::max(local_grid_radius_m_, source_grid.resolution());
    const int width_cells = positiveCellCount(2.0 * radius, source_grid.resolution());
    const GridBounds local_bounds{current_pose_.pose.position.x - radius,
                                  current_pose_.pose.position.y - radius,
                                  source_grid.resolution(), width_cells, width_cells};
    OccupancyGrid2D local_grid{local_bounds};
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.info.map_load_time = stamp;
    msg.info.resolution = static_cast<float>(local_grid.resolution());
    msg.info.width = static_cast<std::uint32_t>(local_grid.width());
    msg.info.height = static_cast<std::uint32_t>(local_grid.height());
    msg.info.origin.position.x = local_grid.originX();
    msg.info.origin.position.y = local_grid.originY();
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(local_grid.cellCount(), static_cast<std::int8_t>(-1));

    for (int y = 0; y < local_grid.height(); ++y) {
      for (int x = 0; x < local_grid.width(); ++x) {
        const GridIndex local_cell{x, y};
        const auto source_cell =
            source_grid.worldToCell(local_grid.cellCenter(local_cell));
        if (source_cell.has_value()) {
          msg.data[local_grid.linearIndex(local_cell)] =
              occupancyValue(source_grid, *source_cell, true);
        }
      }
    }

    if (!local_grid_seen_) {
      local_grid_seen_ = true;
      RCLCPP_INFO(get_logger(),
                  "Published obstacle memory local grid: center=(%.2f, %.2f) "
                  "radius=%.2f cells=%dx%d",
                  current_pose_.pose.position.x, current_pose_.pose.position.y, radius,
                  local_grid.width(), local_grid.height());
    } else {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Published obstacle memory local grid: center=(%.2f, %.2f) radius=%.2f "
          "cells=%dx%d",
          current_pose_.pose.position.x, current_pose_.pose.position.y, radius,
          local_grid.width(), local_grid.height());
    }
    return msg;
  }

  std::unique_ptr<ObstacleMemoryGrid> memory_;
  ObstacleMemoryConfig memory_config_{};
  GpsCompassConfig gps_config_{};
  GeoReference gps_origin_{};
  NavigationPose2D current_pose_{};
  std::optional<GpsFixSample> last_gps_;
  CompassYawState compass_yaw_{};

  std::string frame_id_{"map"};
  std::string pose_source_{"px4_local_position"};
  double min_mapping_altitude_m_{0.0};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t last_pose_update_ns_{0};
  std::int64_t max_compass_staleness_ns_{1'000'000'000};
  double inflation_radius_m_{2.5};
  double local_grid_radius_m_{45.0};
  double scan_yaw_offset_rad_{0.0};
  double initial_heading_rad_{0.0};
  bool swap_lidar_xy_to_local_frame_{false};
  bool use_px4_heading_for_scan_{true};
  bool pose_seen_{false};
  bool scan_seen_{false};
  bool local_grid_seen_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_grid_pub_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::ObstacleMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
