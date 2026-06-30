#include "drone_city_nav/grid_config.hpp"
#include "drone_city_nav/lidar_motion_compensation.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
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

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int8_t rawOccupancyValue(const OccupancyGrid2D& grid,
                                            const GridIndex cell) {
  if (grid.isOccupied(cell)) {
    return static_cast<std::int8_t>(100);
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
    const double requested_resolution_m =
        declare_parameter<double>("grid_resolution_m", 0.5);
    const double width_m = declare_parameter<double>("grid_width_m", 120.0);
    const double height_m = declare_parameter<double>("grid_height_m", 80.0);
    const double origin_x = declare_parameter<double>("grid_origin_x", -20.0);
    const double origin_y = declare_parameter<double>("grid_origin_y", -40.0);

    const GridBounds memory_bounds = boundedGridBounds(
        origin_x, origin_y, requested_resolution_m, width_m, height_m);
    memory_ = std::make_unique<ObstacleMemoryGrid>(memory_bounds);

    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    use_px4_heading_for_scan_ =
        declare_parameter<bool>("use_px4_heading_for_scan", true);
    motion_compensate_lidar_pose_ =
        declare_parameter<bool>("motion_compensate_lidar_pose", true);
    lidar_pose_latency_s_ =
        std::clamp(declare_parameter<double>("lidar_pose_latency_s", 0.05), 0.0, 1.0);
    min_mapping_altitude_m_ = declare_parameter<double>("min_mapping_altitude_m", 0.0);
    max_pose_staleness_ns_ = static_cast<std::int64_t>(
        std::clamp<double>(declare_parameter<double>("max_pose_staleness_s", 1.0), 0.0,
                           3600.0) *
        1.0e9);
    memory_config_.max_lidar_range_m =
        declare_parameter<double>("max_lidar_range_m", 35.0);
    memory_config_.range_hit_epsilon_m =
        declare_parameter<double>("range_hit_epsilon_m", 0.05);
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
    memory_config_.occupied_score =
        std::clamp(memory_config_.occupied_score, memory_config_.free_score + 1,
                   memory_config_.max_score);
    memory_config_.free_score =
        std::clamp(memory_config_.free_score, memory_config_.min_score,
                   memory_config_.occupied_score - 1);
    scan_yaw_offset_rad_ = declare_parameter<double>("scan_yaw_offset_rad", 0.0);
    compensate_lidar_attitude_ =
        declare_parameter<bool>("compensate_lidar_attitude", true);
    lidar_mount_roll_rad_ = declare_parameter<double>("lidar_mount_roll_rad", 0.0);
    lidar_mount_pitch_rad_ = declare_parameter<double>("lidar_mount_pitch_rad", 0.0);
    lidar_mount_yaw_rad_ = declare_parameter<double>("lidar_mount_yaw_rad", 0.0);
    lidar_z_offset_m_ = declare_parameter<double>("lidar_z_offset_m", 0.0);
    min_projected_lidar_altitude_m_ =
        declare_parameter<double>("min_projected_lidar_altitude_m", 0.0);
    max_projected_lidar_altitude_m_ =
        declare_parameter<double>("max_projected_lidar_altitude_m", 100000.0);

    const bool use_initial_pose =
        declare_parameter<bool>("use_initial_pose_until_px4", true);
    initial_heading_rad_ = declare_parameter<double>("initial_heading_rad", 0.0);
    px4_local_pose_config_ =
        Px4LocalPoseConfig{use_px4_heading_for_scan_, initial_heading_rad_,
                           declare_parameter<double>("px4_local_origin_x_m", 0.0),
                           declare_parameter<double>("px4_local_origin_y_m", 0.0)};
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
    const std::string attitude_topic = declare_parameter<std::string>(
        "px4_vehicle_attitude_topic", "/fmu/out/vehicle_attitude");

    raw_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("obstacle_memory_grid_topic",
                                       "/drone_city_nav/obstacle_memory_grid"),
        rclcpp::QoS{1}.transient_local());

    const auto sensor_qos = rclcpp::SensorDataQoS{};
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, sensor_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { onScan(*msg); });
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
          onAttitude(*msg);
        });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic, sensor_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          onLocalPosition(*msg);
        });

    RCLCPP_INFO(get_logger(),
                "Obstacle memory ready: pose=px4_local_position grid=%dx%d "
                "resolution=%.2fm origin=(%.1f, %.1f) lidar='%s' attitude='%s'",
                memory_->rawGrid().width(), memory_->rawGrid().height(),
                memory_->rawGrid().resolution(), memory_->rawGrid().originX(),
                memory_->rawGrid().originY(), lidar_topic.c_str(),
                attitude_topic.c_str());
    RCLCPP_INFO(get_logger(),
                "Obstacle memory config: max_range=%.2f stride=%d "
                "raw_memory_only=true "
                "score[min=%d max=%d free<=%d occupied>=%d] "
                "yaw_source=%s compensate_attitude=%s lidar_z_offset=%.2f "
                "projected_altitude_range=[%.2f, %.2f] "
                "motion_compensation=%s pose_latency=%.3fs "
                "lidar_mount_rpy=(%.3f, %.3f, %.3f)",
                memory_config_.max_lidar_range_m, memory_config_.scan_stride,
                memory_config_.min_score, memory_config_.max_score,
                memory_config_.free_score, memory_config_.occupied_score,
                use_px4_heading_for_scan_ ? "px4_heading" : "initial_map_aligned",
                compensate_lidar_attitude_ ? "true" : "false", lidar_z_offset_m_,
                min_projected_lidar_altitude_m_, max_projected_lidar_altitude_m_,
                motion_compensate_lidar_pose_ ? "true" : "false", lidar_pose_latency_s_,
                lidar_mount_roll_rad_, lidar_mount_pitch_rad_, lidar_mount_yaw_rad_);
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
        sample, px4_local_pose_config_, current_pose_);
    if (status == Px4LocalPoseUpdateStatus::kInvalidPosition) {
      last_pose_update_ns_ = 0;
      current_velocity_ = Point2{};
      current_velocity_valid_ = false;
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
      current_velocity_ = Point2{};
      current_velocity_valid_ = false;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Obstacle memory invalidated cached pose after PX4 local position without "
          "valid heading: heading_good_for_control=%s heading=%.3f",
          msg.heading_good_for_control ? "true" : "false",
          static_cast<double>(msg.heading));
      return;
    }

    last_pose_update_ns_ = get_clock()->now().nanoseconds();
    if (msg.v_xy_valid && std::isfinite(msg.vx) && std::isfinite(msg.vy)) {
      current_velocity_ =
          Point2{static_cast<double>(msg.vx), static_cast<double>(msg.vy)};
      current_velocity_valid_ = true;
    } else {
      current_velocity_ = Point2{};
      current_velocity_valid_ = false;
    }
    logFirstPose("px4_local_position");
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

    const double pose_lag_s = poseReceiveLagSeconds(now_ns);
    const LidarPoseMotionCompensationResult motion_compensation =
        compensateLidarPoseForLatency(current_pose_.pose.position, current_velocity_,
                                      motion_compensate_lidar_pose_,
                                      current_velocity_valid_, pose_lag_s,
                                      lidar_pose_latency_s_);
    Pose2 scan_pose = current_pose_.pose;
    scan_pose.position = motion_compensation.position;

    LaserScan2DView scan_view{};
    scan_view.ranges = std::span<const float>{scan.ranges.data(), scan.ranges.size()};
    scan_view.angle_min_rad = static_cast<double>(scan.angle_min);
    scan_view.angle_increment_rad = static_cast<double>(scan.angle_increment);
    scan_view.range_min_m = static_cast<double>(scan.range_min);
    scan_view.range_max_m = static_cast<double>(scan.range_max);
    scan_view.scan_yaw_offset_rad = scan_yaw_offset_rad_;
    scan_view.origin_altitude_m = current_pose_.altitude_m;
    scan_view.roll_rad = current_attitude_.roll_rad;
    scan_view.pitch_rad = current_attitude_.pitch_rad;
    scan_view.lidar_z_offset_m = lidar_z_offset_m_;
    scan_view.min_projected_altitude_m = min_projected_lidar_altitude_m_;
    scan_view.max_projected_altitude_m = max_projected_lidar_altitude_m_;
    scan_view.altitude_valid = current_pose_.altitude_valid;
    scan_view.attitude_valid = attitude_valid_;
    scan_view.compensate_attitude = compensate_lidar_attitude_;
    scan_view.lidar_mount_roll_rad = lidar_mount_roll_rad_;
    scan_view.lidar_mount_pitch_rad = lidar_mount_pitch_rad_;
    scan_view.lidar_mount_yaw_rad = lidar_mount_yaw_rad_;
    const ObstacleMemoryStats stats =
        memory_->integrateScan(scan_pose, scan_view, memory_config_);

    if (!scan_seen_) {
      scan_seen_ = true;
      RCLCPP_INFO(
          get_logger(),
          "First lidar scan: beams=%zu processed=%zu hits=%zu range=[%.2f, %.2f] "
          "angle=[%.2f, %.2f] attitude_valid=%s",
          scan.ranges.size(), stats.processed_beams, stats.hit_beams,
          static_cast<double>(scan.range_min), static_cast<double>(scan.range_max),
          static_cast<double>(scan.angle_min), static_cast<double>(scan.angle_max),
          attitude_valid_ ? "true" : "false");
    }

    publishMemoryGrid();

    const GridCellCounts raw_counts = memory_->countRawCells();
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Obstacle memory update: pose=(%.2f, %.2f, altitude=%.2f, yaw=%.2f) "
        "scan_pose=(%.2f, %.2f) pose_lag=%.3fs pose_latency=%.3fs "
        "motion_shift=(%.2f, %.2f) motion_shift_m=%.2f "
        "roll=%.3f pitch=%.3f attitude_valid=%s processed=%zu hits=%zu invalid=%zu "
        "altitude_rejected=%zu clipped=%zu outside_hits=%zu free_updates=%zu "
        "occupied_updates=%zu raw[occupied=%zu free=%zu unknown=%zu]",
        current_pose_.pose.position.x, current_pose_.pose.position.y,
        current_pose_.altitude_m, current_pose_.pose.yaw_rad, scan_pose.position.x,
        scan_pose.position.y, motion_compensation.pose_lag_s,
        motion_compensation.latency_s, motion_compensation.applied_shift.x,
        motion_compensation.applied_shift.y, motion_compensation.applied_shift_m,
        current_attitude_.roll_rad, current_attitude_.pitch_rad,
        attitude_valid_ ? "true" : "false", stats.processed_beams, stats.hit_beams,
        stats.invalid_ranges, stats.altitude_rejected_beams, stats.clipped_rays,
        stats.outside_hit_endpoints, stats.free_cells_updated,
        stats.occupied_cells_updated, raw_counts.occupied_cells, raw_counts.free_cells,
        raw_counts.unknown_cells);
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
    current_velocity_ = Point2{};
    current_velocity_valid_ = false;
  }

  [[nodiscard]] double poseAgeSeconds(const std::int64_t now_ns) const {
    if (last_pose_update_ns_ <= 0 || now_ns <= last_pose_update_ns_) {
      return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(now_ns - last_pose_update_ns_) / 1.0e9;
  }

  [[nodiscard]] double poseReceiveLagSeconds(const std::int64_t scan_receive_ns) const {
    if (scan_receive_ns > 0 && last_pose_update_ns_ > 0 &&
        scan_receive_ns > last_pose_update_ns_) {
      return static_cast<double>(scan_receive_ns - last_pose_update_ns_) / 1.0e9;
    }
    return 0.0;
  }

  void publishMemoryGrid() {
    const rclcpp::Time stamp = now();
    raw_grid_pub_->publish(makeOccupancyGridMessage(memory_->rawGrid(), stamp));
  }

  [[nodiscard]] nav_msgs::msg::OccupancyGrid
  makeOccupancyGridMessage(const OccupancyGrid2D& grid,
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
        msg.data[grid.linearIndex(cell)] = rawOccupancyValue(grid, cell);
      }
    }
    return msg;
  }

  std::unique_ptr<ObstacleMemoryGrid> memory_;
  ObstacleMemoryConfig memory_config_{};
  Px4LocalPoseConfig px4_local_pose_config_{};
  NavigationPose2D current_pose_{};
  AttitudeEuler current_attitude_{};
  Point2 current_velocity_{};

  std::string frame_id_{"map"};
  double min_mapping_altitude_m_{0.0};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t last_pose_update_ns_{0};
  double scan_yaw_offset_rad_{0.0};
  double initial_heading_rad_{0.0};
  double lidar_z_offset_m_{0.0};
  double lidar_mount_roll_rad_{0.0};
  double lidar_mount_pitch_rad_{0.0};
  double lidar_mount_yaw_rad_{0.0};
  double min_projected_lidar_altitude_m_{0.0};
  double max_projected_lidar_altitude_m_{100000.0};
  double lidar_pose_latency_s_{0.05};
  bool use_px4_heading_for_scan_{true};
  bool motion_compensate_lidar_pose_{true};
  bool compensate_lidar_attitude_{true};
  bool pose_seen_{false};
  bool scan_seen_{false};
  bool attitude_valid_{false};
  bool current_velocity_valid_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_grid_pub_;
};

} // namespace drone_city_nav

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::ObstacleMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
