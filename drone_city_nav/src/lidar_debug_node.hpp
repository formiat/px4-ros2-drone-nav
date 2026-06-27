#pragma once

#include "drone_city_nav/debug_image.hpp"
#include "drone_city_nav/lidar_debug_renderer.hpp"
#include "drone_city_nav/lidar_motion_compensation.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/lidar_radar_markers.hpp"
#include "drone_city_nav/lidar_snapshot_writer.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {

inline constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
inline constexpr double kGroundDebugZ = 0.05;

[[nodiscard]] inline bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] inline std::string zeroPadded(const std::uint64_t value,
                                            const int width) {
  std::ostringstream stream;
  stream << std::setw(width) << std::setfill('0') << value;
  return stream.str();
}

[[nodiscard]] inline std::int64_t
toNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] inline double ageSecondsOrNan(const std::int64_t stamp_ns,
                                            const std::int64_t now_ns) noexcept {
  if (stamp_ns <= 0 || now_ns <= 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (stamp_ns >= now_ns) {
    return 0.0;
  }
  return static_cast<double>(now_ns - stamp_ns) /
         static_cast<double>(kNanosecondsPerSecond);
}

[[nodiscard]] inline double yawDeltaRad(const double lhs_rad,
                                        const double rhs_rad) noexcept {
  if (!std::isfinite(lhs_rad) || !std::isfinite(rhs_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::remainder(lhs_rad - rhs_rad, 2.0 * std::numbers::pi);
}

class LidarDebugNode final : public rclcpp::Node {
public:
  LidarDebugNode();

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg);

  void onAttitude(const px4_msgs::msg::VehicleAttitude& msg);

  void onScan(const sensor_msgs::msg::LaserScan& msg);

  void writeSnapshot();

  [[nodiscard]] double scanRangeMax() const;

  [[nodiscard]] const char* yawSourceName() const noexcept;

  [[nodiscard]] double projectionYawRad() const noexcept;

  [[nodiscard]] double
  scanTimeIncrementSeconds(const sensor_msgs::msg::LaserScan& scan) const noexcept;

  [[nodiscard]] double
  scanDurationSeconds(const sensor_msgs::msg::LaserScan& scan) const noexcept;

  [[nodiscard]] double poseReceiveLagSeconds() const noexcept;

  [[nodiscard]] double beamAgeSeconds(const std::size_t beam_index) const noexcept;

  [[nodiscard]] LidarProjectionPose
  lidarProjectionPoseForBeam(const std::size_t beam_index) const;

  [[nodiscard]] LidarProjectionConfig lidarProjectionConfig() const;

  [[nodiscard]] LidarBeamProjection projectScanBeam(const std::size_t beam_index,
                                                    const float raw_range) const;

  [[nodiscard]] Point2 headingDirection(const double projection_yaw_rad) const;

  [[nodiscard]] std::vector<LidarSnapshotCsvRow>
  collectScanRows(LidarSnapshotStats& stats) const;

  [[nodiscard]] std::optional<GridImageView> gridImageView() const;

  void countGrid(LidarSnapshotStats& stats) const;

  [[nodiscard]] std::vector<Point2>
  collectGridPoints(const nav_msgs::msg::OccupancyGrid& grid,
                    const std::uint8_t min_value, const std::uint8_t max_value) const;

  [[nodiscard]] std::vector<Point2> collectProhibitedGridPoints() const;

  [[nodiscard]] std::vector<Point2>
  collectOccupiedGridPoints(const nav_msgs::msg::OccupancyGrid& grid) const;

  void publishProhibitedPointCloud();

  [[nodiscard]] std::pair<int, int> hitMemoryKey(const Point2 point) const;

  [[nodiscard]] bool rememberedHitsAllowed() const;

  void rememberHitPoints(const std::vector<Point2>& hit_points);

  [[nodiscard]] std::vector<Point2> pathPoints() const;

  void writeSummary(const std::string& prefix, const std::filesystem::path& image_path,
                    const std::filesystem::path& csv_path,
                    const LidarSnapshotStats& stats, const bool image_ok);

  void publishPointCloud(
      const std::vector<Point2>& points, const double z_m,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);

  [[nodiscard]] std::vector<LidarBeamProjection> collectRadarScanProjections() const;

  void publishRadarMarkers();

  std::string output_dir_;
  std::string pointcloud_topic_;
  std::string remembered_pointcloud_topic_;
  std::string prohibited_pointcloud_topic_;
  std::string raw_memory_pointcloud_topic_;
  std::string marker_topic_;
  std::filesystem::path summary_path_;
  std::ofstream summary_stream_;
  sensor_msgs::msg::LaserScan last_scan_;
  nav_msgs::msg::OccupancyGrid last_grid_;
  nav_msgs::msg::Path last_path_;
  Pose2 current_pose_{};
  Point2 px4_local_origin_{};
  Point2 current_velocity_{};
  AttitudeEuler attitude_{};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double horizontal_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double attitude_tilt_rad_{std::numeric_limits<double>::quiet_NaN()};
  double snapshot_period_s_{1.0};
  double view_radius_m_{45.0};
  double max_lidar_range_m_{35.0};
  double range_hit_epsilon_m_{0.05};
  double initial_heading_rad_{0.0};
  double scan_yaw_offset_rad_{0.0};
  double lidar_pose_latency_s_{0.05};
  double lidar_scan_duration_override_s_{0.0};
  double hit_memory_resolution_m_{0.25};
  double min_remember_altitude_m_{0.0};
  double lidar_z_offset_m_{0.0};
  double lidar_mount_roll_rad_{0.0};
  double lidar_mount_pitch_rad_{0.0};
  double lidar_mount_yaw_rad_{0.0};
  double min_projected_lidar_altitude_m_{0.0};
  double max_projected_lidar_altitude_m_{100000.0};
  double current_pointcloud_z_m_{kGroundDebugZ};
  double remembered_pointcloud_z_m_{kGroundDebugZ};
  double prohibited_pointcloud_z_m_{kGroundDebugZ};
  double raw_memory_pointcloud_z_m_{kGroundDebugZ};
  double marker_z_m_{kGroundDebugZ};
  int image_size_px_{900};
  std::size_t beam_csv_stride_{1U};
  std::size_t max_logged_hit_points_{256U};
  std::size_t max_remembered_hit_points_{50000U};
  std::uint64_t max_snapshots_{0U};
  std::uint64_t snapshot_index_{0U};
  std::int64_t last_scan_receive_ns_{0};
  std::int64_t last_scan_stamp_ns_{0};
  std::int64_t last_pose_receive_ns_{0};
  std::int64_t last_heading_receive_ns_{0};
  std::int64_t last_attitude_receive_ns_{0};
  std::int64_t last_projected_pose_receive_ns_{0};
  std::int64_t last_projected_heading_receive_ns_{0};
  std::int64_t last_projected_attitude_receive_ns_{0};
  std::set<std::pair<int, int>> remembered_hit_cells_;
  std::vector<LidarSnapshotCsvRow> last_scan_rows_;
  LidarSnapshotStats last_scan_stats_{};
  std::vector<Point2> last_scan_hit_points_;
  std::vector<Point2> remembered_hit_points_;
  Pose2 last_projected_pose_{};
  Point2 last_projected_velocity_{};
  AttitudeEuler last_projected_attitude_{};
  double last_projected_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double last_projected_horizontal_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_projected_attitude_tilt_rad_{std::numeric_limits<double>::quiet_NaN()};
  double last_projected_projection_yaw_rad_{std::numeric_limits<double>::quiet_NaN()};
  double last_projected_pose_lag_s_{0.0};
  double last_projected_pose_latency_s_{0.0};
  double last_projected_motion_time_offset_s_{0.0};
  double last_projected_motion_shift_m_{0.0};
  double last_projected_scan_duration_s_{0.0};
  double last_projected_scan_time_increment_s_{0.0};
  Point2 last_projected_motion_shift_{};
  bool scan_seen_{false};
  bool last_scan_projection_seen_{false};
  bool grid_seen_{false};
  bool path_seen_{false};
  bool pose_seen_{false};
  bool altitude_valid_{false};
  bool last_projected_altitude_valid_{false};
  bool horizontal_speed_valid_{false};
  bool last_projected_horizontal_speed_valid_{false};
  bool attitude_valid_{false};
  bool last_projected_attitude_valid_{false};
  bool px4_heading_seen_{false};
  bool last_projected_px4_heading_seen_{false};
  bool use_px4_heading_for_scan_{false};
  bool motion_compensate_lidar_pose_{true};
  bool lidar_scan_deskew_{false};
  bool compensate_lidar_attitude_{false};
  bool publish_lidar_radar_markers_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr prohibited_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr memory_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      remembered_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      prohibited_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      raw_memory_pointcloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav
