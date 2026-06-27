#pragma once

#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <limits>
#include <string>
#include <vector>

namespace drone_city_nav {

struct LidarSnapshotStats {
  std::size_t processed_beams{0U};
  std::size_t accepted_beams{0U};
  std::size_t hit_beams{0U};
  std::size_t altitude_rejected_beams{0U};
  std::size_t invalid_range_beams{0U};
  std::size_t invalid_scan_beams{0U};
  std::size_t projection_rejected_beams{0U};
  std::size_t grid_unknown{0U};
  std::size_t grid_free{0U};
  std::size_t grid_inflated{0U};
  std::size_t grid_occupied{0U};
  double endpoint_altitude_min_m{std::numeric_limits<double>::infinity()};
  double endpoint_altitude_max_m{-std::numeric_limits<double>::infinity()};
  std::vector<Point2> hit_points;
};

struct LidarSnapshotCsvRow {
  std::size_t beam_index{0U};
  double angle_rad{0.0};
  double raw_range_m{std::numeric_limits<double>::quiet_NaN()};
  double used_range_m{0.0};
  bool hit{false};
  double end_x_m{std::numeric_limits<double>::quiet_NaN()};
  double end_y_m{std::numeric_limits<double>::quiet_NaN()};
  double end_altitude_m{std::numeric_limits<double>::quiet_NaN()};
  LidarBeamProjectionStatus status{LidarBeamProjectionStatus::kInvalidScan};
  Point3 lidar_direction{};
  Point3 body_frd_direction{};
  Point3 ned_direction{};
};

struct LidarSnapshotRecord {
  std::string snapshot;
  double time_s{0.0};
  Point2 position{};
  double yaw_rad{0.0};
  double altitude_m{std::numeric_limits<double>::quiet_NaN()};
  double horizontal_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  bool horizontal_speed_valid{false};
  bool attitude_valid{false};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double attitude_yaw_rad{0.0};
  double tilt_rad{std::numeric_limits<double>::quiet_NaN()};
  std::string yaw_source;
  double projection_yaw_rad{0.0};
  bool px4_heading_valid{false};
  double yaw_delta_to_attitude_rad{std::numeric_limits<double>::quiet_NaN()};
  double scan_receive_age_s{std::numeric_limits<double>::quiet_NaN()};
  double scan_stamp_age_s{std::numeric_limits<double>::quiet_NaN()};
  double pose_receive_age_s{std::numeric_limits<double>::quiet_NaN()};
  double heading_receive_age_s{std::numeric_limits<double>::quiet_NaN()};
  double attitude_receive_age_s{std::numeric_limits<double>::quiet_NaN()};
  bool motion_compensation_enabled{false};
  bool scan_deskew_enabled{false};
  double pose_lag_s{0.0};
  double pose_latency_s{0.0};
  double motion_time_offset_s{0.0};
  Point2 motion_shift{};
  double motion_shift_m{0.0};
  double scan_duration_s{0.0};
  double scan_time_increment_s{0.0};
  std::size_t scan_beams{0U};
  double scan_range_min_m{0.0};
  double scan_range_max_m{0.0};
  double scan_angle_min_rad{0.0};
  double scan_angle_max_rad{0.0};
  bool compensate_attitude{false};
  bool use_px4_heading_for_scan{false};
  double initial_heading_rad{0.0};
  double scan_yaw_offset_rad{0.0};
  double lidar_mount_roll_rad{0.0};
  double lidar_mount_pitch_rad{0.0};
  double lidar_mount_yaw_rad{0.0};
  double min_projected_altitude_m{0.0};
  double max_projected_altitude_m{0.0};
  bool grid_seen{false};
  bool path_seen{false};
  std::size_t path_waypoints{0U};
  std::size_t remembered_hits{0U};
  bool image_ok{false};
  std::filesystem::path image_path;
  std::filesystem::path scan_csv_path;
  std::size_t max_logged_hit_points{0U};
  LidarSnapshotStats stats{};
};

[[nodiscard]] const char*
projectionStatusName(LidarBeamProjectionStatus status) noexcept;

[[nodiscard]] bool writeLidarScanCsv(const std::filesystem::path& path,
                                     const std::vector<LidarSnapshotCsvRow>& rows);

void writeLidarSnapshotSummary(std::ostream& stream, const LidarSnapshotRecord& record);

} // namespace drone_city_nav
