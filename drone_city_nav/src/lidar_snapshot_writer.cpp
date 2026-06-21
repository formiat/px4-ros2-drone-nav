#include "drone_city_nav/lidar_snapshot_writer.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string_view>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::string jsonString(const std::string& value) {
  std::ostringstream stream;
  stream << '"';
  for (const char c : value) {
    switch (c) {
      case '\\':
        stream << "\\\\";
        break;
      case '"':
        stream << "\\\"";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        stream << c;
        break;
    }
  }
  stream << '"';
  return stream.str();
}

void writeJsonNumberOrNull(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
    return;
  }
  stream << "null";
}

void writeJsonNumberField(std::ostream& stream, const std::string_view name,
                          const double value) {
  stream << '"' << name << "\":";
  writeJsonNumberOrNull(stream, value);
}

} // namespace

const char* projectionStatusName(const LidarBeamProjectionStatus status) noexcept {
  switch (status) {
    case LidarBeamProjectionStatus::kAccepted:
      return "accepted";
    case LidarBeamProjectionStatus::kInvalidScan:
      return "invalid_scan";
    case LidarBeamProjectionStatus::kInvalidRange:
      return "invalid_range";
    case LidarBeamProjectionStatus::kAltitudeRejected:
      return "altitude_rejected";
  }
  return "unknown";
}

bool writeLidarScanCsv(const std::filesystem::path& path,
                       const std::vector<LidarSnapshotCsvRow>& rows) {
  std::ofstream csv{path, std::ios::out | std::ios::trunc};
  if (!csv.is_open()) {
    return false;
  }

  csv << "beam_index,angle_rad,raw_range_m,used_range_m,hit,end_x_m,end_y_m,"
         "end_altitude_m,status,lidar_dir_x,lidar_dir_y,lidar_dir_z,"
         "body_frd_x,body_frd_y,body_frd_z,"
         "ned_x,ned_y,ned_z\n";
  for (const LidarSnapshotCsvRow& row : rows) {
    csv << row.beam_index << ',' << row.angle_rad << ',' << row.raw_range_m << ','
        << row.used_range_m << ',' << (row.hit ? 1 : 0) << ',' << row.end_x_m << ','
        << row.end_y_m << ',' << row.end_altitude_m << ','
        << projectionStatusName(row.status) << ',' << row.lidar_direction.x << ','
        << row.lidar_direction.y << ',' << row.lidar_direction.z << ','
        << row.body_frd_direction.x << ',' << row.body_frd_direction.y << ','
        << row.body_frd_direction.z << ',' << row.ned_direction.x << ','
        << row.ned_direction.y << ',' << row.ned_direction.z << '\n';
  }
  return csv.good();
}

void writeLidarSnapshotSummary(std::ostream& stream,
                               const LidarSnapshotRecord& record) {
  stream << std::fixed << std::setprecision(4);
  stream << "{\"snapshot\":" << jsonString(record.snapshot) << ',';
  writeJsonNumberField(stream, "time_s", record.time_s);
  stream << ',';
  stream << "\"pose\":{";
  writeJsonNumberField(stream, "x", record.position.x);
  stream << ',';
  writeJsonNumberField(stream, "y", record.position.y);
  stream << ',';
  writeJsonNumberField(stream, "yaw_rad", record.yaw_rad);
  stream << ',';
  writeJsonNumberField(stream, "altitude_m", record.altitude_m);
  stream << "},";
  stream << "\"motion\":{";
  writeJsonNumberField(stream, "horizontal_speed_mps", record.horizontal_speed_mps);
  stream << ",\"horizontal_speed_valid\":"
         << (record.horizontal_speed_valid ? "true" : "false") << "},";
  stream << "\"attitude\":{\"valid\":" << (record.attitude_valid ? "true" : "false")
         << ',';
  writeJsonNumberField(stream, "roll_rad", record.roll_rad);
  stream << ',';
  writeJsonNumberField(stream, "pitch_rad", record.pitch_rad);
  stream << ',';
  writeJsonNumberField(stream, "yaw_rad", record.attitude_yaw_rad);
  stream << ',';
  writeJsonNumberField(stream, "tilt_rad", record.tilt_rad);
  stream << "},";
  stream << "\"projection\":{\"yaw_source\":" << jsonString(record.yaw_source) << ',';
  writeJsonNumberField(stream, "yaw_rad", record.projection_yaw_rad);
  stream << ",\"px4_heading_valid\":" << (record.px4_heading_valid ? "true" : "false")
         << ",\"yaw_delta_to_attitude_rad\":";
  writeJsonNumberOrNull(stream, record.yaw_delta_to_attitude_rad);
  stream << "},";
  stream << "\"timing\":{\"scan_receive_age_s\":";
  writeJsonNumberOrNull(stream, record.scan_receive_age_s);
  stream << ",\"scan_stamp_age_s\":";
  writeJsonNumberOrNull(stream, record.scan_stamp_age_s);
  stream << ",\"pose_receive_age_s\":";
  writeJsonNumberOrNull(stream, record.pose_receive_age_s);
  stream << ",\"heading_receive_age_s\":";
  writeJsonNumberOrNull(stream, record.heading_receive_age_s);
  stream << ",\"attitude_receive_age_s\":";
  writeJsonNumberOrNull(stream, record.attitude_receive_age_s);
  stream << "},";
  stream << "\"scan\":{\"beams\":" << record.scan_beams
         << ",\"processed\":" << record.stats.processed_beams
         << ",\"hits\":" << record.stats.hit_beams << ',';
  writeJsonNumberField(stream, "range_min", record.scan_range_min_m);
  stream << ',';
  writeJsonNumberField(stream, "range_max", record.scan_range_max_m);
  stream << ',';
  writeJsonNumberField(stream, "angle_min", record.scan_angle_min_rad);
  stream << ',';
  writeJsonNumberField(stream, "angle_max", record.scan_angle_max_rad);
  stream << ",\"altitude_rejected\":" << record.stats.altitude_rejected_beams
         << ",\"projection_rejected\":" << record.stats.projection_rejected_beams
         << "},";
  stream << "\"projection_config\":{\"compensate_attitude\":"
         << (record.compensate_attitude ? "true" : "false")
         << ",\"use_px4_heading_for_scan\":"
         << (record.use_px4_heading_for_scan ? "true" : "false") << ',';
  writeJsonNumberField(stream, "initial_heading_rad", record.initial_heading_rad);
  stream << ',';
  writeJsonNumberField(stream, "scan_yaw_offset_rad", record.scan_yaw_offset_rad);
  stream << ',';
  writeJsonNumberField(stream, "lidar_mount_roll_rad", record.lidar_mount_roll_rad);
  stream << ',';
  writeJsonNumberField(stream, "lidar_mount_pitch_rad", record.lidar_mount_pitch_rad);
  stream << ',';
  writeJsonNumberField(stream, "lidar_mount_yaw_rad", record.lidar_mount_yaw_rad);
  stream << ',';
  writeJsonNumberField(stream, "min_projected_altitude_m",
                       record.min_projected_altitude_m);
  stream << ',';
  writeJsonNumberField(stream, "max_projected_altitude_m",
                       record.max_projected_altitude_m);
  stream << "},";
  stream << "\"projection_stats\":{\"accepted\":" << record.stats.accepted_beams
         << ",\"hit\":" << record.stats.hit_beams
         << ",\"altitude_rejected\":" << record.stats.altitude_rejected_beams
         << ",\"invalid_range\":" << record.stats.invalid_range_beams
         << ",\"invalid_scan\":" << record.stats.invalid_scan_beams
         << ",\"endpoint_altitude_min_m\":";
  writeJsonNumberOrNull(stream, record.stats.endpoint_altitude_min_m);
  stream << ",\"endpoint_altitude_max_m\":";
  writeJsonNumberOrNull(stream, record.stats.endpoint_altitude_max_m);
  stream << "},";
  stream << "\"grid\":{\"seen\":" << (record.grid_seen ? "true" : "false")
         << ",\"unknown\":" << record.stats.grid_unknown
         << ",\"free\":" << record.stats.grid_free
         << ",\"inflated\":" << record.stats.grid_inflated
         << ",\"occupied\":" << record.stats.grid_occupied << "},";
  stream << "\"path\":{\"seen\":" << (record.path_seen ? "true" : "false")
         << ",\"waypoints\":" << record.path_waypoints << "},";
  stream << "\"remembered_hits\":" << record.remembered_hits << ',';
  stream << "\"candidate_hits\":" << record.candidate_hits << ',';
  stream << "\"image_ok\":" << (record.image_ok ? "true" : "false") << ',';
  stream << "\"image\":" << jsonString(record.image_path.string()) << ',';
  stream << "\"scan_csv\":" << jsonString(record.scan_csv_path.string()) << ',';
  stream << "\"hit_points\":[";
  const std::size_t logged_hit_count =
      std::min(record.stats.hit_points.size(), record.max_logged_hit_points);
  for (std::size_t i = 0U; i < logged_hit_count; ++i) {
    if (i != 0U) {
      stream << ',';
    }
    stream << '{';
    writeJsonNumberField(stream, "x", record.stats.hit_points[i].x);
    stream << ',';
    writeJsonNumberField(stream, "y", record.stats.hit_points[i].y);
    stream << '}';
  }
  stream << "]}\n";
  stream.flush();
}

} // namespace drone_city_nav
