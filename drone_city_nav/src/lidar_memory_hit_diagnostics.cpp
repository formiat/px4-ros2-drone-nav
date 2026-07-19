#include "drone_city_nav/lidar_memory_hit_diagnostics.hpp"

#include "drone_city_nav/known_passage_solid_volumes.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <ostream>
#include <sstream>
#include <string_view>
#include <utility>

namespace drone_city_nav {
namespace {

void writeNumberOrNull(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
  } else {
    stream << "null";
  }
}

void writePoint2(std::ostream& stream, const Point2 point) {
  stream << '{';
  stream << "\"x\":";
  writeNumberOrNull(stream, point.x);
  stream << ",\"y\":";
  writeNumberOrNull(stream, point.y);
  stream << '}';
}

void writePoint3(std::ostream& stream, const Point3& point) {
  stream << '{';
  stream << "\"x\":";
  writeNumberOrNull(stream, point.x);
  stream << ",\"y\":";
  writeNumberOrNull(stream, point.y);
  stream << ",\"z\":";
  writeNumberOrNull(stream, point.z);
  stream << '}';
}

void writeStamp(std::ostream& stream, const char* name, const std::int64_t stamp_ns,
                const bool valid) {
  stream << '"' << name << "\":{\"valid\":" << (valid ? "true" : "false")
         << ",\"ns\":" << stamp_ns << '}';
}

void writeKnownSurface(std::ostream& stream,
                       const std::optional<KnownStaticExpectedSurface>& surface) {
  if (!surface.has_value()) {
    stream << "null";
    return;
  }
  const KnownStaticExpectedSurface& value = *surface;
  stream << "{\"range_m\":";
  writeNumberOrNull(stream, value.range_m);
  stream << ",\"intersection_map_m\":";
  writePoint3(stream, value.intersection_map_m);
  stream << ",\"part\":\"" << knownPassageSolidPartKindName(value.part_kind)
         << "\",\"structure_id\":\"" << value.structure_id << "\",\"opening_id\":\""
         << value.opening_id << "\",\"part_id\":\"" << value.part_id
         << "\",\"volume\":{\"center_xy\":";
  writePoint2(stream, value.volume_center);
  stream << ",\"normal_xy\":";
  writePoint2(stream, value.volume_normal_xy);
  stream << ",\"lateral_xy\":";
  writePoint2(stream, value.volume_lateral_xy);
  stream << ",\"depth_m\":";
  writeNumberOrNull(stream, value.volume_depth_m);
  stream << ",\"width_m\":";
  writeNumberOrNull(stream, value.volume_width_m);
  stream << ",\"min_z_m\":";
  writeNumberOrNull(stream, value.volume_min_z_m);
  stream << ",\"max_z_m\":";
  writeNumberOrNull(stream, value.volume_max_z_m);
  stream << "},\"confident_face_interior\":"
         << (value.confident_face_interior ? "true" : "false") << '}';
}

void writePassageMemoryHitDiagnostic(std::ostream& stream,
                                     const std::uint64_t dump_record_index,
                                     const std::string_view structure_id,
                                     const ObstacleMemoryOccupiedTransition& transition,
                                     const Point3& vehicle_position_map_m,
                                     const Point2& scan_pose_map_m) {
  const MemoryCellProvenance& provenance = transition.provenance;
  const LidarBeamObservation& observation = provenance.occupancy_trigger.beam;
  const KnownStaticClassificationSnapshot& classification =
      provenance.occupancy_trigger.known_static;
  const LidarIngestionDecisionSnapshot& ingestion =
      provenance.occupancy_trigger.ingestion_decision;
  const LidarBeamProjection& projection = observation.projection;
  stream << "PASSAGE_MEMORY_HIT transition=occupied dump_record=" << dump_record_index
         << " structure=" << structure_id << " beam=" << observation.beam_index
         << " cell=(" << provenance.cell.x << ", " << provenance.cell.y
         << ") endpoint=(" << std::fixed << std::setprecision(3)
         << projection.endpoint_map_m.x << ", " << projection.endpoint_map_m.y << ", "
         << projection.endpoint_map_m.z << ") range=" << observation.measured_range_m
         << " score=" << transition.score_before << "->" << transition.score_after
         << " pose=(" << vehicle_position_map_m.x << ", " << vehicle_position_map_m.y
         << ", " << vehicle_position_map_m.z << ") scan_pose=(" << scan_pose_map_m.x
         << ", " << scan_pose_map_m.y << ") source_attitude=(valid="
         << (observation.source_attitude_valid ? "true" : "false")
         << " roll=" << observation.source_roll_rad
         << " pitch=" << observation.source_pitch_rad
         << " tilt=" << observation.source_tilt_rad << ") applied_attitude=(applied="
         << (projection.attitude_compensation_applied ? "true" : "false")
         << " roll=" << projection.applied_roll_rad
         << " pitch=" << projection.applied_pitch_rad
         << " tilt=" << projection.applied_tilt_rad
         << ") acquisition_stamp_ns=" << observation.acquisition_stamp_ns
         << " acquisition_stamp_valid="
         << (observation.acquisition_stamp_valid ? "true" : "false")
         << " receive_stamp_ns=" << observation.receive_stamp_ns
         << " receive_stamp_valid="
         << (observation.receive_stamp_valid ? "true" : "false") << " ray_origin=("
         << projection.ray_origin_map_m.x << ", " << projection.ray_origin_map_m.y
         << ", " << projection.ray_origin_map_m.z << ") ray_dir=("
         << std::setprecision(5) << projection.ray_direction_map.x << ", "
         << projection.ray_direction_map.y << ", " << projection.ray_direction_map.z
         << ") classifier_applied="
         << (classification.classifier_applied ? "true" : "false") << " classification="
         << knownStaticLidarHitClassificationName(classification.classification)
         << " volume_matched=" << (classification.volume_matched ? "true" : "false")
         << " confident_face="
         << (classification.confident_face_interior ? "true" : "false")
         << " known_structure="
         << (classification.structure_id.empty() ? "<none>"
                                                 : classification.structure_id)
         << " opening="
         << (classification.opening_id.empty() ? "<none>" : classification.opening_id)
         << " part="
         << (classification.part_id.empty() ? "<none>" : classification.part_id)
         << " expected_range=" << std::setprecision(3)
         << classification.expected_range_m << " delta=" << classification.range_delta_m
         << " ingestion[action=" << lidarIngestionActionName(ingestion.action)
         << " reason=" << lidarIngestionReasonName(ingestion.reason)
         << " surface=" << lidarExpectedSurfaceKindName(ingestion.expected_surface)
         << " expected_range=" << ingestion.expected_range_m
         << " delta=" << ingestion.range_delta_m << ']';
}

} // namespace

LidarMemoryHitDumpOpenStatus
LidarMemoryHitDumpWriter::open(LidarMemoryHitDumpConfig config) {
  stream_.close();
  stream_.clear();
  config_ = std::move(config);
  records_written_ = 0U;
  limit_reported_ = false;
  if (!config_.enabled) {
    return LidarMemoryHitDumpOpenStatus::kDisabled;
  }

  std::error_code error;
  if (!config_.path.parent_path().empty()) {
    std::filesystem::create_directories(config_.path.parent_path(), error);
  }
  if (error) {
    config_.enabled = false;
    return LidarMemoryHitDumpOpenStatus::kCreateDirectoryFailed;
  }
  stream_.open(config_.path, std::ios::out | std::ios::trunc);
  if (!stream_.is_open()) {
    config_.enabled = false;
    return LidarMemoryHitDumpOpenStatus::kOpenFailed;
  }
  return LidarMemoryHitDumpOpenStatus::kReady;
}

LidarMemoryHitDumpWriteResult
LidarMemoryHitDumpWriter::write(const LidarMemoryHitDiagnosticRecord& record) {
  if (!config_.enabled || !stream_.is_open()) {
    return {};
  }
  if (records_written_ >= config_.max_records) {
    const bool first_limit_reached = !limit_reported_;
    limit_reported_ = true;
    return LidarMemoryHitDumpWriteResult{LidarMemoryHitDumpWriteStatus::kLimitReached,
                                         0U, first_limit_reached};
  }
  const std::uint64_t record_index = records_written_ + 1U;
  LidarMemoryHitDiagnosticRecord numbered_record = record;
  numbered_record.record_index = record_index;
  writeLidarMemoryHitDiagnosticJson(stream_, numbered_record);
  if (!stream_.good()) {
    config_.enabled = false;
    return LidarMemoryHitDumpWriteResult{LidarMemoryHitDumpWriteStatus::kWriteFailed,
                                         0U, false};
  }
  records_written_ = record_index;
  return LidarMemoryHitDumpWriteResult{LidarMemoryHitDumpWriteStatus::kWritten,
                                       record_index, false};
}

const std::filesystem::path& LidarMemoryHitDumpWriter::path() const noexcept {
  return config_.path;
}

std::string formatPassageMemoryHitDiagnostic(
    const std::uint64_t dump_record_index, const std::string_view structure_id,
    const ObstacleMemoryOccupiedTransition& transition,
    const Point3& vehicle_position_map_m, const Point2& scan_pose_map_m) {
  std::ostringstream stream;
  writePassageMemoryHitDiagnostic(stream, dump_record_index, structure_id, transition,
                                  vehicle_position_map_m, scan_pose_map_m);
  return stream.str();
}

bool isRetainedExpectedSurfaceHit(const LidarIngestionDecision& decision) noexcept {
  return decision.action == LidarIngestionAction::kIntegrateFreeAndHit &&
         decision.reason == LidarIngestionReason::kObstacleBeforeExpectedSurface &&
         decision.expected_surface != LidarExpectedSurfaceKind::kNone;
}

void writeLidarMemoryHitDiagnosticJson(std::ostream& stream,
                                       const LidarMemoryHitDiagnosticRecord& record) {
  const LidarBeamObservation& observation =
      record.transition.provenance.occupancy_trigger.beam;
  const LidarBeamProjection& projection = observation.projection;
  const LidarIngestionDecision& decision = record.transition.trigger_decision;
  const LidarMemoryHitDiagnosticContext& context = record.context;
  const double beam_angle_rad =
      context.scan_angle_min_rad +
      static_cast<double>(observation.beam_index) * context.scan_angle_increment_rad;
  const double beam_time_offset_s =
      static_cast<double>(observation.beam_index) * context.scan_time_increment_s;

  stream << std::setprecision(12);
  stream << "{\"record_index\":" << record.record_index
         << ",\"event\":\"occupied_memory_transition\",\"cell\":{\"x\":"
         << record.transition.provenance.cell.x
         << ",\"y\":" << record.transition.provenance.cell.y
         << "},\"score\":{\"before\":" << record.transition.score_before
         << ",\"after\":" << record.transition.score_after
         << "},\"callback_stamp_ns\":" << context.callback_stamp_ns << ",\"scan\":{";
  writeStamp(stream, "acquisition_stamp", observation.acquisition_stamp_ns,
             observation.acquisition_stamp_valid);
  stream << ',';
  writeStamp(stream, "receive_stamp", observation.receive_stamp_ns,
             observation.receive_stamp_valid);
  stream << ",\"beam_index\":" << observation.beam_index << ",\"beam_angle_rad\":";
  writeNumberOrNull(stream, beam_angle_rad);
  stream << ",\"beam_time_offset_s\":";
  writeNumberOrNull(stream, beam_time_offset_s);
  stream << ",\"range_min_m\":";
  writeNumberOrNull(stream, context.scan_range_min_m);
  stream << ",\"range_max_m\":";
  writeNumberOrNull(stream, context.scan_range_max_m);
  stream << ",\"time_increment_s\":";
  writeNumberOrNull(stream, context.scan_time_increment_s);
  stream << ",\"duration_s\":";
  writeNumberOrNull(stream, context.scan_duration_s);
  stream << ",\"measured_range_m\":";
  writeNumberOrNull(stream, observation.measured_range_m);
  stream << ",\"effective_max_range_m\":";
  writeNumberOrNull(stream, observation.effective_max_range_m);
  stream << "},\"pose_used\":{";
  writeStamp(stream, "px4_sample_stamp", context.pose_sample_stamp_ns,
             context.pose_sample_stamp_valid);
  stream << ',';
  writeStamp(stream, "callback_receive_stamp", context.pose_receive_stamp_ns,
             context.pose_receive_stamp_valid);
  stream << ",\"vehicle_position_xy\":";
  writePoint2(stream, context.vehicle_pose.position);
  stream << ",\"vehicle_altitude_m\":";
  writeNumberOrNull(stream, context.vehicle_pose.altitude_m);
  stream << ",\"vehicle_yaw_rad\":";
  writeNumberOrNull(stream, context.vehicle_pose.yaw_rad);
  stream << ",\"altitude_valid\":"
         << (context.vehicle_pose.altitude_valid ? "true" : "false")
         << ",\"attitude_valid\":"
         << (context.vehicle_pose.attitude_valid ? "true" : "false")
         << ",\"horizontal_velocity_xy\":";
  writePoint2(stream, context.horizontal_velocity);
  stream << ",\"horizontal_velocity_valid\":"
         << (context.horizontal_velocity_valid ? "true" : "false") << "},";
  stream << "\"attitude_used\":{";
  writeStamp(stream, "px4_sample_stamp", context.attitude_sample_stamp_ns,
             context.attitude_sample_stamp_valid);
  stream << ',';
  writeStamp(stream, "callback_receive_stamp", context.attitude_receive_stamp_ns,
             context.attitude_receive_stamp_valid);
  stream << ",\"roll_rad\":";
  writeNumberOrNull(stream, observation.source_roll_rad);
  stream << ",\"pitch_rad\":";
  writeNumberOrNull(stream, observation.source_pitch_rad);
  stream << ",\"tilt_rad\":";
  writeNumberOrNull(stream, observation.source_tilt_rad);
  stream << ",\"source_valid\":"
         << (observation.source_attitude_valid ? "true" : "false") << "},";
  stream << "\"motion_compensation\":{\"applied\":"
         << (context.motion_compensation.applied ? "true" : "false")
         << ",\"pose_lag_s\":";
  writeNumberOrNull(stream, context.motion_compensation.pose_lag_s);
  stream << ",\"latency_s\":";
  writeNumberOrNull(stream, context.motion_compensation.latency_s);
  stream << ",\"signed_time_offset_s\":";
  writeNumberOrNull(stream, context.motion_compensation.signed_time_offset_s);
  stream << ",\"applied_shift_xy\":";
  writePoint2(stream, context.motion_compensation.applied_shift);
  stream << ",\"compensated_position_xy\":";
  writePoint2(stream, context.motion_compensation.position);
  stream << "},\"projection\":{\"ray_origin_map_m\":";
  writePoint3(stream, projection.ray_origin_map_m);
  stream << ",\"ray_direction_map\":";
  writePoint3(stream, projection.ray_direction_map);
  stream << ",\"endpoint_map_m\":";
  writePoint3(stream, projection.endpoint_map_m);
  stream << ",\"lidar_direction_flu\":";
  writePoint3(stream, projection.lidar_direction);
  stream << ",\"body_direction_frd\":";
  writePoint3(stream, projection.body_frd_direction);
  stream << ",\"ned_direction\":";
  writePoint3(stream, projection.ned_direction);
  stream << ",\"hit\":" << (projection.hit ? "true" : "false")
         << ",\"endpoint_xyz_valid\":"
         << (projection.endpoint_xyz_valid ? "true" : "false")
         << ",\"attitude_compensation_applied\":"
         << (projection.attitude_compensation_applied ? "true" : "false")
         << ",\"applied_roll_rad\":";
  writeNumberOrNull(stream, projection.applied_roll_rad);
  stream << ",\"applied_pitch_rad\":";
  writeNumberOrNull(stream, projection.applied_pitch_rad);
  stream << ",\"applied_tilt_rad\":";
  writeNumberOrNull(stream, projection.applied_tilt_rad);
  stream << "},\"expected_surfaces\":{\"ground_range_m\":";
  if (decision.expected_ground_range_m.has_value()) {
    writeNumberOrNull(stream, *decision.expected_ground_range_m);
  } else {
    stream << "null";
  }
  stream << ",\"ground_z_m\":";
  writeNumberOrNull(stream, context.ground_config.ground_altitude_m);
  stream << ",\"ground_closer_tolerance_m\":";
  writeNumberOrNull(stream, context.ground_config.closer_range_tolerance_m);
  stream << ",\"ground_farther_tolerance_m\":";
  writeNumberOrNull(stream, context.ground_config.farther_range_tolerance_m);
  stream << ",\"known_static\":";
  writeKnownSurface(stream, decision.known_static_surface);
  stream << ",\"known_static_closer_tolerance_m\":";
  writeNumberOrNull(stream, context.known_static_closer_range_tolerance_m);
  stream << ",\"known_static_farther_tolerance_m\":";
  writeNumberOrNull(stream, context.known_static_farther_range_tolerance_m);
  stream << ",\"known_static_endpoint_volume_tolerance_m\":";
  writeNumberOrNull(stream, context.known_static_endpoint_volume_tolerance_m);
  stream << "},\"decision\":{\"action\":\"" << lidarIngestionActionName(decision.action)
         << "\",\"reason\":\"" << lidarIngestionReasonName(decision.reason)
         << "\",\"selected_surface\":\""
         << lidarExpectedSurfaceKindName(decision.expected_surface)
         << "\",\"selected_expected_range_m\":";
  writeNumberOrNull(stream, decision.expected_range_m);
  stream << ",\"selected_range_delta_m\":";
  writeNumberOrNull(stream, decision.range_delta_m);
  stream << "},\"projection_config\":{\"lidar_z_offset_m\":";
  writeNumberOrNull(stream, context.projection_config.lidar_z_offset_m);
  stream << ",\"scan_yaw_offset_rad\":";
  writeNumberOrNull(stream, context.projection_config.scan_yaw_offset_rad);
  stream << ",\"mount_roll_rad\":";
  writeNumberOrNull(stream, context.projection_config.lidar_mount_roll_rad);
  stream << ",\"mount_pitch_rad\":";
  writeNumberOrNull(stream, context.projection_config.lidar_mount_pitch_rad);
  stream << ",\"mount_yaw_rad\":";
  writeNumberOrNull(stream, context.projection_config.lidar_mount_yaw_rad);
  stream << ",\"compensate_attitude\":"
         << (context.projection_config.compensate_attitude ? "true" : "false")
         << "},\"provenance\":{\"accepted_hit_count\":"
         << record.transition.provenance.accepted_hit_count << ",\"min_endpoint_z_m\":";
  if (record.transition.provenance.min_endpoint_z_m.has_value()) {
    writeNumberOrNull(stream, *record.transition.provenance.min_endpoint_z_m);
  } else {
    stream << "null";
  }
  stream << ",\"max_endpoint_z_m\":";
  if (record.transition.provenance.max_endpoint_z_m.has_value()) {
    writeNumberOrNull(stream, *record.transition.provenance.max_endpoint_z_m);
  } else {
    stream << "null";
  }
  stream << "}}\n";
  stream.flush();
}

std::optional<std::int64_t>
px4TimestampNanoseconds(const std::uint64_t timestamp_us) noexcept {
  if (timestamp_us == 0U ||
      timestamp_us > static_cast<std::uint64_t>(
                         std::numeric_limits<std::int64_t>::max() / 1000LL)) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(timestamp_us) * 1000LL;
}

double lidarScanDurationSeconds(const double scan_time_s, const double time_increment_s,
                                const std::size_t beam_count) noexcept {
  if (std::isfinite(scan_time_s) && scan_time_s > 0.0) {
    return scan_time_s;
  }
  if (std::isfinite(time_increment_s) && time_increment_s > 0.0 && beam_count > 1U) {
    return time_increment_s * static_cast<double>(beam_count - 1U);
  }
  return 0.0;
}

} // namespace drone_city_nav
