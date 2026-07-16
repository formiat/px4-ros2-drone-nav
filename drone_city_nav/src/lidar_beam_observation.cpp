#include "drone_city_nav/lidar_beam_observation.hpp"

#include <cmath>
#include <limits>

namespace drone_city_nav {

LidarBeamTimestamp
lidarBeamAcquisitionTimestamp(const LaserScanTiming& timing,
                              const std::size_t beam_index) noexcept {
  if (!timing.first_beam_stamp_valid || timing.first_beam_stamp_ns <= 0) {
    return {};
  }
  if (beam_index == 0U) {
    return LidarBeamTimestamp{timing.first_beam_stamp_ns, true};
  }
  if (!std::isfinite(timing.time_increment_s) || timing.time_increment_s < 0.0) {
    return {};
  }

  constexpr long double kNanosecondsPerSecond = 1.0e9L;
  const long double offset_ns = static_cast<long double>(beam_index) *
                                static_cast<long double>(timing.time_increment_s) *
                                kNanosecondsPerSecond;
  const long double max_offset =
      static_cast<long double>(std::numeric_limits<std::int64_t>::max()) -
      static_cast<long double>(timing.first_beam_stamp_ns);
  if (!std::isfinite(offset_ns) || offset_ns < 0.0L || offset_ns > max_offset) {
    return {};
  }
  const auto rounded_offset_ns = static_cast<std::int64_t>(std::round(offset_ns));
  return LidarBeamTimestamp{timing.first_beam_stamp_ns + rounded_offset_ns, true};
}

LidarBeamObservation
makeLidarBeamObservation(const LaserScanTiming& timing, const std::size_t beam_index,
                         const LidarBeamProjection& projection,
                         const double effective_max_range_m,
                         const LidarProjectionPose& source_pose,
                         const LidarProjectionConfig& projection_config) {
  const LidarBeamTimestamp acquisition_stamp =
      lidarBeamAcquisitionTimestamp(timing, beam_index);
  const bool source_attitude_valid = source_pose.attitude_valid &&
                                     std::isfinite(source_pose.roll_rad) &&
                                     std::isfinite(source_pose.pitch_rad);
  return LidarBeamObservation{
      .beam_index = beam_index,
      .acquisition_stamp_ns = acquisition_stamp.stamp_ns,
      .acquisition_stamp_valid = acquisition_stamp.valid,
      .receive_stamp_ns = timing.receive_stamp_ns,
      .receive_stamp_valid = timing.receive_stamp_valid,
      .projection = projection,
      .measured_range_m = projection.used_range_m,
      .effective_max_range_m = effective_max_range_m,
      .attitude_compensation_required = projection_config.compensate_attitude,
      .source_attitude_valid = source_attitude_valid,
      .source_roll_rad = source_attitude_valid
                             ? source_pose.roll_rad
                             : std::numeric_limits<double>::quiet_NaN(),
      .source_pitch_rad = source_attitude_valid
                              ? source_pose.pitch_rad
                              : std::numeric_limits<double>::quiet_NaN(),
      .source_tilt_rad = source_attitude_valid
                             ? std::hypot(source_pose.roll_rad, source_pose.pitch_rad)
                             : std::numeric_limits<double>::quiet_NaN(),
  };
}

KnownStaticClassificationSnapshot
makeKnownStaticClassificationSnapshot(const bool classifier_applied,
                                      const KnownStaticLidarHitResult& classification) {
  return KnownStaticClassificationSnapshot{
      .classifier_applied = classifier_applied,
      .classification = classification.classification,
      .volume_matched = classification.volume_matched,
      .confident_face_interior = classification.confident_face_interior,
      .part_kind_valid = classification.volume_matched,
      .part_kind = classification.part_kind,
      .structure_id = std::string{classification.structure_id},
      .opening_id = std::string{classification.opening_id},
      .part_id = std::string{classification.part_id},
      .expected_range_m = classification.expected_range_m,
      .range_delta_m = classification.range_delta_m,
  };
}

} // namespace drone_city_nav
