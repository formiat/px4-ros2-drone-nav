#pragma once

#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"
#include "drone_city_nav/lidar_projection.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace drone_city_nav {

struct LaserScanTiming {
  std::int64_t first_beam_stamp_ns{0};
  bool first_beam_stamp_valid{false};
  double time_increment_s{0.0};
  std::int64_t receive_stamp_ns{0};
  bool receive_stamp_valid{false};
};

struct LidarBeamTimestamp {
  std::int64_t stamp_ns{0};
  bool valid{false};
};

[[nodiscard]] LidarBeamTimestamp
lidarBeamAcquisitionTimestamp(const LaserScanTiming& timing,
                              std::size_t beam_index) noexcept;

struct KnownStaticClassificationSnapshot {
  bool classifier_applied{false};
  KnownStaticLidarHitClassification classification{
      KnownStaticLidarHitClassification::kAmbiguous};
  bool volume_matched{false};
  bool confident_face_interior{false};
  bool part_kind_valid{false};
  KnownPassageSolidPartKind part_kind{KnownPassageSolidPartKind::kLeft};
  std::string structure_id;
  std::string opening_id;
  std::string part_id;
  double expected_range_m{std::numeric_limits<double>::quiet_NaN()};
  double range_delta_m{std::numeric_limits<double>::quiet_NaN()};
};

struct LidarBeamObservation {
  std::size_t beam_index{0U};
  std::int64_t scan_stamp_ns{0};
  bool scan_stamp_valid{false};
  std::int64_t acquisition_stamp_ns{0};
  bool acquisition_stamp_valid{false};
  std::int64_t receive_stamp_ns{0};
  bool receive_stamp_valid{false};
  bool timestamp_aligned_pose{false};
  LidarBeamProjection projection{};
  double measured_range_m{std::numeric_limits<double>::quiet_NaN()};
  double effective_max_range_m{std::numeric_limits<double>::quiet_NaN()};
  bool attitude_compensation_required{false};
  bool source_attitude_valid{false};
  double source_roll_rad{std::numeric_limits<double>::quiet_NaN()};
  double source_pitch_rad{std::numeric_limits<double>::quiet_NaN()};
  double source_tilt_rad{std::numeric_limits<double>::quiet_NaN()};
};

[[nodiscard]] LidarBeamObservation
makeLidarBeamObservation(const LaserScanTiming& timing, std::size_t beam_index,
                         const LidarBeamProjection& projection,
                         double effective_max_range_m,
                         const LidarProjectionPose& source_pose,
                         const LidarProjectionConfig& projection_config,
                         bool timestamp_aligned_pose = false);

[[nodiscard]] KnownStaticClassificationSnapshot
makeKnownStaticClassificationSnapshot(bool classifier_applied,
                                      const KnownStaticLidarHitResult& classification);

} // namespace drone_city_nav
