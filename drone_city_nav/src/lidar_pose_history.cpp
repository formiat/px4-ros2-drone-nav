#include "drone_city_nav/lidar_pose_history.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <sstream>

namespace drone_city_nav {
namespace {

constexpr double kPi{std::numbers::pi};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] double normalizedAngle(double angle_rad) noexcept {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

[[nodiscard]] std::array<double, 4>
normalizedQuaternion(const std::array<double, 4>& quaternion) noexcept {
  const double norm =
      std::sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    return {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0};
  }
  return {quaternion[0] / norm, quaternion[1] / norm, quaternion[2] / norm,
          quaternion[3] / norm};
}

[[nodiscard]] bool finiteQuaternion(const std::array<double, 4>& quaternion) noexcept {
  return std::all_of(quaternion.begin(), quaternion.end(),
                     [](const double value) { return std::isfinite(value); });
}

[[nodiscard]] std::array<double, 4> slerp(std::array<double, 4> from,
                                          std::array<double, 4> to,
                                          const double ratio) noexcept {
  double dot = from[0] * to[0] + from[1] * to[1] + from[2] * to[2] + from[3] * to[3];
  if (dot < 0.0) {
    for (double& value : to) {
      value = -value;
    }
    dot = -dot;
  }
  dot = std::clamp(dot, -1.0, 1.0);
  if (dot > 0.9995) {
    const std::array<double, 4> blended{
        from[0] + ratio * (to[0] - from[0]),
        from[1] + ratio * (to[1] - from[1]),
        from[2] + ratio * (to[2] - from[2]),
        from[3] + ratio * (to[3] - from[3]),
    };
    return normalizedQuaternion(blended);
  }
  const double theta = std::acos(dot);
  const double sin_theta = std::sin(theta);
  const double from_weight = std::sin((1.0 - ratio) * theta) / sin_theta;
  const double to_weight = std::sin(ratio * theta) / sin_theta;
  return {from_weight * from[0] + to_weight * to[0],
          from_weight * from[1] + to_weight * to[1],
          from_weight * from[2] + to_weight * to[2],
          from_weight * from[3] + to_weight * to[3]};
}

[[nodiscard]] std::optional<AttitudeEuler>
quaternionEuler(const std::array<double, 4>& quaternion) noexcept {
  const auto normalized = normalizedQuaternion(quaternion);
  if (!finiteQuaternion(normalized)) {
    return std::nullopt;
  }
  const double w = normalized[0];
  const double x = normalized[1];
  const double y = normalized[2];
  const double z = normalized[3];
  const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) {
    return std::nullopt;
  }
  return AttitudeEuler{roll, pitch, yaw};
}

template<typename Sample>
[[nodiscard]] std::int64_t sampleStamp(const Sample& sample,
                                       const LidarPoseTimeBasis time_basis) noexcept {
  return time_basis == LidarPoseTimeBasis::kPx4AcquisitionTime
             ? sample.acquisition_stamp_ns
             : sample.stamp_ns;
}

template<typename Sample>
[[nodiscard]] std::pair<std::size_t, std::size_t>
bracketingSamples(const std::deque<Sample>& samples, const std::int64_t stamp_ns,
                  const LidarPoseTimeBasis time_basis) {
  const auto upper =
      std::lower_bound(samples.begin(), samples.end(), stamp_ns,
                       [time_basis](const Sample& sample, const std::int64_t stamp) {
                         return sampleStamp(sample, time_basis) < stamp;
                       });
  if (upper == samples.begin()) {
    return {0U, 0U};
  }
  if (upper == samples.end()) {
    const std::size_t last = samples.size() - 1U;
    return {last, last};
  }
  const std::size_t upper_index = static_cast<std::size_t>(upper - samples.begin());
  return {upper_index - 1U, upper_index};
}

[[nodiscard]] double
interpolationRatio(const std::int64_t from_stamp_ns, const std::int64_t to_stamp_ns,
                   const std::int64_t requested_stamp_ns) noexcept {
  if (from_stamp_ns == to_stamp_ns) {
    return 0.0;
  }
  return std::clamp(static_cast<double>(requested_stamp_ns - from_stamp_ns) /
                        static_cast<double>(to_stamp_ns - from_stamp_ns),
                    0.0, 1.0);
}

[[nodiscard]] std::int64_t nearestStampError(const std::int64_t requested_stamp_ns,
                                             const std::int64_t from_stamp_ns,
                                             const std::int64_t to_stamp_ns) noexcept {
  if (requested_stamp_ns < from_stamp_ns) {
    return from_stamp_ns - requested_stamp_ns;
  }
  if (requested_stamp_ns > to_stamp_ns) {
    return requested_stamp_ns - to_stamp_ns;
  }
  return 0;
}

template<typename Sample>
[[nodiscard]] LidarPoseTemporalAlignment
temporalAlignment(const Sample& from, const Sample& to,
                  const std::int64_t requested_stamp_ns,
                  const LidarPoseTimeBasis time_basis) noexcept {
  const std::int64_t from_stamp_ns = sampleStamp(from, time_basis);
  const std::int64_t to_stamp_ns = sampleStamp(to, time_basis);
  LidarPoseTemporalMode mode = LidarPoseTemporalMode::kExact;
  std::int64_t signed_extrapolation_ns = 0;
  if (requested_stamp_ns < from_stamp_ns) {
    mode = LidarPoseTemporalMode::kExtrapolatedBefore;
    signed_extrapolation_ns = requested_stamp_ns - from_stamp_ns;
  } else if (requested_stamp_ns > to_stamp_ns) {
    mode = LidarPoseTemporalMode::kExtrapolatedAfter;
    signed_extrapolation_ns = requested_stamp_ns - to_stamp_ns;
  } else if (from_stamp_ns != to_stamp_ns) {
    mode = LidarPoseTemporalMode::kInterpolated;
  }
  return LidarPoseTemporalAlignment{
      .mode = mode,
      .requested_stamp_ns = requested_stamp_ns,
      .from_receive_stamp_ns = from.stamp_ns,
      .to_receive_stamp_ns = to.stamp_ns,
      .from_acquisition_stamp_ns = from.acquisition_stamp_ns,
      .to_acquisition_stamp_ns = to.acquisition_stamp_ns,
      .from_source_stamp_ns = from.source_stamp_ns,
      .to_source_stamp_ns = to.source_stamp_ns,
      .signed_extrapolation_ns = signed_extrapolation_ns,
      .interpolation_ratio =
          interpolationRatio(from_stamp_ns, to_stamp_ns, requested_stamp_ns),
  };
}

} // namespace

LidarPoseHistory::LidarPoseHistory(LidarPoseHistoryConfig config)
    : config_{config} {
  config_.retention_ns = std::max<std::int64_t>(1, config_.retention_ns);
  config_.max_extrapolation_ns =
      std::clamp<std::int64_t>(config_.max_extrapolation_ns, 0, config_.retention_ns);
}

void LidarPoseHistory::addPosition(const std::int64_t stamp_ns,
                                   const Point3& position_map_m, const double yaw_rad,
                                   const bool yaw_valid,
                                   const std::int64_t acquisition_stamp_ns,
                                   const std::int64_t source_stamp_ns) {
  if (stamp_ns <= 0 || !finitePoint(position_map_m) || !yaw_valid ||
      !std::isfinite(yaw_rad)) {
    return;
  }
  if (!positions_.empty() && stamp_ns < positions_.back().stamp_ns) {
    return;
  }
  if (!positions_.empty() && acquisition_stamp_ns > 0 &&
      positions_.back().acquisition_stamp_ns > 0 &&
      acquisition_stamp_ns < positions_.back().acquisition_stamp_ns) {
    return;
  }
  positions_.push_back(
      PositionSample{stamp_ns, acquisition_stamp_ns,
                     source_stamp_ns > 0 ? source_stamp_ns : acquisition_stamp_ns,
                     position_map_m, normalizedAngle(yaw_rad)});
  prune(stamp_ns);
}

void LidarPoseHistory::addAttitude(const std::int64_t stamp_ns,
                                   const std::array<float, 4>& quaternion,
                                   const std::int64_t acquisition_stamp_ns,
                                   const std::int64_t source_stamp_ns) {
  const std::array<double, 4> converted{
      static_cast<double>(quaternion[0]), static_cast<double>(quaternion[1]),
      static_cast<double>(quaternion[2]), static_cast<double>(quaternion[3])};
  const auto normalized = normalizedQuaternion(converted);
  if (stamp_ns <= 0 || !finiteQuaternion(normalized)) {
    return;
  }
  if (!attitudes_.empty() && stamp_ns < attitudes_.back().stamp_ns) {
    return;
  }
  if (!attitudes_.empty() && acquisition_stamp_ns > 0 &&
      attitudes_.back().acquisition_stamp_ns > 0 &&
      acquisition_stamp_ns < attitudes_.back().acquisition_stamp_ns) {
    return;
  }
  attitudes_.push_back(AttitudeSample{
      stamp_ns, acquisition_stamp_ns,
      source_stamp_ns > 0 ? source_stamp_ns : acquisition_stamp_ns, normalized});
  prune(stamp_ns);
}

std::optional<TimestampAlignedLidarPose>
LidarPoseHistory::sample(const std::int64_t stamp_ns) const noexcept {
  return sampleWithDiagnostics(stamp_ns, LidarPoseTimeBasis::kReceiveTime).aligned_pose;
}

LidarPoseSampleResult LidarPoseHistory::sampleWithDiagnostics(
    const std::int64_t stamp_ns, const LidarPoseTimeBasis time_basis) const noexcept {
  LidarPoseSampleResult result{};
  if (stamp_ns <= 0) {
    result.status = LidarPoseAlignmentStatus::kInvalidBeamStamp;
    return result;
  }
  if (positions_.empty()) {
    result.status = LidarPoseAlignmentStatus::kPositionHistoryEmpty;
    return result;
  }
  if (attitudes_.empty()) {
    result.status = LidarPoseAlignmentStatus::kAttitudeHistoryEmpty;
    return result;
  }
  const auto [position_from_index, position_to_index] =
      bracketingSamples(positions_, stamp_ns, time_basis);
  const auto [attitude_from_index, attitude_to_index] =
      bracketingSamples(attitudes_, stamp_ns, time_basis);
  const PositionSample& position_from = positions_[position_from_index];
  const PositionSample& position_to = positions_[position_to_index];
  const AttitudeSample& attitude_from = attitudes_[attitude_from_index];
  const AttitudeSample& attitude_to = attitudes_[attitude_to_index];
  if (time_basis == LidarPoseTimeBasis::kPx4AcquisitionTime &&
      (position_from.acquisition_stamp_ns <= 0 ||
       position_to.acquisition_stamp_ns <= 0 ||
       attitude_from.acquisition_stamp_ns <= 0 ||
       attitude_to.acquisition_stamp_ns <= 0)) {
    result.status = LidarPoseAlignmentStatus::kInvalidBeamStamp;
    return result;
  }
  result.position_timing =
      temporalAlignment(position_from, position_to, stamp_ns, time_basis);
  result.attitude_timing =
      temporalAlignment(attitude_from, attitude_to, stamp_ns, time_basis);
  const std::int64_t position_from_stamp_ns = sampleStamp(position_from, time_basis);
  const std::int64_t position_to_stamp_ns = sampleStamp(position_to, time_basis);
  const std::int64_t attitude_from_stamp_ns = sampleStamp(attitude_from, time_basis);
  const std::int64_t attitude_to_stamp_ns = sampleStamp(attitude_to, time_basis);
  result.position_stamp_error_ns =
      nearestStampError(stamp_ns, position_from_stamp_ns, position_to_stamp_ns);
  result.attitude_stamp_error_ns =
      nearestStampError(stamp_ns, attitude_from_stamp_ns, attitude_to_stamp_ns);
  if (result.position_stamp_error_ns > config_.max_extrapolation_ns ||
      result.attitude_stamp_error_ns > config_.max_extrapolation_ns) {
    result.status = LidarPoseAlignmentStatus::kExtrapolationExceeded;
    return result;
  }

  const double position_ratio =
      interpolationRatio(position_from_stamp_ns, position_to_stamp_ns, stamp_ns);
  const Point3 position{position_from.position_map_m.x +
                            position_ratio * (position_to.position_map_m.x -
                                              position_from.position_map_m.x),
                        position_from.position_map_m.y +
                            position_ratio * (position_to.position_map_m.y -
                                              position_from.position_map_m.y),
                        position_from.position_map_m.z +
                            position_ratio * (position_to.position_map_m.z -
                                              position_from.position_map_m.z)};
  const double attitude_ratio =
      interpolationRatio(attitude_from_stamp_ns, attitude_to_stamp_ns, stamp_ns);
  const auto attitude_quaternion =
      slerp(attitude_from.quaternion, attitude_to.quaternion, attitude_ratio);
  const std::optional<AttitudeEuler> attitude = quaternionEuler(attitude_quaternion);
  if (!attitude.has_value()) {
    result.status = LidarPoseAlignmentStatus::kAttitudeInvalid;
    return result;
  }
  result.aligned_pose = TimestampAlignedLidarPose{
      .pose =
          LidarProjectionPose{
              .position = Point2{position.x, position.y},
              .altitude_m = position.z,
              .yaw_rad = attitude->yaw_rad,
              .roll_rad = attitude->roll_rad,
              .pitch_rad = attitude->pitch_rad,
              .altitude_valid = true,
              .attitude_valid = true,
              .body_to_ned_quaternion = attitude_quaternion,
              .body_to_ned_quaternion_valid = true,
          },
      .requested_stamp_ns = stamp_ns,
      .position_stamp_error_ns = result.position_stamp_error_ns,
      .attitude_stamp_error_ns = result.attitude_stamp_error_ns,
      .position_interpolated = position_from_index != position_to_index,
      .attitude_interpolated = attitude_from_index != attitude_to_index,
      .position_timing = result.position_timing,
      .attitude_timing = result.attitude_timing,
  };
  result.status = LidarPoseAlignmentStatus::kAligned;
  return result;
}

void LidarPoseHistory::clear() noexcept {
  positions_.clear();
  attitudes_.clear();
}

std::size_t LidarPoseHistory::positionSampleCount() const noexcept {
  return positions_.size();
}

std::size_t LidarPoseHistory::attitudeSampleCount() const noexcept {
  return attitudes_.size();
}

void LidarPoseHistory::prune(const std::int64_t newest_stamp_ns) {
  const std::int64_t oldest_stamp_ns = newest_stamp_ns - config_.retention_ns;
  while (positions_.size() > 1U && positions_[1].stamp_ns < oldest_stamp_ns) {
    positions_.pop_front();
  }
  while (attitudes_.size() > 1U && attitudes_[1].stamp_ns < oldest_stamp_ns) {
    attitudes_.pop_front();
  }
}

std::optional<std::vector<LidarProjectionPose>> timestampAlignedLidarBeamPoses(
    const LidarPoseHistory& history, const LaserScanTiming& timing,
    const std::size_t beam_count, const std::optional<double> fixed_yaw_rad,
    const Px4RosTimeMapper* const time_mapper) {
  LidarBeamPoseAlignmentResult result = timestampAlignedLidarBeamPosesWithDiagnostics(
      history, timing, beam_count, fixed_yaw_rad, time_mapper);
  if (!result.aligned()) {
    return std::nullopt;
  }
  return std::move(result.poses);
}

LidarBeamPoseAlignmentResult timestampAlignedLidarBeamPosesWithDiagnostics(
    const LidarPoseHistory& history, const LaserScanTiming& timing,
    const std::size_t beam_count, const std::optional<double> fixed_yaw_rad,
    const Px4RosTimeMapper* const time_mapper) {
  LidarBeamPoseAlignmentResult result{};
  result.position_sample_count = history.positionSampleCount();
  result.attitude_sample_count = history.attitudeSampleCount();
  if (time_mapper != nullptr) {
    result.time_mapping = time_mapper->diagnostics();
  }
  if (beam_count == 0U) {
    result.status = LidarPoseAlignmentStatus::kEmptyScan;
    return result;
  }
  if (!timing.first_beam_stamp_valid || timing.first_beam_stamp_ns <= 0) {
    result.status = LidarPoseAlignmentStatus::kInvalidScanStamp;
    return result;
  }
  const auto build_poses =
      [&](const LidarPoseTimeBasis time_basis,
          const LidarPoseAlignmentSource source) -> LidarBeamPoseAlignmentResult {
    LidarBeamPoseAlignmentResult attempt = result;
    attempt.source = source;
    attempt.poses.reserve(beam_count);
    for (std::size_t beam_index = 0U; beam_index < beam_count; ++beam_index) {
      const LidarBeamTimestamp beam_stamp =
          lidarBeamAcquisitionTimestamp(timing, beam_index);
      if (!beam_stamp.valid) {
        attempt.status = LidarPoseAlignmentStatus::kInvalidBeamStamp;
        attempt.failed_beam_index = beam_index;
        return attempt;
      }
      std::int64_t requested_stamp_ns = beam_stamp.stamp_ns;
      if (time_basis == LidarPoseTimeBasis::kPx4AcquisitionTime) {
        const auto mapped_stamp =
            time_mapper != nullptr
                ? time_mapper->rosToPx4LocalTimeNs(beam_stamp.stamp_ns)
                : std::nullopt;
        if (!mapped_stamp.has_value()) {
          attempt.status = LidarPoseAlignmentStatus::kInvalidBeamStamp;
          attempt.failed_beam_index = beam_index;
          attempt.poses.clear();
          return attempt;
        }
        requested_stamp_ns = *mapped_stamp;
      }
      attempt.requested_stamp_ns = requested_stamp_ns;
      const LidarPoseSampleResult sample =
          history.sampleWithDiagnostics(requested_stamp_ns, time_basis);
      attempt.position_stamp_error_ns = sample.position_stamp_error_ns;
      attempt.attitude_stamp_error_ns = sample.attitude_stamp_error_ns;
      attempt.position_timing = sample.position_timing;
      attempt.attitude_timing = sample.attitude_timing;
      if (!sample.aligned_pose.has_value()) {
        attempt.status = sample.status;
        attempt.failed_beam_index = beam_index;
        attempt.poses.clear();
        return attempt;
      }
      LidarProjectionPose pose = sample.aligned_pose->pose;
      if (fixed_yaw_rad.has_value()) {
        pose.yaw_rad = *fixed_yaw_rad;
        pose.body_to_ned_quaternion_valid = false;
      }
      attempt.poses.push_back(pose);
    }
    attempt.status = LidarPoseAlignmentStatus::kAligned;
    attempt.failed_beam_index = beam_count;
    return attempt;
  };

  if (time_mapper != nullptr && time_mapper->ready()) {
    LidarBeamPoseAlignmentResult source_attempt =
        build_poses(LidarPoseTimeBasis::kPx4AcquisitionTime,
                    LidarPoseAlignmentSource::kSourceTimestampAligned);
    if (source_attempt.aligned()) {
      const auto map_acquisition_stamps =
          [time_mapper](LidarPoseTemporalAlignment& temporal_alignment) {
            temporal_alignment.from_acquisition_ros_stamp_ns =
                time_mapper
                    ->px4LocalToRosTimeNs(temporal_alignment.from_acquisition_stamp_ns)
                    .value_or(0);
            temporal_alignment.to_acquisition_ros_stamp_ns =
                time_mapper
                    ->px4LocalToRosTimeNs(temporal_alignment.to_acquisition_stamp_ns)
                    .value_or(0);
          };
      map_acquisition_stamps(source_attempt.position_timing);
      map_acquisition_stamps(source_attempt.attitude_timing);
      return source_attempt;
    }
  }
  return build_poses(LidarPoseTimeBasis::kReceiveTime,
                     LidarPoseAlignmentSource::kReceiveTimestampAlignedFallback);
}

const char*
lidarPoseAlignmentStatusName(const LidarPoseAlignmentStatus status) noexcept {
  switch (status) {
    case LidarPoseAlignmentStatus::kAligned:
      return "aligned";
    case LidarPoseAlignmentStatus::kEmptyScan:
      return "empty_scan";
    case LidarPoseAlignmentStatus::kInvalidScanStamp:
      return "invalid_scan_stamp";
    case LidarPoseAlignmentStatus::kInvalidBeamStamp:
      return "invalid_beam_stamp";
    case LidarPoseAlignmentStatus::kPositionHistoryEmpty:
      return "position_history_empty";
    case LidarPoseAlignmentStatus::kAttitudeHistoryEmpty:
      return "attitude_history_empty";
    case LidarPoseAlignmentStatus::kExtrapolationExceeded:
      return "extrapolation_exceeded";
    case LidarPoseAlignmentStatus::kAttitudeInvalid:
      return "attitude_invalid";
  }
  return "unknown";
}

const char* lidarPoseTemporalModeName(const LidarPoseTemporalMode mode) noexcept {
  switch (mode) {
    case LidarPoseTemporalMode::kUnavailable:
      return "unavailable";
    case LidarPoseTemporalMode::kExact:
      return "exact";
    case LidarPoseTemporalMode::kInterpolated:
      return "interpolated";
    case LidarPoseTemporalMode::kExtrapolatedBefore:
      return "extrapolated_before";
    case LidarPoseTemporalMode::kExtrapolatedAfter:
      return "extrapolated_after";
  }
  return "unknown";
}

const char*
lidarPoseAlignmentSourceName(const LidarPoseAlignmentSource source) noexcept {
  switch (source) {
    case LidarPoseAlignmentSource::kUnavailable:
      return "unavailable";
    case LidarPoseAlignmentSource::kSourceTimestampAligned:
      return "source_timestamp_aligned";
    case LidarPoseAlignmentSource::kReceiveTimestampAlignedFallback:
      return "receive_timestamp_aligned_fallback";
  }
  return "unknown";
}

std::int64_t
lidarPoseSourceTimestampNanoseconds(const std::uint64_t timestamp_us) noexcept {
  constexpr std::uint64_t kNanosecondsPerMicrosecond{1000U};
  const std::uint64_t max_timestamp_us =
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) /
      kNanosecondsPerMicrosecond;
  if (timestamp_us == 0U || timestamp_us > max_timestamp_us) {
    return 0;
  }
  return static_cast<std::int64_t>(timestamp_us * kNanosecondsPerMicrosecond);
}

std::string formatLidarPoseAlignmentDiagnostic(
    const char* const prefix, const LidarBeamPoseAlignmentResult& result,
    const LaserScanTiming& timing, const std::int64_t receive_stamp_ns) {
  std::ostringstream stream;
  stream << prefix << ": reason=" << lidarPoseAlignmentStatusName(result.status)
         << " source=" << lidarPoseAlignmentSourceName(result.source)
         << " failed_beam=" << result.failed_beam_index
         << " scan_stamp_ns=" << timing.first_beam_stamp_ns
         << " requested_stamp_ns=" << result.requested_stamp_ns
         << " receive_stamp_ns=" << receive_stamp_ns << " clock_delta_ms="
         << 1.0e-6 * static_cast<double>(receive_stamp_ns - timing.first_beam_stamp_ns)
         << " position_samples=" << result.position_sample_count
         << " attitude_samples=" << result.attitude_sample_count
         << " time_increment_s=" << timing.time_increment_s << " position_error_ms="
         << 1.0e-6 * static_cast<double>(result.position_stamp_error_ns)
         << " attitude_error_ms="
         << 1.0e-6 * static_cast<double>(result.attitude_stamp_error_ns)
         << " position_timing[mode="
         << lidarPoseTemporalModeName(result.position_timing.mode)
         << " receive=" << result.position_timing.from_receive_stamp_ns << ".."
         << result.position_timing.to_receive_stamp_ns
         << " acquisition=" << result.position_timing.from_acquisition_stamp_ns << ".."
         << result.position_timing.to_acquisition_stamp_ns
         << " source=" << result.position_timing.from_source_stamp_ns << ".."
         << result.position_timing.to_source_stamp_ns
         << " acquisition_ros=" << result.position_timing.from_acquisition_ros_stamp_ns
         << ".." << result.position_timing.to_acquisition_ros_stamp_ns
         << " ratio=" << result.position_timing.interpolation_ratio
         << " extrapolation_ms="
         << 1.0e-6 * static_cast<double>(result.position_timing.signed_extrapolation_ns)
         << "] attitude_timing[mode="
         << lidarPoseTemporalModeName(result.attitude_timing.mode)
         << " receive=" << result.attitude_timing.from_receive_stamp_ns << ".."
         << result.attitude_timing.to_receive_stamp_ns
         << " acquisition=" << result.attitude_timing.from_acquisition_stamp_ns << ".."
         << result.attitude_timing.to_acquisition_stamp_ns
         << " source=" << result.attitude_timing.from_source_stamp_ns << ".."
         << result.attitude_timing.to_source_stamp_ns
         << " acquisition_ros=" << result.attitude_timing.from_acquisition_ros_stamp_ns
         << ".." << result.attitude_timing.to_acquisition_ros_stamp_ns
         << " ratio=" << result.attitude_timing.interpolation_ratio
         << " extrapolation_ms="
         << 1.0e-6 * static_cast<double>(result.attitude_timing.signed_extrapolation_ns)
         << "] time_mapping[ready=" << (result.time_mapping.ready ? "true" : "false")
         << " samples=" << result.time_mapping.sample_count
         << " scale=" << result.time_mapping.scale
         << " offset_ms=" << 1.0e-6 * result.time_mapping.offset_ns
         << " max_residual_ms=" << 1.0e-6 * result.time_mapping.max_fit_residual_ns
         << " estimated_offset_ms="
         << 1.0e-6 * static_cast<double>(result.time_mapping.latest_estimated_offset_ns)
         << ']';
  return stream.str();
}

} // namespace drone_city_nav
