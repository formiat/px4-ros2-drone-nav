#include "drone_city_nav/lidar_pose_history.hpp"

#include <algorithm>
#include <cmath>
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
[[nodiscard]] std::pair<std::size_t, std::size_t>
bracketingSamples(const std::deque<Sample>& samples, const std::int64_t stamp_ns) {
  const auto upper =
      std::lower_bound(samples.begin(), samples.end(), stamp_ns,
                       [](const Sample& sample, const std::int64_t stamp) {
                         return sample.stamp_ns < stamp;
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

} // namespace

LidarPoseHistory::LidarPoseHistory(LidarPoseHistoryConfig config)
    : config_{config} {
  config_.retention_ns = std::max<std::int64_t>(1, config_.retention_ns);
  config_.max_extrapolation_ns =
      std::clamp<std::int64_t>(config_.max_extrapolation_ns, 0, config_.retention_ns);
}

void LidarPoseHistory::addPosition(const std::int64_t stamp_ns,
                                   const Point3& position_map_m, const double yaw_rad,
                                   const bool yaw_valid) {
  if (stamp_ns <= 0 || !finitePoint(position_map_m) || !yaw_valid ||
      !std::isfinite(yaw_rad)) {
    return;
  }
  if (!positions_.empty() && stamp_ns < positions_.back().stamp_ns) {
    return;
  }
  positions_.push_back(
      PositionSample{stamp_ns, position_map_m, normalizedAngle(yaw_rad)});
  prune(stamp_ns);
}

void LidarPoseHistory::addAttitude(const std::int64_t stamp_ns,
                                   const std::array<float, 4>& quaternion) {
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
  attitudes_.push_back(AttitudeSample{stamp_ns, normalized});
  prune(stamp_ns);
}

std::optional<TimestampAlignedLidarPose>
LidarPoseHistory::sample(const std::int64_t stamp_ns) const noexcept {
  return sampleWithDiagnostics(stamp_ns).aligned_pose;
}

LidarPoseSampleResult
LidarPoseHistory::sampleWithDiagnostics(const std::int64_t stamp_ns) const noexcept {
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
      bracketingSamples(positions_, stamp_ns);
  const auto [attitude_from_index, attitude_to_index] =
      bracketingSamples(attitudes_, stamp_ns);
  const PositionSample& position_from = positions_[position_from_index];
  const PositionSample& position_to = positions_[position_to_index];
  const AttitudeSample& attitude_from = attitudes_[attitude_from_index];
  const AttitudeSample& attitude_to = attitudes_[attitude_to_index];
  result.position_stamp_error_ns =
      nearestStampError(stamp_ns, position_from.stamp_ns, position_to.stamp_ns);
  result.attitude_stamp_error_ns =
      nearestStampError(stamp_ns, attitude_from.stamp_ns, attitude_to.stamp_ns);
  if (result.position_stamp_error_ns > config_.max_extrapolation_ns ||
      result.attitude_stamp_error_ns > config_.max_extrapolation_ns) {
    result.status = LidarPoseAlignmentStatus::kExtrapolationExceeded;
    return result;
  }

  const double position_ratio =
      interpolationRatio(position_from.stamp_ns, position_to.stamp_ns, stamp_ns);
  const Point3 position{position_from.position_map_m.x +
                            position_ratio * (position_to.position_map_m.x -
                                              position_from.position_map_m.x),
                        position_from.position_map_m.y +
                            position_ratio * (position_to.position_map_m.y -
                                              position_from.position_map_m.y),
                        position_from.position_map_m.z +
                            position_ratio * (position_to.position_map_m.z -
                                              position_from.position_map_m.z)};
  const double yaw_delta = normalizedAngle(position_to.yaw_rad - position_from.yaw_rad);
  const double yaw =
      normalizedAngle(position_from.yaw_rad + position_ratio * yaw_delta);

  const double attitude_ratio =
      interpolationRatio(attitude_from.stamp_ns, attitude_to.stamp_ns, stamp_ns);
  const auto attitude_quaternion =
      slerp(attitude_from.quaternion, attitude_to.quaternion, attitude_ratio);
  const std::optional<AttitudeEuler> attitude = quaternionEuler(attitude_quaternion);
  if (!attitude.has_value()) {
    result.status = LidarPoseAlignmentStatus::kAttitudeInvalid;
    return result;
  }
  result.aligned_pose = TimestampAlignedLidarPose{
      .pose = LidarProjectionPose{Point2{position.x, position.y}, position.z, yaw,
                                  attitude->roll_rad, attitude->pitch_rad, true, true},
      .requested_stamp_ns = stamp_ns,
      .position_stamp_error_ns = result.position_stamp_error_ns,
      .attitude_stamp_error_ns = result.attitude_stamp_error_ns,
      .position_interpolated = position_from_index != position_to_index,
      .attitude_interpolated = attitude_from_index != attitude_to_index,
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
    const std::size_t beam_count, const std::optional<double> fixed_yaw_rad) {
  LidarBeamPoseAlignmentResult result = timestampAlignedLidarBeamPosesWithDiagnostics(
      history, timing, beam_count, fixed_yaw_rad);
  if (!result.aligned()) {
    return std::nullopt;
  }
  return std::move(result.poses);
}

LidarBeamPoseAlignmentResult timestampAlignedLidarBeamPosesWithDiagnostics(
    const LidarPoseHistory& history, const LaserScanTiming& timing,
    const std::size_t beam_count, const std::optional<double> fixed_yaw_rad) {
  LidarBeamPoseAlignmentResult result{};
  result.position_sample_count = history.positionSampleCount();
  result.attitude_sample_count = history.attitudeSampleCount();
  if (beam_count == 0U) {
    result.status = LidarPoseAlignmentStatus::kEmptyScan;
    return result;
  }
  if (!timing.first_beam_stamp_valid || timing.first_beam_stamp_ns <= 0) {
    result.status = LidarPoseAlignmentStatus::kInvalidScanStamp;
    return result;
  }
  result.poses.reserve(beam_count);
  for (std::size_t beam_index = 0U; beam_index < beam_count; ++beam_index) {
    const LidarBeamTimestamp beam_stamp =
        lidarBeamAcquisitionTimestamp(timing, beam_index);
    if (!beam_stamp.valid) {
      result.status = LidarPoseAlignmentStatus::kInvalidBeamStamp;
      result.failed_beam_index = beam_index;
      return result;
    }
    result.requested_stamp_ns = beam_stamp.stamp_ns;
    const LidarPoseSampleResult sample =
        history.sampleWithDiagnostics(beam_stamp.stamp_ns);
    result.position_stamp_error_ns = sample.position_stamp_error_ns;
    result.attitude_stamp_error_ns = sample.attitude_stamp_error_ns;
    if (!sample.aligned_pose.has_value()) {
      result.status = sample.status;
      result.failed_beam_index = beam_index;
      result.poses.clear();
      return result;
    }
    LidarProjectionPose pose = sample.aligned_pose->pose;
    if (fixed_yaw_rad.has_value()) {
      pose.yaw_rad = *fixed_yaw_rad;
    }
    result.poses.push_back(pose);
  }
  result.status = LidarPoseAlignmentStatus::kAligned;
  result.failed_beam_index = beam_count;
  return result;
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

std::string formatLidarPoseAlignmentDiagnostic(
    const char* const prefix, const LidarBeamPoseAlignmentResult& result,
    const LaserScanTiming& timing, const std::int64_t receive_stamp_ns) {
  std::ostringstream stream;
  stream << prefix << ": reason=" << lidarPoseAlignmentStatusName(result.status)
         << " failed_beam=" << result.failed_beam_index
         << " scan_stamp_ns=" << timing.first_beam_stamp_ns
         << " receive_stamp_ns=" << receive_stamp_ns << " clock_delta_ms="
         << 1.0e-6 * static_cast<double>(receive_stamp_ns - timing.first_beam_stamp_ns)
         << " position_samples=" << result.position_sample_count
         << " attitude_samples=" << result.attitude_sample_count
         << " position_error_ms="
         << 1.0e-6 * static_cast<double>(result.position_stamp_error_ns)
         << " attitude_error_ms="
         << 1.0e-6 * static_cast<double>(result.attitude_stamp_error_ns);
  return stream.str();
}

} // namespace drone_city_nav
