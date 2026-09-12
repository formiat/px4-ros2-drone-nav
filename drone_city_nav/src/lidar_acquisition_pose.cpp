#include "drone_city_nav/lidar_acquisition_pose.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace drone_city_nav {
namespace {

constexpr double kNanosecondsPerSecond{1.0e9};

[[nodiscard]] bool checkedAdd(const std::int64_t value, const std::int64_t offset,
                              std::int64_t& result) noexcept {
  if ((offset > 0 && value > std::numeric_limits<std::int64_t>::max() - offset) ||
      (offset < 0 && value < std::numeric_limits<std::int64_t>::min() - offset)) {
    return false;
  }
  result = value + offset;
  return true;
}

} // namespace

LidarPoseSourceStampResult
resolveLidarPoseSourceStamp(const Px4RosTimeMapper& time_mapper,
                            const std::uint64_t source_timestamp_us,
                            const std::int64_t receive_stamp_ns,
                            const LidarPoseSourceStampConfig& config) noexcept {
  LidarPoseSourceStampResult result{};
  if (!time_mapper.ready()) {
    return result;
  }
  const std::optional<std::int64_t> acquisition_stamp_ns =
      time_mapper.recoverPx4LocalTimeNsClosestToRosTime(source_timestamp_us,
                                                        receive_stamp_ns);
  if (!acquisition_stamp_ns.has_value()) {
    result.status = LidarPoseSourceStampStatus::kSourceTimestampInvalid;
    return result;
  }
  const std::optional<std::int64_t> mapped_ros_stamp_ns =
      time_mapper.px4LocalToRosTimeNs(*acquisition_stamp_ns);
  if (!mapped_ros_stamp_ns.has_value() || receive_stamp_ns <= 0) {
    result.status = LidarPoseSourceStampStatus::kSourceTimestampInvalid;
    return result;
  }
  result.acquisition_stamp_ns = *acquisition_stamp_ns;
  result.mapped_ros_stamp_ns = *mapped_ros_stamp_ns;
  result.receive_delta_ns = receive_stamp_ns - *mapped_ros_stamp_ns;
  const std::int64_t maximum_receive_delay_ns =
      std::max<std::int64_t>(0, config.maximum_receive_delay_ns);
  const std::int64_t maximum_future_skew_ns =
      std::max<std::int64_t>(0, config.maximum_future_skew_ns);
  if (result.receive_delta_ns > maximum_receive_delay_ns ||
      result.receive_delta_ns < -maximum_future_skew_ns) {
    result.status = LidarPoseSourceStampStatus::kReceiveTimeMismatch;
    return result;
  }
  result.status = LidarPoseSourceStampStatus::kResolved;
  return result;
}

const char*
lidarPoseSourceStampStatusName(const LidarPoseSourceStampStatus status) noexcept {
  switch (status) {
    case LidarPoseSourceStampStatus::kResolved:
      return "resolved";
    case LidarPoseSourceStampStatus::kTimeMapperUnavailable:
      return "time_mapper_unavailable";
    case LidarPoseSourceStampStatus::kSourceTimestampInvalid:
      return "source_timestamp_invalid";
    case LidarPoseSourceStampStatus::kReceiveTimeMismatch:
      return "receive_time_mismatch";
  }
  return "unknown";
}

LidarAcquisitionPoseResult resolveLidarAcquisitionBeamPoses(
    const LidarPoseHistory& history, const LaserScanTiming& timing,
    const std::size_t beam_count, const LidarAcquisitionPoseConfig& config,
    const std::optional<double> fixed_yaw_rad,
    const Px4RosTimeMapper* const time_mapper) noexcept {
  LidarAcquisitionPoseResult result{};
  if (!timing.first_beam_stamp_valid || timing.first_beam_stamp_ns <= 0) {
    return result;
  }
  const auto valid_offset = [](const double offset_s) {
    return std::isfinite(offset_s) && std::abs(offset_s) <= 1.0;
  };
  if (!valid_offset(config.position_source_time_offset_s) ||
      !valid_offset(config.attitude_source_time_offset_s)) {
    result.status = LidarAcquisitionPoseStatus::kInvalidSourceTimeOffset;
    return result;
  }
  const auto offset_ns = [&config](const double offset_s) {
    return config.apply_source_time_offsets ? static_cast<std::int64_t>(std::llround(
                                                  offset_s * kNanosecondsPerSecond))
                                            : 0;
  };
  result.position_source_time_offset_ns =
      offset_ns(config.position_source_time_offset_s);
  result.attitude_source_time_offset_ns =
      offset_ns(config.attitude_source_time_offset_s);
  std::int64_t shifted_stamp_ns{0};
  if (!checkedAdd(timing.first_beam_stamp_ns, result.position_source_time_offset_ns,
                  shifted_stamp_ns) ||
      shifted_stamp_ns <= 0 ||
      !checkedAdd(timing.first_beam_stamp_ns, result.attitude_source_time_offset_ns,
                  shifted_stamp_ns) ||
      shifted_stamp_ns <= 0) {
    result.status = LidarAcquisitionPoseStatus::kInvalidScanTimestamp;
    return result;
  }
  result.alignment = timestampAlignedLidarBeamPosesWithDiagnostics(
      history, timing, beam_count, fixed_yaw_rad, time_mapper,
      LidarPoseSourceTimeOffsets{
          .position_ns = result.position_source_time_offset_ns,
          .attitude_ns = result.attitude_source_time_offset_ns,
      });
  if (!result.alignment.aligned()) {
    result.status = LidarAcquisitionPoseStatus::kPoseAlignmentFailed;
    return result;
  }
  if (config.require_source_timestamp_alignment && !result.alignment.sourceAligned()) {
    result.status = LidarAcquisitionPoseStatus::kSourceTimestampAlignmentRequired;
    return result;
  }
  if (config.require_bracketed_pose && !result.alignment.bracketed()) {
    result.status = LidarAcquisitionPoseStatus::kTemporalBindingUnreliable;
    return result;
  }
  result.status = LidarAcquisitionPoseStatus::kResolved;
  return result;
}

const char*
lidarAcquisitionPoseStatusName(const LidarAcquisitionPoseStatus status) noexcept {
  switch (status) {
    case LidarAcquisitionPoseStatus::kResolved:
      return "resolved";
    case LidarAcquisitionPoseStatus::kInvalidSourceTimeOffset:
      return "invalid_source_time_offset";
    case LidarAcquisitionPoseStatus::kInvalidScanTimestamp:
      return "invalid_scan_timestamp";
    case LidarAcquisitionPoseStatus::kPoseAlignmentFailed:
      return "pose_alignment_failed";
    case LidarAcquisitionPoseStatus::kSourceTimestampAlignmentRequired:
      return "source_timestamp_alignment_required";
    case LidarAcquisitionPoseStatus::kTemporalBindingUnreliable:
      return "temporal_binding_unreliable";
  }
  return "unknown";
}

std::string formatLidarAcquisitionPoseDiagnostic(
    const char* const prefix, const LidarAcquisitionPoseResult& result,
    const LaserScanTiming& timing, const std::int64_t receive_stamp_ns) {
  std::ostringstream stream;
  stream << prefix << ": status=" << lidarAcquisitionPoseStatusName(result.status)
         << " position_source_time_offset_ms="
         << 1.0e-6 * static_cast<double>(result.position_source_time_offset_ns)
         << " attitude_source_time_offset_ms="
         << 1.0e-6 * static_cast<double>(result.attitude_source_time_offset_ns) << ' '
         << formatLidarPoseAlignmentDiagnostic("alignment", result.alignment, timing,
                                               receive_stamp_ns);
  return stream.str();
}

} // namespace drone_city_nav
