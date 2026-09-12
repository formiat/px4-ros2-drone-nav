#pragma once

#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace drone_city_nav {

struct LidarPoseSourceStampConfig {
  std::int64_t maximum_receive_delay_ns{1'000'000'000};
  std::int64_t maximum_future_skew_ns{100'000'000};
};

enum class LidarPoseSourceStampStatus : std::uint8_t {
  kResolved,
  kTimeMapperUnavailable,
  kSourceTimestampInvalid,
  kReceiveTimeMismatch,
};

struct LidarPoseSourceStampResult {
  std::int64_t acquisition_stamp_ns{0};
  std::int64_t mapped_ros_stamp_ns{0};
  std::int64_t receive_delta_ns{0};
  LidarPoseSourceStampStatus status{LidarPoseSourceStampStatus::kTimeMapperUnavailable};

  [[nodiscard]] bool resolved() const noexcept {
    return status == LidarPoseSourceStampStatus::kResolved;
  }
};

[[nodiscard]] LidarPoseSourceStampResult
resolveLidarPoseSourceStamp(const Px4RosTimeMapper& time_mapper,
                            std::uint64_t source_timestamp_us,
                            std::int64_t receive_stamp_ns,
                            const LidarPoseSourceStampConfig& config = {}) noexcept;

[[nodiscard]] const char*
lidarPoseSourceStampStatusName(LidarPoseSourceStampStatus status) noexcept;

// The scan stamp is the acquisition instant. Each pose source is sampled at
// that instant plus its own signed offset, which absorbs the lead or lag of
// that source alone: the PX4 local position estimate describes the vehicle
// about 0.1 s after its stamp (position offset -0.12 s in the urban flights),
// the attitude estimate is in step with the vehicle (offset 0). One offset for
// both sources tilted every return by the attitude the vehicle had 120 ms
// before the scan, which rocked the current cloud and smeared the persistent
// memory on every manoeuvre.
struct LidarAcquisitionPoseConfig {
  bool apply_source_time_offsets{true};
  double position_source_time_offset_s{0.05};
  double attitude_source_time_offset_s{0.0};
  bool require_source_timestamp_alignment{true};
  bool require_bracketed_pose{true};
};

enum class LidarAcquisitionPoseStatus : std::uint8_t {
  kResolved,
  kInvalidSourceTimeOffset,
  kInvalidScanTimestamp,
  kPoseAlignmentFailed,
  kSourceTimestampAlignmentRequired,
  kTemporalBindingUnreliable,
};

struct LidarAcquisitionPoseResult {
  LidarBeamPoseAlignmentResult alignment{};
  std::int64_t position_source_time_offset_ns{0};
  std::int64_t attitude_source_time_offset_ns{0};
  LidarAcquisitionPoseStatus status{LidarAcquisitionPoseStatus::kInvalidScanTimestamp};

  [[nodiscard]] bool resolved() const noexcept {
    return status == LidarAcquisitionPoseStatus::kResolved;
  }
};

[[nodiscard]] LidarAcquisitionPoseResult resolveLidarAcquisitionBeamPoses(
    const LidarPoseHistory& history, const LaserScanTiming& timing,
    std::size_t beam_count, const LidarAcquisitionPoseConfig& config,
    std::optional<double> fixed_yaw_rad = std::nullopt,
    const Px4RosTimeMapper* time_mapper = nullptr) noexcept;

[[nodiscard]] const char*
lidarAcquisitionPoseStatusName(LidarAcquisitionPoseStatus status) noexcept;

[[nodiscard]] std::string formatLidarAcquisitionPoseDiagnostic(
    const char* prefix, const LidarAcquisitionPoseResult& result,
    const LaserScanTiming& timing, std::int64_t receive_stamp_ns);

} // namespace drone_city_nav
