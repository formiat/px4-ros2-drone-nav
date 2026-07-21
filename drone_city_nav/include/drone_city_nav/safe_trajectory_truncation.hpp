#pragma once

#include "drone_city_nav/trajectory.hpp"

#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

struct SafeTrajectoryTruncationRequest {
  Point2 current_position{};
  double blocker_path_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double truncation_margin_m{10.0};
};

struct SafeTrajectoryTruncationResult {
  bool applied{false};
  bool immediate_hold{false};
  const char* reason{"not_attempted"};
  double current_s_m{std::numeric_limits<double>::quiet_NaN()};
  double blocker_s_m{std::numeric_limits<double>::quiet_NaN()};
  double stop_s_m{std::numeric_limits<double>::quiet_NaN()};
  std::vector<TrajectoryPointSample> samples;
};

struct TruncatedPrefixStitchRequest {
  Point2 current_position{};
  Point2 truncation_point{};
  double max_join_distance_m{1.0};
};

struct TruncatedPrefixStitchResult {
  bool applied{false};
  const char* reason{"not_attempted"};
  double current_s_m{std::numeric_limits<double>::quiet_NaN()};
  double prefix_join_s_m{std::numeric_limits<double>::quiet_NaN()};
  double join_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double suffix_station_offset_m{0.0};
  std::vector<TrajectoryPointSample> samples;
};

// Keeps only the still-safe prefix of an accepted executable trajectory. The
// blocker distance is measured from the current projection along that same
// trajectory; the fixed margin is a policy buffer, not a braking-distance
// calculation. A stop station behind or effectively at the drone becomes an
// immediate hold instead of creating a reverse trajectory.
[[nodiscard]] SafeTrajectoryTruncationResult
truncateTrajectoryBeforeBlocker(std::span<const TrajectoryPointSample> samples,
                                const SafeTrajectoryTruncationRequest& request);

[[nodiscard]] std::uint64_t
trajectoryPrefixFingerprint(std::span<const TrajectoryPointSample> samples) noexcept;

// Retains the executable prefix from the current projection through the
// confirmed truncation point, then appends a suffix that starts at that point.
// Re-stationing and geometry reconstruction remove the temporary terminal
// slowdown from the join; the combined trajectory has only its mission endpoint.
[[nodiscard]] TruncatedPrefixStitchResult
stitchTruncatedPrefixWithSuffix(std::span<const TrajectoryPointSample> prefix,
                                std::span<const TrajectoryPointSample> suffix,
                                const TruncatedPrefixStitchRequest& request);

} // namespace drone_city_nav
