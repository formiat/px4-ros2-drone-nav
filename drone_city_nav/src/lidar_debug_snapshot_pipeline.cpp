#include "drone_city_nav/lidar_debug_snapshot_pipeline.hpp"

#include <cmath>
#include <iomanip>
#include <numbers>
#include <sstream>

namespace drone_city_nav {

[[nodiscard]] std::string lidarSnapshotPrefix(const std::uint64_t snapshot_index) {
  std::ostringstream stream;
  stream << "snapshot_" << std::setw(6) << std::setfill('0') << snapshot_index;
  return stream.str();
}

[[nodiscard]] double ageSecondsOrNan(const std::int64_t stamp_ns,
                                     const std::int64_t now_ns) noexcept {
  if (stamp_ns <= 0 || now_ns <= 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (stamp_ns >= now_ns) {
    return 0.0;
  }
  constexpr double kNanosecondsPerSecond = 1.0e9;
  return static_cast<double>(now_ns - stamp_ns) / kNanosecondsPerSecond;
}

[[nodiscard]] double yawDeltaRad(const double lhs_rad, const double rhs_rad) noexcept {
  if (!std::isfinite(lhs_rad) || !std::isfinite(rhs_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::remainder(lhs_rad - rhs_rad, 2.0 * std::numbers::pi);
}

[[nodiscard]] double lidarScanDurationSeconds(const double scan_time_s,
                                              const double time_increment_s,
                                              const std::size_t beam_count,
                                              const double override_s) noexcept {
  if (override_s > 0.0) {
    return override_s;
  }
  if (std::isfinite(scan_time_s) && scan_time_s > 0.0) {
    return scan_time_s;
  }
  if (std::isfinite(time_increment_s) && time_increment_s > 0.0 && beam_count > 1U) {
    return time_increment_s * static_cast<double>(beam_count - 1U);
  }
  return 0.0;
}

[[nodiscard]] double
lidarScanTimeIncrementSeconds(const double scan_time_s, const double time_increment_s,
                              const std::size_t beam_count) noexcept {
  if (std::isfinite(time_increment_s) && time_increment_s > 0.0) {
    return time_increment_s;
  }
  if (scan_time_s > 0.0 && beam_count > 1U) {
    return scan_time_s / static_cast<double>(beam_count - 1U);
  }
  return 0.0;
}

} // namespace drone_city_nav
