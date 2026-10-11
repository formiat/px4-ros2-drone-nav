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

} // namespace drone_city_nav
