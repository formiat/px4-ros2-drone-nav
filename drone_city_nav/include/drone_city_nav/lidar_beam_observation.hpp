#pragma once

#include <cstddef>
#include <cstdint>

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

} // namespace drone_city_nav
