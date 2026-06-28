#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace drone_city_nav {

[[nodiscard]] std::string lidarSnapshotPrefix(std::uint64_t snapshot_index);

[[nodiscard]] double ageSecondsOrNan(std::int64_t stamp_ns,
                                     std::int64_t now_ns) noexcept;

[[nodiscard]] double yawDeltaRad(double lhs_rad, double rhs_rad) noexcept;

[[nodiscard]] double lidarScanDurationSeconds(double scan_time_s,
                                              double time_increment_s,
                                              std::size_t beam_count,
                                              double override_s) noexcept;

[[nodiscard]] double lidarScanTimeIncrementSeconds(double scan_time_s,
                                                   double time_increment_s,
                                                   std::size_t beam_count) noexcept;

} // namespace drone_city_nav
