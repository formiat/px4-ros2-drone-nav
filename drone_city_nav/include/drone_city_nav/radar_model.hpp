#pragma once

#include "drone_city_nav/intercept_mission.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct RadarDetectionSample {
  std::uint64_t detection_id{0U};
  double range_m{0.0};
  double azimuth_rad{0.0};
  double elevation_rad{0.0};
  double radial_velocity_mps{0.0};
};

[[nodiscard]] std::optional<RadarDetectionSample>
simulateIdealRadarDetection(const TimedVehicleState& radar,
                            const TimedVehicleState& target,
                            std::uint64_t detection_id = 1U) noexcept;

[[nodiscard]] std::optional<Point3>
radarDetectionPositionMap(const TimedVehicleState& radar,
                          const RadarDetectionSample& detection) noexcept;

[[nodiscard]] std::optional<Vec3>
radarDetectionLineOfSightMap(const TimedVehicleState& radar,
                             const RadarDetectionSample& detection) noexcept;

} // namespace drone_city_nav
