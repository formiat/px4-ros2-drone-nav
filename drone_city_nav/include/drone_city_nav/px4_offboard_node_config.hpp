#pragma once

#include "drone_city_nav/velocity_control_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace drone_city_nav {

struct Px4OffboardNodeConfig {
  double cruise_altitude_m{12.0};
  double min_navigation_altitude_m{0.0};
  double takeoff_hover_s{2.0};
  double acceptance_radius_m{1.5};
  double turn_preview_distance_m{32.0};
  double command_resend_period_s{2.0};
  std::int64_t max_pose_staleness_ns{1'000'000'000};
  VelocityFollowerConfig velocity_follower{};
  std::string flight_blackbox_path{"log/offboard_blackbox.jsonl"};
};

[[nodiscard]] double boundedFiniteDouble(double value, double fallback,
                                         double min_value, double max_value) noexcept;

[[nodiscard]] std::uint8_t boundedUint8(std::int64_t value);

[[nodiscard]] std::uint16_t boundedUint16(std::int64_t value);

void sanitizePx4OffboardNodeConfig(Px4OffboardNodeConfig& config);

} // namespace drone_city_nav
