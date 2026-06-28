#include "drone_city_nav/px4_offboard_node_config.hpp"

namespace drone_city_nav {

[[nodiscard]] double boundedFiniteDouble(const double value, const double fallback,
                                         const double min_value,
                                         const double max_value) noexcept {
  if (!std::isfinite(value)) {
    return fallback;
  }
  return std::clamp(value, min_value, max_value);
}

[[nodiscard]] std::uint8_t boundedUint8(const std::int64_t value) {
  return static_cast<std::uint8_t>(std::clamp<std::int64_t>(value, 0, 255));
}

[[nodiscard]] std::uint16_t boundedUint16(const std::int64_t value) {
  return static_cast<std::uint16_t>(std::clamp<std::int64_t>(value, 0, 65535));
}

void sanitizePx4OffboardNodeConfig(Px4OffboardNodeConfig& config) {
  config.min_navigation_altitude_m = std::clamp(config.min_navigation_altitude_m, 0.0,
                                                std::abs(config.cruise_altitude_m));
  config.takeoff_hover_s = std::clamp(config.takeoff_hover_s, 0.0, 30.0);
  config.acceptance_radius_m =
      boundedFiniteDouble(config.acceptance_radius_m, 1.5, 0.0, 100.0);
  config.turn_preview_distance_m =
      boundedFiniteDouble(config.turn_preview_distance_m, 32.0, 0.0, 500.0);
  config.command_resend_period_s =
      boundedFiniteDouble(config.command_resend_period_s, 2.0, 0.05, 60.0);
  config.velocity_follower.min_turn_speed_mps =
      std::clamp(config.velocity_follower.min_turn_speed_mps, 0.0,
                 config.velocity_follower.cruise_speed_mps);
  config.velocity_follower.speed_profile_lookahead_max_m =
      std::max(config.velocity_follower.speed_profile_lookahead_max_m,
               config.velocity_follower.speed_profile_lookahead_min_m);
  config.velocity_follower.final_acceptance_radius_m = config.acceptance_radius_m;
}

} // namespace drone_city_nav
