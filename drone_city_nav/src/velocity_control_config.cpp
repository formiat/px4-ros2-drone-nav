#include "drone_city_nav/velocity_control_config.hpp"

#include <array>
#include <bit>
#include <cstdint>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void mix(std::uint64_t& hash, const std::uint64_t value) noexcept {
  hash ^= value;
  hash *= kFnvPrime;
}

void mixDouble(std::uint64_t& hash, const double value) noexcept {
  mix(hash, std::bit_cast<std::uint64_t>(value));
}

} // namespace

std::uint64_t
velocityControlConfigFingerprint(const VelocityFollowerConfig& config) noexcept {
  std::uint64_t hash = kFnvOffsetBasis;
  const std::array<double, 14U> values{
      config.cruise_speed_mps,
      config.min_turn_speed_mps,
      config.speed_profile_accel_mps2,
      config.speed_profile_decel_mps2,
      config.turn_speed_lateral_accel_mps2,
      config.speed_profile_sample_step_m,
      config.speed_profile_lookahead_time_s,
      config.speed_profile_lookahead_min_m,
      config.speed_profile_lookahead_max_m,
      config.setpoint_forward_accel_mps2,
      config.setpoint_forward_decel_mps2,
      config.setpoint_lateral_response_accel_mps2,
      config.max_velocity_jerk_mps3,
      config.max_lateral_velocity_jerk_mps3,
  };
  for (const double value : values) {
    mixDouble(hash, value);
  }
  return hash;
}

} // namespace drone_city_nav
