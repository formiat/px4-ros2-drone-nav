#pragma once

#include <cmath>
#include <cstddef>
#include <limits>

#if defined(__CUDACC__)
#define DRONE_CITY_NAV_TRANSLATIONAL_SPEED_LIMIT_FN __host__ __device__ inline
#else
#define DRONE_CITY_NAV_TRANSLATIONAL_SPEED_LIMIT_FN inline
#endif

namespace drone_city_nav {

// The translational speed the sensor-braking contract admits along a
// direction of motion, read by the direction's vertical share |vz| / |v| at
// kSamples evenly spaced shares from level flight (0) to a pure climb or
// descent (1). Between two samples a lookup takes the smaller neighbour, so
// it never exceeds the contract; at a sample it is the sample. Every entry
// defaults to no bound; the configuration fills it from the contract.
struct TranslationalSpeedLimitByVerticalShare3D {
  static constexpr std::size_t kSamples{33U};
  float limit_mps[kSamples]; // NOLINT(cppcoreguidelines-avoid-c-arrays)

  constexpr TranslationalSpeedLimitByVerticalShare3D() noexcept
      : limit_mps{} {
    for (float& limit : limit_mps) {
      limit = std::numeric_limits<float>::max();
    }
  }
};

[[nodiscard]] DRONE_CITY_NAV_TRANSLATIONAL_SPEED_LIMIT_FN float
translationalSpeedLimitForVerticalShare3D(
    const TranslationalSpeedLimitByVerticalShare3D& table,
    const float vertical_share) noexcept {
  constexpr std::size_t kLast = TranslationalSpeedLimitByVerticalShare3D::kSamples - 1U;
  const float scaled =
      std::fmin(std::fmax(vertical_share, 0.0F), 1.0F) * static_cast<float>(kLast);
  std::size_t lower = static_cast<std::size_t>(scaled);
  if (lower > kLast) {
    lower = kLast;
  }
  const bool between = scaled > static_cast<float>(lower) && lower < kLast;
  const std::size_t upper = between ? lower + 1U : lower;
  return std::fmin(table.limit_mps[lower], table.limit_mps[upper]);
}

// Every entry finite, positive and within the bound over every direction.
[[nodiscard]] inline bool translationalSpeedLimitByVerticalShareValid3D(
    const TranslationalSpeedLimitByVerticalShare3D& table,
    const float bound_mps) noexcept {
  for (const float limit : table.limit_mps) {
    if (!std::isfinite(limit) || !(limit > 0.0F) || limit > bound_mps) {
      return false;
    }
  }
  return true;
}

} // namespace drone_city_nav
