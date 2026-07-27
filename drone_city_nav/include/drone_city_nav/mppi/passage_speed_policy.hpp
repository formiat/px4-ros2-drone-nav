#pragma once

namespace drone_city_nav::mppi {

struct PassageSpeedPolicy {
  bool use_static_map{true};
  float static_limit_mps{10.0F};
  float no_static_limit_mps{5.0F};
};

[[nodiscard]] constexpr float
activePassageSpeedLimitMps(const PassageSpeedPolicy& policy) noexcept {
  return policy.use_static_map ? policy.static_limit_mps : policy.no_static_limit_mps;
}

} // namespace drone_city_nav::mppi
