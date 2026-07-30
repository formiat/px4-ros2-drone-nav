#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"

#include <span>

namespace drone_city_nav {

enum class EsdfQueryStatus {
  kValid,
  kOutsideGrid,
  kInvalidDistance,
};

struct EsdfQueryResult {
  float clearance_m{0.0F};
  EsdfQueryStatus status{EsdfQueryStatus::kOutsideGrid};
};

[[nodiscard]] EsdfQueryResult queryConservativeEsdf(const mppi::EsdfGrid& grid,
                                                    std::span<const float> esdf_m,
                                                    float x_m, float y_m) noexcept;

} // namespace drone_city_nav
