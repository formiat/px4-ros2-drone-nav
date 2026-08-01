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
  bool raw_occupied{true};
};

[[nodiscard]] EsdfQueryResult queryConservativeEsdf(const mppi::EsdfGrid& grid,
                                                    std::span<const float> esdf_m,
                                                    float x_m, float y_m) noexcept;

[[nodiscard]] EsdfQueryResult queryConservativeEsdf3D(const mppi::EsdfGrid& grid,
                                                      std::span<const float> esdf_m,
                                                      float x_m, float y_m,
                                                      float z_m) noexcept;

} // namespace drone_city_nav
