#pragma once

#include "drone_city_nav/esdf_grid_3d.hpp"

#include <span>

namespace drone_city_nav {

enum class EsdfQueryStatus {
  kValid,
  kOutsideGrid,
  kUnknownSpace,
  kInvalidDistance,
};

struct EsdfQueryResult {
  float clearance_m{0.0F};
  EsdfQueryStatus status{EsdfQueryStatus::kOutsideGrid};
  // The queried point lies inside a raw occupied voxel. Unlike the
  // conservative clearance this is an exact fact, not a bound.
  bool inside_occupied{false};
};

[[nodiscard]] EsdfQueryResult queryConservativeEsdf(const EsdfGrid3D& grid,
                                                    std::span<const float> esdf_m,
                                                    float x_m, float y_m) noexcept;

[[nodiscard]] EsdfQueryResult queryConservativeEsdf3D(const EsdfGrid3D& grid,
                                                      std::span<const float> esdf_m,
                                                      float x_m, float y_m,
                                                      float z_m) noexcept;

} // namespace drone_city_nav
