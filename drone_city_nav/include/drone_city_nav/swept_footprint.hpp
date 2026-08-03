#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <span>

namespace drone_city_nav {

struct SweptFootprintConfig {
  double radius_m{0.82};
  std::size_t perimeter_samples{12U};
  double sweep_step_m{0.25};
};

enum class SweptFootprintStatus {
  kValid,
  kOutsideGrid,
  kInvalidEsdf,
  kRawCollision,
};

struct SweptFootprintResult {
  SweptFootprintStatus status{SweptFootprintStatus::kInvalidEsdf};
  Point3 failure_point{};
  double minimum_clearance_m{0.0};

  [[nodiscard]] bool accepted() const noexcept {
    return status == SweptFootprintStatus::kValid;
  }
};

[[nodiscard]] SweptFootprintResult
validateFootprintAt(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                    const Point3& position,
                    const SweptFootprintConfig& config) noexcept;

[[nodiscard]] SweptFootprintResult
validateSweptFootprint(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                       const Point3& first, const Point3& second,
                       const SweptFootprintConfig& config) noexcept;

} // namespace drone_city_nav
