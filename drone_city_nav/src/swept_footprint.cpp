#include "drone_city_nav/swept_footprint.hpp"

#include "drone_city_nav/esdf_query.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] SweptFootprintResult queryPoint(const mppi::EsdfGrid& grid,
                                              const std::span<const float> esdf_m,
                                              const Point3& center,
                                              const double offset_x,
                                              const double offset_y) noexcept {
  const Point3 query_point{center.x + offset_x, center.y + offset_y, center.z};
  const EsdfQueryResult query = queryConservativeEsdf3D(
      grid, esdf_m, static_cast<float>(query_point.x),
      static_cast<float>(query_point.y), static_cast<float>(query_point.z));
  if (query.status == EsdfQueryStatus::kOutsideGrid) {
    return {.status = SweptFootprintStatus::kOutsideGrid, .failure_point = query_point};
  }
  if (query.status != EsdfQueryStatus::kValid) {
    return {.status = SweptFootprintStatus::kInvalidEsdf, .failure_point = query_point};
  }
  if (query.raw_occupied) {
    return {.status = SweptFootprintStatus::kRawCollision,
            .failure_point = query_point};
  }
  return {.status = SweptFootprintStatus::kValid,
          .minimum_clearance_m = query.clearance_m};
}

} // namespace

SweptFootprintResult validateFootprintAt(const mppi::EsdfGrid& grid,
                                         const std::span<const float> esdf_m,
                                         const Point3& position,
                                         const SweptFootprintConfig& config) noexcept {
  SweptFootprintResult result = queryPoint(grid, esdf_m, position, 0.0, 0.0);
  if (!result.accepted()) {
    return result;
  }
  const double radius_m = std::max(0.0, config.radius_m);
  if (!(radius_m > 0.0) || config.perimeter_samples == 0U) {
    return result;
  }
  constexpr double kTwoPi{6.28318530717958647692};
  for (std::size_t sample = 0U; sample < config.perimeter_samples; ++sample) {
    const double angle = kTwoPi * static_cast<double>(sample) /
                         static_cast<double>(config.perimeter_samples);
    const SweptFootprintResult query = queryPoint(
        grid, esdf_m, position, radius_m * std::cos(angle), radius_m * std::sin(angle));
    if (!query.accepted()) {
      return query;
    }
    result.minimum_clearance_m =
        std::min(result.minimum_clearance_m, query.minimum_clearance_m);
  }
  return result;
}

SweptFootprintResult
validateSweptFootprint(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
                       const Point3& first, const Point3& second,
                       const SweptFootprintConfig& config) noexcept {
  const double length_m = distance3D(first, second);
  const double step_m = std::max(1.0e-3, config.sweep_step_m);
  const std::size_t samples =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length_m / step_m)));
  SweptFootprintResult result{.status = SweptFootprintStatus::kValid,
                              .minimum_clearance_m =
                                  std::numeric_limits<double>::infinity()};
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
    const Point3 position{std::lerp(first.x, second.x, ratio),
                          std::lerp(first.y, second.y, ratio),
                          std::lerp(first.z, second.z, ratio)};
    const SweptFootprintResult point =
        validateFootprintAt(grid, esdf_m, position, config);
    if (!point.accepted()) {
      return point;
    }
    result.minimum_clearance_m =
        std::min(result.minimum_clearance_m, point.minimum_clearance_m);
  }
  return result;
}

} // namespace drone_city_nav
