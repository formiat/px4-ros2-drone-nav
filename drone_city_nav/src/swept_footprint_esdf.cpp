#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>

#include "swept_footprint_internal.hpp"

namespace drone_city_nav {
namespace {

using swept_footprint_detail::bodyPoint;
using swept_footprint_detail::cross;
using swept_footprint_detail::makeKnownClearanceResult;
using swept_footprint_detail::makeStatusResult;
using swept_footprint_detail::mergeEvidence;
using swept_footprint_detail::normalized;
using swept_footprint_detail::subtractKnownClearance;

[[nodiscard]] SweptFootprintResult queryPoint(const mppi::EsdfGrid& grid,
                                              const std::span<const float> esdf_m,
                                              const Point3& query_point) noexcept {
  const EsdfQueryResult query = queryConservativeEsdf3D(
      grid, esdf_m, static_cast<float>(query_point.x),
      static_cast<float>(query_point.y), static_cast<float>(query_point.z));
  if (query.status == EsdfQueryStatus::kOutsideGrid) {
    return makeStatusResult(SweptFootprintStatus::kOutsideGrid, query_point);
  }
  if (query.status == EsdfQueryStatus::kUnknownSpace) {
    SweptFootprintResult result =
        makeStatusResult(SweptFootprintStatus::kUnknownSpace, query_point);
    result.evidence.outside_grid_exposure =
        grid.outside_is_unknown &&
        (query_point.x < static_cast<double>(grid.origin_x_m) ||
         query_point.y < static_cast<double>(grid.origin_y_m) ||
         query_point.z < static_cast<double>(grid.origin_z_m) ||
         query_point.x >= static_cast<double>(grid.origin_x_m) +
                              static_cast<double>(grid.width) * grid.resolution_m ||
         query_point.y >= static_cast<double>(grid.origin_y_m) +
                              static_cast<double>(grid.height) * grid.resolution_m ||
         (grid.depth > 1 &&
          query_point.z >= static_cast<double>(grid.origin_z_m) +
                               static_cast<double>(grid.depth) * grid.resolution_m));
    return result;
  }
  if (query.status != EsdfQueryStatus::kValid) {
    return makeStatusResult(SweptFootprintStatus::kInvalidEsdf, query_point);
  }
  if (query.raw_occupied) {
    SweptFootprintResult result =
        makeStatusResult(SweptFootprintStatus::kRawCollision, query_point);
    result.evidence.known_clearance_observed = true;
    result.evidence.minimum_known_clearance_m = 0.0;
    return result;
  }
  return makeKnownClearanceResult(query.clearance_m);
}

[[nodiscard]] SweptFootprintResult
validatePlanarCircleRawCells(const mppi::EsdfGrid& grid,
                             const std::span<const float> esdf_m,
                             const Point3& position, const double radius_m,
                             SweptFootprintResult result) noexcept {
  const double minimum_x = position.x - radius_m;
  const double maximum_x = position.x + radius_m;
  const double minimum_y = position.y - radius_m;
  const double maximum_y = position.y + radius_m;
  const double world_maximum_x =
      static_cast<double>(grid.origin_x_m) +
      static_cast<double>(grid.width) * static_cast<double>(grid.resolution_m);
  const double world_maximum_y =
      static_cast<double>(grid.origin_y_m) +
      static_cast<double>(grid.height) * static_cast<double>(grid.resolution_m);
  if (minimum_x < static_cast<double>(grid.origin_x_m) || maximum_x > world_maximum_x ||
      minimum_y < static_cast<double>(grid.origin_y_m) || maximum_y > world_maximum_y) {
    SweptFootprintResult boundary =
        makeStatusResult(grid.outside_is_unknown ? SweptFootprintStatus::kUnknownSpace
                                                 : SweptFootprintStatus::kOutsideGrid,
                         position);
    boundary.evidence.outside_grid_exposure = true;
    mergeEvidence(result, boundary);
  }

  const auto minimumCell = [&](const double coordinate, const double origin) noexcept {
    return static_cast<int>(
        std::floor((coordinate - origin) / static_cast<double>(grid.resolution_m)));
  };
  const auto maximumCell = [&](const double coordinate, const double origin) noexcept {
    return static_cast<int>(std::ceil((coordinate - origin) /
                                      static_cast<double>(grid.resolution_m))) -
           1;
  };
  const int minimum_cell_x = std::max(0, minimumCell(minimum_x, grid.origin_x_m));
  const int maximum_cell_x =
      std::min(grid.width - 1, maximumCell(maximum_x, grid.origin_x_m));
  const int minimum_cell_y = std::max(0, minimumCell(minimum_y, grid.origin_y_m));
  const int maximum_cell_y =
      std::min(grid.height - 1, maximumCell(maximum_y, grid.origin_y_m));
  const double radius_squared = radius_m * radius_m;

  for (int cell_y = minimum_cell_y; cell_y <= maximum_cell_y; ++cell_y) {
    for (int cell_x = minimum_cell_x; cell_x <= maximum_cell_x; ++cell_x) {
      const std::size_t index =
          static_cast<std::size_t>(cell_y) * static_cast<std::size_t>(grid.width) +
          static_cast<std::size_t>(cell_x);
      if (index >= esdf_m.size()) {
        mergeEvidence(result,
                      makeStatusResult(SweptFootprintStatus::kInvalidEsdf, position));
        continue;
      }
      const float center_distance_m = esdf_m[index];
      if (center_distance_m == mppi::kUnknownEsdfDistanceM) {
        mergeEvidence(result,
                      makeStatusResult(SweptFootprintStatus::kUnknownSpace, position));
        continue;
      }
      if (std::isinf(center_distance_m) && center_distance_m > 0.0F) {
        continue;
      }
      if (!std::isfinite(center_distance_m) || center_distance_m < 0.0F) {
        mergeEvidence(result,
                      makeStatusResult(SweptFootprintStatus::kInvalidEsdf, position));
        continue;
      }
      if (center_distance_m != 0.0F) {
        continue;
      }

      const double cell_minimum_x =
          static_cast<double>(grid.origin_x_m) +
          static_cast<double>(cell_x) * static_cast<double>(grid.resolution_m);
      const double cell_minimum_y =
          static_cast<double>(grid.origin_y_m) +
          static_cast<double>(cell_y) * static_cast<double>(grid.resolution_m);
      const double cell_maximum_x =
          cell_minimum_x + static_cast<double>(grid.resolution_m);
      const double cell_maximum_y =
          cell_minimum_y + static_cast<double>(grid.resolution_m);
      const double nearest_x = std::clamp(position.x, cell_minimum_x, cell_maximum_x);
      const double nearest_y = std::clamp(position.y, cell_minimum_y, cell_maximum_y);
      const double dx = position.x - nearest_x;
      const double dy = position.y - nearest_y;
      if (dx * dx + dy * dy <= radius_squared) {
        mergeEvidence(result,
                      makeStatusResult(SweptFootprintStatus::kRawCollision,
                                       Point3{nearest_x, nearest_y, position.z}));
        return result;
      }
    }
  }

  subtractKnownClearance(result, radius_m);
  return result;
}

[[nodiscard]] SweptFootprintClearanceProfile sampleSweptFootprint(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point3& first, const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const double critical_distance_m, const double preferred_distance_m) noexcept {
  const double length_m = distance3D(first, second);
  const double step_m = std::max(1.0e-3, config.sweep_step_m);
  const std::size_t samples =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length_m / step_m)));
  const double exposure_per_sample = length_m / static_cast<double>(samples);
  SweptFootprintClearanceProfile profile{
      .validation = {.status = SweptFootprintStatus::kValid}};
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
    const Point3 position{std::lerp(first.x, second.x, ratio),
                          std::lerp(first.y, second.y, ratio),
                          std::lerp(first.z, second.z, ratio)};
    const FootprintBodyAxis body_axis = normalized(FootprintBodyAxis{
        std::lerp(first_body_axis.x, second_body_axis.x, ratio),
        std::lerp(first_body_axis.y, second_body_axis.y, ratio),
        std::lerp(first_body_axis.z, second_body_axis.z, ratio),
    });
    const SweptFootprintResult point =
        validateFootprintAt(grid, esdf_m, position, body_axis, config);
    mergeEvidence(profile.validation, point);
    if (point.evidence.raw_collision) {
      return profile;
    }
    if (sample == 0U || !point.evidence.known_clearance_observed) {
      continue;
    }
    if (point.evidence.minimum_known_clearance_m < critical_distance_m) {
      profile.critical_exposure_m += exposure_per_sample;
    } else if (point.evidence.minimum_known_clearance_m < preferred_distance_m) {
      profile.planning_exposure_m += exposure_per_sample;
    }
  }
  return profile;
}

} // namespace

SweptFootprintResult validateFootprintAt(const mppi::EsdfGrid& grid,
                                         const std::span<const float> esdf_m,
                                         const Point3& position,
                                         const SweptFootprintConfig& config) noexcept {
  return validateFootprintAt(grid, esdf_m, position, FootprintBodyAxis{}, config);
}

SweptFootprintResult validateFootprintAt(const mppi::EsdfGrid& grid,
                                         const std::span<const float> esdf_m,
                                         const Point3& position,
                                         const FootprintBodyAxis& requested_body_axis,
                                         const SweptFootprintConfig& config) noexcept {
  SweptFootprintResult result = queryPoint(grid, esdf_m, position);
  if (result.evidence.raw_collision) {
    return result;
  }
  const double radius_m = std::max(0.0, config.radius_m);
  if (!(radius_m > 0.0) || config.perimeter_samples == 0U) {
    return result;
  }
  if (grid.depth <= 1) {
    return validatePlanarCircleRawCells(grid, esdf_m, position, radius_m, result);
  }
  const FootprintBodyAxis axis = normalized(requested_body_axis);
  const FootprintBodyAxis reference = std::abs(axis.z) < 0.9
                                          ? FootprintBodyAxis{0.0, 0.0, 1.0}
                                          : FootprintBodyAxis{1.0, 0.0, 0.0};
  const FootprintBodyAxis radial_x = normalized(cross(axis, reference));
  const FootprintBodyAxis radial_y = normalized(cross(axis, radial_x));
  const double bounding_radius_m =
      std::hypot(radius_m, std::max(std::max(0.0, config.lower_extent_m),
                                    std::max(0.0, config.upper_extent_m)));
  if (result.status == SweptFootprintStatus::kValid &&
      result.evidence.known_clearance_observed &&
      config.safe_clearance_threshold_m > 0.0 &&
      result.evidence.minimum_known_clearance_m - bounding_radius_m >=
          config.safe_clearance_threshold_m) {
    subtractKnownClearance(result, bounding_radius_m);
    return result;
  }
  const std::size_t axial_samples = std::max<std::size_t>(2U, config.axial_samples);
  const std::size_t radial_rings = std::max<std::size_t>(1U, config.radial_rings);
  constexpr double kTwoPi{6.28318530717958647692};
  for (std::size_t axial_sample = 0U; axial_sample < axial_samples; ++axial_sample) {
    const double axial_ratio =
        static_cast<double>(axial_sample) / static_cast<double>(axial_samples - 1U);
    const double axial_offset_m =
        std::lerp(-std::max(0.0, config.lower_extent_m),
                  std::max(0.0, config.upper_extent_m), axial_ratio);
    const SweptFootprintResult axis_query = queryPoint(
        grid, esdf_m,
        bodyPoint(position, axis, radial_x, radial_y, axial_offset_m, 0.0, 0.0));
    mergeEvidence(result, axis_query);
    if (axis_query.evidence.raw_collision) {
      return result;
    }
    for (std::size_t ring = 1U; ring <= radial_rings; ++ring) {
      const double radial_offset_m =
          radius_m * static_cast<double>(ring) / static_cast<double>(radial_rings);
      for (std::size_t sample = 0U; sample < config.perimeter_samples; ++sample) {
        const double angle = kTwoPi * static_cast<double>(sample) /
                             static_cast<double>(config.perimeter_samples);
        const SweptFootprintResult query =
            queryPoint(grid, esdf_m,
                       bodyPoint(position, axis, radial_x, radial_y, axial_offset_m,
                                 radial_offset_m, angle));
        mergeEvidence(result, query);
        if (query.evidence.raw_collision) {
          return result;
        }
      }
    }
  }
  return result;
}

SweptFootprintResult
validateSweptFootprint(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
                       const Point3& first, const Point3& second,
                       const SweptFootprintConfig& config) noexcept {
  return validateSweptFootprint(grid, esdf_m, first, FootprintBodyAxis{}, second,
                                FootprintBodyAxis{}, config);
}

SweptFootprintResult
validateSweptFootprint(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
                       const Point3& first, const FootprintBodyAxis& first_body_axis,
                       const Point3& second, const FootprintBodyAxis& second_body_axis,
                       const SweptFootprintConfig& config) noexcept {
  return sampleSweptFootprint(grid, esdf_m, first, first_body_axis, second,
                              second_body_axis, config, 0.0, 0.0)
      .validation;
}

SweptFootprintClearanceProfile profileSweptFootprintClearance(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point3& first, const Point3& second, const SweptFootprintConfig& config,
    const double critical_distance_m, const double preferred_distance_m) noexcept {
  return sampleSweptFootprint(grid, esdf_m, first, FootprintBodyAxis{}, second,
                              FootprintBodyAxis{}, config, critical_distance_m,
                              preferred_distance_m);
}

} // namespace drone_city_nav
