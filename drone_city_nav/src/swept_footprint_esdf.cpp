#include "drone_city_nav/derived_clearance_3d.hpp"

#include <algorithm>
#include <cmath>

#include "swept_footprint_internal.hpp"

namespace drone_city_nav {
namespace {

using swept_footprint_detail::bodyPoint;
using swept_footprint_detail::cross;
using swept_footprint_detail::inflatedSweepFootprint;
using swept_footprint_detail::interpolateSweepAxis;
using swept_footprint_detail::interpolateSweepPosition;
using swept_footprint_detail::makeConservativeSweepCover3D;
using swept_footprint_detail::normalized;

[[nodiscard]] int statusPriority(const EsdfQueryStatus status) noexcept {
  switch (status) {
    case EsdfQueryStatus::kValid:
      return 0;
    case EsdfQueryStatus::kUnknownSpace:
      return 1;
    case EsdfQueryStatus::kOutsideGrid:
      return 2;
    case EsdfQueryStatus::kInvalidDistance:
      return 3;
  }
  return 3;
}

[[nodiscard]] DerivedFootprintClearance3D
makeStatusResult(const EsdfQueryStatus status,
                 const Point3& diagnostic_point) noexcept {
  DerivedFootprintClearance3D result{
      .status = status,
      .diagnostic_point = diagnostic_point,
  };
  switch (status) {
    case EsdfQueryStatus::kValid:
      break;
    case EsdfQueryStatus::kOutsideGrid:
      result.evidence.outside_grid_exposure = true;
      break;
    case EsdfQueryStatus::kUnknownSpace:
      result.evidence.unknown_exposure = true;
      break;
    case EsdfQueryStatus::kInvalidDistance:
      result.evidence.invalid_esdf_exposure = true;
      break;
  }
  return result;
}

[[nodiscard]] DerivedFootprintClearance3D
makeKnownClearanceResult(const double clearance_m) noexcept {
  DerivedFootprintClearance3D result{.status = EsdfQueryStatus::kValid};
  result.evidence.known_clearance_observed = true;
  result.evidence.minimum_known_clearance_m = clearance_m;
  return result;
}

void mergeEvidence(DerivedFootprintClearance3D& target,
                   const DerivedFootprintClearance3D& source) noexcept {
  target.evidence.outside_grid_exposure =
      target.evidence.outside_grid_exposure || source.evidence.outside_grid_exposure;
  target.evidence.unknown_exposure =
      target.evidence.unknown_exposure || source.evidence.unknown_exposure;
  target.evidence.invalid_esdf_exposure =
      target.evidence.invalid_esdf_exposure || source.evidence.invalid_esdf_exposure;
  if (source.evidence.known_clearance_observed) {
    target.evidence.known_clearance_observed = true;
    target.evidence.minimum_known_clearance_m =
        std::min(target.evidence.minimum_known_clearance_m,
                 source.evidence.minimum_known_clearance_m);
  }
  if (statusPriority(source.status) > statusPriority(target.status)) {
    target.status = source.status;
    target.diagnostic_point = source.diagnostic_point;
  }
}

void subtractKnownClearance(DerivedFootprintClearance3D& result,
                            const double extent_m) noexcept {
  if (result.evidence.known_clearance_observed) {
    result.evidence.minimum_known_clearance_m =
        std::max(0.0, result.evidence.minimum_known_clearance_m - extent_m);
  }
}

[[nodiscard]] DerivedFootprintClearance3D
queryPoint(const EsdfGrid3D& grid, const std::span<const float> esdf_m,
           const Point3& query_point) noexcept {
  const EsdfQueryResult query = queryConservativeEsdf3D(
      grid, esdf_m, static_cast<float>(query_point.x),
      static_cast<float>(query_point.y), static_cast<float>(query_point.z));
  if (query.status == EsdfQueryStatus::kOutsideGrid) {
    return makeStatusResult(EsdfQueryStatus::kOutsideGrid, query_point);
  }
  if (query.status == EsdfQueryStatus::kUnknownSpace) {
    DerivedFootprintClearance3D result =
        makeStatusResult(EsdfQueryStatus::kUnknownSpace, query_point);
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
    return makeStatusResult(EsdfQueryStatus::kInvalidDistance, query_point);
  }
  return makeKnownClearanceResult(query.clearance_m);
}

[[nodiscard]] DerivedFootprintClearance3D samplePlanarCircleClearanceCells(
    const EsdfGrid3D& grid, const std::span<const float> esdf_m, const Point3& position,
    const double radius_m, DerivedFootprintClearance3D result) noexcept {
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
    DerivedFootprintClearance3D boundary =
        makeStatusResult(grid.outside_is_unknown ? EsdfQueryStatus::kUnknownSpace
                                                 : EsdfQueryStatus::kOutsideGrid,
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
  for (int cell_y = minimum_cell_y; cell_y <= maximum_cell_y; ++cell_y) {
    for (int cell_x = minimum_cell_x; cell_x <= maximum_cell_x; ++cell_x) {
      const std::size_t index =
          static_cast<std::size_t>(cell_y) * static_cast<std::size_t>(grid.width) +
          static_cast<std::size_t>(cell_x);
      if (index >= esdf_m.size()) {
        mergeEvidence(result,
                      makeStatusResult(EsdfQueryStatus::kInvalidDistance, position));
        continue;
      }
      const float center_distance_m = esdf_m[index];
      if (center_distance_m == kUnknownEsdfDistanceM) {
        mergeEvidence(result,
                      makeStatusResult(EsdfQueryStatus::kUnknownSpace, position));
        continue;
      }
      if (std::isinf(center_distance_m) && center_distance_m > 0.0F) {
        continue;
      }
      if (!std::isfinite(center_distance_m) || center_distance_m < 0.0F) {
        mergeEvidence(result,
                      makeStatusResult(EsdfQueryStatus::kInvalidDistance, position));
        continue;
      }
      if (center_distance_m == 0.0F) {
        result.evidence.known_clearance_observed = true;
        result.evidence.minimum_known_clearance_m = 0.0;
      }
    }
  }

  subtractKnownClearance(result, radius_m);
  return result;
}

[[nodiscard]] DerivedSweptClearanceProfile3D sampleSweptFootprint(
    const EsdfGrid3D& grid, const std::span<const float> esdf_m, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const double critical_distance_m, const double preferred_distance_m) noexcept {
  const double length_m = distance3D(first, second);
  DerivedSweptClearanceProfile3D profile{
      .clearance = {.status = EsdfQueryStatus::kValid}};
  const auto cover = makeConservativeSweepCover3D(first, first_body_axis, second,
                                                  second_body_axis, config);
  if (!cover.valid()) {
    profile.clearance = makeStatusResult(EsdfQueryStatus::kInvalidDistance, first);
    return profile;
  }
  const double exposure_per_sample =
      length_m / static_cast<double>(cover.interval_count);
  for (std::size_t interval = 0U; interval < cover.interval_count; ++interval) {
    const double ratio = (static_cast<double>(interval) + 0.5) /
                         static_cast<double>(cover.interval_count);
    const FootprintBodyAxis midpoint_axis = interpolateSweepAxis(cover, ratio);
    const SweptFootprintConfig inflated_config =
        inflatedSweepFootprint(config, cover, midpoint_axis);
    const DerivedFootprintClearance3D point =
        queryFootprintClearance3D(grid, esdf_m, interpolateSweepPosition(cover, ratio),
                                  midpoint_axis, inflated_config);
    mergeEvidence(profile.clearance, point);
    if (!point.evidence.known_clearance_observed) {
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

DerivedFootprintClearance3D
queryFootprintClearance3D(const EsdfGrid3D& grid, const std::span<const float> esdf_m,
                          const Point3& position,
                          const SweptFootprintConfig& config) noexcept {
  return queryFootprintClearance3D(grid, esdf_m, position, FootprintBodyAxis{}, config);
}

DerivedFootprintClearance3D
queryFootprintClearance3D(const EsdfGrid3D& grid, const std::span<const float> esdf_m,
                          const Point3& position,
                          const FootprintBodyAxis& requested_body_axis,
                          const SweptFootprintConfig& config) noexcept {
  DerivedFootprintClearance3D result = queryPoint(grid, esdf_m, position);
  const double radius_m = std::max(0.0, config.radius_m);
  if (!(radius_m > 0.0) || config.perimeter_samples == 0U) {
    return result;
  }
  if (grid.depth <= 1) {
    return samplePlanarCircleClearanceCells(grid, esdf_m, position, radius_m, result);
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
  if (result.status == EsdfQueryStatus::kValid &&
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
    const DerivedFootprintClearance3D axis_query = queryPoint(
        grid, esdf_m,
        bodyPoint(position, axis, radial_x, radial_y, axial_offset_m, 0.0, 0.0));
    mergeEvidence(result, axis_query);
    for (std::size_t ring = 1U; ring <= radial_rings; ++ring) {
      const double radial_offset_m =
          radius_m * static_cast<double>(ring) / static_cast<double>(radial_rings);
      for (std::size_t sample = 0U; sample < config.perimeter_samples; ++sample) {
        const double angle = kTwoPi * static_cast<double>(sample) /
                             static_cast<double>(config.perimeter_samples);
        const DerivedFootprintClearance3D query =
            queryPoint(grid, esdf_m,
                       bodyPoint(position, axis, radial_x, radial_y, axial_offset_m,
                                 radial_offset_m, angle));
        mergeEvidence(result, query);
      }
    }
  }
  return result;
}

DerivedFootprintClearance3D querySweptFootprintClearance3D(
    const EsdfGrid3D& grid, const std::span<const float> esdf_m, const Point3& first,
    const Point3& second, const SweptFootprintConfig& config) noexcept {
  return querySweptFootprintClearance3D(grid, esdf_m, first, FootprintBodyAxis{},
                                        second, FootprintBodyAxis{}, config);
}

DerivedFootprintClearance3D querySweptFootprintClearance3D(
    const EsdfGrid3D& grid, const std::span<const float> esdf_m, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis,
    const SweptFootprintConfig& config) noexcept {
  return sampleSweptFootprint(grid, esdf_m, first, first_body_axis, second,
                              second_body_axis, config, 0.0, 0.0)
      .clearance;
}

DerivedSweptClearanceProfile3D profileSweptFootprintClearance3D(
    const EsdfGrid3D& grid, const std::span<const float> esdf_m, const Point3& first,
    const Point3& second, const SweptFootprintConfig& config,
    const double critical_distance_m, const double preferred_distance_m) noexcept {
  return sampleSweptFootprint(grid, esdf_m, first, FootprintBodyAxis{}, second,
                              FootprintBodyAxis{}, config, critical_distance_m,
                              preferred_distance_m);
}

} // namespace drone_city_nav
