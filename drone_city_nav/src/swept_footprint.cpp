#include "drone_city_nav/swept_footprint.hpp"

#include "drone_city_nav/esdf_query.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] SweptFootprintResult queryPoint(const mppi::EsdfGrid& grid,
                                              const std::span<const float> esdf_m,
                                              const Point3& query_point) noexcept {
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

[[nodiscard]] FootprintBodyAxis normalized(const FootprintBodyAxis& axis) noexcept {
  const double length = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  if (!(length > 1.0e-9) || !std::isfinite(length)) {
    return {};
  }
  return FootprintBodyAxis{axis.x / length, axis.y / length, axis.z / length};
}

[[nodiscard]] FootprintBodyAxis cross(const FootprintBodyAxis& first,
                                      const FootprintBodyAxis& second) noexcept {
  return FootprintBodyAxis{first.y * second.z - first.z * second.y,
                           first.z * second.x - first.x * second.z,
                           first.x * second.y - first.y * second.x};
}

[[nodiscard]] Point3 bodyPoint(const Point3& center, const FootprintBodyAxis& axis,
                               const FootprintBodyAxis& radial_x,
                               const FootprintBodyAxis& radial_y,
                               const double axial_offset_m,
                               const double radial_offset_m,
                               const double angle_rad) noexcept {
  const double cos_angle = std::cos(angle_rad);
  const double sin_angle = std::sin(angle_rad);
  return Point3{
      center.x + axial_offset_m * axis.x +
          radial_offset_m * (cos_angle * radial_x.x + sin_angle * radial_y.x),
      center.y + axial_offset_m * axis.y +
          radial_offset_m * (cos_angle * radial_x.y + sin_angle * radial_y.y),
      center.z + axial_offset_m * axis.z +
          radial_offset_m * (cos_angle * radial_x.z + sin_angle * radial_y.z),
  };
}

[[nodiscard]] double squaredDistanceToBox(const Point3& point, const Point3& minimum,
                                          const Point3& maximum) noexcept {
  const double dx =
      point.x < minimum.x ? minimum.x - point.x : std::max(0.0, point.x - maximum.x);
  const double dy =
      point.y < minimum.y ? minimum.y - point.y : std::max(0.0, point.y - maximum.y);
  const double dz =
      point.z < minimum.z ? minimum.z - point.z : std::max(0.0, point.z - maximum.z);
  return dx * dx + dy * dy + dz * dz;
}

[[nodiscard]] double squaredDistanceSegmentToBox(const Point3& first,
                                                 const Point3& second,
                                                 const Point3& minimum,
                                                 const Point3& maximum) noexcept {
  const auto distance_at = [&](const double ratio) noexcept {
    return squaredDistanceToBox(Point3{std::lerp(first.x, second.x, ratio),
                                       std::lerp(first.y, second.y, ratio),
                                       std::lerp(first.z, second.z, ratio)},
                                minimum, maximum);
  };
  double lower = 0.0;
  double upper = 1.0;
  for (std::size_t iteration = 0U; iteration < 24U; ++iteration) {
    const double first_third = std::lerp(lower, upper, 1.0 / 3.0);
    const double second_third = std::lerp(lower, upper, 2.0 / 3.0);
    if (distance_at(first_third) <= distance_at(second_third)) {
      upper = second_third;
    } else {
      lower = first_third;
    }
  }
  return std::min(
      {distance_at(0.0), distance_at(1.0), distance_at(0.5 * (lower + upper))});
}

[[nodiscard]] int minimumCell(const double coordinate, const double origin,
                              const double resolution) noexcept {
  return static_cast<int>(std::floor((coordinate - origin) / resolution));
}

[[nodiscard]] int maximumCell(const double coordinate, const double origin,
                              const double resolution) noexcept {
  return static_cast<int>(std::ceil((coordinate - origin) / resolution)) - 1;
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
    return {.status = SweptFootprintStatus::kOutsideGrid, .failure_point = position};
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
  const int minimum_cell_x = minimumCell(minimum_x, grid.origin_x_m);
  const int maximum_cell_x = maximumCell(maximum_x, grid.origin_x_m);
  const int minimum_cell_y = minimumCell(minimum_y, grid.origin_y_m);
  const int maximum_cell_y = maximumCell(maximum_y, grid.origin_y_m);
  const double radius_squared = radius_m * radius_m;

  for (int cell_y = minimum_cell_y; cell_y <= maximum_cell_y; ++cell_y) {
    for (int cell_x = minimum_cell_x; cell_x <= maximum_cell_x; ++cell_x) {
      const std::size_t index =
          static_cast<std::size_t>(cell_y) * static_cast<std::size_t>(grid.width) +
          static_cast<std::size_t>(cell_x);
      if (index >= esdf_m.size()) {
        return {.status = SweptFootprintStatus::kInvalidEsdf,
                .failure_point = position};
      }
      const float center_distance_m = esdf_m[index];
      if (std::isinf(center_distance_m) && center_distance_m > 0.0F) {
        continue;
      }
      if (!std::isfinite(center_distance_m) || center_distance_m < 0.0F) {
        return {.status = SweptFootprintStatus::kInvalidEsdf,
                .failure_point = position};
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
        return {.status = SweptFootprintStatus::kRawCollision,
                .failure_point = Point3{nearest_x, nearest_y, position.z},
                .minimum_clearance_m = 0.0};
      }
    }
  }

  result.minimum_clearance_m = std::max(0.0, result.minimum_clearance_m - radius_m);
  return result;
}

void accumulate(SweptFootprintResult& result,
                const SweptFootprintResult& query) noexcept {
  result.minimum_clearance_m =
      std::min(result.minimum_clearance_m, query.minimum_clearance_m);
}

[[nodiscard]] SweptFootprintResult validRawFootprint() noexcept {
  return {.status = SweptFootprintStatus::kValid,
          .minimum_clearance_m = std::numeric_limits<double>::infinity()};
}

template<typename Occupancy>
[[nodiscard]] SweptFootprintResult
validateRawFootprintAt2D(const Occupancy& occupancy, const Point3& position,
                         const SweptFootprintConfig& config) noexcept {
  const double radius_m = std::max(0.0, config.radius_m);
  if (!(radius_m > 0.0)) {
    const std::optional<GridIndex> cell =
        occupancy.worldToCell(Point2{position.x, position.y});
    return cell.has_value() && occupancy.isOccupied(*cell)
               ? SweptFootprintResult{.status = SweptFootprintStatus::kRawCollision,
                                      .failure_point = position}
               : validRawFootprint();
  }
  const GridBounds& bounds = occupancy.bounds();
  const int minimum_x = std::max(
      0, minimumCell(position.x - radius_m, bounds.origin_x, bounds.resolution_m));
  const int maximum_x = std::min(
      bounds.width_cells - 1,
      maximumCell(position.x + radius_m, bounds.origin_x, bounds.resolution_m));
  const int minimum_y = std::max(
      0, minimumCell(position.y - radius_m, bounds.origin_y, bounds.resolution_m));
  const int maximum_y = std::min(
      bounds.height_cells - 1,
      maximumCell(position.y + radius_m, bounds.origin_y, bounds.resolution_m));
  const double radius_squared = radius_m * radius_m;
  for (int y = minimum_y; y <= maximum_y; ++y) {
    for (int x = minimum_x; x <= maximum_x; ++x) {
      const GridIndex cell{x, y};
      if (!occupancy.isOccupied(cell)) {
        continue;
      }
      const double cell_minimum_x = bounds.origin_x + x * bounds.resolution_m;
      const double cell_minimum_y = bounds.origin_y + y * bounds.resolution_m;
      const double nearest_x =
          std::clamp(position.x, cell_minimum_x, cell_minimum_x + bounds.resolution_m);
      const double nearest_y =
          std::clamp(position.y, cell_minimum_y, cell_minimum_y + bounds.resolution_m);
      const double dx = position.x - nearest_x;
      const double dy = position.y - nearest_y;
      if (dx * dx + dy * dy <= radius_squared) {
        return {.status = SweptFootprintStatus::kRawCollision,
                .failure_point = Point3{nearest_x, nearest_y, position.z}};
      }
    }
  }
  return validRawFootprint();
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
  if (!result.accepted()) {
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
    if (!axis_query.accepted()) {
      return axis_query;
    }
    accumulate(result, axis_query);
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
        if (!query.accepted()) {
          return query;
        }
        accumulate(result, query);
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
    const FootprintBodyAxis body_axis = normalized(FootprintBodyAxis{
        std::lerp(first_body_axis.x, second_body_axis.x, ratio),
        std::lerp(first_body_axis.y, second_body_axis.y, ratio),
        std::lerp(first_body_axis.z, second_body_axis.z, ratio),
    });
    const SweptFootprintResult point =
        validateFootprintAt(grid, esdf_m, position, body_axis, config);
    if (!point.accepted()) {
      return point;
    }
    result.minimum_clearance_m =
        std::min(result.minimum_clearance_m, point.minimum_clearance_m);
  }
  return result;
}

SweptFootprintResult
validateRawFootprintAt(const OccupancyGrid2D& occupancy, const Point3& position,
                       const SweptFootprintConfig& config) noexcept {
  return validateRawFootprintAt2D(occupancy, position, config);
}

SweptFootprintResult
validateRawFootprintAt(const RawOccupancyGridView2D& occupancy, const Point3& position,
                       const SweptFootprintConfig& config) noexcept {
  return validateRawFootprintAt2D(occupancy, position, config);
}

SweptFootprintResult
validateRawSweptFootprint(const OccupancyGrid2D& occupancy, const Point3& first,
                          const Point3& second,
                          const SweptFootprintConfig& config) noexcept {
  const double length_m = std::hypot(second.x - first.x, second.y - first.y);
  const double step_m = std::max(1.0e-3, config.sweep_step_m);
  const std::size_t samples =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length_m / step_m)));
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
    const SweptFootprintResult result = validateRawFootprintAt(
        occupancy,
        Point3{std::lerp(first.x, second.x, ratio), std::lerp(first.y, second.y, ratio),
               std::lerp(first.z, second.z, ratio)},
        config);
    if (!result.accepted()) {
      return result;
    }
  }
  return validRawFootprint();
}

SweptFootprintResult
validateRawFootprintAt(const OccupancyGrid3D& occupancy, const Point3& position,
                       const FootprintBodyAxis& requested_body_axis,
                       const SweptFootprintConfig& config) noexcept {
  const double radius_m = std::max(0.0, config.radius_m);
  if (!(radius_m > 0.0)) {
    const std::optional<GridIndex3D> cell = occupancy.worldToCell(position);
    return cell.has_value() && occupancy.isOccupied(*cell)
               ? SweptFootprintResult{.status = SweptFootprintStatus::kRawCollision,
                                      .failure_point = position}
               : validRawFootprint();
  }
  const FootprintBodyAxis axis = normalized(requested_body_axis);
  const Point3 lower{position.x - std::max(0.0, config.lower_extent_m) * axis.x,
                     position.y - std::max(0.0, config.lower_extent_m) * axis.y,
                     position.z - std::max(0.0, config.lower_extent_m) * axis.z};
  const Point3 upper{position.x + std::max(0.0, config.upper_extent_m) * axis.x,
                     position.y + std::max(0.0, config.upper_extent_m) * axis.y,
                     position.z + std::max(0.0, config.upper_extent_m) * axis.z};
  const GridBounds3D& bounds = occupancy.bounds();
  const int minimum_x = std::max(0, minimumCell(std::min(lower.x, upper.x) - radius_m,
                                                bounds.origin_x, bounds.resolution_m));
  const int maximum_x = std::min(bounds.width_cells - 1,
                                 maximumCell(std::max(lower.x, upper.x) + radius_m,
                                             bounds.origin_x, bounds.resolution_m));
  const int minimum_y = std::max(0, minimumCell(std::min(lower.y, upper.y) - radius_m,
                                                bounds.origin_y, bounds.resolution_m));
  const int maximum_y = std::min(bounds.height_cells - 1,
                                 maximumCell(std::max(lower.y, upper.y) + radius_m,
                                             bounds.origin_y, bounds.resolution_m));
  const int minimum_z = std::max(0, minimumCell(std::min(lower.z, upper.z) - radius_m,
                                                bounds.origin_z, bounds.resolution_m));
  const int maximum_z = std::min(bounds.depth_cells - 1,
                                 maximumCell(std::max(lower.z, upper.z) + radius_m,
                                             bounds.origin_z, bounds.resolution_m));
  const double radius_squared = radius_m * radius_m;
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        const GridIndex3D cell{x, y, z};
        if (!occupancy.isOccupied(cell)) {
          continue;
        }
        const Point3 cell_minimum{bounds.origin_x + x * bounds.resolution_m,
                                  bounds.origin_y + y * bounds.resolution_m,
                                  bounds.origin_z + z * bounds.resolution_m};
        const Point3 cell_maximum{cell_minimum.x + bounds.resolution_m,
                                  cell_minimum.y + bounds.resolution_m,
                                  cell_minimum.z + bounds.resolution_m};
        if (squaredDistanceSegmentToBox(lower, upper, cell_minimum, cell_maximum) <=
            radius_squared) {
          return {.status = SweptFootprintStatus::kRawCollision,
                  .failure_point = occupancy.cellCenter(cell)};
        }
      }
    }
  }
  return validRawFootprint();
}

SweptFootprintResult
validateRawSweptFootprint(const OccupancyGrid3D& occupancy, const Point3& first,
                          const FootprintBodyAxis& first_body_axis,
                          const Point3& second,
                          const FootprintBodyAxis& second_body_axis,
                          const SweptFootprintConfig& config) noexcept {
  const double length_m = distance3D(first, second);
  const double step_m = std::max(1.0e-3, config.sweep_step_m);
  const std::size_t samples =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length_m / step_m)));
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
    const SweptFootprintResult result = validateRawFootprintAt(
        occupancy,
        Point3{std::lerp(first.x, second.x, ratio), std::lerp(first.y, second.y, ratio),
               std::lerp(first.z, second.z, ratio)},
        normalized(
            FootprintBodyAxis{std::lerp(first_body_axis.x, second_body_axis.x, ratio),
                              std::lerp(first_body_axis.y, second_body_axis.y, ratio),
                              std::lerp(first_body_axis.z, second_body_axis.z, ratio)}),
        config);
    if (!result.accepted()) {
      return result;
    }
  }
  return validRawFootprint();
}

FootprintBodyAxis bodyAxisFromWorldAcceleration(const Vec3& acceleration_mps2,
                                                const double gravity_mps2) noexcept {
  return normalized(FootprintBodyAxis{acceleration_mps2.x, acceleration_mps2.y,
                                      acceleration_mps2.z + gravity_mps2});
}

} // namespace drone_city_nav
