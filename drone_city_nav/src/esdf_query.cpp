#include "drone_city_nav/esdf_query.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {

EsdfQueryResult queryConservativeEsdf(const mppi::EsdfGrid& grid,
                                      const std::span<const float> esdf_m,
                                      const float x_m, const float y_m) noexcept {
  if (grid.width <= 0 || grid.height <= 0 || !(grid.resolution_m > 0.0F) ||
      !std::isfinite(x_m) || !std::isfinite(y_m)) {
    return {};
  }
  const float cell_x_float = (x_m - grid.origin_x_m) / grid.resolution_m;
  const float cell_y_float = (y_m - grid.origin_y_m) / grid.resolution_m;
  const int cell_x = static_cast<int>(std::floor(cell_x_float));
  const int cell_y = static_cast<int>(std::floor(cell_y_float));
  if (cell_x < 0 || cell_y < 0 || cell_x >= grid.width || cell_y >= grid.height) {
    return {.clearance_m = std::numeric_limits<float>::infinity(),
            .status = EsdfQueryStatus::kOutsideGrid,
            .raw_occupied = false};
  }
  const std::size_t index =
      static_cast<std::size_t>(cell_y) * static_cast<std::size_t>(grid.width) +
      static_cast<std::size_t>(cell_x);
  if (index >= esdf_m.size()) {
    return {};
  }
  const float center_distance_m = esdf_m[index];
  if (std::isinf(center_distance_m) && center_distance_m > 0.0F) {
    return {.clearance_m = std::numeric_limits<float>::infinity(),
            .status = EsdfQueryStatus::kValid,
            .raw_occupied = false};
  }
  if (!std::isfinite(center_distance_m) || center_distance_m < 0.0F) {
    return {.clearance_m = 0.0F,
            .status = EsdfQueryStatus::kInvalidDistance,
            .raw_occupied = true};
  }

  const float center_x_m =
      grid.origin_x_m + (static_cast<float>(cell_x) + 0.5F) * grid.resolution_m;
  const float center_y_m =
      grid.origin_y_m + (static_cast<float>(cell_y) + 0.5F) * grid.resolution_m;
  const float query_to_center_m = std::hypot(x_m - center_x_m, y_m - center_y_m);
  constexpr float kHalfDiagonalScale{0.70710678118654752440F};
  const float occupied_cell_radius_m = kHalfDiagonalScale * grid.resolution_m;
  return {
      .clearance_m = std::max(0.0F, center_distance_m - query_to_center_m -
                                        occupied_cell_radius_m),
      .status = EsdfQueryStatus::kValid,
      .raw_occupied = center_distance_m == 0.0F,
  };
}

EsdfQueryResult queryConservativeEsdf3D(const mppi::EsdfGrid& grid,
                                        const std::span<const float> esdf_m,
                                        const float x_m, const float y_m,
                                        const float z_m) noexcept {
  if (grid.depth <= 1) {
    return queryConservativeEsdf(grid, esdf_m, x_m, y_m);
  }
  if (grid.width <= 0 || grid.height <= 0 || grid.depth <= 0 ||
      !(grid.resolution_m > 0.0F) || !std::isfinite(x_m) || !std::isfinite(y_m) ||
      !std::isfinite(z_m)) {
    return {};
  }
  const int cell_x =
      static_cast<int>(std::floor((x_m - grid.origin_x_m) / grid.resolution_m));
  const int cell_y =
      static_cast<int>(std::floor((y_m - grid.origin_y_m) / grid.resolution_m));
  const int cell_z =
      static_cast<int>(std::floor((z_m - grid.origin_z_m) / grid.resolution_m));
  if (cell_x < 0 || cell_y < 0 || cell_z < 0 || cell_x >= grid.width ||
      cell_y >= grid.height || cell_z >= grid.depth) {
    return {.clearance_m = std::numeric_limits<float>::infinity(),
            .status = EsdfQueryStatus::kOutsideGrid,
            .raw_occupied = false};
  }
  const std::size_t index =
      (static_cast<std::size_t>(cell_z) * static_cast<std::size_t>(grid.height) +
       static_cast<std::size_t>(cell_y)) *
          static_cast<std::size_t>(grid.width) +
      static_cast<std::size_t>(cell_x);
  if (index >= esdf_m.size()) {
    return {};
  }
  const float center_distance_m = esdf_m[index];
  if (std::isinf(center_distance_m) && center_distance_m > 0.0F) {
    return {.clearance_m = std::numeric_limits<float>::infinity(),
            .status = EsdfQueryStatus::kValid,
            .raw_occupied = false};
  }
  if (!std::isfinite(center_distance_m) || center_distance_m < 0.0F) {
    return {.clearance_m = 0.0F,
            .status = EsdfQueryStatus::kInvalidDistance,
            .raw_occupied = true};
  }
  const float center_x_m =
      grid.origin_x_m + (static_cast<float>(cell_x) + 0.5F) * grid.resolution_m;
  const float center_y_m =
      grid.origin_y_m + (static_cast<float>(cell_y) + 0.5F) * grid.resolution_m;
  const float center_z_m =
      grid.origin_z_m + (static_cast<float>(cell_z) + 0.5F) * grid.resolution_m;
  const float dx = x_m - center_x_m;
  const float dy = y_m - center_y_m;
  const float dz = z_m - center_z_m;
  constexpr float kHalfVoxelDiagonalScale{0.86602540378443864676F};
  const float correction_m = std::sqrt(dx * dx + dy * dy + dz * dz) +
                             kHalfVoxelDiagonalScale * grid.resolution_m;
  return {.clearance_m = std::max(0.0F, center_distance_m - correction_m),
          .status = EsdfQueryStatus::kValid,
          .raw_occupied = center_distance_m == 0.0F};
}

} // namespace drone_city_nav
