#include "drone_city_nav/grid_config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr double kMinGridResolutionM = 0.01;

[[nodiscard]] double usableResolution(const double resolution_m) noexcept {
  if (!std::isfinite(resolution_m) || resolution_m <= 0.0) {
    return kMinGridResolutionM;
  }
  return std::max(resolution_m, kMinGridResolutionM);
}

} // namespace

int boundedPositiveCellCount(const double length_m,
                             const double resolution_m) noexcept {
  const double resolution = usableResolution(resolution_m);
  if (!std::isfinite(length_m) || length_m <= 0.0) {
    return 1;
  }

  const double raw_count = std::ceil(length_m / resolution);
  if (!std::isfinite(raw_count) || raw_count <= 1.0) {
    return 1;
  }
  if (raw_count >= static_cast<double>(kMaxGridAxisCells)) {
    return kMaxGridAxisCells;
  }
  return static_cast<int>(raw_count);
}

GridBounds boundedGridBounds(const double origin_x, const double origin_y,
                             const double resolution_m, const double width_m,
                             const double height_m) noexcept {
  const double resolution = usableResolution(resolution_m);
  int width_cells = boundedPositiveCellCount(width_m, resolution);
  int height_cells = boundedPositiveCellCount(height_m, resolution);

  const std::size_t width = static_cast<std::size_t>(width_cells);
  const std::size_t height = static_cast<std::size_t>(height_cells);
  if (width > 0U && height > 0U && width > kMaxGridCellCount / height) {
    height_cells = std::max(
        1, static_cast<int>(kMaxGridCellCount / std::max<std::size_t>(1U, width)));
  }

  return GridBounds{std::isfinite(origin_x) ? origin_x : 0.0,
                    std::isfinite(origin_y) ? origin_y : 0.0, resolution, width_cells,
                    height_cells};
}

bool gridBoundsUsable(const GridBounds& bounds) noexcept {
  if (!std::isfinite(bounds.origin_x) || !std::isfinite(bounds.origin_y) ||
      !std::isfinite(bounds.resolution_m) || bounds.resolution_m <= 0.0 ||
      bounds.width_cells <= 0 || bounds.height_cells <= 0) {
    return false;
  }
  const auto width = static_cast<std::size_t>(bounds.width_cells);
  const auto height = static_cast<std::size_t>(bounds.height_cells);
  return width <= kMaxGridCellCount / height;
}

std::size_t gridBoundsCellCount(const GridBounds& bounds) noexcept {
  if (!gridBoundsUsable(bounds)) {
    return 0U;
  }
  return static_cast<std::size_t>(bounds.width_cells) *
         static_cast<std::size_t>(bounds.height_cells);
}

} // namespace drone_city_nav
