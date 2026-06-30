#include "drone_city_nav/distance_field.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kLargeSquaredDistance = 1.0e30;

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

[[nodiscard]] bool isSourceCell(const OccupancyGrid2D& grid, const GridIndex cell,
                                const DistanceFieldSource source) {
  switch (source) {
    case DistanceFieldSource::kOccupied:
      return grid.isOccupied(cell);
    case DistanceFieldSource::kProhibited:
      return grid.isProhibited(cell);
  }
  return false;
}

[[nodiscard]] std::size_t linearIndex(const GridBounds& bounds, const GridIndex cell) {
  if (cell.x < 0 || cell.y < 0 || cell.x >= bounds.width_cells ||
      cell.y >= bounds.height_cells) {
    throw std::out_of_range{"Distance field cell is outside grid bounds"};
  }
  return static_cast<std::size_t>(cell.y) *
             static_cast<std::size_t>(bounds.width_cells) +
         static_cast<std::size_t>(cell.x);
}

void distanceTransform1D(const std::vector<double>& f, std::vector<double>& d) {
  const int n = static_cast<int>(f.size());
  if (n <= 0) {
    d.clear();
    return;
  }

  std::vector<int> v(static_cast<std::size_t>(n), 0);
  std::vector<double> z(static_cast<std::size_t>(n + 1), 0.0);
  int k = 0;
  v[0] = 0;
  z[0] = -kInfinity;
  z[1] = kInfinity;

  for (int q = 1; q < n; ++q) {
    double s = 0.0;
    while (k >= 0) {
      const int vk = v[static_cast<std::size_t>(k)];
      const double qd = static_cast<double>(q);
      const double vkd = static_cast<double>(vk);
      s = ((f[static_cast<std::size_t>(q)] + (qd * qd)) -
           (f[static_cast<std::size_t>(vk)] + (vkd * vkd))) /
          (2.0 * static_cast<double>(q - vk));
      if (s > z[static_cast<std::size_t>(k)]) {
        break;
      }
      --k;
    }

    if (k < 0) {
      k = 0;
      v[0] = q;
      z[0] = -kInfinity;
      z[1] = kInfinity;
      continue;
    }

    ++k;
    v[static_cast<std::size_t>(k)] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k) + 1U] = kInfinity;
  }

  d.assign(f.size(), kLargeSquaredDistance);
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<std::size_t>(k) + 1U] < static_cast<double>(q)) {
      ++k;
    }
    const int vk = v[static_cast<std::size_t>(k)];
    const double delta = static_cast<double>(q - vk);
    d[static_cast<std::size_t>(q)] = delta * delta + f[static_cast<std::size_t>(vk)];
  }
}

} // namespace

DistanceField2D DistanceField2D::build(const OccupancyGrid2D& grid,
                                       const double max_distance_m,
                                       const DistanceFieldSource source) {
  const auto started_at = std::chrono::steady_clock::now();
  DistanceField2D field{};
  field.bounds_ = grid.bounds();
  field.max_distance_m_ =
      std::isfinite(max_distance_m) && max_distance_m > 0.0 ? max_distance_m : 0.0;
  field.source_ = source;
  field.stats_.width_cells = static_cast<std::size_t>(grid.width());
  field.stats_.height_cells = static_cast<std::size_t>(grid.height());
  field.distance_m_.assign(grid.cellCount(), kInfinity);

  if (grid.width() <= 0 || grid.height() <= 0 || !(grid.resolution() > 0.0)) {
    field.stats_.duration_ms = elapsedMilliseconds(started_at);
    return field;
  }

  const std::size_t width = static_cast<std::size_t>(grid.width());
  const std::size_t height = static_cast<std::size_t>(grid.height());
  std::vector<double> row_input(width, kLargeSquaredDistance);
  std::vector<double> row_output(width, kLargeSquaredDistance);
  std::vector<double> column_input(height, kLargeSquaredDistance);
  std::vector<double> column_output(height, kLargeSquaredDistance);
  std::vector<double> x_pass(grid.cellCount(), kLargeSquaredDistance);

  for (int y = 0; y < grid.height(); ++y) {
    std::fill(row_input.begin(), row_input.end(), kLargeSquaredDistance);
    for (int x = 0; x < grid.width(); ++x) {
      const GridIndex cell{x, y};
      if (isSourceCell(grid, cell, source)) {
        row_input[static_cast<std::size_t>(x)] = 0.0;
        ++field.stats_.source_cells;
      }
    }
    distanceTransform1D(row_input, row_output);
    for (int x = 0; x < grid.width(); ++x) {
      x_pass[grid.linearIndex(GridIndex{x, y})] =
          row_output[static_cast<std::size_t>(x)];
    }
  }

  if (field.stats_.source_cells == 0U) {
    field.stats_.duration_ms = elapsedMilliseconds(started_at);
    return field;
  }

  const double resolution_m = grid.resolution();
  const bool has_max_distance = field.max_distance_m_ > 0.0;
  for (int x = 0; x < grid.width(); ++x) {
    for (int y = 0; y < grid.height(); ++y) {
      column_input[static_cast<std::size_t>(y)] =
          x_pass[grid.linearIndex(GridIndex{x, y})];
    }
    distanceTransform1D(column_input, column_output);
    for (int y = 0; y < grid.height(); ++y) {
      const double distance_cells_sq = column_output[static_cast<std::size_t>(y)];
      if (!(distance_cells_sq < kLargeSquaredDistance * 0.5)) {
        continue;
      }
      const double distance_m = std::sqrt(distance_cells_sq) * resolution_m;
      field.distance_m_[grid.linearIndex(GridIndex{x, y})] =
          !has_max_distance || distance_m <= field.max_distance_m_ ? distance_m
                                                                   : kInfinity;
    }
  }

  field.stats_.duration_ms = elapsedMilliseconds(started_at);
  return field;
}

const GridBounds& DistanceField2D::bounds() const noexcept {
  return bounds_;
}

double DistanceField2D::maxDistanceM() const noexcept {
  return max_distance_m_;
}

DistanceFieldSource DistanceField2D::source() const noexcept {
  return source_;
}

bool DistanceField2D::contains(const GridIndex cell) const noexcept {
  return cell.x >= 0 && cell.y >= 0 && cell.x < bounds_.width_cells &&
         cell.y < bounds_.height_cells;
}

double DistanceField2D::distanceAt(const GridIndex cell) const {
  return distance_m_.at(linearIndex(bounds_, cell));
}

std::span<const double> DistanceField2D::distancesM() const noexcept {
  return distance_m_;
}

const DistanceFieldBuildStats& DistanceField2D::stats() const noexcept {
  return stats_;
}

} // namespace drone_city_nav
