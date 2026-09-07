#include "drone_city_nav/indexed_point_cloud_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace drone_city_nav {
namespace {

// Cell coordinates are packed into 21 bits each around a mid-range offset, so
// a key orders cells lexicographically by (x, y, z) and a cloud may span
// about a million cells along each axis.
constexpr std::int64_t kCellCoordinateBits{21};
constexpr std::int64_t kCellCoordinateOffset{std::int64_t{1}
                                             << (kCellCoordinateBits - 1)};
constexpr std::int64_t kCellCoordinateMask{(std::int64_t{1} << kCellCoordinateBits) -
                                           1};

[[nodiscard]] std::int64_t cellCoordinate(const double value,
                                          const double cell_size_m) noexcept {
  const double cell = std::floor(value / cell_size_m);
  const double clamped = std::clamp(cell, static_cast<double>(-kCellCoordinateOffset),
                                    static_cast<double>(kCellCoordinateOffset - 1));
  return static_cast<std::int64_t>(clamped);
}

[[nodiscard]] std::int64_t packKey(const std::int64_t x, const std::int64_t y,
                                   const std::int64_t z) noexcept {
  return (((x + kCellCoordinateOffset) & kCellCoordinateMask)
          << (2 * kCellCoordinateBits)) |
         (((y + kCellCoordinateOffset) & kCellCoordinateMask) << kCellCoordinateBits) |
         ((z + kCellCoordinateOffset) & kCellCoordinateMask);
}

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool insideBox(const Point3& point, const Point3& minimum,
                             const Point3& maximum) noexcept {
  return point.x >= minimum.x && point.x <= maximum.x && point.y >= minimum.y &&
         point.y <= maximum.y && point.z >= minimum.z && point.z <= maximum.z;
}

} // namespace

std::int64_t indexedPointCloudCellKey3D(const Point3& point,
                                        const double cell_size_m) noexcept {
  if (!finitePoint(point) || !(cell_size_m > 0.0)) {
    return 0;
  }
  return packKey(cellCoordinate(point.x, cell_size_m),
                 cellCoordinate(point.y, cell_size_m),
                 cellCoordinate(point.z, cell_size_m));
}

IndexedPointCloudView3D::IndexedPointCloudView3D(
    const std::span<const Point3> points,
    const std::span<const IndexedPointCloudCell3D> cells,
    const double cell_size_m) noexcept
    : points_{points},
      cells_{cells},
      cell_size_m_{cell_size_m} {
}

void IndexedPointCloudView3D::collectInsideBox(const Point3& minimum,
                                               const Point3& maximum,
                                               std::vector<Point3>& output) const {
  if (points_.empty() || !finitePoint(minimum) || !finitePoint(maximum)) {
    return;
  }
  if (!indexed()) {
    for (const Point3& point : points_) {
      if (insideBox(point, minimum, maximum)) {
        output.push_back(point);
      }
    }
    return;
  }
  const std::int64_t minimum_x = cellCoordinate(minimum.x, cell_size_m_);
  const std::int64_t minimum_y = cellCoordinate(minimum.y, cell_size_m_);
  const std::int64_t minimum_z = cellCoordinate(minimum.z, cell_size_m_);
  const std::int64_t maximum_x = cellCoordinate(maximum.x, cell_size_m_);
  const std::int64_t maximum_y = cellCoordinate(maximum.y, cell_size_m_);
  const std::int64_t maximum_z = cellCoordinate(maximum.z, cell_size_m_);
  // A box that overlaps more cells than the cloud has is cheaper to answer
  // with the linear pass the unindexed view makes.
  const double overlapped_cells = static_cast<double>(maximum_x - minimum_x + 1) *
                                  static_cast<double>(maximum_y - minimum_y + 1) *
                                  static_cast<double>(maximum_z - minimum_z + 1);
  if (overlapped_cells >= static_cast<double>(cells_.size())) {
    for (const Point3& point : points_) {
      if (insideBox(point, minimum, maximum)) {
        output.push_back(point);
      }
    }
    return;
  }
  for (std::int64_t x = minimum_x; x <= maximum_x; ++x) {
    for (std::int64_t y = minimum_y; y <= maximum_y; ++y) {
      for (std::int64_t z = minimum_z; z <= maximum_z; ++z) {
        const std::int64_t key = packKey(x, y, z);
        const auto cell = std::ranges::lower_bound(
            cells_, key, {},
            [](const IndexedPointCloudCell3D& entry) { return entry.key; });
        if (cell == cells_.end() || cell->key != key) {
          continue;
        }
        for (std::uint32_t index = cell->begin; index < cell->end; ++index) {
          const Point3& point = points_[index];
          if (insideBox(point, minimum, maximum)) {
            output.push_back(point);
          }
        }
      }
    }
  }
}

IndexedPointCloud3D::IndexedPointCloud3D(std::vector<Point3> points,
                                         const double cell_size_m)
    : points_{std::move(points)},
      cell_size_m_{cell_size_m} {
  if (points_.empty() || !(cell_size_m_ > 0.0) || !std::isfinite(cell_size_m_) ||
      points_.size() >
          static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    cell_size_m_ = 0.0;
    return;
  }
  std::vector<std::int64_t> keys(points_.size());
  for (std::size_t index = 0U; index < points_.size(); ++index) {
    keys[index] = indexedPointCloudCellKey3D(points_[index], cell_size_m_);
  }
  std::vector<std::uint32_t> order(points_.size());
  std::iota(order.begin(), order.end(), 0U);
  std::ranges::stable_sort(order, {},
                           [&keys](const std::uint32_t index) { return keys[index]; });
  std::vector<Point3> sorted(points_.size());
  for (std::size_t index = 0U; index < order.size(); ++index) {
    sorted[index] = points_[order[index]];
  }
  points_ = std::move(sorted);
  cells_.clear();
  for (std::size_t index = 0U; index < order.size(); ++index) {
    const std::int64_t key = keys[order[index]];
    if (cells_.empty() || cells_.back().key != key) {
      cells_.push_back(
          IndexedPointCloudCell3D{.key = key,
                                  .begin = static_cast<std::uint32_t>(index),
                                  .end = static_cast<std::uint32_t>(index + 1U)});
    } else {
      cells_.back().end = static_cast<std::uint32_t>(index + 1U);
    }
  }
}

IndexedPointCloudView3D IndexedPointCloud3D::view() const noexcept {
  return IndexedPointCloudView3D{points_, cells_, cell_size_m_};
}

} // namespace drone_city_nav
