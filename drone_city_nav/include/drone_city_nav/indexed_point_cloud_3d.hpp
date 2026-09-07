#pragma once

#include "drone_city_nav/types.hpp"

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <span>
#include <vector>

namespace drone_city_nav {

// One cubic cell of an indexed point cloud: the points with this cell key
// occupy [begin, end) of the cloud's point array.
struct IndexedPointCloudCell3D {
  std::int64_t key{0};
  std::uint32_t begin{0U};
  std::uint32_t end{0U};
};

// A non-owning view of a point cloud, optionally bucketed into cubic cells so
// that the points inside an axis-aligned box are found by visiting the cells
// the box overlaps instead of every point. A view built from a bare range of
// points carries no cells and answers a box query with one linear pass, so
// callers need not care which they were handed; an indexed view answers the
// same question from the cells.
//
// The lidar scan the execution path is validated against has tens of
// thousands of returns, and every swept segment of the horizon asks which of
// them can touch the body. Scanning the whole return set per segment was the
// largest single cost of horizon assembly and of the commit revalidation.
class IndexedPointCloudView3D final {
public:
  IndexedPointCloudView3D() = default;

  // An unindexed view over a contiguous range of points.
  template<std::ranges::contiguous_range Range>
    requires std::same_as<std::ranges::range_value_t<Range>, Point3>
  IndexedPointCloudView3D(const Range& points) // NOLINT(google-explicit-constructor)
      : points_{std::ranges::data(points), std::ranges::size(points)} {
  }

  IndexedPointCloudView3D(std::span<const Point3> points,
                          std::span<const IndexedPointCloudCell3D> cells,
                          double cell_size_m) noexcept;

  [[nodiscard]] std::span<const Point3> points() const noexcept {
    return points_;
  }

  [[nodiscard]] bool empty() const noexcept {
    return points_.empty();
  }

  [[nodiscard]] std::size_t size() const noexcept {
    return points_.size();
  }

  [[nodiscard]] bool indexed() const noexcept {
    return !cells_.empty() && cell_size_m_ > 0.0;
  }

  [[nodiscard]] double cellSizeM() const noexcept {
    return cell_size_m_;
  }

  // Appends every point inside the closed box [minimum, maximum] to `output`.
  void collectInsideBox(const Point3& minimum, const Point3& maximum,
                        std::vector<Point3>& output) const;

private:
  std::span<const Point3> points_;
  std::span<const IndexedPointCloudCell3D> cells_;
  double cell_size_m_{0.0};
};

// An owning point cloud sorted by cell: the points are reordered so that each
// cell's points are contiguous, and the cells are kept sorted by key for a
// binary search per cell the query box overlaps.
class IndexedPointCloud3D final {
public:
  IndexedPointCloud3D() = default;
  IndexedPointCloud3D(std::vector<Point3> points, double cell_size_m);

  [[nodiscard]] IndexedPointCloudView3D view() const noexcept;

  [[nodiscard]] std::span<const Point3> points() const noexcept {
    return points_;
  }

  [[nodiscard]] std::span<const IndexedPointCloudCell3D> cells() const noexcept {
    return cells_;
  }

  [[nodiscard]] double cellSizeM() const noexcept {
    return cell_size_m_;
  }

private:
  std::vector<Point3> points_;
  std::vector<IndexedPointCloudCell3D> cells_;
  double cell_size_m_{0.0};
};

// The cell key of a point at `cell_size_m`, the value the index sorts by.
[[nodiscard]] std::int64_t indexedPointCloudCellKey3D(const Point3& point,
                                                      double cell_size_m) noexcept;

} // namespace drone_city_nav
