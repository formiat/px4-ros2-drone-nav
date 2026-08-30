#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>

#include "swept_footprint_internal.hpp"

namespace drone_city_nav {
namespace {

using swept_footprint_detail::makeStatusResult;

struct SegmentBoxDistance2D {
  double squared_distance{std::numeric_limits<double>::infinity()};
  double segment_ratio{0.0};
  Point2 nearest_box_point{};
};

struct ContactCellRange {
  int minimum{0};
  int maximum{-1};
};

[[nodiscard]] bool conservativeLessOrEqual(const double value,
                                           const double limit) noexcept {
  constexpr double kContactTolerance = 64.0 * std::numeric_limits<double>::epsilon();
  const double scale = std::max({1.0, std::abs(value), std::abs(limit)});
  return value <= limit + kContactTolerance * scale;
}

[[nodiscard]] SegmentBoxDistance2D
squaredDistanceSegmentToBox(const Point2& first, const Point2& second,
                            const Point2& minimum, const Point2& maximum) noexcept {
  const Point2 direction{second.x - first.x, second.y - first.y};
  SegmentBoxDistance2D best{};
  const auto consider = [&](const double ratio) noexcept {
    const Point2 point{first.x + direction.x * ratio, first.y + direction.y * ratio};
    const Point2 nearest{std::clamp(point.x, minimum.x, maximum.x),
                         std::clamp(point.y, minimum.y, maximum.y)};
    const double dx = point.x - nearest.x;
    const double dy = point.y - nearest.y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < best.squared_distance) {
      best = {.squared_distance = distance_squared,
              .segment_ratio = ratio,
              .nearest_box_point = nearest};
    }
  };

  // Point-to-box squared distance is a convex piecewise quadratic along the
  // segment. Box-face crossings delimit every piece, whose exact minimum is at
  // an endpoint or at its stationary point.
  std::array<double, 6U> breakpoints{};
  breakpoints[0] = 0.0;
  breakpoints[1] = 1.0;
  std::size_t breakpoint_count{2U};
  const auto add_axis_breakpoints = [&](const double start, const double delta,
                                        const double lower,
                                        const double upper) noexcept {
    if (delta == 0.0) {
      return;
    }
    for (const double boundary : {lower, upper}) {
      const double ratio = (boundary - start) / delta;
      if (ratio > 0.0 && ratio < 1.0) {
        breakpoints[breakpoint_count++] = ratio;
      }
    }
  };
  add_axis_breakpoints(first.x, direction.x, minimum.x, maximum.x);
  add_axis_breakpoints(first.y, direction.y, minimum.y, maximum.y);
  std::sort(breakpoints.begin(), breakpoints.begin() + breakpoint_count);
  const auto unique_end =
      std::unique(breakpoints.begin(), breakpoints.begin() + breakpoint_count);
  breakpoint_count =
      static_cast<std::size_t>(std::distance(breakpoints.begin(), unique_end));

  for (std::size_t index = 0U; index < breakpoint_count; ++index) {
    consider(breakpoints[index]);
    if (index + 1U >= breakpoint_count ||
        !(breakpoints[index + 1U] > breakpoints[index])) {
      continue;
    }
    const double midpoint = 0.5 * (breakpoints[index] + breakpoints[index + 1U]);
    double derivative_constant{0.0};
    double derivative_slope{0.0};
    const auto add_active_axis = [&](const double start, const double delta,
                                     const double lower, const double upper) noexcept {
      const double coordinate = start + delta * midpoint;
      const std::optional<double> active_boundary =
          coordinate < lower   ? std::optional<double>{lower}
          : coordinate > upper ? std::optional<double>{upper}
                               : std::nullopt;
      if (!active_boundary.has_value()) {
        return;
      }
      derivative_constant += delta * (start - *active_boundary);
      derivative_slope += delta * delta;
    };
    add_active_axis(first.x, direction.x, minimum.x, maximum.x);
    add_active_axis(first.y, direction.y, minimum.y, maximum.y);
    if (derivative_slope > 0.0) {
      consider(std::clamp(-derivative_constant / derivative_slope, breakpoints[index],
                          breakpoints[index + 1U]));
    }
  }
  return best;
}

[[nodiscard]] std::optional<ContactCellRange>
contactCellRange(const double query_minimum, const double query_maximum,
                 const double origin, const double grid_maximum,
                 const double resolution, const int cell_count) noexcept {
  if (query_maximum < origin || query_minimum > grid_maximum) {
    return std::nullopt;
  }
  const double clamped_minimum = std::clamp(query_minimum, origin, grid_maximum);
  const double clamped_maximum = std::clamp(query_maximum, origin, grid_maximum);
  // Include one adjacent cell on each side. Exact box distance removes the
  // false positives, while this broad phase remains conservative at rounded
  // cell-boundary contacts.
  const int minimum = std::clamp(
      static_cast<int>(std::floor((clamped_minimum - origin) / resolution)) - 1, 0,
      cell_count - 1);
  const int maximum = std::clamp(
      static_cast<int>(std::floor((clamped_maximum - origin) / resolution)) + 1, 0,
      cell_count - 1);
  return ContactCellRange{minimum, maximum};
}

[[nodiscard]] SweptFootprintResult validRawFootprint() noexcept {
  return {.status = SweptFootprintStatus::kValid};
}

template<typename Occupancy>
[[nodiscard]] SweptFootprintResult
validateRawCapsule2D(const Occupancy& occupancy, const Point3& first,
                     const Point3& second,
                     const SweptFootprintConfig& config) noexcept {
  if (!std::isfinite(first.x) || !std::isfinite(first.y) || !std::isfinite(first.z) ||
      !std::isfinite(second.x) || !std::isfinite(second.y) ||
      !std::isfinite(second.z) || !std::isfinite(config.radius_m)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }
  const double radius_m = std::max(0.0, config.radius_m);
  const double radius_squared = radius_m * radius_m;
  const GridBounds& bounds = occupancy.bounds();
  const double grid_maximum_x =
      bounds.origin_x + static_cast<double>(bounds.width_cells) * bounds.resolution_m;
  const double grid_maximum_y =
      bounds.origin_y + static_cast<double>(bounds.height_cells) * bounds.resolution_m;
  const Point2 direction{second.x - first.x, second.y - first.y};
  const double query_minimum_x = std::nextafter(
      std::min(first.x, second.x) - radius_m, -std::numeric_limits<double>::infinity());
  const double query_maximum_x = std::nextafter(
      std::max(first.x, second.x) + radius_m, std::numeric_limits<double>::infinity());
  const double query_minimum_y = std::nextafter(
      std::min(first.y, second.y) - radius_m, -std::numeric_limits<double>::infinity());
  const double query_maximum_y = std::nextafter(
      std::max(first.y, second.y) + radius_m, std::numeric_limits<double>::infinity());
  if (!std::isfinite(radius_squared) || !std::isfinite(grid_maximum_x) ||
      !std::isfinite(grid_maximum_y) || !(grid_maximum_x > bounds.origin_x) ||
      !(grid_maximum_y > bounds.origin_y) || !std::isfinite(direction.x) ||
      !std::isfinite(direction.y) || !std::isfinite(query_minimum_x) ||
      !std::isfinite(query_maximum_x) || !std::isfinite(query_minimum_y) ||
      !std::isfinite(query_maximum_y)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }

  const std::optional<ContactCellRange> x_range =
      contactCellRange(query_minimum_x, query_maximum_x, bounds.origin_x,
                       grid_maximum_x, bounds.resolution_m, bounds.width_cells);
  const std::optional<ContactCellRange> y_range =
      contactCellRange(query_minimum_y, query_maximum_y, bounds.origin_y,
                       grid_maximum_y, bounds.resolution_m, bounds.height_cells);
  if (!x_range.has_value() || !y_range.has_value()) {
    return validRawFootprint();
  }

  for (int y = y_range->minimum; y <= y_range->maximum; ++y) {
    for (int x = x_range->minimum; x <= x_range->maximum; ++x) {
      const GridIndex cell{x, y};
      if (!occupancy.isOccupied(cell)) {
        continue;
      }
      const double cell_minimum_x = bounds.origin_x + x * bounds.resolution_m;
      const double cell_minimum_y = bounds.origin_y + y * bounds.resolution_m;
      const SegmentBoxDistance2D distance = squaredDistanceSegmentToBox(
          Point2{first.x, first.y}, Point2{second.x, second.y},
          Point2{cell_minimum_x, cell_minimum_y},
          Point2{cell_minimum_x + bounds.resolution_m,
                 cell_minimum_y + bounds.resolution_m});
      if (conservativeLessOrEqual(distance.squared_distance, radius_squared)) {
        return makeStatusResult(
            SweptFootprintStatus::kRawCollision,
            Point3{distance.nearest_box_point.x, distance.nearest_box_point.y,
                   first.z + (second.z - first.z) * distance.segment_ratio});
      }
    }
  }
  return validRawFootprint();
}

template<typename Occupancy>
[[nodiscard]] SweptFootprintResult
validateRawFootprintAt2D(const Occupancy& occupancy, const Point3& position,
                         const SweptFootprintConfig& config) noexcept {
  return validateRawCapsule2D(occupancy, position, position, config);
}

} // namespace

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
  return validateRawCapsule2D(occupancy, first, second, config);
}

} // namespace drone_city_nav
