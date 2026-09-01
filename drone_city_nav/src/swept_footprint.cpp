#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

#include "swept_footprint_internal.hpp"

namespace drone_city_nav {
namespace {

using swept_footprint_detail::inflatedSweepFootprint;
using swept_footprint_detail::interpolateSweepAxis;
using swept_footprint_detail::interpolateSweepPosition;
using swept_footprint_detail::makeConservativeSweepCover3D;
using swept_footprint_detail::makeStatusResult;
using swept_footprint_detail::normalized;

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

[[nodiscard]] bool conservativeLessOrEqual(const double value,
                                           const double limit) noexcept {
  constexpr double kContactTolerance = 64.0 * std::numeric_limits<double>::epsilon();
  const double scale = std::max({1.0, std::abs(value), std::abs(limit)});
  return value <= limit + kContactTolerance * scale;
}

[[nodiscard]] double squaredDistanceSegmentToBox(const Point3& first,
                                                 const Point3& second,
                                                 const Point3& minimum,
                                                 const Point3& maximum) noexcept {
  const Point3 direction{second.x - first.x, second.y - first.y, second.z - first.z};
  const auto distance_at = [&](const double ratio) noexcept {
    return squaredDistanceToBox(Point3{first.x + direction.x * ratio,
                                       first.y + direction.y * ratio,
                                       first.z + direction.z * ratio},
                                minimum, maximum);
  };

  // Point-to-box squared distance is a convex piecewise quadratic along the
  // segment. Box-face crossing ratios delimit every quadratic piece, so the
  // exact minimum is one of the breakpoints or a stationary point inside a
  // piece. A fixed-iteration search can overestimate this minimum and miss a
  // tangent or shallow collision.
  std::array<double, 8U> breakpoints{};
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
  add_axis_breakpoints(first.z, direction.z, minimum.z, maximum.z);
  std::sort(breakpoints.begin(), breakpoints.begin() + breakpoint_count);
  const auto unique_end =
      std::unique(breakpoints.begin(), breakpoints.begin() + breakpoint_count);
  breakpoint_count =
      static_cast<std::size_t>(std::distance(breakpoints.begin(), unique_end));

  double minimum_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < breakpoint_count; ++index) {
    minimum_distance_squared =
        std::min(minimum_distance_squared, distance_at(breakpoints[index]));
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
    add_active_axis(first.z, direction.z, minimum.z, maximum.z);
    if (derivative_slope > 0.0) {
      const double stationary_ratio =
          std::clamp(-derivative_constant / derivative_slope, breakpoints[index],
                     breakpoints[index + 1U]);
      minimum_distance_squared =
          std::min(minimum_distance_squared, distance_at(stationary_ratio));
    }
  }
  return minimum_distance_squared;
}

[[nodiscard]] bool boxIntersectsFiniteCylinder(const Point3& center,
                                               const FootprintBodyAxis& axis,
                                               const Point3& lower, const Point3& upper,
                                               const double lower_extent_m,
                                               const double upper_extent_m,
                                               const double radius_squared) noexcept {
  const Point3 box_center{0.5 * (lower.x + upper.x), 0.5 * (lower.y + upper.y),
                          0.5 * (lower.z + upper.z)};
  const Point3 box_half_extent{0.5 * (upper.x - lower.x), 0.5 * (upper.y - lower.y),
                               0.5 * (upper.z - lower.z)};
  const Point3 relative_center{box_center.x - center.x, box_center.y - center.y,
                               box_center.z - center.z};
  const double projected_center = relative_center.x * axis.x +
                                  relative_center.y * axis.y +
                                  relative_center.z * axis.z;
  const double projected_half_extent = box_half_extent.x * std::abs(axis.x) +
                                       box_half_extent.y * std::abs(axis.y) +
                                       box_half_extent.z * std::abs(axis.z);
  if (!conservativeLessOrEqual(-lower_extent_m,
                               projected_center + projected_half_extent) ||
      !conservativeLessOrEqual(projected_center - projected_half_extent,
                               upper_extent_m)) {
    return false;
  }

  const auto squared_interval_distance = [](const double value, const double minimum,
                                            const double maximum) noexcept {
    if (value < minimum) {
      const double distance = minimum - value;
      return distance * distance;
    }
    if (value > maximum) {
      const double distance = value - maximum;
      return distance * distance;
    }
    return 0.0;
  };
  constexpr double kCardinalAxisTolerance{1.0e-12};
  if (std::abs(axis.x) >= 1.0 - kCardinalAxisTolerance) {
    return conservativeLessOrEqual(
        squared_interval_distance(center.y, lower.y, upper.y) +
            squared_interval_distance(center.z, lower.z, upper.z),
        radius_squared);
  }
  if (std::abs(axis.y) >= 1.0 - kCardinalAxisTolerance) {
    return conservativeLessOrEqual(
        squared_interval_distance(center.x, lower.x, upper.x) +
            squared_interval_distance(center.z, lower.z, upper.z),
        radius_squared);
  }
  if (std::abs(axis.z) >= 1.0 - kCardinalAxisTolerance) {
    return conservativeLessOrEqual(
        squared_interval_distance(center.x, lower.x, upper.x) +
            squared_interval_distance(center.y, lower.y, upper.y),
        radius_squared);
  }

  const Point3 axis_lower{center.x - lower_extent_m * axis.x,
                          center.y - lower_extent_m * axis.y,
                          center.z - lower_extent_m * axis.z};
  const Point3 axis_upper{center.x + upper_extent_m * axis.x,
                          center.y + upper_extent_m * axis.y,
                          center.z + upper_extent_m * axis.z};
  return conservativeLessOrEqual(
      squaredDistanceSegmentToBox(axis_lower, axis_upper, lower, upper),
      radius_squared);
}

[[nodiscard]] std::pair<double, double>
projectedBoxInterval(const Point3& minimum, const Point3& maximum, const Point3& origin,
                     const FootprintBodyAxis& axis) noexcept {
  const Point3 center{0.5 * (minimum.x + maximum.x), 0.5 * (minimum.y + maximum.y),
                      0.5 * (minimum.z + maximum.z)};
  const Point3 half_extent{0.5 * (maximum.x - minimum.x), 0.5 * (maximum.y - minimum.y),
                           0.5 * (maximum.z - minimum.z)};
  const Point3 relative{center.x - origin.x, center.y - origin.y, center.z - origin.z};
  const double projected_center =
      relative.x * axis.x + relative.y * axis.y + relative.z * axis.z;
  const double projected_half_extent = half_extent.x * std::abs(axis.x) +
                                       half_extent.y * std::abs(axis.y) +
                                       half_extent.z * std::abs(axis.z);
  return {projected_center - projected_half_extent,
          projected_center + projected_half_extent};
}

[[nodiscard]] bool
seedAllowsSupportContact(const ProprioceptiveFreeSpaceSeed3D& seed,
                         const Point3& box_minimum, const Point3& box_maximum,
                         const double occupancy_resolution_m) noexcept {
  const FootprintBodyAxis seed_axis = normalized(seed.body_axis);
  if (!boxIntersectsFiniteCylinder(seed.position, seed_axis, box_minimum, box_maximum,
                                   std::max(0.0, seed.footprint.lower_extent_m),
                                   std::max(0.0, seed.footprint.upper_extent_m),
                                   std::max(0.0, seed.footprint.radius_m) *
                                       std::max(0.0, seed.footprint.radius_m))) {
    return false;
  }
  const auto [box_minimum_axial, box_maximum_axial] =
      projectedBoxInterval(box_minimum, box_maximum, seed.position, seed_axis);
  const double lower_cap = -std::max(0.0, seed.footprint.lower_extent_m);
  constexpr double kContactToleranceM{1.0e-9};
  const double quantization_tolerance_m = std::max(0.0, occupancy_resolution_m);
  return box_minimum_axial <=
             lower_cap + quantization_tolerance_m + kContactToleranceM &&
         box_maximum_axial >=
             lower_cap - quantization_tolerance_m - kContactToleranceM &&
         box_minimum_axial < kContactToleranceM;
}

[[nodiscard]] bool
candidateRemainsInsideLaunchSupportEnvelope(const LaunchSupportContact3D& contact,
                                            const Point3& candidate_position) noexcept {
  const FootprintBodyAxis seed_axis = normalized(contact.seed.body_axis);
  const Point3 delta{candidate_position.x - contact.seed.position.x,
                     candidate_position.y - contact.seed.position.y,
                     candidate_position.z - contact.seed.position.z};
  const double axial =
      delta.x * seed_axis.x + delta.y * seed_axis.y + delta.z * seed_axis.z;
  const double lateral_squared = std::max(0.0, delta.x * delta.x + delta.y * delta.y +
                                                   delta.z * delta.z - axial * axial);
  constexpr double kDepartureToleranceM{1.0e-9};
  return axial >= contact.minimum_axial_departure_m - kDepartureToleranceM &&
         lateral_squared <=
             contact.maximum_lateral_departure_m * contact.maximum_lateral_departure_m +
                 kDepartureToleranceM;
}

[[nodiscard]] bool sameBox(const AxisAlignedBox3D& first, const Point3& second_minimum,
                           const Point3& second_maximum) noexcept {
  constexpr double kCellAlignmentToleranceM{1.0e-6};
  return std::abs(first.minimum.x - second_minimum.x) <= kCellAlignmentToleranceM &&
         std::abs(first.minimum.y - second_minimum.y) <= kCellAlignmentToleranceM &&
         std::abs(first.minimum.z - second_minimum.z) <= kCellAlignmentToleranceM &&
         std::abs(first.maximum.x - second_maximum.x) <= kCellAlignmentToleranceM &&
         std::abs(first.maximum.y - second_maximum.y) <= kCellAlignmentToleranceM &&
         std::abs(first.maximum.z - second_maximum.z) <= kCellAlignmentToleranceM;
}

[[nodiscard]] bool launchSupportContainsCell(const LaunchSupportContact3D& contact,
                                             const Point3& box_minimum,
                                             const Point3& box_maximum) noexcept {
  return std::ranges::any_of(contact.contact_cells, [&](const AxisAlignedBox3D& box) {
    return sameBox(box, box_minimum, box_maximum);
  });
}

[[nodiscard]] bool pointInsideBox(const Point3& point,
                                  const AxisAlignedBox3D& box) noexcept {
  constexpr double kContactToleranceM{1.0e-6};
  return point.x >= box.minimum.x - kContactToleranceM &&
         point.x <= box.maximum.x + kContactToleranceM &&
         point.y >= box.minimum.y - kContactToleranceM &&
         point.y <= box.maximum.y + kContactToleranceM &&
         point.z >= box.minimum.z - kContactToleranceM &&
         point.z <= box.maximum.z + kContactToleranceM;
}

[[nodiscard]] bool launchSupportAllowsPoint(const LaunchSupportContact3D& contact,
                                            const Point3& point,
                                            const Point3& candidate_position) noexcept {
  return candidateRemainsInsideLaunchSupportEnvelope(contact, candidate_position) &&
         std::ranges::any_of(contact.contact_cells, [&](const AxisAlignedBox3D& box) {
           return pointInsideBox(point, box);
         });
}

[[nodiscard]] int minimumContactCell(const double coordinate, const double origin,
                                     const double resolution) noexcept {
  return static_cast<int>(std::ceil((coordinate - origin) / resolution)) - 1;
}

[[nodiscard]] int maximumContactCell(const double coordinate, const double origin,
                                     const double resolution) noexcept {
  return static_cast<int>(std::floor((coordinate - origin) / resolution));
}

[[nodiscard]] SweptFootprintResult validRawFootprint() noexcept {
  return {.status = SweptFootprintStatus::kValid};
}

[[nodiscard]] bool pointIntersectsBody(const Point3& point, const Point3& center,
                                       const FootprintBodyAxis& requested_axis,
                                       const SweptFootprintConfig& config) noexcept {
  const FootprintBodyAxis axis = normalized(requested_axis);
  const Point3 delta{point.x - center.x, point.y - center.y, point.z - center.z};
  const double axial = delta.x * axis.x + delta.y * axis.y + delta.z * axis.z;
  if (axial < -std::max(0.0, config.lower_extent_m) ||
      axial > std::max(0.0, config.upper_extent_m)) {
    return false;
  }
  const double distance_squared =
      delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
  const double radial_squared = std::max(0.0, distance_squared - axial * axial);
  const double radius = std::max(0.0, config.radius_m);
  return radial_squared <= radius * radius;
}

template<typename Occupancy>
[[nodiscard]] SweptFootprintResult validateRawFootprintAt3D(
    const Occupancy& occupancy, const Point3& position,
    const FootprintBodyAxis& requested_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact = nullptr) noexcept {
  const double radius_m = std::max(0.0, config.radius_m);
  if (!(radius_m > 0.0)) {
    const std::optional<GridIndex3D> cell = occupancy.worldToCell(position);
    if (!cell.has_value()) {
      return validRawFootprint();
    }
    return occupancy.isOccupied(*cell)
               ? makeStatusResult(SweptFootprintStatus::kRawCollision, position)
               : validRawFootprint();
  }

  const FootprintBodyAxis axis = normalized(requested_body_axis);
  const double lower_extent_m = std::max(0.0, config.lower_extent_m);
  const double upper_extent_m = std::max(0.0, config.upper_extent_m);
  const Point3 lower{position.x - lower_extent_m * axis.x,
                     position.y - lower_extent_m * axis.y,
                     position.z - lower_extent_m * axis.z};
  const Point3 upper{position.x + upper_extent_m * axis.x,
                     position.y + upper_extent_m * axis.y,
                     position.z + upper_extent_m * axis.z};
  const Point3 radial_extent{radius_m * std::sqrt(std::max(0.0, 1.0 - axis.x * axis.x)),
                             radius_m * std::sqrt(std::max(0.0, 1.0 - axis.y * axis.y)),
                             radius_m *
                                 std::sqrt(std::max(0.0, 1.0 - axis.z * axis.z))};
  const GridBounds3D& bounds = occupancy.bounds();
  const Point3 body_minimum{std::min(lower.x, upper.x) - radial_extent.x,
                            std::min(lower.y, upper.y) - radial_extent.y,
                            std::min(lower.z, upper.z) - radial_extent.z};
  const Point3 body_maximum{std::max(lower.x, upper.x) + radial_extent.x,
                            std::max(lower.y, upper.y) + radial_extent.y,
                            std::max(lower.z, upper.z) + radial_extent.z};
  const int requested_minimum_x =
      minimumContactCell(body_minimum.x, bounds.origin_x, bounds.resolution_m);
  const int requested_maximum_x =
      maximumContactCell(body_maximum.x, bounds.origin_x, bounds.resolution_m);
  const int requested_minimum_y =
      minimumContactCell(body_minimum.y, bounds.origin_y, bounds.resolution_m);
  const int requested_maximum_y =
      maximumContactCell(body_maximum.y, bounds.origin_y, bounds.resolution_m);
  const int requested_minimum_z =
      minimumContactCell(body_minimum.z, bounds.origin_z, bounds.resolution_m);
  const int requested_maximum_z =
      maximumContactCell(body_maximum.z, bounds.origin_z, bounds.resolution_m);
  const int minimum_x = std::max(0, requested_minimum_x);
  const int maximum_x = std::min(bounds.width_cells - 1, requested_maximum_x);
  const int minimum_y = std::max(0, requested_minimum_y);
  const int maximum_y = std::min(bounds.height_cells - 1, requested_maximum_y);
  const int minimum_z = std::max(0, requested_minimum_z);
  const int maximum_z = std::min(bounds.depth_cells - 1, requested_maximum_z);
  const double radius_squared = radius_m * radius_m;
  OccupancyChunkIndex3D cached_chunk_index{};
  const ObservedOccupancyGrid3D::Chunk* cached_chunk{nullptr};
  bool cached_chunk_initialized{false};
  const auto occupied_at = [&](const GridIndex3D cell) noexcept {
    if constexpr (!std::is_same_v<Occupancy, ObservedOccupancyGrid3D>) {
      return occupancy.isOccupied(cell);
    } else {
      const OccupancyChunkIndex3D chunk_index =
          ObservedOccupancyGrid3D::chunkIndex(cell);
      if (!cached_chunk_initialized || chunk_index != cached_chunk_index) {
        cached_chunk_index = chunk_index;
        cached_chunk = occupancy.findChunk(chunk_index);
        cached_chunk_initialized = true;
      }
      return cached_chunk != nullptr &&
             ObservedOccupancyGrid3D::chunkState(
                 *cached_chunk, ObservedOccupancyGrid3D::localBitIndex(cell)) ==
                 ObservedVoxelState::kOccupied;
    }
  };
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        const GridIndex3D cell{x, y, z};
        if (!occupied_at(cell)) {
          continue;
        }
        const Point3 cell_minimum{bounds.origin_x + x * bounds.resolution_m,
                                  bounds.origin_y + y * bounds.resolution_m,
                                  bounds.origin_z + z * bounds.resolution_m};
        const Point3 cell_maximum{cell_minimum.x + bounds.resolution_m,
                                  cell_minimum.y + bounds.resolution_m,
                                  cell_minimum.z + bounds.resolution_m};
        if (!boxIntersectsFiniteCylinder(position, axis, cell_minimum, cell_maximum,
                                         lower_extent_m, upper_extent_m,
                                         radius_squared)) {
          continue;
        }
        const bool launch_support_cell_allowed =
            launch_support_contact != nullptr &&
            candidateRemainsInsideLaunchSupportEnvelope(*launch_support_contact,
                                                        position) &&
            launchSupportContainsCell(*launch_support_contact, cell_minimum,
                                      cell_maximum);
        if (launch_support_cell_allowed) {
          continue;
        }
        return makeStatusResult(SweptFootprintStatus::kRawCollision,
                                occupancy.cellCenter(cell));
      }
    }
  }
  return validRawFootprint();
}

template<typename Occupancy>
[[nodiscard]] SweptFootprintResult validateRawSweptFootprint3D(
    const Occupancy& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact = nullptr) noexcept {
  const auto cover = makeConservativeSweepCover3D(first, first_body_axis, second,
                                                  second_body_axis, config);
  if (!cover.valid()) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }
  const SweptFootprintResult first_result = validateRawFootprintAt3D(
      occupancy, cover.first, cover.first_axis, config, launch_support_contact);
  if (!first_result.accepted()) {
    return first_result;
  }
  for (std::size_t interval = 0U; interval < cover.interval_count; ++interval) {
    const double interval_begin_ratio =
        static_cast<double>(interval) / static_cast<double>(cover.interval_count);
    const double interval_end_ratio =
        static_cast<double>(interval + 1U) / static_cast<double>(cover.interval_count);
    const double ratio = 0.5 * (interval_begin_ratio + interval_end_ratio);
    const FootprintBodyAxis midpoint_axis = interpolateSweepAxis(cover, ratio);
    const SweptFootprintConfig inflated_config =
        inflatedSweepFootprint(config, cover, midpoint_axis);
    const LaunchSupportContact3D* interval_launch_support = launch_support_contact;
    if (launch_support_contact != nullptr &&
        (!candidateRemainsInsideLaunchSupportEnvelope(
             *launch_support_contact,
             interpolateSweepPosition(cover, interval_begin_ratio)) ||
         !candidateRemainsInsideLaunchSupportEnvelope(
             *launch_support_contact,
             interpolateSweepPosition(cover, interval_end_ratio)))) {
      // The launch envelope is convex. Both interval endpoints must be inside
      // before its contact cells can be exempted for the complete interval.
      interval_launch_support = nullptr;
    }
    const SweptFootprintResult result = validateRawFootprintAt3D(
        occupancy, interpolateSweepPosition(cover, ratio), midpoint_axis,
        inflated_config, interval_launch_support);
    if (!result.accepted()) {
      return result;
    }
  }
  const SweptFootprintResult second_result = validateRawFootprintAt3D(
      occupancy, cover.second, cover.second_axis, config, launch_support_contact);
  if (!second_result.accepted()) {
    return second_result;
  }
  return validRawFootprint();
}

} // namespace

bool proprioceptiveSeedAllowsSupportContact(
    const ProprioceptiveFreeSpaceSeed3D& seed, const Point3& box_minimum,
    const Point3& box_maximum, const double occupancy_resolution_m) noexcept {
  return seedAllowsSupportContact(seed, box_minimum, box_maximum,
                                  occupancy_resolution_m);
}

SweptFootprintResult
validateRawFootprintAt(const OccupancyGrid3D& occupancy, const Point3& position,
                       const FootprintBodyAxis& requested_body_axis,
                       const SweptFootprintConfig& config) noexcept {
  return validateRawFootprintAt3D(occupancy, position, requested_body_axis, config);
}

SweptFootprintResult
validateRawSweptFootprint(const OccupancyGrid3D& occupancy, const Point3& first,
                          const FootprintBodyAxis& first_body_axis,
                          const Point3& second,
                          const FootprintBodyAxis& second_body_axis,
                          const SweptFootprintConfig& config) noexcept {
  return validateRawSweptFootprint3D(occupancy, first, first_body_axis, second,
                                     second_body_axis, config);
}

SweptFootprintResult validateRawFootprintAt(
    const ObservedOccupancyGrid3D& occupancy, const Point3& position,
    const FootprintBodyAxis& requested_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  if (launch_support_contact != nullptr &&
      !launchSupportContactValid3D(*launch_support_contact)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, position);
  }
  return validateRawFootprintAt3D(occupancy, position, requested_body_axis, config,
                                  launch_support_contact);
}

SweptFootprintResult validateRawSweptFootprint(
    const ObservedOccupancyGrid3D& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  if (launch_support_contact != nullptr &&
      !launchSupportContactValid3D(*launch_support_contact)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }
  return validateRawSweptFootprint3D(occupancy, first, first_body_axis, second,
                                     second_body_axis, config, launch_support_contact);
}

bool footprintIntersectsAxisAlignedBox(const Point3& position,
                                       const FootprintBodyAxis& requested_body_axis,
                                       const SweptFootprintConfig& config,
                                       const Point3& box_minimum,
                                       const Point3& box_maximum) noexcept {
  const FootprintBodyAxis axis = normalized(requested_body_axis);
  return boxIntersectsFiniteCylinder(
      position, axis, box_minimum, box_maximum, std::max(0.0, config.lower_extent_m),
      std::max(0.0, config.upper_extent_m),
      std::max(0.0, config.radius_m) * std::max(0.0, config.radius_m));
}

SweptFootprintResult validateRawPointCloudFootprintAt(
    const std::span<const Point3> obstacle_points, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  if (launch_support_contact != nullptr &&
      !launchSupportContactValid3D(*launch_support_contact)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, position);
  }
  for (const Point3& point : obstacle_points) {
    if (pointIntersectsBody(point, position, body_axis, config)) {
      if (launch_support_contact != nullptr &&
          launchSupportAllowsPoint(*launch_support_contact, point, position)) {
        continue;
      }
      return makeStatusResult(SweptFootprintStatus::kRawCollision, point);
    }
  }
  return validRawFootprint();
}

SweptFootprintResult validateRawPointCloudSweptFootprint(
    const std::span<const Point3> obstacle_points, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  if (launch_support_contact != nullptr &&
      !launchSupportContactValid3D(*launch_support_contact)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }
  const auto cover = makeConservativeSweepCover3D(first, first_body_axis, second,
                                                  second_body_axis, config);
  if (!cover.valid()) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }
  const SweptFootprintResult first_result = validateRawPointCloudFootprintAt(
      obstacle_points, cover.first, cover.first_axis, config, launch_support_contact);
  if (!first_result.accepted()) {
    return first_result;
  }
  for (std::size_t interval = 0U; interval < cover.interval_count; ++interval) {
    const double interval_begin_ratio =
        static_cast<double>(interval) / static_cast<double>(cover.interval_count);
    const double interval_end_ratio =
        static_cast<double>(interval + 1U) / static_cast<double>(cover.interval_count);
    const double ratio = 0.5 * (interval_begin_ratio + interval_end_ratio);
    const FootprintBodyAxis midpoint_axis = interpolateSweepAxis(cover, ratio);
    const SweptFootprintConfig inflated_config =
        inflatedSweepFootprint(config, cover, midpoint_axis);
    const LaunchSupportContact3D* interval_launch_support = launch_support_contact;
    if (launch_support_contact != nullptr &&
        (!candidateRemainsInsideLaunchSupportEnvelope(
             *launch_support_contact,
             interpolateSweepPosition(cover, interval_begin_ratio)) ||
         !candidateRemainsInsideLaunchSupportEnvelope(
             *launch_support_contact,
             interpolateSweepPosition(cover, interval_end_ratio)))) {
      interval_launch_support = nullptr;
    }
    const SweptFootprintResult result = validateRawPointCloudFootprintAt(
        obstacle_points, interpolateSweepPosition(cover, ratio), midpoint_axis,
        inflated_config, interval_launch_support);
    if (!result.accepted()) {
      return result;
    }
  }
  return validateRawPointCloudFootprintAt(
      obstacle_points, cover.second, cover.second_axis, config, launch_support_contact);
}

FootprintBodyAxis bodyAxisFromWorldAcceleration(const Vec3& acceleration_mps2,
                                                const double gravity_mps2) noexcept {
  return normalized(FootprintBodyAxis{acceleration_mps2.x, acceleration_mps2.y,
                                      acceleration_mps2.z + gravity_mps2});
}

const char* sweptFootprintStatusName(const SweptFootprintStatus status) noexcept {
  switch (status) {
    case SweptFootprintStatus::kValid:
      return "valid";
    case SweptFootprintStatus::kInvalidInput:
      return "invalid_input";
    case SweptFootprintStatus::kRawCollision:
      return "raw_collision";
  }
  return "invalid_status";
}

} // namespace drone_city_nav
