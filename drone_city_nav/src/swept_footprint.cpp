#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "swept_footprint_internal.hpp"

namespace drone_city_nav {
namespace {

using swept_footprint_detail::makeStatusResult;
using swept_footprint_detail::mergeEvidence;
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
  if (projected_center + projected_half_extent < -lower_extent_m ||
      projected_center - projected_half_extent > upper_extent_m) {
    return false;
  }

  const Point3 axis_lower{center.x - lower_extent_m * axis.x,
                          center.y - lower_extent_m * axis.y,
                          center.z - lower_extent_m * axis.z};
  const Point3 axis_upper{center.x + upper_extent_m * axis.x,
                          center.y + upper_extent_m * axis.y,
                          center.z + upper_extent_m * axis.z};
  return squaredDistanceSegmentToBox(axis_lower, axis_upper, lower, upper) <=
         radius_squared;
}

[[nodiscard]] bool
pointInsideFiniteCylinder(const Point3& point, const Point3& center,
                          const FootprintBodyAxis& axis,
                          const SweptFootprintConfig& config) noexcept {
  const Point3 delta{point.x - center.x, point.y - center.y, point.z - center.z};
  const double axial = delta.x * axis.x + delta.y * axis.y + delta.z * axis.z;
  if (axial < -std::max(0.0, config.lower_extent_m) ||
      axial > std::max(0.0, config.upper_extent_m)) {
    return false;
  }
  const double distance_squared =
      delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
  const double radial_squared = std::max(0.0, distance_squared - axial * axial);
  const double radius_m = std::max(0.0, config.radius_m);
  return radial_squared <= radius_m * radius_m;
}

[[nodiscard]] bool
boxInsideFiniteCylinder(const Point3& minimum, const Point3& maximum,
                        const Point3& center, const FootprintBodyAxis& axis,
                        const SweptFootprintConfig& config) noexcept {
  for (const double z : {minimum.z, maximum.z}) {
    for (const double y : {minimum.y, maximum.y}) {
      for (const double x : {minimum.x, maximum.x}) {
        if (!pointInsideFiniteCylinder(Point3{x, y, z}, center, axis, config)) {
          return false;
        }
      }
    }
  }
  return true;
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

[[nodiscard]] bool alignedCandidateIntersectionCoveredBySeed(
    const Point3& box_minimum, const Point3& box_maximum,
    const Point3& candidate_position, const FootprintBodyAxis& candidate_axis,
    const SweptFootprintConfig& candidate_config,
    const ProprioceptiveFreeSpaceSeed3D& seed) noexcept {
  FootprintBodyAxis seed_axis = normalized(seed.body_axis);
  double seed_lower_extent_m = std::max(0.0, seed.footprint.lower_extent_m);
  double seed_upper_extent_m = std::max(0.0, seed.footprint.upper_extent_m);
  double alignment = candidate_axis.x * seed_axis.x + candidate_axis.y * seed_axis.y +
                     candidate_axis.z * seed_axis.z;
  if (alignment < 0.0) {
    seed_axis = FootprintBodyAxis{-seed_axis.x, -seed_axis.y, -seed_axis.z};
    std::swap(seed_lower_extent_m, seed_upper_extent_m);
    alignment = -alignment;
  }
  constexpr double kAxisAlignmentTolerance{1.0e-9};
  if (alignment < 1.0 - kAxisAlignmentTolerance) {
    return false;
  }

  const Point3 center_delta{candidate_position.x - seed.position.x,
                            candidate_position.y - seed.position.y,
                            candidate_position.z - seed.position.z};
  const double axial_offset = center_delta.x * seed_axis.x +
                              center_delta.y * seed_axis.y +
                              center_delta.z * seed_axis.z;
  const double center_distance_squared = center_delta.x * center_delta.x +
                                         center_delta.y * center_delta.y +
                                         center_delta.z * center_delta.z;
  const double perpendicular_offset =
      std::sqrt(std::max(0.0, center_distance_squared - axial_offset * axial_offset));
  const double candidate_radius_m = std::max(0.0, candidate_config.radius_m);
  const double seed_radius_m = std::max(0.0, seed.footprint.radius_m);
  constexpr double kContainmentToleranceM{1.0e-9};
  if (perpendicular_offset + candidate_radius_m >
      seed_radius_m + kContainmentToleranceM) {
    return false;
  }

  const double candidate_minimum =
      axial_offset - std::max(0.0, candidate_config.lower_extent_m);
  const double candidate_maximum =
      axial_offset + std::max(0.0, candidate_config.upper_extent_m);
  const auto [box_minimum_axial, box_maximum_axial] =
      projectedBoxInterval(box_minimum, box_maximum, seed.position, seed_axis);
  const double intersection_minimum = std::max(candidate_minimum, box_minimum_axial);
  const double intersection_maximum = std::min(candidate_maximum, box_maximum_axial);
  if (intersection_minimum > intersection_maximum + kContainmentToleranceM) {
    return true;
  }
  return intersection_minimum >= -seed_lower_extent_m - kContainmentToleranceM &&
         intersection_maximum <= seed_upper_extent_m + kContainmentToleranceM;
}

[[nodiscard]] bool
candidateIntersectionCoveredBySeed(const Point3& box_minimum, const Point3& box_maximum,
                                   const Point3& candidate_position,
                                   const FootprintBodyAxis& candidate_axis,
                                   const SweptFootprintConfig& candidate_config,
                                   const ProprioceptiveFreeSpaceSeed3D& seed,
                                   const std::size_t subdivision_depth = 0U) noexcept {
  if (!boxIntersectsFiniteCylinder(candidate_position, candidate_axis, box_minimum,
                                   box_maximum,
                                   std::max(0.0, candidate_config.lower_extent_m),
                                   std::max(0.0, candidate_config.upper_extent_m),
                                   std::max(0.0, candidate_config.radius_m) *
                                       std::max(0.0, candidate_config.radius_m))) {
    return true;
  }
  if (alignedCandidateIntersectionCoveredBySeed(box_minimum, box_maximum,
                                                candidate_position, candidate_axis,
                                                candidate_config, seed)) {
    return true;
  }

  const FootprintBodyAxis seed_axis = normalized(seed.body_axis);
  if (boxInsideFiniteCylinder(box_minimum, box_maximum, seed.position, seed_axis,
                              seed.footprint)) {
    return true;
  }
  constexpr std::size_t kMaximumSubdivisionDepth{4U};
  if (subdivision_depth >= kMaximumSubdivisionDepth) {
    return false;
  }

  const Point3 midpoint{0.5 * (box_minimum.x + box_maximum.x),
                        0.5 * (box_minimum.y + box_maximum.y),
                        0.5 * (box_minimum.z + box_maximum.z)};
  for (int z = 0; z < 2; ++z) {
    for (int y = 0; y < 2; ++y) {
      for (int x = 0; x < 2; ++x) {
        const Point3 child_minimum{x == 0 ? box_minimum.x : midpoint.x,
                                   y == 0 ? box_minimum.y : midpoint.y,
                                   z == 0 ? box_minimum.z : midpoint.z};
        const Point3 child_maximum{x == 0 ? midpoint.x : box_maximum.x,
                                   y == 0 ? midpoint.y : box_maximum.y,
                                   z == 0 ? midpoint.z : box_maximum.z};
        if (!candidateIntersectionCoveredBySeed(
                child_minimum, child_maximum, candidate_position, candidate_axis,
                candidate_config, seed, subdivision_depth + 1U)) {
          return false;
        }
      }
    }
  }
  return true;
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

[[nodiscard]] int minimumCell(const double coordinate, const double origin,
                              const double resolution) noexcept {
  return static_cast<int>(std::floor((coordinate - origin) / resolution));
}

[[nodiscard]] int maximumCell(const double coordinate, const double origin,
                              const double resolution) noexcept {
  return static_cast<int>(std::ceil((coordinate - origin) / resolution)) - 1;
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
[[nodiscard]] SweptFootprintResult
validateRawFootprintAt2D(const Occupancy& occupancy, const Point3& position,
                         const SweptFootprintConfig& config) noexcept {
  const double radius_m = std::max(0.0, config.radius_m);
  if (!(radius_m > 0.0)) {
    const std::optional<GridIndex> cell =
        occupancy.worldToCell(Point2{position.x, position.y});
    return cell.has_value() && occupancy.isOccupied(*cell)
               ? makeStatusResult(SweptFootprintStatus::kRawCollision, position)
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
        return makeStatusResult(SweptFootprintStatus::kRawCollision,
                                Point3{nearest_x, nearest_y, position.z});
      }
    }
  }
  return validRawFootprint();
}

template<bool RequireKnownFree, bool PreserveFailureCategory, typename Occupancy>
[[nodiscard]] SweptFootprintResult validateRawFootprintAt3D(
    const Occupancy& occupancy, const Point3& position,
    const FootprintBodyAxis& requested_body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed = nullptr,
    const LaunchSupportContact3D* const launch_support_contact = nullptr) noexcept {
  const double radius_m = std::max(0.0, config.radius_m);
  if (!(radius_m > 0.0)) {
    const std::optional<GridIndex3D> cell = occupancy.worldToCell(position);
    if (!cell.has_value()) {
      if constexpr (RequireKnownFree) {
        return makeStatusResult(SweptFootprintStatus::kUnknownSpace, position);
      }
      return validRawFootprint();
    }
    if constexpr (RequireKnownFree) {
      if (!occupancy.isKnown(*cell)) {
        return makeStatusResult(SweptFootprintStatus::kUnknownSpace, position);
      }
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
  const int requested_minimum_x =
      minimumCell(std::min(lower.x, upper.x) - radial_extent.x, bounds.origin_x,
                  bounds.resolution_m);
  const int requested_maximum_x =
      maximumCell(std::max(lower.x, upper.x) + radial_extent.x, bounds.origin_x,
                  bounds.resolution_m);
  const int requested_minimum_y =
      minimumCell(std::min(lower.y, upper.y) - radial_extent.y, bounds.origin_y,
                  bounds.resolution_m);
  const int requested_maximum_y =
      maximumCell(std::max(lower.y, upper.y) + radial_extent.y, bounds.origin_y,
                  bounds.resolution_m);
  const int requested_minimum_z =
      minimumCell(std::min(lower.z, upper.z) - radial_extent.z, bounds.origin_z,
                  bounds.resolution_m);
  const int requested_maximum_z =
      maximumCell(std::max(lower.z, upper.z) + radial_extent.z, bounds.origin_z,
                  bounds.resolution_m);
  SweptFootprintResult result = validRawFootprint();
  if constexpr (RequireKnownFree) {
    if (requested_minimum_x < 0 || requested_minimum_y < 0 || requested_minimum_z < 0 ||
        requested_maximum_x >= bounds.width_cells ||
        requested_maximum_y >= bounds.height_cells ||
        requested_maximum_z >= bounds.depth_cells) {
      if constexpr (!PreserveFailureCategory) {
        return makeStatusResult(SweptFootprintStatus::kOutsideGrid, position);
      }
      mergeEvidence(result,
                    makeStatusResult(SweptFootprintStatus::kOutsideGrid, position));
    }
  }
  const int minimum_x = std::max(0, requested_minimum_x);
  const int maximum_x = std::min(bounds.width_cells - 1, requested_maximum_x);
  const int minimum_y = std::max(0, requested_minimum_y);
  const int maximum_y = std::min(bounds.height_cells - 1, requested_maximum_y);
  const int minimum_z = std::max(0, requested_minimum_z);
  const int maximum_z = std::min(bounds.depth_cells - 1, requested_maximum_z);
  const double radius_squared = radius_m * radius_m;
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        const GridIndex3D cell{x, y, z};
        ObservedVoxelState observed_state{ObservedVoxelState::kUnknown};
        if constexpr (RequireKnownFree) {
          observed_state = occupancy.state(cell);
          if (observed_state == ObservedVoxelState::kFree) {
            continue;
          }
        } else {
          if (!occupancy.isOccupied(cell)) {
            continue;
          }
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
        if constexpr (RequireKnownFree) {
          if (observed_state == ObservedVoxelState::kUnknown &&
              !launch_support_cell_allowed &&
              (free_space_seed == nullptr ||
               !candidateIntersectionCoveredBySeed(cell_minimum, cell_maximum, position,
                                                   axis, config, *free_space_seed))) {
            if constexpr (!PreserveFailureCategory) {
              return makeStatusResult(SweptFootprintStatus::kUnknownSpace,
                                      occupancy.cellCenter(cell));
            }
            mergeEvidence(result, makeStatusResult(SweptFootprintStatus::kUnknownSpace,
                                                   occupancy.cellCenter(cell)));
          }
        }
        const bool occupied = [&]() noexcept {
          if constexpr (RequireKnownFree) {
            return observed_state == ObservedVoxelState::kOccupied;
          }
          return true;
        }();
        if (occupied) {
          if (launch_support_cell_allowed) {
            continue;
          }
          mergeEvidence(result, makeStatusResult(SweptFootprintStatus::kRawCollision,
                                                 occupancy.cellCenter(cell)));
          return result;
        }
      }
    }
  }
  return result;
}

template<bool RequireKnownFree, bool PreserveFailureCategory, typename Occupancy>
[[nodiscard]] SweptFootprintResult validateRawSweptFootprint3D(
    const Occupancy& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed = nullptr,
    const LaunchSupportContact3D* const launch_support_contact = nullptr) noexcept {
  const double length_m = distance3D(first, second);
  const double step_m = std::max(1.0e-3, config.sweep_step_m);
  const std::size_t samples =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length_m / step_m)));
  SweptFootprintResult aggregate = validRawFootprint();
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
    const SweptFootprintResult result =
        validateRawFootprintAt3D<RequireKnownFree, PreserveFailureCategory>(
            occupancy,
            Point3{std::lerp(first.x, second.x, ratio),
                   std::lerp(first.y, second.y, ratio),
                   std::lerp(first.z, second.z, ratio)},
            normalized(FootprintBodyAxis{
                std::lerp(first_body_axis.x, second_body_axis.x, ratio),
                std::lerp(first_body_axis.y, second_body_axis.y, ratio),
                std::lerp(first_body_axis.z, second_body_axis.z, ratio)}),
            config, free_space_seed, launch_support_contact);
    if constexpr (!PreserveFailureCategory) {
      if (!result.accepted()) {
        return result;
      }
    } else {
      mergeEvidence(aggregate, result);
      if (result.evidence.raw_collision) {
        return aggregate;
      }
    }
  }
  return aggregate;
}

} // namespace

bool proprioceptiveSeedAllowsSupportContact(
    const ProprioceptiveFreeSpaceSeed3D& seed, const Point3& box_minimum,
    const Point3& box_maximum, const double occupancy_resolution_m) noexcept {
  return seedAllowsSupportContact(seed, box_minimum, box_maximum,
                                  occupancy_resolution_m);
}

bool updateLaunchSupportSettling(LaunchSupportContact3D& contact,
                                 const Point3& observed_position) noexcept {
  const FootprintBodyAxis seed_axis = normalized(contact.seed.body_axis);
  const Point3 delta{observed_position.x - contact.seed.position.x,
                     observed_position.y - contact.seed.position.y,
                     observed_position.z - contact.seed.position.z};
  const double axial =
      delta.x * seed_axis.x + delta.y * seed_axis.y + delta.z * seed_axis.z;
  constexpr double kSettlingToleranceM{1.0e-9};
  if (axial >= contact.minimum_axial_departure_m - kSettlingToleranceM ||
      axial < -contact.maximum_axial_settling_m - kSettlingToleranceM) {
    return false;
  }
  contact.minimum_axial_departure_m = axial;
  return true;
}

const char* sweptFootprintStatusName(const SweptFootprintStatus status) noexcept {
  switch (status) {
    case SweptFootprintStatus::kValid:
      return "valid";
    case SweptFootprintStatus::kOutsideGrid:
      return "outside_grid";
    case SweptFootprintStatus::kUnknownSpace:
      return "unknown_space";
    case SweptFootprintStatus::kInvalidEsdf:
      return "invalid_esdf";
    case SweptFootprintStatus::kRawCollision:
      return "raw_collision";
  }
  return "invalid_status";
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
  return validateRawFootprintAt3D<false, true>(occupancy, position, requested_body_axis,
                                               config);
}

SweptFootprintResult
validateRawSweptFootprint(const OccupancyGrid3D& occupancy, const Point3& first,
                          const FootprintBodyAxis& first_body_axis,
                          const Point3& second,
                          const FootprintBodyAxis& second_body_axis,
                          const SweptFootprintConfig& config) noexcept {
  return validateRawSweptFootprint3D<false, true>(occupancy, first, first_body_axis,
                                                  second, second_body_axis, config);
}

SweptFootprintResult validateRawFootprintAt(
    const ObservedOccupancyGrid3D& occupancy, const Point3& position,
    const FootprintBodyAxis& requested_body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  return validateRawFootprintAt3D<true, true>(occupancy, position, requested_body_axis,
                                              config, free_space_seed,
                                              launch_support_contact);
}

SweptFootprintResult validateRawSweptFootprint(
    const ObservedOccupancyGrid3D& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  return validateRawSweptFootprint3D<true, true>(
      occupancy, first, first_body_axis, second, second_body_axis, config,
      free_space_seed, launch_support_contact);
}

bool rawFootprintIsNavigableAt(const OccupancyGrid3D& occupancy, const Point3& position,
                               const FootprintBodyAxis& body_axis,
                               const SweptFootprintConfig& config) noexcept {
  return validateRawFootprintAt3D<false, false>(occupancy, position, body_axis, config)
      .accepted();
}

bool rawSweptFootprintIsNavigable(const OccupancyGrid3D& occupancy, const Point3& first,
                                  const FootprintBodyAxis& first_body_axis,
                                  const Point3& second,
                                  const FootprintBodyAxis& second_body_axis,
                                  const SweptFootprintConfig& config) noexcept {
  return validateRawSweptFootprint3D<false, false>(occupancy, first, first_body_axis,
                                                   second, second_body_axis, config)
      .accepted();
}

bool rawFootprintIsNavigableAt(
    const ObservedOccupancyGrid3D& occupancy, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  return validateRawFootprintAt3D<true, false>(occupancy, position, body_axis, config,
                                               free_space_seed, launch_support_contact)
      .accepted();
}

bool rawSweptFootprintIsNavigable(
    const ObservedOccupancyGrid3D& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
    const LaunchSupportContact3D* const launch_support_contact) noexcept {
  return validateRawSweptFootprint3D<true, false>(
             occupancy, first, first_body_axis, second, second_body_axis, config,
             free_space_seed, launch_support_contact)
      .accepted();
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
  const double length_m = distance3D(first, second);
  const double step_m = std::max(1.0e-3, config.sweep_step_m);
  const std::size_t samples =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length_m / step_m)));
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
    const Point3 position{std::lerp(first.x, second.x, ratio),
                          std::lerp(first.y, second.y, ratio),
                          std::lerp(first.z, second.z, ratio)};
    const FootprintBodyAxis body_axis{
        std::lerp(first_body_axis.x, second_body_axis.x, ratio),
        std::lerp(first_body_axis.y, second_body_axis.y, ratio),
        std::lerp(first_body_axis.z, second_body_axis.z, ratio),
    };
    const SweptFootprintResult result = validateRawPointCloudFootprintAt(
        obstacle_points, position, body_axis, config, launch_support_contact);
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
