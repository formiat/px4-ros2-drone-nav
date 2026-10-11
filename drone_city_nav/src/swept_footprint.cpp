#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>

#include "swept_footprint_internal.hpp"

namespace drone_city_nav {
namespace {

using swept_footprint_detail::boxIntersectsFiniteCylinder;
using swept_footprint_detail::inflatedSweepFootprint;
using swept_footprint_detail::interpolateSweepAxis;
using swept_footprint_detail::interpolateSweepPosition;
using swept_footprint_detail::makeConservativeSweepCover3D;
using swept_footprint_detail::makeStatusResult;
using swept_footprint_detail::normalized;
using swept_footprint_detail::physicalBody;
using swept_footprint_detail::pointIntersectsBody;

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
  return launchSupportEnvelopeContains3D(contact, candidate_position) &&
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

struct AxisAlignedExtent3D {
  Point3 minimum{};
  Point3 maximum{};
};

// Conservative axis-aligned box of a body at one pose: the axial extents along
// the body axis plus the full radius in every direction.
[[nodiscard]] AxisAlignedExtent3D
bodyExtent(const Point3& center, const FootprintBodyAxis& axis,
           const SweptFootprintConfig& config) noexcept {
  const double radius_m = std::max(0.0, config.radius_m);
  const double lower_extent_m = std::max(0.0, config.lower_extent_m);
  const double upper_extent_m = std::max(0.0, config.upper_extent_m);
  const Point3 lower{center.x - lower_extent_m * axis.x,
                     center.y - lower_extent_m * axis.y,
                     center.z - lower_extent_m * axis.z};
  const Point3 upper{center.x + upper_extent_m * axis.x,
                     center.y + upper_extent_m * axis.y,
                     center.z + upper_extent_m * axis.z};
  return AxisAlignedExtent3D{
      .minimum = Point3{std::min(lower.x, upper.x) - radius_m,
                        std::min(lower.y, upper.y) - radius_m,
                        std::min(lower.z, upper.z) - radius_m},
      .maximum = Point3{std::max(lower.x, upper.x) + radius_m,
                        std::max(lower.y, upper.y) + radius_m,
                        std::max(lower.z, upper.z) + radius_m},
  };
}

// The rotating, translating body over a whole sweep stays inside the union of
// its end poses, grown by the largest body radius times the rotation angle.
[[nodiscard]] AxisAlignedExtent3D
sweepExtent(const swept_footprint_detail::ConservativeSweepCover3D& cover,
            const SweptFootprintConfig& config) noexcept {
  const AxisAlignedExtent3D first = bodyExtent(cover.first, cover.first_axis, config);
  const AxisAlignedExtent3D second =
      bodyExtent(cover.second, cover.second_axis, config);
  const double body_radius_m = std::hypot(
      std::max(0.0, config.radius_m), std::max(std::max(0.0, config.lower_extent_m),
                                               std::max(0.0, config.upper_extent_m)));
  const double rotation_margin_m = body_radius_m * cover.angle_rad;
  return AxisAlignedExtent3D{
      .minimum =
          Point3{std::min(first.minimum.x, second.minimum.x) - rotation_margin_m,
                 std::min(first.minimum.y, second.minimum.y) - rotation_margin_m,
                 std::min(first.minimum.z, second.minimum.z) - rotation_margin_m},
      .maximum =
          Point3{std::max(first.maximum.x, second.maximum.x) + rotation_margin_m,
                 std::max(first.maximum.y, second.maximum.y) + rotation_margin_m,
                 std::max(first.maximum.z, second.maximum.z) + rotation_margin_m},
  };
}

template<typename Chunk>
[[nodiscard]] bool chunkHasOccupiedBits(const Chunk& chunk) noexcept {
  if constexpr (std::is_same_v<Chunk, ObservedOccupancyGrid3D::Chunk>) {
    return std::ranges::any_of(chunk.occupied,
                               [](const std::uint64_t word) { return word != 0U; });
  } else {
    return std::ranges::any_of(chunk,
                               [](const std::uint64_t word) { return word != 0U; });
  }
}

// Exact early-out: when no chunk overlapping the extent holds an occupied cell,
// nothing inside the extent can intersect the body. Chunks are 16-cubed so an
// extent of a few metres touches at most a handful of them.
template<typename Occupancy>
[[nodiscard]] bool
extentMayContainOccupied(const Occupancy& occupancy,
                         const AxisAlignedExtent3D& extent) noexcept {
  constexpr int kChunkSize{Occupancy::kChunkSize};
  const GridBounds3D& bounds = occupancy.bounds();
  const int minimum_x = std::max(
      0, minimumContactCell(extent.minimum.x, bounds.origin_x, bounds.resolution_m));
  const int maximum_x = std::min(
      bounds.width_cells - 1,
      maximumContactCell(extent.maximum.x, bounds.origin_x, bounds.resolution_m));
  const int minimum_y = std::max(
      0, minimumContactCell(extent.minimum.y, bounds.origin_y, bounds.resolution_m));
  const int maximum_y = std::min(
      bounds.height_cells - 1,
      maximumContactCell(extent.maximum.y, bounds.origin_y, bounds.resolution_m));
  const int minimum_z = std::max(
      0, minimumContactCell(extent.minimum.z, bounds.origin_z, bounds.resolution_m));
  const int maximum_z = std::min(
      bounds.depth_cells - 1,
      maximumContactCell(extent.maximum.z, bounds.origin_z, bounds.resolution_m));
  if (minimum_x > maximum_x || minimum_y > maximum_y || minimum_z > maximum_z) {
    return false;
  }
  for (int chunk_z = minimum_z / kChunkSize; chunk_z <= maximum_z / kChunkSize;
       ++chunk_z) {
    for (int chunk_y = minimum_y / kChunkSize; chunk_y <= maximum_y / kChunkSize;
         ++chunk_y) {
      for (int chunk_x = minimum_x / kChunkSize; chunk_x <= maximum_x / kChunkSize;
           ++chunk_x) {
        const auto* const chunk =
            occupancy.findChunk(OccupancyChunkIndex3D{chunk_x, chunk_y, chunk_z});
        if (chunk != nullptr && chunkHasOccupiedBits(*chunk)) {
          return true;
        }
      }
    }
  }
  return false;
}

[[nodiscard]] bool pointInsideExtent(const Point3& point,
                                     const AxisAlignedExtent3D& extent) noexcept {
  return point.x >= extent.minimum.x && point.x <= extent.maximum.x &&
         point.y >= extent.minimum.y && point.y <= extent.maximum.y &&
         point.z >= extent.minimum.z && point.z <= extent.maximum.z;
}

// Points that can touch the body over a sweep or at a pose: the cloud's own
// cells answer for the extent, so a segment costs the few cells it overlaps
// rather than one pass over the whole scan.
[[nodiscard]] std::span<const Point3>
pointsInsideExtent(const IndexedPointCloudView3D& points,
                   const AxisAlignedExtent3D& extent) {
  thread_local std::vector<Point3> scratch;
  scratch.clear();
  points.collectInsideBox(extent.minimum, extent.maximum, scratch);
  return scratch;
}

// Contact evidence accepted by one validation: the anchored launch support and
// the proprioceptive seed. Every listed body position must be exempt for the
// evidence to stay suppressed, so a sweep interval lists its begin, midpoint,
// and end and never exempts evidence its interior may approach.
struct ContactExemption3D {
  const LaunchSupportContact3D* launch_support{nullptr};
  const ProprioceptiveFreeSpaceSeed3D* seed{nullptr};
  std::span<const Point3> positions;

  [[nodiscard]] bool exemptsBox(const Point3& box_minimum,
                                const Point3& box_maximum) const noexcept {
    if (positions.empty()) {
      return false;
    }
    if (launch_support != nullptr &&
        launchSupportContactContainsCell3D(*launch_support, box_minimum, box_maximum) &&
        std::ranges::all_of(positions, [&](const Point3& position) {
          return launchSupportEnvelopeContains3D(*launch_support, position);
        })) {
      return true;
    }
    return seed != nullptr && proprioceptiveSeedExemptsBoxAtAll(
                                  *seed, positions, box_minimum, box_maximum);
  }

  [[nodiscard]] bool exemptsPoint(const Point3& obstacle_point) const noexcept {
    if (positions.empty()) {
      return false;
    }
    if (launch_support != nullptr &&
        std::ranges::all_of(positions, [&](const Point3& position) {
          return launchSupportAllowsPoint(*launch_support, obstacle_point, position);
        })) {
      return true;
    }
    return seed != nullptr &&
           proprioceptiveSeedExemptsPointAtAll(*seed, positions, obstacle_point);
  }
};

[[nodiscard]] ContactExemption3D
poseExemption(const LaunchSupportContact3D* const launch_support,
              const ProprioceptiveFreeSpaceSeed3D* const seed,
              const Point3& position) noexcept {
  return ContactExemption3D{.launch_support = launch_support,
                            .seed = seed,
                            .positions = std::span<const Point3>{&position, 1U}};
}

// Exact per-pose validation. Callers that already proved the surrounding
// chunks empty for a whole sweep skip the per-pose broad phase.
template<typename Occupancy>
[[nodiscard]] SweptFootprintResult
validateRawFootprintAt3DExact(const Occupancy& occupancy, const Point3& position,
                              const FootprintBodyAxis& requested_body_axis,
                              const SweptFootprintConfig& config,
                              const ContactExemption3D& exemption) noexcept {
  const double radius_m = std::max(0.0, config.radius_m);
  if (!(radius_m > 0.0)) {
    const std::optional<GridIndex3D> cell = occupancy.worldToCell(position);
    if (!cell.has_value() || !occupancy.isOccupied(*cell)) {
      return validRawFootprint();
    }
    const GridBounds3D& bounds = occupancy.bounds();
    const Point3 cell_minimum{bounds.origin_x + cell->x * bounds.resolution_m,
                              bounds.origin_y + cell->y * bounds.resolution_m,
                              bounds.origin_z + cell->z * bounds.resolution_m};
    const Point3 cell_maximum{cell_minimum.x + bounds.resolution_m,
                              cell_minimum.y + bounds.resolution_m,
                              cell_minimum.z + bounds.resolution_m};
    return exemption.exemptsBox(cell_minimum, cell_maximum)
               ? validRawFootprint()
               : makeStatusResult(SweptFootprintStatus::kRawCollision, position);
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
        if (exemption.exemptsBox(cell_minimum, cell_maximum)) {
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
[[nodiscard]] SweptFootprintResult validateRawFootprintAt3D(
    const Occupancy& occupancy, const Point3& position,
    const FootprintBodyAxis& requested_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact,
    const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_seed) noexcept {
  if (config.radius_m > 0.0 &&
      !extentMayContainOccupied(
          occupancy, bodyExtent(position, normalized(requested_body_axis), config))) {
    return validRawFootprint();
  }
  return validateRawFootprintAt3DExact(
      occupancy, position, requested_body_axis, config,
      poseExemption(launch_support_contact, proprioceptive_seed, position));
}

// Body positions whose exemption must hold for one sweep interval: the launch
// envelope is convex and the seed's admissible region is bounded away from
// the contact, so begin, midpoint, and end together cover the interval up to
// the sweep step the inflated midpoint footprint already absorbs.
struct SweepIntervalPositions3D {
  std::array<Point3, 3U> positions{};

  [[nodiscard]] std::span<const Point3> span() const noexcept {
    return std::span<const Point3>{positions};
  }
};

[[nodiscard]] SweepIntervalPositions3D
sweepIntervalPositions(const swept_footprint_detail::ConservativeSweepCover3D& cover,
                       const double begin_ratio, const double midpoint_ratio,
                       const double end_ratio) noexcept {
  return SweepIntervalPositions3D{
      .positions = {interpolateSweepPosition(cover, begin_ratio),
                    interpolateSweepPosition(cover, midpoint_ratio),
                    interpolateSweepPosition(cover, end_ratio)}};
}

template<typename Occupancy>
[[nodiscard]] SweptFootprintResult validateRawSweptFootprint3D(
    const Occupancy& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact,
    const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_seed) noexcept {
  const auto cover = makeConservativeSweepCover3D(first, first_body_axis, second,
                                                  second_body_axis, config);
  if (!cover.valid()) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }
  if (!extentMayContainOccupied(occupancy, sweepExtent(cover, config))) {
    return validRawFootprint();
  }
  const SweptFootprintResult first_result = validateRawFootprintAt3DExact(
      occupancy, cover.first, cover.first_axis, config,
      poseExemption(launch_support_contact, proprioceptive_seed, cover.first));
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
    const SweepIntervalPositions3D interval_positions =
        sweepIntervalPositions(cover, interval_begin_ratio, ratio, interval_end_ratio);
    const SweptFootprintResult result = validateRawFootprintAt3DExact(
        occupancy, interval_positions.positions[1U], midpoint_axis, inflated_config,
        ContactExemption3D{.launch_support = launch_support_contact,
                           .seed = proprioceptive_seed,
                           .positions = interval_positions.span()});
    if (!result.accepted()) {
      return result;
    }
  }
  const SweptFootprintResult second_result = validateRawFootprintAt3DExact(
      occupancy, cover.second, cover.second_axis, config,
      poseExemption(launch_support_contact, proprioceptive_seed, cover.second));
  if (!second_result.accepted()) {
    return second_result;
  }
  return validRawFootprint();
}

[[nodiscard]] SweptFootprintResult validateRawPointCloudFootprintAt3D(
    const std::span<const Point3> obstacle_points, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const ContactExemption3D& exemption) noexcept {
  for (const Point3& point : obstacle_points) {
    if (pointIntersectsBody(point, position, body_axis, config) &&
        !exemption.exemptsPoint(point)) {
      return makeStatusResult(SweptFootprintStatus::kRawCollision, point);
    }
  }
  return validRawFootprint();
}

[[nodiscard]] bool
validProprioceptiveSeed(const ProprioceptiveFreeSpaceSeed3D* const seed) noexcept {
  if (seed == nullptr) {
    return true;
  }
  const double axis_norm =
      std::hypot(std::hypot(seed->body_axis.x, seed->body_axis.y), seed->body_axis.z);
  return swept_footprint_detail::finitePoint(seed->position) &&
         std::isfinite(axis_norm) && axis_norm > 1.0e-9 &&
         std::isfinite(seed->footprint.radius_m) && seed->footprint.radius_m >= 0.0 &&
         std::isfinite(seed->footprint.body_radius_m) &&
         seed->footprint.body_radius_m >= 0.0 &&
         std::isfinite(seed->footprint.body_lower_extent_m) &&
         seed->footprint.body_lower_extent_m >= 0.0 &&
         std::isfinite(seed->footprint.body_upper_extent_m) &&
         seed->footprint.body_upper_extent_m >= 0.0 &&
         std::isfinite(seed->footprint.lower_extent_m) &&
         seed->footprint.lower_extent_m >= 0.0 &&
         std::isfinite(seed->footprint.upper_extent_m) &&
         seed->footprint.upper_extent_m >= 0.0 &&
         std::isfinite(seed->contact_tolerance_m) && seed->contact_tolerance_m >= 0.0 &&
         std::ranges::all_of(seed->departure_chain, [](const Point3& pose) noexcept {
           return swept_footprint_detail::finitePoint(pose);
         });
}

} // namespace

SweptFootprintResult validateRawFootprintAt(
    const OccupancyGrid3D& occupancy, const Point3& position,
    const FootprintBodyAxis& requested_body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_seed) noexcept {
  if (!validProprioceptiveSeed(proprioceptive_seed)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, position);
  }
  return validateRawFootprintAt3D(occupancy, position, requested_body_axis, config,
                                  nullptr, proprioceptive_seed);
}

SweptFootprintResult validateRawSweptFootprint(
    const OccupancyGrid3D& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_seed) noexcept {
  if (!validProprioceptiveSeed(proprioceptive_seed)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }
  return validateRawSweptFootprint3D(occupancy, first, first_body_axis, second,
                                     second_body_axis, config, nullptr,
                                     proprioceptive_seed);
}

SweptFootprintResult validateRawFootprintAt(
    const ObservedOccupancyGrid3D& occupancy, const Point3& position,
    const FootprintBodyAxis& requested_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact,
    const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_seed) noexcept {
  if ((launch_support_contact != nullptr &&
       !launchSupportContactValid3D(*launch_support_contact)) ||
      !validProprioceptiveSeed(proprioceptive_seed)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, position);
  }
  return validateRawFootprintAt3D(occupancy, position, requested_body_axis, config,
                                  launch_support_contact, proprioceptive_seed);
}

SweptFootprintResult validateRawSweptFootprint(
    const ObservedOccupancyGrid3D& occupancy, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact,
    const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_seed) noexcept {
  if ((launch_support_contact != nullptr &&
       !launchSupportContactValid3D(*launch_support_contact)) ||
      !validProprioceptiveSeed(proprioceptive_seed)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }
  return validateRawSweptFootprint3D(occupancy, first, first_body_axis, second,
                                     second_body_axis, config, launch_support_contact,
                                     proprioceptive_seed);
}

SweptFootprintConfig
physicalBodyFootprint(const SweptFootprintConfig& footprint) noexcept {
  return physicalBody(footprint);
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
    const IndexedPointCloudView3D& obstacle_points, const Point3& position,
    const FootprintBodyAxis& body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact,
    const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_seed) noexcept {
  if ((launch_support_contact != nullptr &&
       !launchSupportContactValid3D(*launch_support_contact)) ||
      !validProprioceptiveSeed(proprioceptive_seed)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, position);
  }
  return validateRawPointCloudFootprintAt3D(
      pointsInsideExtent(obstacle_points, bodyExtent(position, body_axis, config)),
      position, body_axis, config,
      poseExemption(launch_support_contact, proprioceptive_seed, position));
}

SweptFootprintResult validateRawPointCloudSweptFootprint(
    const IndexedPointCloudView3D& all_obstacle_points, const Point3& first,
    const FootprintBodyAxis& first_body_axis, const Point3& second,
    const FootprintBodyAxis& second_body_axis, const SweptFootprintConfig& config,
    const LaunchSupportContact3D* const launch_support_contact,
    const ProprioceptiveFreeSpaceSeed3D* const proprioceptive_seed) noexcept {
  if ((launch_support_contact != nullptr &&
       !launchSupportContactValid3D(*launch_support_contact)) ||
      !validProprioceptiveSeed(proprioceptive_seed)) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }
  const auto cover = makeConservativeSweepCover3D(first, first_body_axis, second,
                                                  second_body_axis, config);
  if (!cover.valid()) {
    return makeStatusResult(SweptFootprintStatus::kInvalidInput, first);
  }
  const std::span<const Point3> obstacle_points =
      pointsInsideExtent(all_obstacle_points, sweepExtent(cover, config));
  if (obstacle_points.empty()) {
    return validRawFootprint();
  }
  const SweptFootprintResult first_result = validateRawPointCloudFootprintAt3D(
      obstacle_points, cover.first, cover.first_axis, config,
      poseExemption(launch_support_contact, proprioceptive_seed, cover.first));
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
    const SweepIntervalPositions3D interval_positions =
        sweepIntervalPositions(cover, interval_begin_ratio, ratio, interval_end_ratio);
    const SweptFootprintResult result = validateRawPointCloudFootprintAt3D(
        obstacle_points, interval_positions.positions[1U], midpoint_axis,
        inflated_config,
        ContactExemption3D{.launch_support = launch_support_contact,
                           .seed = proprioceptive_seed,
                           .positions = interval_positions.span()});
    if (!result.accepted()) {
      return result;
    }
  }
  return validateRawPointCloudFootprintAt3D(
      obstacle_points, cover.second, cover.second_axis, config,
      poseExemption(launch_support_contact, proprioceptive_seed, cover.second));
}

} // namespace drone_city_nav
