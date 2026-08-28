#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool sameResolution(const GridBounds3D& first,
                                  const GridBounds3D& second) noexcept {
  return std::abs(first.resolution_m - second.resolution_m) <= 1.0e-9;
}

[[nodiscard]] int clampedCell(const double coordinate, const double origin,
                              const double resolution_m,
                              const int cell_count) noexcept {
  return std::clamp(static_cast<int>(std::floor((coordinate - origin) / resolution_m)),
                    0, cell_count - 1);
}

struct LaunchSupportCellCandidate {
  GridIndex3D index{};
  AxisAlignedBox3D bounds{};
};

[[nodiscard]] bool launchSupportEnvelopeIntersectsCell(
    const ProprioceptiveFreeSpaceSeed3D& seed, const double resolution_m,
    const Point3& cell_minimum, const Point3& cell_maximum) noexcept {
  const double axis_norm =
      std::hypot(std::hypot(seed.body_axis.x, seed.body_axis.y), seed.body_axis.z);
  if (!(axis_norm > 1.0e-9) || !(resolution_m > 0.0)) {
    return false;
  }
  const FootprintBodyAxis axis{seed.body_axis.x / axis_norm,
                               seed.body_axis.y / axis_norm,
                               seed.body_axis.z / axis_norm};
  const double lower_extent_m = std::max(0.0, seed.footprint.lower_extent_m);
  const double maximum_axial_extent_m =
      std::max(lower_extent_m, std::max(0.0, seed.footprint.upper_extent_m));
  const Point3 contact_center{seed.position.x - lower_extent_m * axis.x,
                              seed.position.y - lower_extent_m * axis.y,
                              seed.position.z - lower_extent_m * axis.z};
  const SweptFootprintConfig contact_envelope{
      .radius_m =
          std::hypot(std::max(0.0, seed.footprint.radius_m), maximum_axial_extent_m) +
          resolution_m,
      .lower_extent_m = resolution_m,
      .upper_extent_m = resolution_m,
  };
  return footprintIntersectsAxisAlignedBox(contact_center, axis, contact_envelope,
                                           cell_minimum, cell_maximum);
}

[[nodiscard]] std::vector<LaunchSupportCellCandidate>
launchSupportCellCandidates(const GridBounds3D& bounds,
                            const ProprioceptiveFreeSpaceSeed3D& seed) {
  const double broad_extent_m = std::max(0.0, seed.footprint.radius_m) +
                                std::max(std::max(0.0, seed.footprint.lower_extent_m),
                                         std::max(0.0, seed.footprint.upper_extent_m)) +
                                bounds.resolution_m;
  const int minimum_x = clampedCell(seed.position.x - broad_extent_m, bounds.origin_x,
                                    bounds.resolution_m, bounds.width_cells);
  const int maximum_x = clampedCell(seed.position.x + broad_extent_m, bounds.origin_x,
                                    bounds.resolution_m, bounds.width_cells);
  const int minimum_y = clampedCell(seed.position.y - broad_extent_m, bounds.origin_y,
                                    bounds.resolution_m, bounds.height_cells);
  const int maximum_y = clampedCell(seed.position.y + broad_extent_m, bounds.origin_y,
                                    bounds.resolution_m, bounds.height_cells);
  const int minimum_z = clampedCell(seed.position.z - broad_extent_m, bounds.origin_z,
                                    bounds.resolution_m, bounds.depth_cells);
  const int maximum_z = clampedCell(seed.position.z + broad_extent_m, bounds.origin_z,
                                    bounds.resolution_m, bounds.depth_cells);

  std::vector<LaunchSupportCellCandidate> result;
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        const Point3 minimum{
            bounds.origin_x + static_cast<double>(x) * bounds.resolution_m,
            bounds.origin_y + static_cast<double>(y) * bounds.resolution_m,
            bounds.origin_z + static_cast<double>(z) * bounds.resolution_m};
        const Point3 maximum{minimum.x + bounds.resolution_m,
                             minimum.y + bounds.resolution_m,
                             minimum.z + bounds.resolution_m};
        if (launchSupportEnvelopeIntersectsCell(seed, bounds.resolution_m, minimum,
                                                maximum)) {
          result.push_back(LaunchSupportCellCandidate{
              .index = GridIndex3D{x, y, z},
              .bounds = AxisAlignedBox3D{.minimum = minimum, .maximum = maximum},
          });
        }
      }
    }
  }
  return result;
}

[[nodiscard]] LaunchSupportContact3D
makeLaunchSupportContact3D(const GridBounds3D& bounds,
                           const ProprioceptiveFreeSpaceSeed3D& seed,
                           const LaunchSupportEvidenceSource evidence_source,
                           const std::size_t occupied_evidence_cells,
                           const std::vector<LaunchSupportCellCandidate>& candidates) {
  LaunchSupportContact3D contact{
      .seed = seed,
      .contact_cells = {},
      .occupied_evidence_cells = occupied_evidence_cells,
      .evidence_source = evidence_source,
      .maximum_lateral_departure_m = bounds.resolution_m,
      .minimum_axial_departure_m = 0.0,
      .maximum_axial_settling_m = bounds.resolution_m,
  };
  contact.contact_cells.reserve(candidates.size());
  std::ranges::transform(
      candidates, std::back_inserter(contact.contact_cells),
      [](const LaunchSupportCellCandidate& candidate) { return candidate.bounds; });
  return contact;
}

} // namespace

GridBounds3D selectLocalObservedEsdfBounds(const GridBounds3D& world_bounds,
                                           const Point3& position,
                                           const LocalObservedEsdfWindow3D& window) {
  if (!(world_bounds.resolution_m > 0.0) || world_bounds.width_cells <= 0 ||
      world_bounds.height_cells <= 0 || world_bounds.depth_cells <= 0 ||
      !localObservedEsdfWindow3DIsValid(window) || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(position.z)) {
    throw std::invalid_argument{"invalid local observed ESDF bounds request"};
  }
  const int minimum_x =
      clampedCell(position.x - window.horizontal_half_extent_m, world_bounds.origin_x,
                  world_bounds.resolution_m, world_bounds.width_cells);
  const int maximum_x =
      clampedCell(position.x + window.horizontal_half_extent_m, world_bounds.origin_x,
                  world_bounds.resolution_m, world_bounds.width_cells);
  const int minimum_y =
      clampedCell(position.y - window.horizontal_half_extent_m, world_bounds.origin_y,
                  world_bounds.resolution_m, world_bounds.height_cells);
  const int maximum_y =
      clampedCell(position.y + window.horizontal_half_extent_m, world_bounds.origin_y,
                  world_bounds.resolution_m, world_bounds.height_cells);
  const int minimum_z =
      clampedCell(position.z - window.vertical_half_extent_m, world_bounds.origin_z,
                  world_bounds.resolution_m, world_bounds.depth_cells);
  const int maximum_z =
      clampedCell(position.z + window.vertical_half_extent_m, world_bounds.origin_z,
                  world_bounds.resolution_m, world_bounds.depth_cells);
  return GridBounds3D{
      .origin_x = world_bounds.origin_x +
                  static_cast<double>(minimum_x) * world_bounds.resolution_m,
      .origin_y = world_bounds.origin_y +
                  static_cast<double>(minimum_y) * world_bounds.resolution_m,
      .origin_z = world_bounds.origin_z +
                  static_cast<double>(minimum_z) * world_bounds.resolution_m,
      .resolution_m = world_bounds.resolution_m,
      .width_cells = maximum_x - minimum_x + 1,
      .height_cells = maximum_y - minimum_y + 1,
      .depth_cells = maximum_z - minimum_z + 1,
  };
}

bool localObservedEsdfNeedsRecenter(const GridBounds3D& local_bounds,
                                    const GridBounds3D& world_bounds,
                                    const Point3& position,
                                    const LocalObservedEsdfWindow3D& window) noexcept {
  if (!sameResolution(local_bounds, world_bounds) ||
      !localObservedEsdfWindow3DIsValid(window) || !std::isfinite(position.x) ||
      !std::isfinite(position.y) || !std::isfinite(position.z)) {
    return true;
  }
  const double local_maximum_x =
      local_bounds.origin_x + local_bounds.width_cells * local_bounds.resolution_m;
  const double local_maximum_y =
      local_bounds.origin_y + local_bounds.height_cells * local_bounds.resolution_m;
  const double local_maximum_z =
      local_bounds.origin_z + local_bounds.depth_cells * local_bounds.resolution_m;
  const double world_maximum_x =
      world_bounds.origin_x + world_bounds.width_cells * world_bounds.resolution_m;
  const double world_maximum_y =
      world_bounds.origin_y + world_bounds.height_cells * world_bounds.resolution_m;
  const double world_maximum_z =
      world_bounds.origin_z + world_bounds.depth_cells * world_bounds.resolution_m;
  const bool room_left = local_bounds.origin_x > world_bounds.origin_x + 1.0e-9;
  const bool room_right = local_maximum_x < world_maximum_x - 1.0e-9;
  const bool room_down = local_bounds.origin_y > world_bounds.origin_y + 1.0e-9;
  const bool room_up = local_maximum_y < world_maximum_y - 1.0e-9;
  const bool room_below = local_bounds.origin_z > world_bounds.origin_z + 1.0e-9;
  const bool room_above = local_maximum_z < world_maximum_z - 1.0e-9;
  return (room_left &&
          position.x - local_bounds.origin_x < window.horizontal_recenter_margin_m) ||
         (room_right &&
          local_maximum_x - position.x < window.horizontal_recenter_margin_m) ||
         (room_down &&
          position.y - local_bounds.origin_y < window.horizontal_recenter_margin_m) ||
         (room_up &&
          local_maximum_y - position.y < window.horizontal_recenter_margin_m) ||
         (room_below &&
          position.z - local_bounds.origin_z < window.vertical_recenter_margin_m) ||
         (room_above &&
          local_maximum_z - position.z < window.vertical_recenter_margin_m);
}

bool localObservedEsdfWindow3DIsValid(
    const LocalObservedEsdfWindow3D& window) noexcept {
  return std::isfinite(window.horizontal_half_extent_m) &&
         window.horizontal_half_extent_m > 0.0 &&
         std::isfinite(window.vertical_half_extent_m) &&
         window.vertical_half_extent_m > 0.0 &&
         std::isfinite(window.horizontal_recenter_margin_m) &&
         window.horizontal_recenter_margin_m >= 0.0 &&
         window.horizontal_recenter_margin_m < window.horizontal_half_extent_m &&
         std::isfinite(window.vertical_recenter_margin_m) &&
         window.vertical_recenter_margin_m >= 0.0 &&
         window.vertical_recenter_margin_m < window.vertical_half_extent_m;
}

std::optional<LaunchSupportContact3D>
detectLaunchSupportContact3D(const ObservedOccupancyGrid3D& occupancy,
                             const ProprioceptiveFreeSpaceSeed3D& seed) {
  const GridBounds3D& bounds = occupancy.bounds();
  const std::vector<LaunchSupportCellCandidate> candidates =
      launchSupportCellCandidates(bounds, seed);
  const std::size_t occupied_evidence_cells = static_cast<std::size_t>(
      std::ranges::count_if(candidates, [&](const LaunchSupportCellCandidate& cell) {
        return occupancy.state(cell.index) == ObservedVoxelState::kOccupied;
      }));
  if (occupied_evidence_cells == 0U) {
    return std::nullopt;
  }
  return makeLaunchSupportContact3D(bounds, seed,
                                    LaunchSupportEvidenceSource::kObservedOccupancy,
                                    occupied_evidence_cells, candidates);
}

LaunchSupportContact3D
makeVehicleLandedSupportContact3D(const GridBounds3D& bounds,
                                  const ProprioceptiveFreeSpaceSeed3D& seed) {
  const std::vector<LaunchSupportCellCandidate> candidates =
      launchSupportCellCandidates(bounds, seed);
  return makeLaunchSupportContact3D(
      bounds, seed, LaunchSupportEvidenceSource::kVehicleLandDetector, 0U, candidates);
}

} // namespace drone_city_nav
