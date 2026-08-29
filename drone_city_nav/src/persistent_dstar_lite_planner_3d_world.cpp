#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ranges>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

namespace drone_city_nav::detail {
namespace {

constexpr double kGeometryTolerance{1.0e-9};

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  return std::abs(first.origin_x - second.origin_x) <= kGeometryTolerance &&
         std::abs(first.origin_y - second.origin_y) <= kGeometryTolerance &&
         std::abs(first.origin_z - second.origin_z) <= kGeometryTolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= kGeometryTolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

[[nodiscard]] bool nodeLess(const PersistentPlannerNode3D& first,
                            const PersistentPlannerNode3D& second) noexcept {
  return std::tuple{first.z, first.y, first.x} <
         std::tuple{second.z, second.y, second.x};
}

[[nodiscard]] GridIndex3D rawCellFromChunkBit(const OccupancyChunkIndex3D& chunk,
                                              const std::size_t bit_index) noexcept {
  const std::size_t layer = static_cast<std::size_t>(OccupancyGrid3D::kChunkSize) *
                            static_cast<std::size_t>(OccupancyGrid3D::kChunkSize);
  const auto local_z = static_cast<int>(bit_index / layer);
  const std::size_t within_layer = bit_index % layer;
  const auto local_y = static_cast<int>(
      within_layer / static_cast<std::size_t>(OccupancyGrid3D::kChunkSize));
  const auto local_x = static_cast<int>(
      within_layer % static_cast<std::size_t>(OccupancyGrid3D::kChunkSize));
  return GridIndex3D{
      chunk.x * OccupancyGrid3D::kChunkSize + local_x,
      chunk.y * OccupancyGrid3D::kChunkSize + local_y,
      chunk.z * OccupancyGrid3D::kChunkSize + local_z,
  };
}

} // namespace

bool PersistentDStarLitePlanner3DImpl::sameGridGeometry(
    const GridBounds3D& bounds) const noexcept {
  return lattice_.width_ > 0 && lattice_.height_ > 0 && lattice_.depth_ > 0 &&
         sameBounds(lattice_.raw_bounds_, bounds);
}

void PersistentDStarLitePlanner3DImpl::configureGridGeometry(
    const GridBounds3D& bounds) {
  lattice_.raw_bounds_ = bounds;
  const double width_m = static_cast<double>(bounds.width_cells) * bounds.resolution_m;
  const double height_m =
      static_cast<double>(bounds.height_cells) * bounds.resolution_m;
  const double depth_m = static_cast<double>(bounds.depth_cells) * bounds.resolution_m;
  lattice_.width_ = std::max(
      1, static_cast<int>(std::floor(width_m / config_.minimum_horizontal_step_m)));
  lattice_.height_ = std::max(
      1, static_cast<int>(std::floor(height_m / config_.minimum_horizontal_step_m)));
  lattice_.depth_ = std::max(
      1, static_cast<int>(std::floor(depth_m / config_.minimum_vertical_step_m)));
}

PersistentPlannerWorldUpdate3D
PersistentDStarLitePlanner3DImpl::updateWorld(const PersistentPlannerWorld3D& world) {
  PersistentPlannerWorldUpdate3D update;
  if (!world.valid() || world.bounds() == nullptr) {
    return update;
  }
  if (!initialized_ && world_.producer_instance_id == 0U) {
    world_ = world;
    configureGridGeometry(*world.bounds());
    update.accepted = true;
    update.requires_reset = true;
    return update;
  }
  if (world_.producer_instance_id == 0U ||
      world.producer_instance_id != world_.producer_instance_id ||
      !sameGridGeometry(*world.bounds()) || world.revision < world_.revision ||
      (world.revision == world_.revision &&
       world.occupied_fingerprint != world_.occupied_fingerprint)) {
    if (world.revision < world_.revision &&
        world.producer_instance_id == world_.producer_instance_id) {
      return update;
    }
    world_ = world;
    configureGridGeometry(*world.bounds());
    dstar_session_.edge_cost_cache_.clear();
    update.accepted = true;
    update.requires_reset = true;
    return update;
  }
  if (world.revision == world_.revision) {
    update.accepted = true;
    update.occupied_world_unchanged = true;
    world_ = world;
    return update;
  }
  if (world.full_reset) {
    world_ = world;
    dstar_session_.edge_cost_cache_.clear();
    update.accepted = true;
    update.requires_reset = true;
    return update;
  }
  if (world.occupied_fingerprint == world_.occupied_fingerprint) {
    world_ = world;
    update.accepted = true;
    update.occupied_world_unchanged = true;
    return update;
  }
  if (world.observed_occupancy == nullptr || world_.observed_occupancy == nullptr) {
    world_ = world;
    dstar_session_.edge_cost_cache_.clear();
    update.accepted = true;
    update.requires_reset = true;
    return update;
  }

  update.changed_cells = changedOccupiedCells(world_, world);
  update.occupied_cells_removed =
      std::ranges::any_of(update.changed_cells, [&](const GridIndex3D cell) {
        return world_.observed_occupancy->isOccupied(cell) &&
               !world.observed_occupancy->isOccupied(cell);
      });
  if (update.changed_cells.empty() ||
      update.changed_cells.size() > config_.maximum_incremental_changed_voxels) {
    update.changed_cells.clear();
    update.requires_reset = true;
  }
  world_ = world;
  update.accepted = true;
  return update;
}

std::vector<GridIndex3D> PersistentDStarLitePlanner3DImpl::changedOccupiedCells(
    const PersistentPlannerWorld3D& previous,
    const PersistentPlannerWorld3D& current) const {
  const ObservedOccupancyGrid3D& before = *previous.observed_occupancy;
  const ObservedOccupancyGrid3D& after = *current.observed_occupancy;
  std::vector<OccupancyChunkIndex3D> chunks = current.dirty_chunks;
  if (chunks.empty()) {
    chunks.reserve(before.chunks().size() + after.chunks().size());
    for (const auto& [index, storage] : before.chunks()) {
      static_cast<void>(storage);
      chunks.push_back(index);
    }
    for (const auto& [index, storage] : after.chunks()) {
      static_cast<void>(storage);
      chunks.push_back(index);
    }
  }
  std::ranges::sort(chunks, {}, [](const OccupancyChunkIndex3D& index) {
    return std::tuple{index.z, index.y, index.x};
  });
  chunks.erase(std::unique(chunks.begin(), chunks.end()), chunks.end());

  std::vector<GridIndex3D> changed;
  for (const OccupancyChunkIndex3D& index : chunks) {
    const ObservedOccupancyChunk3D* const before_chunk = before.findChunk(index);
    const ObservedOccupancyChunk3D* const after_chunk = after.findChunk(index);
    if (before_chunk == after_chunk) {
      continue;
    }
    for (std::size_t word_index = 0U; word_index < OccupancyGrid3D::kWordsPerChunk;
         ++word_index) {
      const std::uint64_t before_word =
          before_chunk != nullptr ? before_chunk->occupied.at(word_index) : 0U;
      const std::uint64_t after_word =
          after_chunk != nullptr ? after_chunk->occupied.at(word_index) : 0U;
      std::uint64_t difference = before_word ^ after_word;
      while (difference != 0U) {
        const int bit_offset = std::countr_zero(difference);
        const std::size_t bit_index =
            word_index * 64U + static_cast<std::size_t>(bit_offset);
        const GridIndex3D cell = rawCellFromChunkBit(index, bit_index);
        if (before.contains(cell)) {
          changed.push_back(cell);
          if (changed.size() > config_.maximum_incremental_changed_voxels) {
            return changed;
          }
        }
        difference &= difference - 1U;
      }
    }
  }
  return changed;
}

bool PersistentDStarLitePlanner3DImpl::nodeInside(
    const PersistentPlannerNode3D node) const noexcept {
  return node.x >= 0 && node.y >= 0 && node.z >= 0 && node.x < lattice_.width_ &&
         node.y < lattice_.height_ && node.z < lattice_.depth_;
}

int PersistentDStarLitePlanner3DImpl::maximumLatticeScale() const noexcept {
  return 1 << config_.maximum_adaptive_lattice_level;
}

std::size_t PersistentDStarLitePlanner3DImpl::latticeLevel(
    const PersistentPlannerNode3D first,
    const PersistentPlannerNode3D second) const noexcept {
  const int scale =
      std::max({std::abs(first.x - second.x), std::abs(first.y - second.y),
                std::abs(first.z - second.z)});
  if (scale <= 1) {
    return 0U;
  }
  return static_cast<std::size_t>(std::countr_zero(static_cast<unsigned int>(scale)));
}

Point3 PersistentDStarLitePlanner3DImpl::pointFor(
    const PersistentPlannerNode3D node) const noexcept {
  return Point3{
      lattice_.raw_bounds_.origin_x +
          (static_cast<double>(node.x) + 0.5) * config_.minimum_horizontal_step_m,
      lattice_.raw_bounds_.origin_y +
          (static_cast<double>(node.y) + 0.5) * config_.minimum_horizontal_step_m,
      lattice_.raw_bounds_.origin_z +
          (static_cast<double>(node.z) + 0.5) * config_.minimum_vertical_step_m,
  };
}

PersistentPlannerNode3D
PersistentDStarLitePlanner3DImpl::nearestNode(const Point3& point) const noexcept {
  const auto clamp_index = [](const double coordinate, const double origin,
                              const double step, const int size) {
    const int index = static_cast<int>(std::floor((coordinate - origin) / step));
    return std::clamp(index, 0, size - 1);
  };
  return PersistentPlannerNode3D{
      clamp_index(point.x, lattice_.raw_bounds_.origin_x,
                  config_.minimum_horizontal_step_m, lattice_.width_),
      clamp_index(point.y, lattice_.raw_bounds_.origin_y,
                  config_.minimum_horizontal_step_m, lattice_.height_),
      clamp_index(point.z, lattice_.raw_bounds_.origin_z,
                  config_.minimum_vertical_step_m, lattice_.depth_),
  };
}

std::optional<PersistentPlannerNode3D>
PersistentDStarLitePlanner3DImpl::selectAnchor(const Point3& point,
                                               const bool start_anchor) const {
  const PersistentPlannerNode3D center = nearestNode(point);
  const auto radius = static_cast<int>(config_.connector_search_radius_cells);
  std::vector<PersistentPlannerNode3D> candidates;
  const int diameter = 2 * radius + 1;
  const auto diameter_size = static_cast<std::size_t>(diameter);
  candidates.reserve(diameter_size * diameter_size * diameter_size);
  for (int z_offset = -radius; z_offset <= radius; ++z_offset) {
    for (int y_offset = -radius; y_offset <= radius; ++y_offset) {
      for (int x_offset = -radius; x_offset <= radius; ++x_offset) {
        const PersistentPlannerNode3D candidate{
            center.x + x_offset, center.y + y_offset, center.z + z_offset};
        if (nodeInside(candidate)) {
          candidates.push_back(candidate);
        }
      }
    }
  }
  std::ranges::sort(candidates, [&](const PersistentPlannerNode3D& first,
                                    const PersistentPlannerNode3D& second) {
    const double first_distance = distance3D(point, pointFor(first));
    const double second_distance = distance3D(point, pointFor(second));
    return first_distance == second_distance ? nodeLess(first, second)
                                             : first_distance < second_distance;
  });
  for (const PersistentPlannerNode3D candidate : candidates) {
    const Point3 anchor = pointFor(candidate);
    if (!nodeValid(candidate)) {
      continue;
    }
    const bool connector_valid = start_anchor ? departureSegmentValid(point, anchor)
                                              : rawSegmentValid(anchor, point);
    if (connector_valid) {
      return candidate;
    }
  }
  return std::nullopt;
}

bool PersistentDStarLitePlanner3DImpl::pointInsideFlightEnvelope(
    const Point3& point) const noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         evaluateFlightEnvelopeAltitude(point.z, config_.flight_envelope) ==
             FlightEnvelopeStatus::kValid;
}

bool PersistentDStarLitePlanner3DImpl::rawSegmentValid(const Point3& first,
                                                       const Point3& second) const {
  if (!pointInsideFlightEnvelope(first) || !pointInsideFlightEnvelope(second)) {
    return false;
  }
  if (world_.observed_occupancy != nullptr) {
    // The resident graph represents persistent raw occupancy only. A moving
    // proprioceptive seed and launch support are local execution evidence; if
    // they changed graph edge costs, every pose refresh would invalidate the
    // complete backward search and its edge cache.
    return validateObservedSweptFootprint(
               *world_.observed_occupancy, first, FootprintBodyAxis{}, second,
               FootprintBodyAxis{}, config_.physical_footprint,
               ObservedSpaceValidationPolicy::kAllowUnknown)
        .accepted();
  }
  return world_.static_occupancy != nullptr &&
         validateKnownStaticSweptFootprint(
             *world_.static_occupancy, first, FootprintBodyAxis{}, second,
             FootprintBodyAxis{}, config_.physical_footprint)
             .accepted();
}

bool PersistentDStarLitePlanner3DImpl::departureSegmentValid(
    const Point3& first, const Point3& second) const {
  if (!pointInsideFlightEnvelope(first) || !pointInsideFlightEnvelope(second)) {
    return false;
  }
  if (world_.observed_occupancy != nullptr) {
    const ProprioceptiveFreeSpaceSeed3D* const seed =
        world_.proprioceptive_free_space_seed.has_value()
            ? std::addressof(*world_.proprioceptive_free_space_seed)
            : nullptr;
    const LaunchSupportContact3D* const support =
        world_.launch_support_contact.has_value()
            ? std::addressof(*world_.launch_support_contact)
            : nullptr;
    return validateObservedSweptFootprint(
               *world_.observed_occupancy, first, FootprintBodyAxis{}, second,
               FootprintBodyAxis{}, config_.physical_footprint,
               ObservedSpaceValidationPolicy::kAllowUnknown, seed, support)
        .accepted();
  }
  return world_.static_occupancy != nullptr &&
         validateKnownStaticSweptFootprint(
             *world_.static_occupancy, first, FootprintBodyAxis{}, second,
             FootprintBodyAxis{}, config_.physical_footprint)
             .accepted();
}

bool PersistentDStarLitePlanner3DImpl::nodeValid(
    const PersistentPlannerNode3D node) const {
  return nodeInside(node) && rawSegmentValid(pointFor(node), pointFor(node));
}

} // namespace drone_city_nav::detail
