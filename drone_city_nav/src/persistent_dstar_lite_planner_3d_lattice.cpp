#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/raw_occupancy_clearance_3d.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"
#include "persistent_dstar_lite_planner_3d_lattice_level_zero.hpp"

namespace drone_city_nav::detail {
namespace {

// Fractions of a lattice step a placed node is probed at, nearest first.
constexpr std::array<double, 2> kNodePlacementFractions{0.25, 0.45};

} // namespace

bool PlannerLattice3D::sameGridGeometry(const GridBounds3D& bounds) const noexcept {
  return width_ > 0 && height_ > 0 && depth_ > 0 && sameBounds(raw_bounds_, bounds);
}

void PlannerLattice3D::configureGridGeometry(const GridBounds3D& bounds) {
  raw_bounds_ = bounds;
  ensureChunkTables(bounds);
  const double width_m = static_cast<double>(bounds.width_cells) * bounds.resolution_m;
  const double height_m =
      static_cast<double>(bounds.height_cells) * bounds.resolution_m;
  const double depth_m = static_cast<double>(bounds.depth_cells) * bounds.resolution_m;
  width_ = std::max(
      1, static_cast<int>(std::floor(width_m / config_->minimum_horizontal_step_m)));
  height_ = std::max(
      1, static_cast<int>(std::floor(height_m / config_->minimum_horizontal_step_m)));
  depth_ = std::max(
      1, static_cast<int>(std::floor(depth_m / config_->minimum_vertical_step_m)));
  // An adaptive edge can be affected by any occupied change within the body
  // margin of its extent, quantized by the raw voxel half diagonal.
  const double raw_half_diagonal = 0.5 * std::numbers::sqrt3 * bounds.resolution_m;
  adaptive_edge_horizontal_margin_m_ =
      config_->physical_footprint.radius_m + raw_half_diagonal;
  adaptive_edge_vertical_margin_m_ =
      std::max(config_->physical_footprint.lower_extent_m,
               config_->physical_footprint.upper_extent_m) +
      raw_half_diagonal;
  level_zero_edge_states_.assign(nodeCount(), 0U);
  const PersistentPlannerNode3D origin{1, 1, 1};
  for (std::size_t direction = 0U; direction < kLevelZeroDirections; ++direction) {
    const LevelZeroOffset3D offset = kLevelZeroOffsets[direction];
    level_zero_edge_time_s_[direction] = minimumFlightTranslationTime3D(
        pointFor(origin),
        pointFor(PersistentPlannerNode3D{origin.x + offset.x, origin.y + offset.y,
                                         origin.z + offset.z}),
        config_->time_model);
  }
}

std::size_t PlannerLattice3D::nodeCount() const noexcept {
  return static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_) *
         static_cast<std::size_t>(depth_);
}

std::size_t
PlannerLattice3D::linearIndex(const PersistentPlannerNode3D node) const noexcept {
  return (static_cast<std::size_t>(node.z) * static_cast<std::size_t>(height_) +
          static_cast<std::size_t>(node.y)) *
             static_cast<std::size_t>(width_) +
         static_cast<std::size_t>(node.x);
}

PersistentPlannerNode3D
PlannerLattice3D::nodeAt(const std::size_t index) const noexcept {
  const auto width = static_cast<std::size_t>(width_);
  const auto height = static_cast<std::size_t>(height_);
  return PersistentPlannerNode3D{
      static_cast<int>(index % width),
      static_cast<int>((index / width) % height),
      static_cast<int>(index / (width * height)),
  };
}

PlannerLattice3D::LevelZeroSlot PlannerLattice3D::levelZeroSlot(
    const PersistentPlannerEdge3D& canonical) const noexcept {
  return LevelZeroSlot{
      .node = linearIndex(canonical.first),
      .direction = levelZeroDirection(canonical.second.x - canonical.first.x,
                                      canonical.second.y - canonical.first.y,
                                      canonical.second.z - canonical.first.z),
  };
}

unsigned PlannerLattice3D::levelZeroState(const LevelZeroSlot slot) const noexcept {
  return (level_zero_edge_states_[slot.node] >> (2U * slot.direction)) & 3U;
}

void PlannerLattice3D::setLevelZeroState(const LevelZeroSlot slot,
                                         const unsigned state) noexcept {
  std::uint32_t& word = level_zero_edge_states_[slot.node];
  word = (word & ~(3U << (2U * slot.direction))) | (state << (2U * slot.direction));
}

double
PlannerLattice3D::clearanceRequiredForSegment(const Point3& first,
                                              const Point3& second) const noexcept {
  const double body_extent_m = std::max({config_->physical_footprint.radius_m,
                                         config_->physical_footprint.lower_extent_m,
                                         config_->physical_footprint.upper_extent_m});
  const double raw_half_diagonal = 0.5 * std::numbers::sqrt3 * raw_bounds_.resolution_m;
  return 0.5 * distance3D(first, second) + body_extent_m + raw_half_diagonal;
}

bool PlannerLattice3D::connectorClearsByClearance(const PersistentPlannerNode3D node,
                                                  const Point3& target,
                                                  const double target_clearance_m) {
  const double cap_m = config_->clearance_ranking_distance_m;
  if (!(config_->clearance_ranking_weight > 0.0) || !(cap_m > 0.0)) {
    return false;
  }
  const double required_m = clearanceRequiredForSegment(pointFor(node), target);
  if (required_m >= cap_m || !(target_clearance_m > required_m)) {
    return false;
  }
  const std::optional<double> node_clearance =
      cachedNodeClearance(node, required_m + 1.0e-6);
  return node_clearance.has_value() && *node_clearance > required_m;
}

bool PlannerLattice3D::endpointClearanceClears(const PersistentPlannerNode3D first,
                                               const PersistentPlannerNode3D second) {
  const double cap_m = config_->clearance_ranking_distance_m;
  if (!(config_->clearance_ranking_weight > 0.0) || !(cap_m > 0.0)) {
    return false;
  }
  const double required_m =
      clearanceRequiredForSegment(pointFor(first), pointFor(second));
  if (required_m >= cap_m) {
    return false;
  }
  // Only clearances already derived for ranking are consulted: deriving one
  // costs more than the swept validation it would save, so a search through
  // an unpriced region sweeps its edges and the ranking prices them later.
  const std::optional<double> first_clearance =
      cachedNodeClearance(first, required_m + 1.0e-6);
  const std::optional<double> second_clearance =
      cachedNodeClearance(second, required_m + 1.0e-6);
  return first_clearance.has_value() && second_clearance.has_value() &&
         *first_clearance > required_m && *second_clearance > required_m;
}

bool PlannerLattice3D::surroundingsUnoccupied(
    const PersistentPlannerNode3D node) const noexcept {
  if (!resident_collision_oracle_.has_value()) {
    return false;
  }
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  const OccupiedCollisionWorld3D& world = resident_collision_oracle_->world();
  const Point3 center = pointFor(node);
  const double horizontal_reach_m =
      config_->minimum_horizontal_step_m + adaptive_edge_horizontal_margin_m_;
  const double vertical_reach_m =
      config_->minimum_vertical_step_m + adaptive_edge_vertical_margin_m_;
  const double resolution_m = raw_bounds_.resolution_m;
  const auto chunk_of = [&](const double coordinate, const double origin) {
    return static_cast<int>(
        std::floor(std::floor((coordinate - origin) / resolution_m) / kChunkSize));
  };
  const int minimum_x = chunk_of(center.x - horizontal_reach_m, raw_bounds_.origin_x);
  const int maximum_x = chunk_of(center.x + horizontal_reach_m, raw_bounds_.origin_x);
  const int minimum_y = chunk_of(center.y - horizontal_reach_m, raw_bounds_.origin_y);
  const int maximum_y = chunk_of(center.y + horizontal_reach_m, raw_bounds_.origin_y);
  const int minimum_z = chunk_of(center.z - vertical_reach_m, raw_bounds_.origin_z);
  const int maximum_z = chunk_of(center.z + vertical_reach_m, raw_bounds_.origin_z);
  const auto unoccupied = [&](const auto& occupancy) {
    for (int z = minimum_z; z <= maximum_z; ++z) {
      for (int y = minimum_y; y <= maximum_y; ++y) {
        for (int x = minimum_x; x <= maximum_x; ++x) {
          if (occupancy.findChunk(OccupancyChunkIndex3D{x, y, z}) != nullptr) {
            return false;
          }
        }
      }
    }
    return true;
  };
  return (world.observed_occupancy == nullptr ||
          unoccupied(*world.observed_occupancy)) &&
         (world.static_occupancy == nullptr || unoccupied(*world.static_occupancy));
}

void PlannerLattice3D::markSurroundingLevelZeroEdgesClear(
    const PersistentPlannerNode3D node) noexcept {
  if (!pointInsideFlightEnvelope(pointFor(node))) {
    return;
  }
  for (int z_offset = -1; z_offset <= 1; ++z_offset) {
    for (int y_offset = -1; y_offset <= 1; ++y_offset) {
      for (int x_offset = -1; x_offset <= 1; ++x_offset) {
        if (x_offset == 0 && y_offset == 0 && z_offset == 0) {
          continue;
        }
        const PersistentPlannerNode3D neighbor{node.x + x_offset, node.y + y_offset,
                                               node.z + z_offset};
        if (!nodeInside(neighbor) || !pointInsideFlightEnvelope(pointFor(neighbor))) {
          continue;
        }
        setLevelZeroState(levelZeroSlot(canonicalEdge(node, neighbor)),
                          kLevelZeroEdgeClear);
      }
    }
  }
}

void PlannerLattice3D::ensureChunkTables(const GridBounds3D& bounds) {
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  const int columns = (bounds.width_cells + kChunkSize - 1) / kChunkSize;
  const int rows = (bounds.height_cells + kChunkSize - 1) / kChunkSize;
  const int layers = (bounds.depth_cells + kChunkSize - 1) / kChunkSize;
  const std::size_t chunk_count = static_cast<std::size_t>(std::max(columns, 0)) *
                                  static_cast<std::size_t>(std::max(rows, 0)) *
                                  static_cast<std::size_t>(std::max(layers, 0));
  if (columns != chunk_columns_ || rows != chunk_rows_ || layers != chunk_layers_ ||
      chunk_change_epoch_.size() != chunk_count) {
    chunk_columns_ = columns;
    chunk_rows_ = rows;
    chunk_layers_ = layers;
    chunk_change_epoch_.assign(chunk_count, 0U);
    chunk_occupied_ring_.assign(chunk_count, 0U);
  }
  const double chunk_span_m = kChunkSize * bounds.resolution_m;
  const double reach_m = std::max(config_->clearance_ranking_distance_m,
                                  config_->feasibility_clearance_ranking_distance_m);
  near_occupied_chunk_radius_ =
      chunk_span_m > 0.0 ? static_cast<int>(std::ceil(reach_m / chunk_span_m)) : 0;
}

void PlannerLattice3D::installWorld(const PersistentPlannerWorld3D& world) {
  if (world.observed_occupancy != nullptr) {
    ensureChunkTables(world.observed_occupancy->bounds());
  } else if (world.static_occupancy != nullptr) {
    ensureChunkTables(world.static_occupancy->bounds());
  }
  if (world.static_occupancy != nullptr) {
    // The static grid enumerates no chunks: every chunk ranks with a query.
    std::ranges::fill(chunk_occupied_ring_, 1U);
  } else if (world.observed_occupancy != nullptr) {
    for (const auto& [chunk, storage] : world.observed_occupancy->chunks()) {
      const bool occupied = std::ranges::any_of(
          storage->occupied, [](const std::uint64_t word) { return word != 0U; });
      if (occupied) {
        markNearOccupied(chunk);
      }
    }
  }
  resident_collision_oracle_.emplace(OccupiedCollisionWorld3D{
      .observed_occupancy = world.observed_occupancy.get(),
      .static_occupancy = world.static_occupancy.get(),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = config_->physical_footprint,
      .flight_envelope = config_->flight_envelope,
  });
  clearance_field_ = world.observed_clearance_field;
  clearance_field_contact_ = world.launch_support_contact
                                 ? std::addressof(*world.launch_support_contact)
                                 : nullptr;
  installDepartureEvidence(world);
}

void PlannerLattice3D::installDepartureEvidence(const PersistentPlannerWorld3D& world) {
  // Departure evidence is local to the vehicle's pose and refreshes with every
  // request; the world owning it must outlive this oracle.
  departure_collision_oracle_.emplace(OccupiedCollisionWorld3D{
      .observed_occupancy = world.observed_occupancy.get(),
      .static_occupancy = world.static_occupancy.get(),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = world.launch_support_contact
                                    ? std::addressof(*world.launch_support_contact)
                                    : nullptr,
      .proprioceptive_free_space_seed =
          world.proprioceptive_free_space_seed
              ? std::addressof(*world.proprioceptive_free_space_seed)
              : nullptr,
      .footprint = config_->departure_footprint.value_or(config_->physical_footprint),
      .flight_envelope = config_->flight_envelope,
  });
  departure_hull_collision_oracle_.emplace(OccupiedCollisionWorld3D{
      .observed_occupancy = world.observed_occupancy.get(),
      .static_occupancy = world.static_occupancy.get(),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = world.launch_support_contact
                                    ? std::addressof(*world.launch_support_contact)
                                    : nullptr,
      .proprioceptive_free_space_seed =
          world.proprioceptive_free_space_seed
              ? std::addressof(*world.proprioceptive_free_space_seed)
              : nullptr,
      .footprint = physicalBodyFootprint(
          config_->departure_footprint.value_or(config_->physical_footprint)),
      .flight_envelope = config_->flight_envelope,
  });
}

bool PlannerLattice3D::nodeInside(const PersistentPlannerNode3D node) const noexcept {
  return node.x >= 0 && node.y >= 0 && node.z >= 0 && node.x < width_ &&
         node.y < height_ && node.z < depth_;
}

int PlannerLattice3D::maximumScale() const noexcept {
  return 1 << config_->maximum_adaptive_lattice_level;
}

std::size_t
PlannerLattice3D::level(const PersistentPlannerNode3D first,
                        const PersistentPlannerNode3D second) const noexcept {
  const int scale =
      std::max({std::abs(first.x - second.x), std::abs(first.y - second.y),
                std::abs(first.z - second.z)});
  if (scale <= 1) {
    return 0U;
  }
  return static_cast<std::size_t>(std::countr_zero(static_cast<unsigned int>(scale)));
}

double PlannerLattice3D::maximumNodePlacementOffsetM() const noexcept {
  const double fraction = *std::ranges::max_element(kNodePlacementFractions);
  return fraction *
         std::max(config_->minimum_horizontal_step_m, config_->minimum_vertical_step_m);
}

bool PlannerLattice3D::nodeDisplaced(
    const PersistentPlannerNode3D node) const noexcept {
  if (node_placements_.empty() || !nodeInside(node)) {
    return false;
  }
  const NodePlacement& placed = node_placements_[linearIndex(node)];
  return placed.x != 0.0F || placed.y != 0.0F || placed.z != 0.0F;
}

Point3 PlannerLattice3D::pointFor(const PersistentPlannerNode3D node) const noexcept {
  const Point3 canonical = canonicalPointFor(node);
  if (node_placements_.empty() || !nodeInside(node)) {
    return canonical;
  }
  const NodePlacement& placed = node_placements_[linearIndex(node)];
  return Point3{canonical.x + placed.x, canonical.y + placed.y, canonical.z + placed.z};
}

void PlannerLattice3D::placeNode(const PersistentPlannerNode3D node) const {
  if (!nodeInside(node) || !resident_collision_oracle_.has_value()) {
    return;
  }
  if (node_placements_.size() != nodeCount()) {
    node_placements_.assign(nodeCount(), NodePlacement{});
  }
  NodePlacement& placement = node_placements_[linearIndex(node)];
  if (placement.placed) {
    return;
  }
  const Point3 canonical = canonicalPointFor(node);
  const auto stands = [&](const Point3& point) {
    return pointInsideFlightEnvelope(point) &&
           resident_collision_oracle_->validatePoint(point).clear();
  };
  Vec3 offset{};
  if (!stands(canonical)) {
    // Nearest first, horizontally before vertically: the body is wider than
    // it is tall, and a passage is missed across the lattice, not along it.
    // Half a step keeps every placed node inside its own lattice cell.
    const double horizontal_m = config_->minimum_horizontal_step_m;
    const double vertical_m = config_->minimum_vertical_step_m;
    const std::array<double, 2>& fractions = kNodePlacementFractions;
    const double diagonal = std::numbers::sqrt2 / 2.0;
    const std::array<Vec3, 8> horizontal_axes{Vec3{1.0, 0.0, 0.0},
                                              Vec3{-1.0, 0.0, 0.0},
                                              Vec3{0.0, 1.0, 0.0},
                                              Vec3{0.0, -1.0, 0.0},
                                              Vec3{diagonal, diagonal, 0.0},
                                              Vec3{-diagonal, diagonal, 0.0},
                                              Vec3{diagonal, -diagonal, 0.0},
                                              Vec3{-diagonal, -diagonal, 0.0}};
    bool placed{false};
    for (const double fraction : fractions) {
      for (const Vec3& axis : horizontal_axes) {
        const Vec3 candidate{fraction * horizontal_m * axis.x,
                             fraction * horizontal_m * axis.y, 0.0};
        if (stands(Point3{canonical.x + candidate.x, canonical.y + candidate.y,
                          canonical.z})) {
          offset = candidate;
          placed = true;
          break;
        }
      }
      if (placed) {
        break;
      }
      for (const double sign : {1.0, -1.0}) {
        const Vec3 candidate{0.0, 0.0, sign * fraction * vertical_m};
        if (stands(Point3{canonical.x, canonical.y, canonical.z + candidate.z})) {
          offset = candidate;
          placed = true;
          break;
        }
      }
      if (placed) {
        break;
      }
    }
  }
  placement = NodePlacement{.x = static_cast<float>(offset.x),
                            .y = static_cast<float>(offset.y),
                            .z = static_cast<float>(offset.z),
                            .placed = true};
}

void PlannerLattice3D::forgetPlacementsNear(
    const std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash>&
        changed_chunks) const {
  if (changed_chunks.empty() || node_placements_.empty()) {
    return;
  }
  // A placed node stands within half a step of its cell and its body reaches
  // less than a chunk beyond that; a change in its chunk or the chunks around
  // it can move it, and the placement is derived again when the node is next
  // consulted.
  const double chunk_span_m =
      static_cast<double>(OccupancyGrid3D::kChunkSize) * raw_bounds_.resolution_m;
  for (const OccupancyChunkIndex3D& chunk : changed_chunks) {
    const Point3 low{raw_bounds_.origin_x + (chunk.x - 1) * chunk_span_m,
                     raw_bounds_.origin_y + (chunk.y - 1) * chunk_span_m,
                     raw_bounds_.origin_z + (chunk.z - 1) * chunk_span_m};
    const Point3 high{raw_bounds_.origin_x + (chunk.x + 2) * chunk_span_m,
                      raw_bounds_.origin_y + (chunk.y + 2) * chunk_span_m,
                      raw_bounds_.origin_z + (chunk.z + 2) * chunk_span_m};
    const PersistentPlannerNode3D first = nearestNode(low);
    const PersistentPlannerNode3D last = nearestNode(high);
    for (int z = first.z; z <= last.z; ++z) {
      for (int y = first.y; y <= last.y; ++y) {
        for (int x = first.x; x <= last.x; ++x) {
          node_placements_[linearIndex(PersistentPlannerNode3D{x, y, z})].placed =
              false;
        }
      }
    }
  }
}

Point3
PlannerLattice3D::canonicalPointFor(const PersistentPlannerNode3D node) const noexcept {
  return Point3{
      raw_bounds_.origin_x +
          (static_cast<double>(node.x) + 0.5) * config_->minimum_horizontal_step_m,
      raw_bounds_.origin_y +
          (static_cast<double>(node.y) + 0.5) * config_->minimum_horizontal_step_m,
      raw_bounds_.origin_z +
          (static_cast<double>(node.z) + 0.5) * config_->minimum_vertical_step_m,
  };
}

PersistentPlannerNode3D
PlannerLattice3D::nearestNode(const Point3& point) const noexcept {
  const auto clamp_index = [](const double coordinate, const double origin,
                              const double step, const int size) {
    const int index = static_cast<int>(std::floor((coordinate - origin) / step));
    return std::clamp(index, 0, size - 1);
  };
  return PersistentPlannerNode3D{
      clamp_index(point.x, raw_bounds_.origin_x, config_->minimum_horizontal_step_m,
                  width_),
      clamp_index(point.y, raw_bounds_.origin_y, config_->minimum_horizontal_step_m,
                  height_),
      clamp_index(point.z, raw_bounds_.origin_z, config_->minimum_vertical_step_m,
                  depth_),
  };
}

} // namespace drone_city_nav::detail
