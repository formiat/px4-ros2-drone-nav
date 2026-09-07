#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/raw_occupancy_clearance_3d.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
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

constexpr unsigned kLevelZeroEdgeUnknown{0U};
constexpr unsigned kLevelZeroEdgeClear{1U};
constexpr unsigned kLevelZeroEdgeBlocked{2U};

struct LevelZeroOffset3D {
  int x{0};
  int y{0};
  int z{0};
};

// The thirteen offsets that are lexicographically positive in (z, y, x)
// order: exactly the second endpoints of canonical level-zero edges.
constexpr std::array<LevelZeroOffset3D, 13U> kLevelZeroOffsets{{
    {1, 0, 0},
    {-1, 1, 0},
    {0, 1, 0},
    {1, 1, 0},
    {-1, -1, 1},
    {0, -1, 1},
    {1, -1, 1},
    {-1, 0, 1},
    {0, 0, 1},
    {1, 0, 1},
    {-1, 1, 1},
    {0, 1, 1},
    {1, 1, 1},
}};

// Index of a canonical offset in kLevelZeroOffsets: the offsets enumerate the
// codes 14..26 of (z + 1) * 9 + (y + 1) * 3 + (x + 1).
[[nodiscard]] constexpr std::size_t levelZeroDirection(const int x, const int y,
                                                       const int z) noexcept {
  return static_cast<std::size_t>((z + 1) * 9 + (y + 1) * 3 + (x + 1) - 14);
}

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

bool PlannerLattice3D::endpointClearanceClears(const PersistentPlannerNode3D first,
                                               const PersistentPlannerNode3D second) {
  const double cap_m = config_->clearance_ranking_distance_m;
  if (!(config_->clearance_ranking_weight > 0.0) || !(cap_m > 0.0)) {
    return false;
  }
  const Point3 first_point = pointFor(first);
  const Point3 second_point = pointFor(second);
  const double body_extent_m = std::max({config_->physical_footprint.radius_m,
                                         config_->physical_footprint.lower_extent_m,
                                         config_->physical_footprint.upper_extent_m});
  const double raw_half_diagonal = 0.5 * std::numbers::sqrt3 * raw_bounds_.resolution_m;
  const double required_m =
      0.5 * distance3D(first_point, second_point) + body_extent_m + raw_half_diagonal;
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
      .footprint = config_->physical_footprint,
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

Point3 PlannerLattice3D::pointFor(const PersistentPlannerNode3D node) const noexcept {
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

std::vector<PersistentPlannerNode3D>
PlannerLattice3D::admissibleAnchors(const Point3& point,
                                    const bool start_anchor) const {
  const PersistentPlannerNode3D center = nearestNode(point);
  const auto radius = static_cast<int>(config_->connector_search_radius_cells);
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
  std::vector<PersistentPlannerNode3D> anchors;
  for (const PersistentPlannerNode3D candidate : candidates) {
    const Point3 anchor = pointFor(candidate);
    if (!nodeValid(candidate)) {
      continue;
    }
    const bool connector_valid = start_anchor ? departureSegmentValid(point, anchor)
                                              : rawSegmentValid(anchor, point);
    if (connector_valid) {
      anchors.push_back(candidate);
    }
  }
  return anchors;
}

std::optional<PersistentPlannerNode3D>
PlannerLattice3D::selectAnchor(const Point3& point, const bool start_anchor) const {
  const std::vector<PersistentPlannerNode3D> anchors =
      admissibleAnchors(point, start_anchor);
  return anchors.empty() ? std::nullopt
                         : std::optional<PersistentPlannerNode3D>{anchors.front()};
}

std::size_t PlannerLattice3D::departureConnectionCount(const Point3& start) const {
  return admissibleAnchors(start, true).size();
}

PlannerLattice3D::DepartureConnection3D PlannerLattice3D::selectDepartureConnection(
    const Point3& start, const std::size_t skipped_connections) const {
  DepartureConnection3D result;
  const std::vector<PersistentPlannerNode3D> anchors = admissibleAnchors(start, true);
  if (!anchors.empty()) {
    result.anchor = anchors[skipped_connections % anchors.size()];
    return result;
  }
  if (config_->departure_refinement_subdivisions == 0U) {
    return result;
  }
  // No node in the connector radius is reachable in one segment. Probe a grid
  // finer than the lattice around the vehicle for a free point it can reach,
  // and from which a node is reachable; the nearest such point wins, so the
  // detour stays as short as the geometry allows.
  const auto subdivisions =
      static_cast<int>(config_->departure_refinement_subdivisions);
  const double horizontal_step_m =
      config_->minimum_horizontal_step_m / static_cast<double>(subdivisions);
  const double vertical_step_m =
      config_->minimum_vertical_step_m / static_cast<double>(subdivisions);
  const auto radius = static_cast<int>(config_->connector_search_radius_cells);
  const int span = radius * subdivisions;
  std::vector<Point3> waypoints;
  const auto span_size = static_cast<std::size_t>(2 * span + 1);
  waypoints.reserve(span_size * span_size * span_size);
  for (int z_offset = -span; z_offset <= span; ++z_offset) {
    for (int y_offset = -span; y_offset <= span; ++y_offset) {
      for (int x_offset = -span; x_offset <= span; ++x_offset) {
        if (x_offset == 0 && y_offset == 0 && z_offset == 0) {
          continue;
        }
        const Point3 waypoint{
            start.x + static_cast<double>(x_offset) * horizontal_step_m,
            start.y + static_cast<double>(y_offset) * horizontal_step_m,
            start.z + static_cast<double>(z_offset) * vertical_step_m};
        if (pointInsideFlightEnvelope(waypoint)) {
          waypoints.push_back(waypoint);
        }
      }
    }
  }
  std::ranges::sort(waypoints, [&](const Point3& first, const Point3& second) {
    const double first_distance = distance3D(start, first);
    const double second_distance = distance3D(start, second);
    if (first_distance != second_distance) {
      return first_distance < second_distance;
    }
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  std::size_t probes{0U};
  for (const Point3& waypoint : waypoints) {
    if (probes >= config_->maximum_departure_refinement_probes) {
      break;
    }
    ++probes;
    if (!departureSegmentValid(start, waypoint)) {
      continue;
    }
    const std::optional<PersistentPlannerNode3D> anchor = selectAnchor(waypoint, false);
    if (anchor.has_value()) {
      result.anchor = anchor;
      result.waypoint = waypoint;
      return result;
    }
  }
  return result;
}

bool PlannerLattice3D::departureReachable(const Point3& start,
                                          const std::optional<Point3>& waypoint,
                                          const Point3& target) const {
  if (!waypoint.has_value()) {
    return departureSegmentValid(start, target);
  }
  return departureSegmentValid(start, *waypoint) && rawSegmentValid(*waypoint, target);
}

bool PlannerLattice3D::pointInsideFlightEnvelope(const Point3& point) const noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         evaluateFlightEnvelopeAltitude(point.z, config_->flight_envelope) ==
             FlightEnvelopeStatus::kValid;
}

bool PlannerLattice3D::rawSegmentValid(const Point3& first,
                                       const Point3& second) const {
  // The resident graph represents persistent raw occupancy only. A moving
  // proprioceptive seed and launch support are local execution evidence; if
  // they changed graph edge costs, every pose refresh would invalidate the
  // complete backward search and its edge cache.
  return resident_collision_oracle_.has_value() &&
         resident_collision_oracle_
             ->validateSegment(first, FootprintBodyAxis{}, second, FootprintBodyAxis{})
             .clear();
}

bool PlannerLattice3D::departureSegmentValid(const Point3& first,
                                             const Point3& second) const {
  return departure_collision_oracle_.has_value() &&
         departure_collision_oracle_
             ->validateSegment(first, FootprintBodyAxis{}, second, FootprintBodyAxis{})
             .clear();
}

bool PlannerLattice3D::nodeValid(const PersistentPlannerNode3D node) const {
  return nodeInside(node) && rawSegmentValid(pointFor(node), pointFor(node));
}

std::vector<PersistentPlannerNode3D>
PlannerLattice3D::adjacentNodes(const PersistentPlannerNode3D node) const {
  std::vector<PersistentPlannerNode3D> result;
  result.reserve(26U * (config_->maximum_adaptive_lattice_level + 1U));
  forEachAdjacentNode(node, [&result](const PersistentPlannerNode3D candidate) {
    result.push_back(candidate);
  });
  return result;
}

double
PlannerLattice3D::heuristic(const PersistentPlannerNode3D first,
                            const PersistentPlannerNode3D second) const noexcept {
  // Every horizontal graph edge is axial or diagonal. Its obstacle-free
  // lattice distance is therefore an admissible, consistent lower bound that
  // is strictly tighter than continuous Euclidean distance for non-45-degree
  // missions. Adaptive edges preserve the same per-metre cost.
  const double delta_x_m = static_cast<double>(std::abs(first.x - second.x)) *
                           config_->minimum_horizontal_step_m;
  const double delta_y_m = static_cast<double>(std::abs(first.y - second.y)) *
                           config_->minimum_horizontal_step_m;
  const double horizontal_diagonal_m =
      std::max(delta_x_m, delta_y_m) +
      (std::numbers::sqrt2 - 1.0) * std::min(delta_x_m, delta_y_m);
  const double vertical_m = static_cast<double>(std::abs(first.z - second.z)) *
                            config_->minimum_vertical_step_m;
  const double euclidean_m = std::hypot(std::hypot(delta_x_m, delta_y_m), vertical_m);
  return std::max(
      {horizontal_diagonal_m / config_->time_model.maximum_horizontal_speed_mps,
       vertical_m / config_->time_model.maximum_vertical_speed_mps,
       horizontal_diagonal_m / config_->time_model.maximum_translational_speed_mps,
       euclidean_m / config_->time_model.maximum_translational_speed_mps});
}

double PlannerLattice3D::rawEdgeCost(const PersistentPlannerNode3D first,
                                     const PersistentPlannerNode3D second) {
  ++edge_queries_;
  if (!nodeInside(first) || !nodeInside(second) || first == second) {
    return std::numeric_limits<double>::infinity();
  }
  const std::size_t queried_level = level(first, second);
  const PersistentPlannerEdge3D edge = canonicalEdge(first, second);
  if (queried_level == 0U) {
    const LevelZeroSlot slot = levelZeroSlot(edge);
    unsigned state = levelZeroState(slot);
    if (state == kLevelZeroEdgeUnknown) {
      // Open space first: with no raw chunk within the swept reach of the
      // canonical endpoint, every level-zero edge leaving it is clear.
      if (surroundingsUnoccupied(edge.first)) {
        markSurroundingLevelZeroEdgesClear(edge.first);
        state = levelZeroState(slot);
      }
      // Every point of the swept body lies within half the edge length plus
      // the body extent of the nearer endpoint; two endpoints whose raw
      // clearance exceeds that cannot touch occupied evidence anywhere along
      // the edge. The clearance is the cached ranking quantity, so open
      // streets price their edges without swept validation.
      if (state == kLevelZeroEdgeUnknown && endpointClearanceClears(first, second)) {
        state = kLevelZeroEdgeClear;
        setLevelZeroState(slot, state);
      }
      if (state == kLevelZeroEdgeUnknown) {
        ++raw_edge_validation_checks_;
        state = rawSegmentValid(pointFor(first), pointFor(second))
                    ? kLevelZeroEdgeClear
                    : kLevelZeroEdgeBlocked;
        setLevelZeroState(slot, state);
      }
    }
    return state == kLevelZeroEdgeClear ? level_zero_edge_time_s_[slot.direction]
                                        : std::numeric_limits<double>::infinity();
  }
  ++adaptive_edge_queries_;
  maximum_queried_level_ = std::max(maximum_queried_level_, queried_level);
  double raw_cost = std::numeric_limits<double>::infinity();
  if (const auto found = adaptive_edge_cost_cache_.find(edge);
      found != adaptive_edge_cost_cache_.end()) {
    raw_cost = found->second;
  } else {
    const Point3 first_point = pointFor(first);
    const Point3 second_point = pointFor(second);
    ++raw_edge_validation_checks_;
    raw_cost = rawSegmentValid(first_point, second_point)
                   ? minimumFlightTranslationTime3D(first_point, second_point,
                                                    config_->time_model)
                   : std::numeric_limits<double>::infinity();
    adaptive_edge_cost_cache_.emplace(edge, raw_cost);
    indexAdaptiveEdge(edge);
  }
  return raw_cost;
}

double voxelBoxDistance3D(const Point3& point, const Point3& box_minimum,
                          const Point3& box_maximum) noexcept {
  const auto gap = [](const double minimum, const double maximum,
                      const double coordinate) {
    return std::max({minimum - coordinate, 0.0, coordinate - maximum});
  };
  const double dx = gap(box_minimum.x, box_maximum.x, point.x);
  const double dy = gap(box_minimum.y, box_maximum.y, point.y);
  const double dz = gap(box_minimum.z, box_maximum.z, point.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

template<typename Visitor>
void PlannerLattice3D::forEachChunkTouching(const Point3& first, const Point3& second,
                                            Visitor&& visitor) const {
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  const double resolution_m = raw_bounds_.resolution_m;
  const auto chunk_of = [&](const double coordinate, const double origin) {
    return static_cast<int>(
        std::floor(std::floor((coordinate - origin) / resolution_m) / kChunkSize));
  };
  const double horizontal_margin_m = adaptive_edge_horizontal_margin_m_;
  const double vertical_margin_m = adaptive_edge_vertical_margin_m_;
  const int minimum_x =
      chunk_of(std::min(first.x, second.x) - horizontal_margin_m, raw_bounds_.origin_x);
  const int maximum_x =
      chunk_of(std::max(first.x, second.x) + horizontal_margin_m, raw_bounds_.origin_x);
  const int minimum_y =
      chunk_of(std::min(first.y, second.y) - horizontal_margin_m, raw_bounds_.origin_y);
  const int maximum_y =
      chunk_of(std::max(first.y, second.y) + horizontal_margin_m, raw_bounds_.origin_y);
  const int minimum_z =
      chunk_of(std::min(first.z, second.z) - vertical_margin_m, raw_bounds_.origin_z);
  const int maximum_z =
      chunk_of(std::max(first.z, second.z) + vertical_margin_m, raw_bounds_.origin_z);
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        visitor(OccupancyChunkIndex3D{x, y, z});
      }
    }
  }
}

void PlannerLattice3D::indexAdaptiveEdge(const PersistentPlannerEdge3D& edge) {
  forEachChunkTouching(pointFor(edge.first), pointFor(edge.second),
                       [&](const OccupancyChunkIndex3D chunk) {
                         adaptive_edges_by_chunk_[chunk].push_back(edge);
                       });
}

void PlannerLattice3D::unindexAdaptiveEdge(const PersistentPlannerEdge3D& edge) {
  forEachChunkTouching(pointFor(edge.first), pointFor(edge.second),
                       [&](const OccupancyChunkIndex3D chunk) {
                         const auto found = adaptive_edges_by_chunk_.find(chunk);
                         if (found == adaptive_edges_by_chunk_.end()) {
                           return;
                         }
                         std::erase(found->second, edge);
                         if (found->second.empty()) {
                           adaptive_edges_by_chunk_.erase(found);
                         }
                       });
}

std::vector<PersistentPlannerEdge3D>
PlannerLattice3D::forgetAdaptiveEdgesTouching(const LatticeChangesByChunk3D& changes,
                                              const LatticeSegmentTouch3D& touches) {
  std::vector<PersistentPlannerEdge3D> forgotten;
  for (const auto& [chunk, cells] : changes) {
    const auto found = adaptive_edges_by_chunk_.find(chunk);
    if (found == adaptive_edges_by_chunk_.end()) {
      continue;
    }
    // Forgetting unindexes the edge, so iterate a copy of the chunk's list.
    const std::vector<PersistentPlannerEdge3D> candidates = found->second;
    for (const PersistentPlannerEdge3D& edge : candidates) {
      const auto cost = adaptive_edge_cost_cache_.find(edge);
      if (cost == adaptive_edge_cost_cache_.end()) {
        continue;
      }
      const bool clear = std::isfinite(cost->second);
      const Point3 first_point = pointFor(edge.first);
      const Point3 second_point = pointFor(edge.second);
      const bool moved =
          std::ranges::any_of(cells, [&](const LatticeChangedCell3D& cell) {
            return cell.occupied_now == clear &&
                   touches(cell.center, first_point, second_point);
          });
      if (!moved) {
        continue;
      }
      adaptive_edge_cost_cache_.erase(cost);
      unindexAdaptiveEdge(edge);
      forgotten.push_back(edge);
    }
  }
  return forgotten;
}

void PlannerLattice3D::reset() noexcept {
  raw_bounds_ = {};
  width_ = 0;
  height_ = 0;
  depth_ = 0;
  level_zero_edge_states_.clear();
  chunk_change_epoch_.clear();
  chunk_occupied_ring_.clear();
  chunk_columns_ = 0;
  chunk_rows_ = 0;
  chunk_layers_ = 0;
  resetEdgeEvidence();
}

void PlannerLattice3D::resetEdgeEvidence() noexcept {
  std::ranges::fill(level_zero_edge_states_, 0U);
  adaptive_edge_cost_cache_.clear();
  resetNodeClearances();
  adaptive_edges_by_chunk_.clear();
  resetEdgeStatistics();
}

void PlannerLattice3D::resetEdgeStatistics() noexcept {
  clearances_rederived_ = 0U;
  edge_queries_ = 0U;
  raw_edge_validation_checks_ = 0U;
  adaptive_edge_queries_ = 0U;
  maximum_queried_level_ = 0U;
}

const GridBounds3D& PlannerLattice3D::bounds() const noexcept {
  return raw_bounds_;
}

bool PlannerLattice3D::edgeTraversable(const PersistentPlannerNode3D first,
                                       const PersistentPlannerNode3D second) {
  return std::isfinite(rawEdgeCost(first, second));
}

bool PlannerLattice3D::pathTraversable(const std::vector<Point3>& path) const {
  return !firstInvalidSegment(path).has_value();
}

std::optional<std::size_t>
PlannerLattice3D::firstInvalidSegment(const std::vector<Point3>& path) const {
  if (path.size() < 2U || !std::ranges::all_of(path, [&](const Point3& point) {
        return pointInsideFlightEnvelope(point);
      })) {
    return 0U;
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const bool segment_valid =
        index == 1U ? departureSegmentValid(path[index - 1U], path[index])
                    : rawSegmentValid(path[index - 1U], path[index]);
    if (!segment_valid) {
      return index;
    }
  }
  return std::nullopt;
}

std::size_t PlannerLattice3D::nodeSpan() const noexcept {
  return static_cast<std::size_t>(width_) + static_cast<std::size_t>(height_) +
         static_cast<std::size_t>(depth_);
}

bool PlannerLattice3D::forgetEdgeCost(const PersistentPlannerEdge3D& edge) {
  if (!nodeInside(edge.first) || !nodeInside(edge.second) ||
      edge.first == edge.second) {
    return false;
  }
  if (level(edge.first, edge.second) == 0U) {
    const LevelZeroSlot slot = levelZeroSlot(canonicalEdge(edge.first, edge.second));
    if (levelZeroState(slot) == kLevelZeroEdgeUnknown) {
      return false;
    }
    setLevelZeroState(slot, kLevelZeroEdgeUnknown);
    return true;
  }
  return adaptive_edge_cost_cache_.erase(edge) != 0U;
}

bool PlannerLattice3D::forgetEdgeCostForChange(const PersistentPlannerEdge3D& edge,
                                               const bool occupied_cell_added,
                                               const bool occupied_cell_removed) {
  if (!nodeInside(edge.first) || !nodeInside(edge.second) ||
      edge.first == edge.second || (!occupied_cell_added && !occupied_cell_removed)) {
    return false;
  }
  if (level(edge.first, edge.second) == 0U) {
    const LevelZeroSlot slot = levelZeroSlot(canonicalEdge(edge.first, edge.second));
    const unsigned state = levelZeroState(slot);
    const bool movable = (state == kLevelZeroEdgeClear && occupied_cell_added) ||
                         (state == kLevelZeroEdgeBlocked && occupied_cell_removed);
    if (!movable) {
      return false;
    }
    setLevelZeroState(slot, kLevelZeroEdgeUnknown);
    return true;
  }
  const auto cost = adaptive_edge_cost_cache_.find(edge);
  if (cost == adaptive_edge_cost_cache_.end()) {
    return false;
  }
  const bool clear = std::isfinite(cost->second);
  if (!((clear && occupied_cell_added) || (!clear && occupied_cell_removed))) {
    return false;
  }
  adaptive_edge_cost_cache_.erase(cost);
  unindexAdaptiveEdge(edge);
  return true;
}

bool PlannerLattice3D::forgetNodeEdgesForChange(const PersistentPlannerNode3D node,
                                                const bool occupied_cell_added,
                                                const bool occupied_cell_removed) {
  if (!nodeInside(node) || (!occupied_cell_added && !occupied_cell_removed)) {
    return false;
  }
  bool forgotten = false;
  for (int z_offset = -1; z_offset <= 1; ++z_offset) {
    for (int y_offset = -1; y_offset <= 1; ++y_offset) {
      for (int x_offset = -1; x_offset <= 1; ++x_offset) {
        if (x_offset == 0 && y_offset == 0 && z_offset == 0) {
          continue;
        }
        const PersistentPlannerNode3D neighbor{node.x + x_offset, node.y + y_offset,
                                               node.z + z_offset};
        if (!nodeInside(neighbor)) {
          continue;
        }
        const LevelZeroSlot slot = levelZeroSlot(canonicalEdge(node, neighbor));
        const unsigned state = levelZeroState(slot);
        const bool movable = (state == kLevelZeroEdgeClear && occupied_cell_added) ||
                             (state == kLevelZeroEdgeBlocked && occupied_cell_removed);
        if (movable) {
          setLevelZeroState(slot, kLevelZeroEdgeUnknown);
          forgotten = true;
        }
      }
    }
  }
  return forgotten;
}

bool PlannerLattice3D::hasEdgeCost(const PersistentPlannerEdge3D& edge) const noexcept {
  if (!nodeInside(edge.first) || !nodeInside(edge.second) ||
      edge.first == edge.second) {
    return false;
  }
  if (level(edge.first, edge.second) == 0U) {
    return levelZeroState(levelZeroSlot(canonicalEdge(edge.first, edge.second))) !=
           kLevelZeroEdgeUnknown;
  }
  return adaptive_edge_cost_cache_.contains(edge);
}

bool PlannerLattice3D::configured() const noexcept {
  return width_ > 0 && height_ > 0 && depth_ > 0;
}

std::size_t PlannerLattice3D::edgeQueries() const noexcept {
  return edge_queries_;
}

std::size_t PlannerLattice3D::rawEdgeValidationChecks() const noexcept {
  return raw_edge_validation_checks_;
}

std::size_t PlannerLattice3D::adaptiveEdgeQueries() const noexcept {
  return adaptive_edge_queries_;
}

std::size_t PlannerLattice3D::maximumQueriedLevel() const noexcept {
  return maximum_queried_level_;
}

} // namespace drone_city_nav::detail
