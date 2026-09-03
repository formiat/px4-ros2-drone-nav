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

[[nodiscard]] bool nodeLess(const PersistentPlannerNode3D& first,
                            const PersistentPlannerNode3D& second) noexcept {
  return std::tuple{first.z, first.y, first.x} <
         std::tuple{second.z, second.y, second.x};
}

[[nodiscard]] PersistentPlannerEdge3D
canonicalEdge(const PersistentPlannerNode3D first,
              const PersistentPlannerNode3D second) noexcept {
  if (nodeLess(first, second)) {
    return PersistentPlannerEdge3D{first, second};
  }
  return PersistentPlannerEdge3D{second, first};
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

void PlannerLattice3D::installWorld(const PersistentPlannerWorld3D& world) {
  resident_collision_oracle_.emplace(OccupiedCollisionWorld3D{
      .observed_occupancy = world.observed_occupancy.get(),
      .static_occupancy = world.static_occupancy.get(),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = config_->physical_footprint,
      .flight_envelope = config_->flight_envelope,
  });
  departure_collision_oracle_.emplace(OccupiedCollisionWorld3D{
      .observed_occupancy = world.observed_occupancy.get(),
      .static_occupancy = world.static_occupancy.get(),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = world.launch_support_contact
                                    ? std::addressof(*world.launch_support_contact)
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

std::optional<PersistentPlannerNode3D>
PlannerLattice3D::selectAnchor(const Point3& point, const bool start_anchor) const {
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

double PlannerLattice3D::rankedEdgeCost(const PersistentPlannerNode3D first,
                                        const PersistentPlannerNode3D second) {
  const double raw_cost = rawEdgeCost(first, second);
  if (!std::isfinite(raw_cost) || config_->clearance_ranking_weight <= 0.0) {
    return raw_cost;
  }
  // Soft ranking only: a traversable edge stays traversable, it just costs
  // more the closer its endpoints run to confirmed occupied evidence. The raw
  // flight time is cached separately from the clearance, so an occupied
  // change inside the ranking reach re-derives only the cheap node clearance
  // and never the swept-footprint validation.
  // The ranking clearance is measured from the body surface, as the execution
  // risk model measures it, so a node centre one body radius from a wall
  // ranks as touching rather than as one metre clear.
  const double clearance_m =
      std::max(0.0, std::min(nodeClearanceM(first), nodeClearanceM(second)) -
                        config_->physical_footprint.radius_m);
  return raw_cost * rankingFactorForBodyClearance(clearance_m);
}

double PlannerLattice3D::rankedEdgeCost(const PersistentPlannerNode3D first,
                                        const PersistentPlannerNode3D second,
                                        const double clearance_reach_m) {
  const double raw_cost = rawEdgeCost(first, second);
  if (!std::isfinite(raw_cost) || config_->clearance_ranking_weight <= 0.0 ||
      !(clearance_reach_m > 0.0)) {
    return raw_cost;
  }
  // The curve is scaled to the reach: an edge clear beyond it costs its raw
  // flight time, so the unranked heuristic stays tight in open space and the
  // search remains directed; only the band near occupied evidence is priced.
  const double clearance_m =
      std::max(0.0, std::min(nodeClearanceWithin(first, clearance_reach_m),
                             nodeClearanceWithin(second, clearance_reach_m)) -
                        config_->physical_footprint.radius_m);
  return raw_cost * rankingFactorForBodyClearance(clearance_m, clearance_reach_m);
}

double PlannerLattice3D::rankingFactorForBodyClearance(
    const double body_clearance_m) const noexcept {
  return rankingFactorForBodyClearance(body_clearance_m,
                                       config_->clearance_ranking_distance_m);
}

double PlannerLattice3D::rankingFactorForBodyClearance(
    const double body_clearance_m, const double distance_m) const noexcept {
  if (!(config_->clearance_ranking_weight > 0.0) || !(distance_m > 0.0) ||
      body_clearance_m >= distance_m) {
    return 1.0;
  }
  const double shortfall = 1.0 - body_clearance_m / distance_m;
  double factor = 1.0 + config_->clearance_ranking_weight * shortfall * shortfall;
  const double critical_m = config_->clearance_ranking_critical_distance_m;
  if (config_->clearance_ranking_critical_weight > 0.0 && critical_m > 0.0 &&
      body_clearance_m < critical_m) {
    const double critical_shortfall = 1.0 - body_clearance_m / critical_m;
    factor += config_->clearance_ranking_critical_weight * critical_shortfall *
              critical_shortfall;
  }
  return factor;
}

double PlannerLattice3D::pointClearanceM(const Point3& point) const {
  const double cap_m = config_->clearance_ranking_distance_m;
  double clearance_m = cap_m;
  if (resident_collision_oracle_.has_value()) {
    const OccupiedCollisionWorld3D& world = resident_collision_oracle_->world();
    if (world.observed_occupancy != nullptr) {
      clearance_m = std::min(
          clearance_m, rawEuclideanClearance3D(*world.observed_occupancy, point, cap_m,
                                               world.launch_support_contact));
    }
    if (world.static_occupancy != nullptr) {
      clearance_m = std::min(
          clearance_m, rawEuclideanClearance3D(*world.static_occupancy, point, cap_m,
                                               world.launch_support_contact));
    }
  }
  return clearance_m;
}

double PlannerLattice3D::rankedSegmentTimeS(const Point3& first, const Point3& second,
                                            const double clearance_reach_m) const {
  const double segment_s =
      minimumFlightTranslationTime3D(first, second, config_->time_model);
  if (!std::isfinite(segment_s) || config_->clearance_ranking_weight <= 0.0 ||
      !(clearance_reach_m > 0.0)) {
    return segment_s;
  }
  const double radius_m = config_->physical_footprint.radius_m;
  const double sample_step_m = std::max(0.5, 0.5 * config_->minimum_horizontal_step_m);
  const double length_m = distance3D(first, second);
  const auto samples = static_cast<std::size_t>(std::ceil(length_m / sample_step_m));
  double worst_factor = 1.0;
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double ratio =
        samples == 0U ? 0.0
                      : static_cast<double>(sample) / static_cast<double>(samples);
    const Point3 point{first.x + ratio * (second.x - first.x),
                       first.y + ratio * (second.y - first.y),
                       first.z + ratio * (second.z - first.z)};
    const double body_clearance_m =
        std::max(0.0, deriveNodeClearance(point, clearance_reach_m) - radius_m);
    worst_factor =
        std::max(worst_factor,
                 rankingFactorForBodyClearance(body_clearance_m, clearance_reach_m));
  }
  return segment_s * worst_factor;
}

double PlannerLattice3D::rankedPathTimeS(const std::vector<Point3>& path,
                                         const FlightPathTimeProfile3D& profile) const {
  if (path.size() < 2U || !profile.valid ||
      profile.arrival_times_s.size() != path.size() ||
      profile.departure_times_s.size() != path.size()) {
    return 0.0;
  }
  const double radius_m = config_->physical_footprint.radius_m;
  const double sample_step_m = std::max(0.5, 0.5 * config_->minimum_horizontal_step_m);
  double ranked_s = 0.0;
  for (std::size_t index = 0U; index + 1U < path.size(); ++index) {
    const Point3& first = path[index];
    const Point3& second = path[index + 1U];
    const double segment_s = std::max(0.0, profile.arrival_times_s[index + 1U] -
                                               profile.departure_times_s[index]);
    const double turn_s = std::max(0.0, profile.departure_times_s[index] -
                                            profile.arrival_times_s[index]);
    const double length_m = distance3D(first, second);
    const auto samples = static_cast<std::size_t>(std::ceil(length_m / sample_step_m));
    double worst_factor = 1.0;
    for (std::size_t sample = 0U; sample <= samples; ++sample) {
      const double ratio =
          samples == 0U ? 0.0
                        : static_cast<double>(sample) / static_cast<double>(samples);
      const Point3 point{first.x + ratio * (second.x - first.x),
                         first.y + ratio * (second.y - first.y),
                         first.z + ratio * (second.z - first.z)};
      const double body_clearance_m = std::max(0.0, pointClearanceM(point) - radius_m);
      worst_factor =
          std::max(worst_factor, rankingFactorForBodyClearance(body_clearance_m));
    }
    ranked_s += turn_s + segment_s * worst_factor;
  }
  return ranked_s;
}

void PlannerLattice3D::resetNodeClearances() noexcept {
  node_clearance_cache_.clear();
  chunk_change_epoch_.clear();
  change_epoch_ = 0U;
  moved_clearances_.clear();
}

void PlannerLattice3D::noteChangedChunks(
    const std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash>&
        changed_chunks) {
  if (changed_chunks.empty() || clearanceRankingReachM() <= 0.0) {
    return;
  }
  ++change_epoch_;
  for (const OccupancyChunkIndex3D& chunk : changed_chunks) {
    chunk_change_epoch_[chunk] = change_epoch_;
  }
}

std::vector<PersistentPlannerNode3D> PlannerLattice3D::takeMovedClearances() {
  return std::exchange(moved_clearances_, {});
}

std::size_t PlannerLattice3D::clearancesRederived() const noexcept {
  return clearances_rederived_;
}

bool PlannerLattice3D::clearanceStale(const Point3& point,
                                      const std::uint64_t change_epoch) const noexcept {
  if (change_epoch == change_epoch_ || chunk_change_epoch_.empty()) {
    return false;
  }
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  const double reach_m = clearanceRankingReachM();
  const double chunk_span_m = kChunkSize * raw_bounds_.resolution_m;
  const auto chunk_of = [&](const double coordinate, const double origin) {
    return static_cast<int>(std::floor((coordinate - origin) / chunk_span_m));
  };
  const int minimum_x = chunk_of(point.x - reach_m, raw_bounds_.origin_x);
  const int maximum_x = chunk_of(point.x + reach_m, raw_bounds_.origin_x);
  const int minimum_y = chunk_of(point.y - reach_m, raw_bounds_.origin_y);
  const int maximum_y = chunk_of(point.y + reach_m, raw_bounds_.origin_y);
  const int minimum_z = chunk_of(point.z - reach_m, raw_bounds_.origin_z);
  const int maximum_z = chunk_of(point.z + reach_m, raw_bounds_.origin_z);
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        const auto found = chunk_change_epoch_.find(OccupancyChunkIndex3D{x, y, z});
        if (found != chunk_change_epoch_.end() && found->second > change_epoch) {
          return true;
        }
      }
    }
  }
  return false;
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

std::optional<double>
PlannerLattice3D::cachedNodeClearance(const PersistentPlannerNode3D node,
                                      const double reach_m) {
  if (!node_clearance_cache_.contains(node)) {
    return std::nullopt;
  }
  return nodeClearanceWithin(node, reach_m);
}

double PlannerLattice3D::clearanceRankingReachM() const noexcept {
  return config_->clearance_ranking_weight > 0.0 ? config_->clearance_ranking_distance_m
                                                 : 0.0;
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

double PlannerLattice3D::deriveNodeClearance(const Point3& point,
                                             const double cap_m) const {
  double clearance_m = cap_m;
  if (resident_collision_oracle_.has_value()) {
    const OccupiedCollisionWorld3D& world = resident_collision_oracle_->world();
    if (world.observed_occupancy != nullptr) {
      clearance_m = std::min(
          clearance_m, rawEuclideanClearance3D(*world.observed_occupancy, point, cap_m,
                                               world.launch_support_contact));
    }
    if (world.static_occupancy != nullptr) {
      clearance_m = std::min(
          clearance_m, rawEuclideanClearance3D(*world.static_occupancy, point, cap_m,
                                               world.launch_support_contact));
    }
  }
  return clearance_m;
}

double PlannerLattice3D::nodeClearanceM(const PersistentPlannerNode3D node) {
  return nodeClearanceWithin(node, config_->clearance_ranking_distance_m);
}

double PlannerLattice3D::nodeClearanceWithin(const PersistentPlannerNode3D node,
                                             const double cap_m) {
  const Point3 point = pointFor(node);
  const auto found = node_clearance_cache_.find(node);
  if (found != node_clearance_cache_.end()) {
    CachedNodeClearance& cached = found->second;
    // A value derived within a shorter reach that reached its cap says only
    // that nothing lies closer than that cap; a longer reach re-derives it.
    const bool reach_sufficient =
        cached.cap_m + 1.0e-9 >= cap_m || cached.clearance_m + 1.0e-9 < cached.cap_m;
    if (reach_sufficient && !clearanceStale(point, cached.change_epoch)) {
      cached.change_epoch = change_epoch_;
      return std::min(cached.clearance_m, cap_m);
    }
    // Occupied evidence within reach changed since this clearance was
    // derived, or a longer reach is asked for: re-derive it, and hand the
    // node to the search when the move matters for its ranked labels.
    const double previous_m = cached.clearance_m;
    const double reach_m = std::max(cached.cap_m, cap_m);
    cached.clearance_m = deriveNodeClearance(point, reach_m);
    cached.cap_m = reach_m;
    cached.change_epoch = change_epoch_;
    ++clearances_rederived_;
    if (rankingRepairRequired(previous_m, cached.clearance_m)) {
      moved_clearances_.push_back(node);
    }
    return std::min(cached.clearance_m, cap_m);
  }
  const double clearance_m = deriveNodeClearance(point, cap_m);
  node_clearance_cache_.emplace(node,
                                CachedNodeClearance{.clearance_m = clearance_m,
                                                    .cap_m = cap_m,
                                                    .change_epoch = change_epoch_});
  return clearance_m;
}

void PlannerLattice3D::reset() noexcept {
  raw_bounds_ = {};
  width_ = 0;
  height_ = 0;
  depth_ = 0;
  level_zero_edge_states_.clear();
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

bool PlannerLattice3D::rankingRepairRequired(
    const double previous_clearance_m,
    const double current_clearance_m) const noexcept {
  constexpr double kRankingRepairRelativeTolerance{0.05};
  const double radius_m = config_->physical_footprint.radius_m;
  const double previous_factor =
      rankingFactorForBodyClearance(std::max(0.0, previous_clearance_m - radius_m));
  const double current_factor =
      rankingFactorForBodyClearance(std::max(0.0, current_clearance_m - radius_m));
  return std::abs(current_factor - previous_factor) >
         kRankingRepairRelativeTolerance * std::max(previous_factor, current_factor);
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
