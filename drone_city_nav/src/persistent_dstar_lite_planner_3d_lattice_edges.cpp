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

void PlannerLattice3D::beginEdgeRefinementBudget() noexcept {
  edge_refinement_probes_ = 0U;
}

bool PlannerLattice3D::edgeRefinementBudgetRemaining() const noexcept {
  return config_->edge_refinement_offsets == 0U ||
         edge_refinement_probes_ < config_->maximum_edge_refinement_probes;
}

std::optional<Point3>
PlannerLattice3D::refineBlockedEdge(const PersistentPlannerNode3D first,
                                    const PersistentPlannerNode3D second) {
  if (config_->edge_refinement_offsets == 0U || !edgeRefinementBudgetRemaining()) {
    return std::nullopt;
  }
  const Point3 first_point = pointFor(first);
  const Point3 second_point = pointFor(second);
  const Vec3 along{second_point.x - first_point.x, second_point.y - first_point.y,
                   second_point.z - first_point.z};
  const double length =
      std::sqrt(along.x * along.x + along.y * along.y + along.z * along.z);
  if (!(length > 1.0e-6)) {
    return std::nullopt;
  }
  const Vec3 unit{along.x / length, along.y / length, along.z / length};
  // Two directions across the edge: one horizontal, one carrying whatever
  // vertical freedom the edge leaves. A doorway is missed across the edge, not
  // along it.
  const double horizontal_length = std::hypot(unit.x, unit.y);
  const Vec3 lateral =
      horizontal_length > 1.0e-6
          ? Vec3{-unit.y / horizontal_length, unit.x / horizontal_length, 0.0}
          : Vec3{1.0, 0.0, 0.0};
  const Vec3 vertical{lateral.y * unit.z - lateral.z * unit.y,
                      lateral.z * unit.x - lateral.x * unit.z,
                      lateral.x * unit.y - lateral.y * unit.x};
  const Point3 middle{0.5 * (first_point.x + second_point.x),
                      0.5 * (first_point.y + second_point.y),
                      0.5 * (first_point.z + second_point.z)};
  const auto offsets = static_cast<int>(config_->edge_refinement_offsets);
  const double step_m =
      config_->minimum_horizontal_step_m / static_cast<double>(offsets + 1);
  // Nearest first: the shortest detour that clears the body wins.
  for (int magnitude = 1; magnitude <= offsets; ++magnitude) {
    const double distance_m = static_cast<double>(magnitude) * step_m;
    for (const Vec3& axis : {lateral, vertical}) {
      for (const double sign : {1.0, -1.0}) {
        const Point3 waypoint{middle.x + sign * distance_m * axis.x,
                              middle.y + sign * distance_m * axis.y,
                              middle.z + sign * distance_m * axis.z};
        if (!pointInsideFlightEnvelope(waypoint)) {
          continue;
        }
        if (!edgeRefinementBudgetRemaining()) {
          return std::nullopt;
        }
        ++edge_refinement_probes_;
        if (routePiecesValid(first_point, waypoint) &&
            routePiecesValid(waypoint, second_point)) {
          return waypoint;
        }
      }
    }
  }
  return std::nullopt;
}

std::optional<Point3>
PlannerLattice3D::edgeWaypoint(const PersistentPlannerNode3D first,
                               const PersistentPlannerNode3D second) const {
  if (!nodeInside(first) || !nodeInside(second) || first == second ||
      level(first, second) != 0U) {
    return std::nullopt;
  }
  const PersistentPlannerEdge3D edge = canonicalEdge(first, second);
  // The state decides. A bulk clear or an occupied change can overwrite it
  // without the map knowing, and a waypoint validated against an older world
  // must never reach a path.
  if (levelZeroState(levelZeroSlot(edge)) != kLevelZeroEdgeRefined) {
    return std::nullopt;
  }
  const auto found = refined_edge_waypoints_.find(edge);
  return found != refined_edge_waypoints_.end() ? std::optional<Point3>{found->second}
                                                : std::nullopt;
}

std::optional<PlannerLattice3D::PathSegmentEdge3D>
PlannerLattice3D::pricedEdgeForSegment(const std::vector<Point3>& path,
                                       const std::size_t segment) const {
  if (segment == 0U || segment >= path.size()) {
    return std::nullopt;
  }
  constexpr double kOnNodeTolerance{1.0e-6};
  const auto node_at =
      [&](const std::size_t index) -> std::optional<PersistentPlannerNode3D> {
    if (index >= path.size() || !pointInsideFlightEnvelope(path[index])) {
      return std::nullopt;
    }
    const PersistentPlannerNode3D node = nearestNode(path[index]);
    if (!nodeInside(node) ||
        distance3D(pointFor(node), path[index]) > kOnNodeTolerance) {
      return std::nullopt;
    }
    return node;
  };
  const auto adjacent = [&](const PersistentPlannerNode3D from,
                            const PersistentPlannerNode3D to) {
    return from != to && level(from, to) == 0U;
  };
  const auto is_waypoint_of = [&](const PersistentPlannerNode3D from,
                                  const PersistentPlannerNode3D to,
                                  const Point3& point) {
    const std::optional<Point3> waypoint =
        adjacent(from, to) ? edgeWaypoint(from, to) : std::nullopt;
    return waypoint.has_value() && distance3D(*waypoint, point) <= kOnNodeTolerance;
  };
  const std::optional<PersistentPlannerNode3D> before = node_at(segment - 1U);
  const std::optional<PersistentPlannerNode3D> after = node_at(segment);
  if (before && after) {
    // A straight lattice edge, level zero or adaptive.
    return before != after ? std::optional{PathSegmentEdge3D{*before, *after}}
                           : std::nullopt;
  }
  if (before && !after) {
    // Leg into a refined edge's waypoint: the edge continues to the next node.
    const std::optional<PersistentPlannerNode3D> next = node_at(segment + 1U);
    if (next && is_waypoint_of(*before, *next, path[segment])) {
      return PathSegmentEdge3D{*before, *next};
    }
    return std::nullopt;
  }
  if (!before && after && segment >= 2U) {
    // Leg out of a refined edge's waypoint: the edge started at the node
    // before it.
    const std::optional<PersistentPlannerNode3D> previous = node_at(segment - 2U);
    if (previous && is_waypoint_of(*previous, *after, path[segment - 1U])) {
      return PathSegmentEdge3D{*previous, *after};
    }
  }
  return std::nullopt;
}

bool PlannerLattice3D::rejectEdgeBySweep(const PersistentPlannerEdge3D& edge) {
  if (!forgetEdgeCost(edge)) {
    return false;
  }
  sweep_rejected_edges_.push_back(canonicalEdge(edge.first, edge.second));
  return true;
}

std::vector<PersistentPlannerEdge3D> PlannerLattice3D::takeSweepRejectedEdges() {
  std::vector<PersistentPlannerEdge3D> taken;
  taken.swap(sweep_rejected_edges_);
  return taken;
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
  if (!resident_collision_oracle_.has_value()) {
    return false;
  }
  const auto sweep_started = std::chrono::steady_clock::now();
  const bool clear =
      resident_collision_oracle_
          ->validateSegment(first, FootprintBodyAxis{}, second, FootprintBodyAxis{})
          .clear();
  raw_sweep_time_ += std::chrono::steady_clock::now() - sweep_started;
  return clear;
}

bool PlannerLattice3D::departureSegmentValid(const Point3& first,
                                             const Point3& second) const {
  return departureSegmentValidation(first, second).clear();
}

void PlannerLattice3D::resetDepartureBody() noexcept {
  departure_uses_hull_ = false;
}

OccupiedCollisionResult3D
PlannerLattice3D::departureSegmentValidation(const Point3& first, const Point3& second,
                                             const bool hull) const {
  const std::optional<OccupiedCollisionOracle3D>& oracle =
      hull || departure_uses_hull_ ? departure_hull_collision_oracle_
                                   : departure_collision_oracle_;
  if (!oracle.has_value()) {
    return {.status = OccupiedCollisionStatus3D::kInvalidInput, .failure_point = first};
  }
  return oracle->validateSegment(first, FootprintBodyAxis{}, second,
                                 FootprintBodyAxis{});
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
  placeNode(first);
  placeNode(second);
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
        // Priced as the path is accepted: an edge clear only when swept
        // whole is refused by the acceptance, and a search pricing it clear
        // finds it again after every refusal (r693 searched 90 s within
        // 0.4 m of the goal, refusing one such edge after another).
        if (routePiecesValid(pointFor(first), pointFor(second))) {
          state = kLevelZeroEdgeClear;
          setLevelZeroState(slot, state);
        } else if (const std::optional<Point3> waypoint =
                       refineBlockedEdge(edge.first, edge.second);
                   waypoint.has_value()) {
          rememberRefinedWaypoint(edge, *waypoint);
          state = kLevelZeroEdgeRefined;
          setLevelZeroState(slot, state);
        } else {
          // The straight sweep answered: this edge is blocked. Refinement is an
          // opportunistic upgrade on top of that answer, so an edge the probe
          // budget could not reach is cached blocked exactly as it was before
          // refinement existed. Leaving it unknown instead would re-run the
          // straight sweep on every later query in the same update, and a
          // search that revalidates the same edges cannot converge inside its
          // budget.
          state = kLevelZeroEdgeBlocked;
          setLevelZeroState(slot, state);
        }
      }
    }
    if (state == kLevelZeroEdgeClear) {
      // The table holds the canonical geometry; an end placed off its
      // canonical point is flown as it stands.
      if (nodeDisplaced(first) || nodeDisplaced(second)) {
        return minimumFlightTranslationTime3D(pointFor(first), pointFor(second),
                                              config_->time_model);
      }
      return level_zero_edge_time_s_[slot.direction];
    }
    if (state == kLevelZeroEdgeRefined) {
      const auto found = refined_edge_waypoints_.find(edge);
      if (found == refined_edge_waypoints_.end()) {
        setLevelZeroState(slot, kLevelZeroEdgeUnknown);
        return std::numeric_limits<double>::infinity();
      }
      // The detour is what the vehicle actually flies, so it is what the edge
      // costs.
      return minimumFlightTranslationTime3D(pointFor(edge.first), found->second,
                                            config_->time_model) +
             minimumFlightTranslationTime3D(found->second, pointFor(edge.second),
                                            config_->time_model);
    }
    return std::numeric_limits<double>::infinity();
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
    raw_cost = routePiecesValid(first_point, second_point)
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

void PlannerLattice3D::rememberRefinedWaypoint(const PersistentPlannerEdge3D& edge,
                                               const Point3& waypoint) {
  forgetRefinedWaypoint(edge);
  refined_edge_waypoints_.insert_or_assign(edge, waypoint);
  std::vector<OccupancyChunkIndex3D> chunks;
  const auto collect = [&](const OccupancyChunkIndex3D chunk) {
    if (std::ranges::find(chunks, chunk) == chunks.end()) {
      chunks.push_back(chunk);
    }
  };
  forEachChunkTouching(pointFor(edge.first), waypoint, collect);
  forEachChunkTouching(waypoint, pointFor(edge.second), collect);
  for (const OccupancyChunkIndex3D& chunk : chunks) {
    refined_edges_by_chunk_[chunk].push_back(edge);
  }
}

void PlannerLattice3D::forgetRefinedWaypoint(const PersistentPlannerEdge3D& edge) {
  const auto found = refined_edge_waypoints_.find(edge);
  if (found == refined_edge_waypoints_.end()) {
    return;
  }
  const Point3 waypoint = found->second;
  refined_edge_waypoints_.erase(found);
  const auto unindex = [&](const OccupancyChunkIndex3D chunk) {
    const auto entry = refined_edges_by_chunk_.find(chunk);
    if (entry == refined_edges_by_chunk_.end()) {
      return;
    }
    std::erase(entry->second, edge);
    if (entry->second.empty()) {
      refined_edges_by_chunk_.erase(entry);
    }
  };
  forEachChunkTouching(pointFor(edge.first), waypoint, unindex);
  forEachChunkTouching(waypoint, pointFor(edge.second), unindex);
}

std::vector<PersistentPlannerEdge3D>
PlannerLattice3D::forgetRefinedEdgesTouching(const LatticeChangesByChunk3D& changes,
                                             const LatticeSegmentTouch3D& touches) {
  std::vector<PersistentPlannerEdge3D> forgotten;
  for (const auto& [chunk, cells] : changes) {
    const auto found = refined_edges_by_chunk_.find(chunk);
    if (found == refined_edges_by_chunk_.end()) {
      continue;
    }
    // Forgetting unindexes the edge, so iterate a copy of the chunk's list.
    const std::vector<PersistentPlannerEdge3D> candidates = found->second;
    for (const PersistentPlannerEdge3D& edge : candidates) {
      const LevelZeroSlot slot = levelZeroSlot(edge);
      const auto waypoint = refined_edge_waypoints_.find(edge);
      if (levelZeroState(slot) != kLevelZeroEdgeRefined ||
          waypoint == refined_edge_waypoints_.end()) {
        continue;
      }
      const Point3 first_point = pointFor(edge.first);
      const Point3 second_point = pointFor(edge.second);
      const Point3 middle = waypoint->second;
      // A refined edge is traversable, so only a cell that appeared can move
      // it, and only through one of its legs.
      const bool moved =
          std::ranges::any_of(cells, [&](const LatticeChangedCell3D& cell) {
            return cell.occupied_now && (touches(cell.center, first_point, middle) ||
                                         touches(cell.center, middle, second_point));
          });
      if (!moved) {
        continue;
      }
      setLevelZeroState(slot, kLevelZeroEdgeUnknown);
      forgetRefinedWaypoint(edge);
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
  refined_edge_waypoints_.clear();
  refined_edges_by_chunk_.clear();
  chunk_change_epoch_.clear();
  chunk_occupied_ring_.clear();
  chunk_columns_ = 0;
  chunk_rows_ = 0;
  chunk_layers_ = 0;
  resetEdgeEvidence();
}

void PlannerLattice3D::resetEdgeEvidence() noexcept {
  std::ranges::fill(level_zero_edge_states_, 0U);
  refined_edge_waypoints_.clear();
  refined_edges_by_chunk_.clear();
  sweep_rejected_edges_.clear();
  adaptive_edge_cost_cache_.clear();
  resetNodeClearances();
  node_placements_.clear();
  adaptive_edges_by_chunk_.clear();
  resetEdgeStatistics();
}

} // namespace drone_city_nav::detail
