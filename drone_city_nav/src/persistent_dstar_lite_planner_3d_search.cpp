#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <numbers>
#include <optional>
#include <ranges>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

namespace drone_city_nav::detail {
namespace {

constexpr double kCostTolerance{1.0e-12};

[[nodiscard]] bool approximatelyEqual(const double first,
                                      const double second) noexcept {
  if (std::isinf(first) || std::isinf(second)) {
    return first == second;
  }
  return std::abs(first - second) <=
         kCostTolerance * std::max({1.0, std::abs(first), std::abs(second)});
}

[[nodiscard]] bool keyLess(const DStarLiteKey3D& first,
                           const DStarLiteKey3D& second) noexcept {
  if (!approximatelyEqual(first.first, second.first)) {
    return first.first < second.first;
  }
  return !approximatelyEqual(first.second, second.second) &&
         first.second < second.second;
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

} // namespace

bool FeasibilityQueueEntryCompare3D::operator()(
    const FeasibilityQueueEntry3D& first,
    const FeasibilityQueueEntry3D& second) const noexcept {
  if (!approximatelyEqual(first.estimated_total_s, second.estimated_total_s)) {
    return first.estimated_total_s > second.estimated_total_s;
  }
  if (!approximatelyEqual(first.cost_from_start_s, second.cost_from_start_s)) {
    return first.cost_from_start_s > second.cost_from_start_s;
  }
  if (first.depth != second.depth) {
    return first.depth < second.depth;
  }
  return first.sequence > second.sequence;
}

void DStarLiteSession3D::begin(const PersistentPlannerNode3D start,
                               const PersistentPlannerNode3D goal) {
  const std::uint64_t previous_generation = search_generation_;
  reset();
  start_ = start;
  goal_ = goal;
  search_generation_ = previous_generation;
  search_generation_ = search_generation_ == std::numeric_limits<std::uint64_t>::max()
                           ? 1U
                           : search_generation_ + 1U;
  DStarLiteRecord3D& goal_record = records_[goal_];
  goal_record.rhs = 0.0;
  enqueue(goal_, goal_record);
}

DStarLiteKey3D DStarLiteSession3D::calculateKey(const PersistentPlannerNode3D node) {
  DStarLiteRecord3D& record = records_[node];
  const double minimum = std::min(record.g, record.rhs);
  return DStarLiteKey3D{minimum + lattice_->heuristic(start_, node) + key_modifier_,
                        minimum};
}

void DStarLiteSession3D::enqueue(const PersistentPlannerNode3D node,
                                 DStarLiteRecord3D& record) {
  ++queue_token_;
  if (queue_token_ == 0U) {
    queue_token_ = 1U;
  }
  ++queue_sequence_;
  if (queue_sequence_ == 0U) {
    queue_sequence_ = 1U;
  }
  record.open_token = queue_token_;
  open_.push(DStarLiteQueueEntry3D{
      .key = calculateKey(node),
      .node = node,
      .token = record.open_token,
      .sequence = queue_sequence_,
  });
}

void DStarLiteSession3D::recomputeRhs(const PersistentPlannerNode3D node,
                                      DStarLiteRecord3D& record) {
  if (node == goal_) {
    return;
  }
  double best = std::numeric_limits<double>::infinity();
  lattice_->forEachAdjacentNode(node, [&](const PersistentPlannerNode3D successor) {
    const double edge_cost = lattice_->rankedEdgeCost(node, successor);
    if (!std::isfinite(edge_cost)) {
      return;
    }
    const auto found = records_.find(successor);
    const double successor_cost = found != records_.end()
                                      ? found->second.g
                                      : std::numeric_limits<double>::infinity();
    best = std::min(best, edge_cost + successor_cost);
  });
  record.rhs = best;
}

void DStarLiteSession3D::updateVertexQueue(const PersistentPlannerNode3D node,
                                           DStarLiteRecord3D& record) {
  record.open_token = 0U;
  if (!approximatelyEqual(record.g, record.rhs)) {
    enqueue(node, record);
  }
}

void DStarLiteSession3D::updateVertex(const PersistentPlannerNode3D node) {
  DStarLiteRecord3D& record = records_[node];
  recomputeRhs(node, record);
  updateVertexQueue(node, record);
}

void DStarLiteSession3D::lowerPredecessors(const PersistentPlannerNode3D node,
                                           const double node_g) {
  lattice_->forEachAdjacentNode(node, [&](const PersistentPlannerNode3D predecessor) {
    if (predecessor == goal_) {
      return;
    }
    const double edge_cost = lattice_->rankedEdgeCost(predecessor, node);
    if (!std::isfinite(edge_cost)) {
      return;
    }
    DStarLiteRecord3D& predecessor_record = records_[predecessor];
    predecessor_record.rhs = std::min(predecessor_record.rhs, edge_cost + node_g);
    updateVertexQueue(predecessor, predecessor_record);
  });
}

void DStarLiteSession3D::raisePredecessors(const PersistentPlannerNode3D node,
                                           const double previous_node_g) {
  lattice_->forEachAdjacentNode(node, [&](const PersistentPlannerNode3D predecessor) {
    if (predecessor == goal_) {
      return;
    }
    const auto found = records_.find(predecessor);
    if (found == records_.end()) {
      return;
    }
    if (std::isfinite(previous_node_g)) {
      const double edge_cost = lattice_->rankedEdgeCost(predecessor, node);
      if (std::isfinite(edge_cost) &&
          approximatelyEqual(found->second.rhs, edge_cost + previous_node_g)) {
        recomputeRhs(predecessor, found->second);
      }
    }
    updateVertexQueue(predecessor, found->second);
  });
}

namespace {
constexpr double kClearanceToleranceM{1.0e-3};
} // namespace

void DStarLiteSession3D::scheduleAffectedVertices(
    const PersistentPlannerWorld3D& world,
    const std::vector<GridIndex3D>& changed_cells, std::size_t& affected_states) {
  const auto schedule_started = std::chrono::steady_clock::now();
  schedule_statistics_ = {};
  // Level-zero edges are found from a per-cell reach box; adaptive long edges
  // are found from the edge cache by chunk overlap, so a large change set
  // never multiplies by the long-edge reach.
  std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> changed_chunks;
  const double raw_half_diagonal =
      0.5 * std::numbers::sqrt3 * lattice_->bounds().resolution_m;
  const double horizontal_margin =
      config_->physical_footprint.radius_m + raw_half_diagonal;
  const double vertical_margin = std::max(config_->physical_footprint.lower_extent_m,
                                          config_->physical_footprint.upper_extent_m) +
                                 raw_half_diagonal;
  const double horizontal_reach =
      horizontal_margin + std::numbers::sqrt2 * config_->minimum_horizontal_step_m;
  const double vertical_reach = vertical_margin + config_->minimum_vertical_step_m;
  for (const GridIndex3D cell : changed_cells) {
    changed_chunks.insert(OccupancyGrid3D::chunkIndex(cell));
  }
  // Ranking clearances are validated lazily against the changed chunks when
  // the search next consults them; nodes whose ranking factor then moves are
  // repaired through scheduleMovedClearances.
  lattice_->noteChangedChunks(changed_chunks);

  // A D* label can depend only on an edge whose cost was evaluated. Invalidate
  // those exact cached dependencies instead of eagerly creating and validating
  // every geometrically possible neighbor in the conservative change radius.
  // Both endpoints are scheduled because an obstacle removal may make a
  // previously infinite undirected edge traversable.
  // Only an edge whose swept body can actually touch a changed cell is
  // forgotten: the distance from the cell centre to the edge segment must be
  // within the body margin, horizontally and vertically. A node beside a
  // change keeps every edge that points away from it. The test is exact per
  // (cell, edge) pair and touches only nodes within the level-zero reach of
  // that cell, so the cost is linear in the change size.
  std::unordered_set<PersistentPlannerNode3D, PersistentPlannerNode3DHash> affected;
  const auto cell_touches_segment = [&](const Point3& center, const Point3& first,
                                        const Point3& second) noexcept {
    const Vec3 direction{second.x - first.x, second.y - first.y, second.z - first.z};
    const double length_squared = direction.x * direction.x +
                                  direction.y * direction.y + direction.z * direction.z;
    const Vec3 offset{center.x - first.x, center.y - first.y, center.z - first.z};
    const double ratio =
        length_squared > 0.0
            ? std::clamp((offset.x * direction.x + offset.y * direction.y +
                          offset.z * direction.z) /
                             length_squared,
                         0.0, 1.0)
            : 0.0;
    const double dz = offset.z - ratio * direction.z;
    if (std::abs(dz) > vertical_margin) {
      return false;
    }
    const double dx = offset.x - ratio * direction.x;
    const double dy = offset.y - ratio * direction.y;
    return dx * dx + dy * dy <= horizontal_margin * horizontal_margin;
  };

  // The change set is bucketed by the lattice node cell that holds each cell
  // centre; every bucket then visits the labelled nodes within reach of that
  // cell and tests only their priced edges against the bucket's cells. The
  // cost is the number of changed node cells times the reach box, plus the
  // exact per-edge tests near labels, never the change size times the box.
  const auto labelled = [&](const PersistentPlannerNode3D node) {
    return node == start_ || node == goal_ || records_.contains(node);
  };
  // Visits every labelled node within the given node-step radii of a cell node.
  const auto for_each_labelled_node_near = [&](const PersistentPlannerNode3D cell_node,
                                               const int horizontal_radius,
                                               const int vertical_radius,
                                               auto&& visitor) {
    for (int z_offset = -vertical_radius; z_offset <= vertical_radius; ++z_offset) {
      for (int y_offset = -horizontal_radius; y_offset <= horizontal_radius;
           ++y_offset) {
        for (int x_offset = -horizontal_radius; x_offset <= horizontal_radius;
             ++x_offset) {
          const PersistentPlannerNode3D node{
              cell_node.x + x_offset, cell_node.y + y_offset, cell_node.z + z_offset};
          if (lattice_->nodeInside(node) && labelled(node)) {
            visitor(node);
          }
        }
      }
    }
  };
  using ChangedCell = LatticeChangedCell3D;

  std::unordered_map<PersistentPlannerNode3D, std::vector<ChangedCell>,
                     PersistentPlannerNode3DHash>
      changes_by_node_cell;
  LatticeChangesByChunk3D changes_by_chunk;
  for (const GridIndex3D cell : changed_cells) {
    const Point3 center = world.observed_occupancy != nullptr
                              ? world.observed_occupancy->cellCenter(cell)
                              : world.static_occupancy->cellCenter(cell);
    const bool occupied_now = world.observed_occupancy != nullptr
                                  ? world.observed_occupancy->isOccupied(cell)
                                  : world.static_occupancy->isOccupied(cell);
    const ChangedCell change{.center = center, .occupied_now = occupied_now};
    changes_by_node_cell[lattice_->nearestNode(center)].push_back(change);
    changes_by_chunk[OccupancyGrid3D::chunkIndex(cell)].push_back(change);
  }

  // A very large change set (several scans absorbed at once) is scheduled
  // coarsely by chunk: every node whose reach touches a changed chunk forgets
  // the level-zero edges the chunk's change polarity can move, and the
  // labelled ones among them are repaired. The cost is bounded by the changed
  // chunks times the node box of a chunk, never by the cell count.
  constexpr std::size_t kExactChangeCellLimit{32768U};
  const bool exact = changed_cells.size() <= kExactChangeCellLimit;
  if (!exact) {
    const double chunk_span_m = static_cast<double>(OccupancyGrid3D::kChunkSize) *
                                lattice_->bounds().resolution_m;
    const GridBounds3D& bounds = lattice_->bounds();
    for (const OccupancyChunkIndex3D& chunk : changed_chunks) {
      bool occupied_cell_added = false;
      bool occupied_cell_removed = false;
      if (const auto cells = changes_by_chunk.find(chunk);
          cells != changes_by_chunk.end()) {
        for (const ChangedCell& change : cells->second) {
          (change.occupied_now ? occupied_cell_added : occupied_cell_removed) = true;
        }
      }
      const Point3 chunk_minimum{
          bounds.origin_x + chunk.x * chunk_span_m - horizontal_reach,
          bounds.origin_y + chunk.y * chunk_span_m - horizontal_reach,
          bounds.origin_z + chunk.z * chunk_span_m - vertical_reach};
      const Point3 chunk_maximum{
          bounds.origin_x + (chunk.x + 1) * chunk_span_m + horizontal_reach,
          bounds.origin_y + (chunk.y + 1) * chunk_span_m + horizontal_reach,
          bounds.origin_z + (chunk.z + 1) * chunk_span_m + vertical_reach};
      const PersistentPlannerNode3D first = lattice_->nearestNode(chunk_minimum);
      const PersistentPlannerNode3D last = lattice_->nearestNode(chunk_maximum);
      for (int z = first.z; z <= last.z; ++z) {
        for (int y = first.y; y <= last.y; ++y) {
          for (int x = first.x; x <= last.x; ++x) {
            const PersistentPlannerNode3D node{x, y, z};
            if (!lattice_->forgetNodeEdgesForChange(node, occupied_cell_added,
                                                    occupied_cell_removed)) {
              continue;
            }
            ++schedule_statistics_.edges_forgotten;
            if (node == start_ || node == goal_ || records_.contains(node)) {
              affected.insert(node);
            }
          }
        }
      }
    }
    for (const PersistentPlannerEdge3D& edge : lattice_->forgetAdaptiveEdgesTouching(
             changes_by_chunk, cell_touches_segment)) {
      affected.insert(edge.first);
      affected.insert(edge.second);
    }
    std::vector<PersistentPlannerNode3D> coarse_ordered{affected.begin(),
                                                        affected.end()};
    std::ranges::sort(coarse_ordered, nodeLess);
    for (const PersistentPlannerNode3D node : coarse_ordered) {
      if (pending_repair_members_.insert(node).second) {
        pending_repair_nodes_.push_back(node);
      }
    }
    affected_states = coarse_ordered.size();
    schedule_statistics_.total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  schedule_started)
            .count();
    return;
  }
  const int horizontal_radius =
      static_cast<int>(
          std::ceil(horizontal_reach / config_->minimum_horizontal_step_m)) +
      1;
  const int vertical_radius =
      static_cast<int>(std::ceil(vertical_reach / config_->minimum_vertical_step_m)) +
      1;
  for (const auto& [cell_node, changes] : changes_by_node_cell) {
    for_each_labelled_node_near(
        cell_node, horizontal_radius, vertical_radius,
        [&](const PersistentPlannerNode3D node) {
          const Point3 node_point = lattice_->pointFor(node);
          // A change beyond the reach of every edge leaving this node cannot
          // touch any of them; most box nodes are filtered here.
          const bool within_reach =
              std::ranges::any_of(changes, [&](const ChangedCell& change) {
                return std::abs(change.center.z - node_point.z) <= vertical_reach &&
                       std::hypot(change.center.x - node_point.x,
                                  change.center.y - node_point.y) <= horizontal_reach;
              });
          if (!within_reach) {
            return;
          }
          for (int z_offset = -1; z_offset <= 1; ++z_offset) {
            for (int y_offset = -1; y_offset <= 1; ++y_offset) {
              for (int x_offset = -1; x_offset <= 1; ++x_offset) {
                if (x_offset == 0 && y_offset == 0 && z_offset == 0) {
                  continue;
                }
                const PersistentPlannerNode3D neighbor{
                    node.x + x_offset, node.y + y_offset, node.z + z_offset};
                if (!lattice_->nodeInside(neighbor)) {
                  continue;
                }
                const PersistentPlannerEdge3D edge = canonicalEdge(node, neighbor);
                if (!lattice_->hasEdgeCost(edge)) {
                  continue;
                }
                const Point3 neighbor_point = lattice_->pointFor(neighbor);
                bool occupied_cell_added = false;
                bool occupied_cell_removed = false;
                for (const ChangedCell& change : changes) {
                  if (cell_touches_segment(change.center, node_point, neighbor_point)) {
                    (change.occupied_now ? occupied_cell_added
                                         : occupied_cell_removed) = true;
                  }
                }
                if (!lattice_->forgetEdgeCostForChange(edge, occupied_cell_added,
                                                       occupied_cell_removed)) {
                  continue;
                }
                ++schedule_statistics_.edges_forgotten;
                affected.insert(edge.first);
                affected.insert(edge.second);
              }
            }
          }
        });
  }
  for (const PersistentPlannerEdge3D& edge :
       lattice_->forgetAdaptiveEdgesTouching(changes_by_chunk, cell_touches_segment)) {
    affected.insert(edge.first);
    affected.insert(edge.second);
  }
  std::vector<PersistentPlannerNode3D> ordered{affected.begin(), affected.end()};
  std::ranges::sort(ordered, nodeLess);
  for (const PersistentPlannerNode3D node : ordered) {
    if (pending_repair_members_.insert(node).second) {
      pending_repair_nodes_.push_back(node);
    }
  }
  affected_states = ordered.size();
  schedule_statistics_.total_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                schedule_started)
          .count();
}

void DStarLiteSession3D::scheduleMovedClearances() {
  for (const PersistentPlannerNode3D node : lattice_->takeMovedClearances()) {
    const auto schedule = [&](const PersistentPlannerNode3D candidate) {
      if (candidate != start_ && candidate != goal_ && !records_.contains(candidate)) {
        return;
      }
      if (pending_repair_members_.insert(candidate).second) {
        pending_repair_nodes_.push_back(candidate);
      }
    };
    schedule(node);
    lattice_->forEachAdjacentNode(node, schedule);
    ++schedule_statistics_.clearances_tightened;
  }
}

bool DStarLiteSession3D::continueAffectedVertexRepair(
    const std::chrono::steady_clock::time_point deadline,
    const std::size_t maximum_vertices, std::size_t& processed_vertices) {
  processed_vertices = 0U;
  while (!pending_repair_nodes_.empty() && processed_vertices < maximum_vertices &&
         std::chrono::steady_clock::now() < deadline) {
    const PersistentPlannerNode3D node = pending_repair_nodes_.front();
    pending_repair_nodes_.pop_front();
    pending_repair_members_.erase(node);
    updateVertex(node);
    ++processed_vertices;
  }
  return pending_repair_nodes_.empty();
}

std::optional<DStarLiteQueueEntry3D> DStarLiteSession3D::currentTop() {
  while (!open_.empty()) {
    const DStarLiteQueueEntry3D& entry = open_.top();
    const auto record = records_.find(entry.node);
    if (record != records_.end() && record->second.open_token == entry.token &&
        entry.token != 0U) {
      return entry;
    }
    open_.pop();
  }
  return std::nullopt;
}

bool DStarLiteSession3D::shortestPathComplete() {
  const auto start_record = records_.find(start_);
  const double start_g = start_record != records_.end()
                             ? start_record->second.g
                             : std::numeric_limits<double>::infinity();
  const double start_rhs = start_record != records_.end()
                               ? start_record->second.rhs
                               : std::numeric_limits<double>::infinity();
  const std::optional<DStarLiteQueueEntry3D> top = currentTop();
  return !top.has_value() || (!keyLess(top->key, calculateKey(start_)) &&
                              approximatelyEqual(start_g, start_rhs));
}

bool DStarLiteSession3D::computeShortestPath(
    const std::chrono::steady_clock::time_point deadline,
    const std::size_t maximum_expansions, std::size_t& expansions) {
  while (!shortestPathComplete()) {
    if (expansions >= maximum_expansions ||
        std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    const std::optional<DStarLiteQueueEntry3D> next = currentTop();
    if (!next.has_value()) {
      return true;
    }
    open_.pop();
    DStarLiteRecord3D& record = records_.at(next->node);
    if (record.open_token != next->token) {
      continue;
    }
    record.open_token = 0U;
    const DStarLiteKey3D current_key = calculateKey(next->node);
    if (keyLess(next->key, current_key)) {
      enqueue(next->node, record);
    } else if (record.g > record.rhs) {
      record.g = record.rhs;
      const double node_g = record.g;
      // Record references may move while predecessors are created below.
      lowerPredecessors(next->node, node_g);
    } else {
      const double previous_node_g = record.g;
      record.g = std::numeric_limits<double>::infinity();
      raisePredecessors(next->node, previous_node_g);
      updateVertexQueue(next->node, records_.at(next->node));
    }
    ++expansions;
  }
  return true;
}

std::vector<Point3> DStarLiteSession3D::extractPath(const Point3& exact_start,
                                                    const Point3& exact_goal,
                                                    std::size_t& adaptive_edges) {
  const auto start_record = records_.find(start_);
  if (start_record == records_.end() || !std::isfinite(start_record->second.g)) {
    return {};
  }
  std::vector<Point3> path;
  path.reserve(std::min(config_->maximum_extracted_path_nodes, lattice_->nodeSpan()));
  path.push_back(exact_start);
  const Point3 start_anchor = lattice_->pointFor(start_);
  if (distance3D(path.back(), start_anchor) > 1.0e-9) {
    path.push_back(start_anchor);
  }
  PersistentPlannerNode3D node = start_;
  std::unordered_set<PersistentPlannerNode3D, PersistentPlannerNode3DHash> visited;
  visited.insert(node);
  while (node != goal_ && path.size() < config_->maximum_extracted_path_nodes) {
    std::optional<PersistentPlannerNode3D> selected;
    double selected_cost = std::numeric_limits<double>::infinity();
    lattice_->forEachAdjacentNode(node, [&](const PersistentPlannerNode3D successor) {
      const double edge_cost = lattice_->rankedEdgeCost(node, successor);
      const auto successor_record = records_.find(successor);
      if (!std::isfinite(edge_cost) || successor_record == records_.end() ||
          !std::isfinite(successor_record->second.g)) {
        return;
      }
      const double cost = edge_cost + successor_record->second.g;
      if (!selected.has_value() || cost < selected_cost - kCostTolerance ||
          (approximatelyEqual(cost, selected_cost) &&
           (lattice_->heuristic(successor, goal_) <
                lattice_->heuristic(*selected, goal_) ||
            (approximatelyEqual(lattice_->heuristic(successor, goal_),
                                lattice_->heuristic(*selected, goal_)) &&
             nodeLess(successor, *selected))))) {
        selected = successor;
        selected_cost = cost;
      }
    });
    if (!selected.has_value() || visited.contains(*selected)) {
      return {};
    }
    const PersistentPlannerNode3D previous_node = node;
    node = *selected;
    adaptive_edges += lattice_->level(previous_node, node) > 0U ? 1U : 0U;
    visited.insert(node);
    const Point3 point = lattice_->pointFor(node);
    if (distance3D(path.back(), point) > 1.0e-9) {
      path.push_back(point);
    }
  }
  if (node != goal_) {
    return {};
  }
  if (distance3D(path.back(), exact_goal) > 1.0e-9) {
    path.push_back(exact_goal);
  }
  return path;
}

std::vector<Point3>
PathPostprocessor3D::shortcut(const std::vector<Point3>& path,
                              const PathPostprocessorContext3D& context,
                              std::size_t& checks, std::size_t& applied) const {
  if (path.size() < 3U || context.maximum_shortcut_checks == 0U ||
      !context.segment_valid || !context.time_profile) {
    return path;
  }
  std::vector<Point3> result = path;
  FlightPathTimeProfile3D current_profile = context.time_profile(result);
  if (!current_profile.valid) {
    return path;
  }
  std::size_t anchor = 0U;
  while (anchor + 2U < result.size()) {
    bool shortcut_applied{false};
    for (std::size_t candidate = result.size() - 1U; candidate > anchor + 1U;
         --candidate) {
      if (checks >= context.maximum_shortcut_checks) {
        break;
      }
      ++checks;
      const bool shortcut_valid =
          context.segment_valid(result[anchor], result[candidate], anchor == 0U);
      if (!shortcut_valid) {
        continue;
      }
      std::vector<Point3> trial = result;
      trial.erase(std::next(trial.begin(), static_cast<std::ptrdiff_t>(anchor + 1U)),
                  std::next(trial.begin(), static_cast<std::ptrdiff_t>(candidate)));
      FlightPathTimeProfile3D trial_profile = context.time_profile(trial);
      if (!trial_profile.valid || trial_profile.travel_time_s >
                                      current_profile.travel_time_s + kCostTolerance) {
        continue;
      }
      applied += candidate - anchor - 1U;
      result = std::move(trial);
      current_profile = std::move(trial_profile);
      shortcut_applied = true;
      break;
    }
    ++anchor;
    if (checks >= context.maximum_shortcut_checks && !shortcut_applied) {
      break;
    }
  }
  return result;
}

std::optional<std::vector<Point3>>
PersistentDStarLitePlanner3DImpl::rebaseIncumbent(const std::vector<Point3>& incumbent,
                                                  const Point3& start,
                                                  const Point3& goal) const {
  if (incumbent.size() < 2U ||
      distance3D(incumbent.back(), goal) > config_.goal_tolerance_m) {
    return std::nullopt;
  }
  std::vector<std::size_t> candidates(incumbent.size());
  for (std::size_t index = 0U; index < candidates.size(); ++index) {
    candidates[index] = index;
  }
  std::ranges::sort(candidates, [&](const std::size_t first, const std::size_t second) {
    return distance3D(start, incumbent[first]) < distance3D(start, incumbent[second]);
  });
  for (const std::size_t candidate : candidates) {
    if (!lattice_.departureSegmentValid(start, incumbent[candidate])) {
      continue;
    }
    std::vector<Point3> rebased;
    rebased.reserve(incumbent.size() - candidate + 1U);
    rebased.push_back(start);
    if (distance3D(start, incumbent[candidate]) > 1.0e-9) {
      rebased.push_back(incumbent[candidate]);
    }
    rebased.insert(
        rebased.end(),
        std::next(incumbent.begin(), static_cast<std::ptrdiff_t>(candidate + 1U)),
        incumbent.end());
    if (rebased.size() >= 2U && lattice_.pathTraversable(rebased)) {
      return rebased;
    }
  }
  return std::nullopt;
}

FlightPathTimeProfile3D
PersistentDStarLitePlanner3DImpl::pathTimeProfile(const std::vector<Point3>& path,
                                                  const Vec3& initial_velocity) const {
  if (path.size() < 2U) {
    return {};
  }
  std::vector<double> speed_limits(
      path.size(), std::min(config_.time_model.maximum_horizontal_speed_mps,
                            config_.time_model.maximum_translational_speed_mps));
  std::vector<std::uint8_t> stop_turn_flags(path.size(), 0U);
  for (std::size_t index = 1U; index + 1U < path.size(); ++index) {
    const Vec3 incoming{path[index].x - path[index - 1U].x,
                        path[index].y - path[index - 1U].y,
                        path[index].z - path[index - 1U].z};
    const Vec3 outgoing{path[index + 1U].x - path[index].x,
                        path[index + 1U].y - path[index].y,
                        path[index + 1U].z - path[index].z};
    stop_turn_flags[index] =
        requiresFlightStopAndTurn3D(incoming, outgoing,
                                    config_.minimum_continuous_turn_alignment)
            ? 1U
            : 0U;
  }
  return parameterizeFlightPathTime3D(path, speed_limits, stop_turn_flags,
                                      initial_velocity, true, config_.time_model);
}

std::optional<SpatialRouteCandidate3D> PersistentDStarLitePlanner3DImpl::makeCandidate(
    std::vector<Point3> path, const SpatialRouteCandidateSource3D source,
    const Vec3& initial_velocity) const {
  if (path.size() < 2U || !lattice_.pathTraversable(path)) {
    return std::nullopt;
  }
  const FlightPathTimeProfile3D profile = pathTimeProfile(path, initial_velocity);
  if (!profile.valid) {
    return std::nullopt;
  }
  double path_length_m = 0.0;
  for (std::size_t index = 1U; index < path.size(); ++index) {
    path_length_m += distance3D(path[index - 1U], path[index]);
  }
  const double ranked_execution_time_s = lattice_.rankedPathTimeS(path, profile);
  return SpatialRouteCandidate3D{
      .points = std::move(path),
      .source = source,
      .path_length_m = path_length_m,
      .estimated_execution_time_s = profile.travel_time_s,
      .estimated_translation_time_s = profile.translation_time_s,
      .estimated_stationary_turn_time_s = profile.stationary_turn_time_s,
      .ranked_execution_time_s = ranked_execution_time_s,
  };
}

void DStarLiteSession3D::reset() noexcept {
  start_ = {};
  goal_ = {};
  search_generation_ = 0U;
  repair_generation_ = 0U;
  queue_token_ = 0U;
  queue_sequence_ = 0U;
  key_modifier_ = 0.0;
  cost_to_goal_heuristic_admissible_ = true;
  open_ = {};
  records_.clear();
  pending_repair_nodes_.clear();
  pending_repair_members_.clear();
}

void DStarLiteSession3D::rebaseStart(const PersistentPlannerNode3D start,
                                     const double key_offset) noexcept {
  key_modifier_ += key_offset;
  start_ = start;
}

void DStarLiteSession3D::markCostToGoalInadmissible() noexcept {
  cost_to_goal_heuristic_admissible_ = false;
}

void DStarLiteSession3D::advanceRepairGeneration() noexcept {
  repair_generation_ = repair_generation_ == std::numeric_limits<std::uint64_t>::max()
                           ? 1U
                           : repair_generation_ + 1U;
}

std::optional<double>
DStarLiteSession3D::costToGoal(const PersistentPlannerNode3D node) const noexcept {
  if (!cost_to_goal_heuristic_admissible_) {
    return std::nullopt;
  }
  const auto record = records_.find(node);
  return record != records_.end() && std::isfinite(record->second.g)
             ? std::optional<double>{record->second.g}
             : std::nullopt;
}

bool DStarLiteSession3D::startResolved(
    const PersistentPlannerNode3D start) const noexcept {
  const auto record = records_.find(start);
  return record != records_.end() && std::isfinite(record->second.g);
}

std::uint64_t DStarLiteSession3D::searchGeneration() const noexcept {
  return search_generation_;
}

std::uint64_t DStarLiteSession3D::repairGeneration() const noexcept {
  return repair_generation_;
}

std::size_t DStarLiteSession3D::records() const noexcept {
  return records_.size();
}

std::size_t DStarLiteSession3D::openEntries() const noexcept {
  return open_.size();
}

std::size_t DStarLiteSession3D::pendingRepairNodes() const noexcept {
  return pending_repair_nodes_.size();
}

} // namespace drone_city_nav::detail
