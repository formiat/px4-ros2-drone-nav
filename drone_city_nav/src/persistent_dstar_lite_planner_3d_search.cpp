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

void DStarLiteSession3D::updateVertex(const PersistentPlannerNode3D node) {
  DStarLiteRecord3D& record = records_[node];
  if (node != goal_) {
    double best = std::numeric_limits<double>::infinity();
    lattice_->forEachAdjacentNode(node, [&](const PersistentPlannerNode3D successor) {
      const double edge_cost = lattice_->rawEdgeCost(node, successor);
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
  record.open_token = 0U;
  if (!approximatelyEqual(record.g, record.rhs)) {
    enqueue(node, record);
  }
}

void DStarLiteSession3D::scheduleAffectedVertices(
    const PersistentPlannerWorld3D& world,
    const std::vector<GridIndex3D>& changed_cells, std::size_t& affected_states) {
  std::unordered_set<PersistentPlannerNode3D, PersistentPlannerNode3DHash>
      resident_candidates;
  const double raw_half_diagonal =
      0.5 * std::numbers::sqrt3 * lattice_->bounds().resolution_m;
  const double ranking_reach = lattice_->clearanceRankingReachM();
  const double horizontal_reach =
      config_->physical_footprint.radius_m + raw_half_diagonal + ranking_reach +
      std::numbers::sqrt2 * config_->minimum_horizontal_step_m *
          static_cast<double>(lattice_->maximumScale());
  const double vertical_reach =
      std::max(config_->physical_footprint.lower_extent_m,
               config_->physical_footprint.upper_extent_m) +
      raw_half_diagonal + ranking_reach +
      config_->minimum_vertical_step_m * static_cast<double>(lattice_->maximumScale());
  const int horizontal_radius =
      static_cast<int>(
          std::ceil(horizontal_reach / config_->minimum_horizontal_step_m)) +
      1;
  const int vertical_radius =
      static_cast<int>(std::ceil(vertical_reach / config_->minimum_vertical_step_m)) +
      1;
  for (const GridIndex3D cell : changed_cells) {
    const Point3 center = world.observed_occupancy != nullptr
                              ? world.observed_occupancy->cellCenter(cell)
                              : world.static_occupancy->cellCenter(cell);
    const PersistentPlannerNode3D nearest = lattice_->nearestNode(center);
    for (int z_offset = -vertical_radius; z_offset <= vertical_radius; ++z_offset) {
      for (int y_offset = -horizontal_radius; y_offset <= horizontal_radius;
           ++y_offset) {
        for (int x_offset = -horizontal_radius; x_offset <= horizontal_radius;
             ++x_offset) {
          const PersistentPlannerNode3D candidate{
              nearest.x + x_offset, nearest.y + y_offset, nearest.z + z_offset};
          if (lattice_->nodeInside(candidate) &&
              (candidate == start_ || candidate == goal_ ||
               records_.contains(candidate))) {
            resident_candidates.insert(candidate);
          }
        }
      }
    }
  }
  // A D* label can depend only on an edge whose cost was evaluated. Invalidate
  // those exact cached dependencies instead of eagerly creating and validating
  // every geometrically possible neighbor in the conservative change radius.
  // Both endpoints are scheduled because an obstacle removal may make a
  // previously infinite undirected edge traversable.
  std::unordered_set<PersistentPlannerNode3D, PersistentPlannerNode3DHash> affected;
  lattice_->forgetNodeClearances(resident_candidates);
  for (const PersistentPlannerNode3D node : resident_candidates) {
    lattice_->forEachAdjacentNode(node, [&](const PersistentPlannerNode3D neighbor) {
      const PersistentPlannerEdge3D edge = canonicalEdge(node, neighbor);
      if (!lattice_->forgetEdgeCost(edge)) {
        return;
      }
      affected.insert(edge.first);
      affected.insert(edge.second);
    });
  }
  std::vector<PersistentPlannerNode3D> ordered{affected.begin(), affected.end()};
  std::ranges::sort(ordered, nodeLess);
  for (const PersistentPlannerNode3D node : ordered) {
    if (pending_repair_members_.insert(node).second) {
      pending_repair_nodes_.push_back(node);
    }
  }
  affected_states = ordered.size();
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
      lattice_->forEachAdjacentNode(next->node,
                                    [this](const PersistentPlannerNode3D predecessor) {
                                      updateVertex(predecessor);
                                    });
    } else {
      record.g = std::numeric_limits<double>::infinity();
      updateVertex(next->node);
      lattice_->forEachAdjacentNode(next->node,
                                    [this](const PersistentPlannerNode3D predecessor) {
                                      updateVertex(predecessor);
                                    });
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
      const double edge_cost = lattice_->rawEdgeCost(node, successor);
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
    }
    if (!selected.has_value() || visited.contains(*selected)) {
      return {};
    });
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
  return SpatialRouteCandidate3D{
      .points = std::move(path),
      .source = source,
      .path_length_m = path_length_m,
      .estimated_execution_time_s = profile.travel_time_s,
      .estimated_translation_time_s = profile.translation_time_s,
      .estimated_stationary_turn_time_s = profile.stationary_turn_time_s,
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
