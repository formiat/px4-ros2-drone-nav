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

void PersistentDStarLitePlanner3DImpl::initializeSearch(
    const PersistentPlannerRequest3D& request, const PersistentPlannerNode3D start,
    const PersistentPlannerNode3D goal) {
  initialized_ = true;
  start_ = start;
  last_start_ = start;
  goal_ = goal;
  exact_start_ = request.start;
  exact_goal_ = request.mission_goal;
  mission_epoch_ = request.mission_epoch;
  dstar_cost_to_goal_heuristic_admissible_ = true;
  key_modifier_ = 0.0;
  queue_token_ = 0U;
  queue_sequence_ = 0U;
  open_ = {};
  records_.clear();
  edge_cost_cache_.clear();
  resetExecutionTimeSearch();
  search_generation_ = search_generation_ == std::numeric_limits<std::uint64_t>::max()
                           ? 1U
                           : search_generation_ + 1U;
  DStarLiteRecord3D& goal_record = records_[goal_];
  goal_record.rhs = 0.0;
  enqueue(goal_, goal_record);
}

std::vector<PersistentPlannerNode3D> PersistentDStarLitePlanner3DImpl::adjacentNodes(
    const PersistentPlannerNode3D node) const {
  std::vector<PersistentPlannerNode3D> result;
  result.reserve(26U * (config_.maximum_adaptive_lattice_level + 1U));
  for (std::size_t level = 0U; level <= config_.maximum_adaptive_lattice_level;
       ++level) {
    const int scale = 1 << level;
    if (node.x % scale != 0 || node.y % scale != 0 || node.z % scale != 0) {
      continue;
    }
    for (int z_offset = -1; z_offset <= 1; ++z_offset) {
      for (int y_offset = -1; y_offset <= 1; ++y_offset) {
        for (int x_offset = -1; x_offset <= 1; ++x_offset) {
          if (x_offset == 0 && y_offset == 0 && z_offset == 0) {
            continue;
          }
          const PersistentPlannerNode3D candidate{node.x + x_offset * scale,
                                                  node.y + y_offset * scale,
                                                  node.z + z_offset * scale};
          if (nodeInside(candidate) && pointInsideFlightEnvelope(pointFor(candidate))) {
            result.push_back(candidate);
          }
        }
      }
    }
  }
  return result;
}

double PersistentDStarLitePlanner3DImpl::heuristic(
    const PersistentPlannerNode3D first,
    const PersistentPlannerNode3D second) const noexcept {
  return minimumFlightTranslationTime3D(pointFor(first), pointFor(second),
                                        config_.time_model);
}

double
PersistentDStarLitePlanner3DImpl::rawEdgeCost(const PersistentPlannerNode3D first,
                                              const PersistentPlannerNode3D second) {
  ++lattice_edge_queries_;
  if (!nodeInside(first) || !nodeInside(second) || first == second) {
    return std::numeric_limits<double>::infinity();
  }
  const std::size_t level = latticeLevel(first, second);
  if (level > 0U) {
    ++adaptive_edge_queries_;
    maximum_queried_lattice_level_ = std::max(maximum_queried_lattice_level_, level);
  }
  const PersistentPlannerEdge3D edge = canonicalEdge(first, second);
  if (const auto found = edge_cost_cache_.find(edge); found != edge_cost_cache_.end()) {
    return found->second;
  }
  const Point3 first_point = pointFor(first);
  const Point3 second_point = pointFor(second);
  ++raw_edge_validation_checks_;
  const double cost = rawSegmentValid(first_point, second_point)
                          ? minimumFlightTranslationTime3D(first_point, second_point,
                                                           config_.time_model)
                          : std::numeric_limits<double>::infinity();
  edge_cost_cache_.emplace(edge, cost);
  return cost;
}

DStarLiteKey3D
PersistentDStarLitePlanner3DImpl::calculateKey(const PersistentPlannerNode3D node) {
  DStarLiteRecord3D& record = records_[node];
  const double minimum = std::min(record.g, record.rhs);
  return DStarLiteKey3D{minimum + heuristic(start_, node) + key_modifier_, minimum};
}

void PersistentDStarLitePlanner3DImpl::enqueue(const PersistentPlannerNode3D node,
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

void PersistentDStarLitePlanner3DImpl::updateVertex(
    const PersistentPlannerNode3D node) {
  DStarLiteRecord3D& record = records_[node];
  if (node != goal_) {
    double best = std::numeric_limits<double>::infinity();
    for (const PersistentPlannerNode3D successor : adjacentNodes(node)) {
      const double edge_cost = rawEdgeCost(node, successor);
      if (!std::isfinite(edge_cost)) {
        continue;
      }
      const auto found = records_.find(successor);
      const double successor_cost = found != records_.end()
                                        ? found->second.g
                                        : std::numeric_limits<double>::infinity();
      best = std::min(best, edge_cost + successor_cost);
    }
    record.rhs = best;
  }
  record.open_token = 0U;
  if (!approximatelyEqual(record.g, record.rhs)) {
    enqueue(node, record);
  }
}

void PersistentDStarLitePlanner3DImpl::updateAffectedVertices(
    const std::vector<GridIndex3D>& changed_cells, std::size_t& affected_states) {
  std::unordered_set<PersistentPlannerNode3D, PersistentPlannerNode3DHash> affected;
  const double raw_half_diagonal = 0.5 * std::numbers::sqrt3 * raw_bounds_.resolution_m;
  const double horizontal_reach =
      config_.physical_footprint.radius_m + raw_half_diagonal +
      std::numbers::sqrt2 * config_.minimum_horizontal_step_m *
          static_cast<double>(maximumLatticeScale());
  const double vertical_reach =
      std::max(config_.physical_footprint.lower_extent_m,
               config_.physical_footprint.upper_extent_m) +
      raw_half_diagonal +
      config_.minimum_vertical_step_m * static_cast<double>(maximumLatticeScale());
  const int horizontal_radius =
      static_cast<int>(
          std::ceil(horizontal_reach / config_.minimum_horizontal_step_m)) +
      1;
  const int vertical_radius =
      static_cast<int>(std::ceil(vertical_reach / config_.minimum_vertical_step_m)) + 1;
  for (const GridIndex3D cell : changed_cells) {
    const Point3 center = world_.observed_occupancy != nullptr
                              ? world_.observed_occupancy->cellCenter(cell)
                              : world_.static_occupancy->cellCenter(cell);
    const PersistentPlannerNode3D nearest = nearestNode(center);
    for (int z_offset = -vertical_radius; z_offset <= vertical_radius; ++z_offset) {
      for (int y_offset = -horizontal_radius; y_offset <= horizontal_radius;
           ++y_offset) {
        for (int x_offset = -horizontal_radius; x_offset <= horizontal_radius;
             ++x_offset) {
          const PersistentPlannerNode3D candidate{
              nearest.x + x_offset, nearest.y + y_offset, nearest.z + z_offset};
          if (nodeInside(candidate) && (candidate == start_ || candidate == goal_ ||
                                        records_.contains(candidate))) {
            affected.insert(candidate);
          }
        }
      }
    }
  }
  // A newly traversable edge must update both endpoints even when only one was
  // resident before the obstacle disappeared. Expanding one graph hop is
  // sufficient because D* Lite propagates the resulting inconsistency through
  // the open queue.
  const std::vector<PersistentPlannerNode3D> resident_affected{affected.begin(),
                                                               affected.end()};
  for (const PersistentPlannerNode3D node : resident_affected) {
    for (const PersistentPlannerNode3D neighbor : adjacentNodes(node)) {
      affected.insert(neighbor);
    }
  }
  std::vector<PersistentPlannerNode3D> ordered{affected.begin(), affected.end()};
  std::ranges::sort(ordered, nodeLess);
  for (const PersistentPlannerNode3D node : ordered) {
    for (const PersistentPlannerNode3D neighbor : adjacentNodes(node)) {
      edge_cost_cache_.erase(canonicalEdge(node, neighbor));
    }
  }
  for (const PersistentPlannerNode3D node : ordered) {
    updateVertex(node);
  }
  affected_states = ordered.size();
}

std::optional<DStarLiteQueueEntry3D> PersistentDStarLitePlanner3DImpl::currentTop() {
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

bool PersistentDStarLitePlanner3DImpl::shortestPathComplete() {
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

bool PersistentDStarLitePlanner3DImpl::computeShortestPath(
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
      for (const PersistentPlannerNode3D predecessor : adjacentNodes(next->node)) {
        updateVertex(predecessor);
      }
    } else {
      record.g = std::numeric_limits<double>::infinity();
      updateVertex(next->node);
      for (const PersistentPlannerNode3D predecessor : adjacentNodes(next->node)) {
        updateVertex(predecessor);
      }
    }
    ++expansions;
  }
  return true;
}

std::vector<Point3> PersistentDStarLitePlanner3DImpl::extractPath() {
  const auto start_record = records_.find(start_);
  if (start_record == records_.end() || !std::isfinite(start_record->second.g)) {
    return {};
  }
  std::vector<Point3> path;
  path.reserve(std::min(config_.maximum_extracted_path_nodes,
                        static_cast<std::size_t>(width_ + height_ + depth_)));
  path.push_back(exact_start_);
  const Point3 start_anchor = pointFor(start_);
  if (distance3D(path.back(), start_anchor) > 1.0e-9) {
    path.push_back(start_anchor);
  }
  PersistentPlannerNode3D node = start_;
  std::unordered_set<PersistentPlannerNode3D, PersistentPlannerNode3DHash> visited;
  visited.insert(node);
  while (node != goal_ && path.size() < config_.maximum_extracted_path_nodes) {
    std::optional<PersistentPlannerNode3D> selected;
    double selected_cost = std::numeric_limits<double>::infinity();
    for (const PersistentPlannerNode3D successor : adjacentNodes(node)) {
      const double edge_cost = rawEdgeCost(node, successor);
      const auto successor_record = records_.find(successor);
      if (!std::isfinite(edge_cost) || successor_record == records_.end() ||
          !std::isfinite(successor_record->second.g)) {
        continue;
      }
      const double cost = edge_cost + successor_record->second.g;
      if (!selected.has_value() || cost < selected_cost - kCostTolerance ||
          (approximatelyEqual(cost, selected_cost) &&
           (heuristic(successor, goal_) < heuristic(*selected, goal_) ||
            (approximatelyEqual(heuristic(successor, goal_),
                                heuristic(*selected, goal_)) &&
             nodeLess(successor, *selected))))) {
        selected = successor;
        selected_cost = cost;
      }
    }
    if (!selected.has_value() || visited.contains(*selected)) {
      return {};
    }
    const PersistentPlannerNode3D previous_node = node;
    node = *selected;
    adaptive_edges_in_extracted_path_ +=
        latticeLevel(previous_node, node) > 0U ? 1U : 0U;
    visited.insert(node);
    const Point3 point = pointFor(node);
    if (distance3D(path.back(), point) > 1.0e-9) {
      path.push_back(point);
    }
  }
  if (node != goal_) {
    return {};
  }
  if (distance3D(path.back(), exact_goal_) > 1.0e-9) {
    path.push_back(exact_goal_);
  }
  return path;
}

std::vector<Point3> PersistentDStarLitePlanner3DImpl::shortcutPath(
    const std::vector<Point3>& path, std::size_t& checks, std::size_t& applied,
    const Vec3& initial_velocity) const {
  if (path.size() < 3U || config_.maximum_shortcut_checks == 0U) {
    return path;
  }
  std::vector<Point3> result = path;
  FlightPathTimeProfile3D current_profile = pathTimeProfile(result, initial_velocity);
  if (!current_profile.valid) {
    return path;
  }
  std::size_t anchor = 0U;
  while (anchor + 2U < result.size()) {
    bool shortcut_applied{false};
    for (std::size_t candidate = result.size() - 1U; candidate > anchor + 1U;
         --candidate) {
      if (checks >= config_.maximum_shortcut_checks) {
        break;
      }
      ++checks;
      const bool shortcut_valid =
          anchor == 0U ? departureSegmentValid(result[anchor], result[candidate])
                       : rawSegmentValid(result[anchor], result[candidate]);
      if (!shortcut_valid) {
        continue;
      }
      std::vector<Point3> trial = result;
      trial.erase(std::next(trial.begin(), static_cast<std::ptrdiff_t>(anchor + 1U)),
                  std::next(trial.begin(), static_cast<std::ptrdiff_t>(candidate)));
      FlightPathTimeProfile3D trial_profile = pathTimeProfile(trial, initial_velocity);
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
    if (checks >= config_.maximum_shortcut_checks && !shortcut_applied) {
      break;
    }
  }
  return result;
}

std::optional<std::vector<Point3>>
PersistentDStarLitePlanner3DImpl::rebaseIncumbent(const Point3& start,
                                                  const Point3& goal) const {
  if (incumbent_.size() < 2U ||
      distance3D(incumbent_.back(), goal) > config_.goal_tolerance_m) {
    return std::nullopt;
  }
  std::vector<std::size_t> candidates(incumbent_.size());
  for (std::size_t index = 0U; index < candidates.size(); ++index) {
    candidates[index] = index;
  }
  std::ranges::sort(candidates, [&](const std::size_t first, const std::size_t second) {
    return distance3D(start, incumbent_[first]) < distance3D(start, incumbent_[second]);
  });
  for (const std::size_t candidate : candidates) {
    if (!departureSegmentValid(start, incumbent_[candidate])) {
      continue;
    }
    std::vector<Point3> rebased;
    rebased.reserve(incumbent_.size() - candidate + 1U);
    rebased.push_back(start);
    if (distance3D(start, incumbent_[candidate]) > 1.0e-9) {
      rebased.push_back(incumbent_[candidate]);
    }
    rebased.insert(
        rebased.end(),
        std::next(incumbent_.begin(), static_cast<std::ptrdiff_t>(candidate + 1U)),
        incumbent_.end());
    if (rebased.size() >= 2U && pathRawValid(rebased)) {
      return rebased;
    }
  }
  return std::nullopt;
}

bool PersistentDStarLitePlanner3DImpl::pathRawValid(
    const std::vector<Point3>& path) const {
  if (path.size() < 2U || !std::ranges::all_of(path, [&](const Point3& point) {
        return pointInsideFlightEnvelope(point);
      })) {
    return false;
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const bool segment_valid =
        index == 1U ? departureSegmentValid(path[index - 1U], path[index])
                    : rawSegmentValid(path[index - 1U], path[index]);
    if (!segment_valid) {
      return false;
    }
  }
  return true;
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

void PersistentDStarLitePlanner3DImpl::populatePathMetrics(
    PersistentPlannerResult3D& result, const Vec3& initial_velocity) const {
  for (std::size_t index = 1U; index < result.points.size(); ++index) {
    result.path_length_m += distance3D(result.points[index - 1U], result.points[index]);
  }
  if (result.points.size() < 2U) {
    return;
  }
  const FlightPathTimeProfile3D profile =
      pathTimeProfile(result.points, initial_velocity);
  if (!profile.valid) {
    return;
  }
  result.estimated_execution_time_s = profile.travel_time_s;
  result.estimated_translation_time_s = profile.translation_time_s;
  result.estimated_stationary_turn_time_s = profile.stationary_turn_time_s;
}

} // namespace drone_city_nav::detail
