#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
  key_modifier_ = 0.0;
  queue_token_ = 0U;
  queue_sequence_ = 0U;
  open_ = {};
  records_.clear();
  edge_cost_cache_.clear();
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
  result.reserve(26U);
  for (int z_offset = -1; z_offset <= 1; ++z_offset) {
    for (int y_offset = -1; y_offset <= 1; ++y_offset) {
      for (int x_offset = -1; x_offset <= 1; ++x_offset) {
        if (x_offset == 0 && y_offset == 0 && z_offset == 0) {
          continue;
        }
        const PersistentPlannerNode3D candidate{node.x + x_offset, node.y + y_offset,
                                                node.z + z_offset};
        if (nodeInside(candidate) && pointInsideFlightEnvelope(pointFor(candidate))) {
          result.push_back(candidate);
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
  if (!nodeInside(first) || !nodeInside(second) || first == second) {
    return std::numeric_limits<double>::infinity();
  }
  const PersistentPlannerEdge3D edge = canonicalEdge(first, second);
  if (const auto found = edge_cost_cache_.find(edge); found != edge_cost_cache_.end()) {
    return found->second;
  }
  const Point3 first_point = pointFor(first);
  const Point3 second_point = pointFor(second);
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
  const double horizontal_reach = config_.physical_footprint.radius_m +
                                  raw_half_diagonal +
                                  std::numbers::sqrt2 * config_.horizontal_step_m;
  const double vertical_reach = std::max(config_.physical_footprint.lower_extent_m,
                                         config_.physical_footprint.upper_extent_m) +
                                raw_half_diagonal + config_.vertical_step_m;
  const int horizontal_radius =
      static_cast<int>(std::ceil(horizontal_reach / config_.horizontal_step_m)) + 1;
  const int vertical_radius =
      static_cast<int>(std::ceil(vertical_reach / config_.vertical_step_m)) + 1;
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
          if (nodeInside(candidate)) {
            affected.insert(candidate);
          }
        }
      }
    }
  }
  std::vector<PersistentPlannerNode3D> ordered{affected.begin(), affected.end()};
  std::ranges::sort(ordered, nodeLess);
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
    node = *selected;
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
    const std::vector<Point3>& path, std::size_t& checks, std::size_t& applied) const {
  if (path.size() < 3U || config_.maximum_shortcut_checks == 0U) {
    return path;
  }
  std::vector<Point3> result;
  result.reserve(path.size());
  std::size_t anchor = 0U;
  result.push_back(path.front());
  while (anchor + 1U < path.size()) {
    std::size_t selected = anchor + 1U;
    for (std::size_t candidate = path.size() - 1U; candidate > anchor + 1U;
         --candidate) {
      if (checks >= config_.maximum_shortcut_checks) {
        break;
      }
      ++checks;
      if (rawSegmentValid(path[anchor], path[candidate])) {
        selected = candidate;
        break;
      }
    }
    applied += selected - anchor - 1U;
    result.push_back(path[selected]);
    anchor = selected;
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
    if (!rawSegmentValid(start, incumbent_[candidate])) {
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
    if (!rawSegmentValid(path[index - 1U], path[index])) {
      return false;
    }
  }
  return true;
}

void PersistentDStarLitePlanner3DImpl::populatePathMetrics(
    PersistentPlannerResult3D& result, const Vec3& initial_velocity) const {
  for (std::size_t index = 1U; index < result.points.size(); ++index) {
    result.path_length_m += distance3D(result.points[index - 1U], result.points[index]);
  }
  if (result.points.size() < 2U) {
    return;
  }
  std::vector<double> speed_limits(result.points.size(),
                                   config_.time_model.maximum_horizontal_speed_mps);
  std::vector<std::uint8_t> stop_turn_flags(result.points.size(), 0U);
  for (std::size_t index = 1U; index + 1U < result.points.size(); ++index) {
    const Point3& first = result.points[index - 1U];
    const Point3& center = result.points[index];
    const Point3& last = result.points[index + 1U];
    const double first_length = distance3D(first, center);
    const double second_length = distance3D(center, last);
    if (!(first_length > kCostTolerance) || !(second_length > kCostTolerance)) {
      continue;
    }
    const double alignment = ((center.x - first.x) * (last.x - center.x) +
                              (center.y - first.y) * (last.y - center.y) +
                              (center.z - first.z) * (last.z - center.z)) /
                             (first_length * second_length);
    if (alignment < config_.minimum_continuous_turn_alignment) {
      stop_turn_flags[index] = 1U;
    }
  }
  const FlightPathTimeProfile3D profile =
      parameterizeFlightPathTime3D(result.points, speed_limits, stop_turn_flags,
                                   initial_velocity, true, config_.time_model);
  if (!profile.valid) {
    return;
  }
  result.estimated_execution_time_s = profile.travel_time_s;
  result.estimated_translation_time_s = profile.translation_time_s;
  result.estimated_stationary_turn_time_s = profile.stationary_turn_time_s;
}

} // namespace drone_city_nav::detail
