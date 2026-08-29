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
  dstar_session_.reset();
  resetFeasibilitySearch();
  resetExecutionTimeSearch();
  dstar_session_.search_generation_ =
      dstar_session_.search_generation_ == std::numeric_limits<std::uint64_t>::max()
          ? 1U
          : dstar_session_.search_generation_ + 1U;
  DStarLiteRecord3D& goal_record = dstar_session_.records_[goal_];
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
  // Every horizontal graph edge is axial or diagonal. Its obstacle-free
  // lattice distance is therefore an admissible, consistent lower bound that
  // is strictly tighter than continuous Euclidean distance for non-45-degree
  // missions. Adaptive edges preserve the same per-metre cost.
  const double delta_x_m = static_cast<double>(std::abs(first.x - second.x)) *
                           config_.minimum_horizontal_step_m;
  const double delta_y_m = static_cast<double>(std::abs(first.y - second.y)) *
                           config_.minimum_horizontal_step_m;
  const double horizontal_diagonal_m =
      std::max(delta_x_m, delta_y_m) +
      (std::numbers::sqrt2 - 1.0) * std::min(delta_x_m, delta_y_m);
  const double vertical_m = static_cast<double>(std::abs(first.z - second.z)) *
                            config_.minimum_vertical_step_m;
  const double euclidean_m = std::hypot(std::hypot(delta_x_m, delta_y_m), vertical_m);
  return std::max(
      {horizontal_diagonal_m / config_.time_model.maximum_horizontal_speed_mps,
       vertical_m / config_.time_model.maximum_vertical_speed_mps,
       horizontal_diagonal_m / config_.time_model.maximum_translational_speed_mps,
       euclidean_m / config_.time_model.maximum_translational_speed_mps});
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
  if (const auto found = dstar_session_.edge_cost_cache_.find(edge);
      found != dstar_session_.edge_cost_cache_.end()) {
    return found->second;
  }
  const Point3 first_point = pointFor(first);
  const Point3 second_point = pointFor(second);
  ++raw_edge_validation_checks_;
  const double cost = rawSegmentValid(first_point, second_point)
                          ? minimumFlightTranslationTime3D(first_point, second_point,
                                                           config_.time_model)
                          : std::numeric_limits<double>::infinity();
  dstar_session_.edge_cost_cache_.emplace(edge, cost);
  return cost;
}

DStarLiteKey3D
PersistentDStarLitePlanner3DImpl::calculateKey(const PersistentPlannerNode3D node) {
  DStarLiteRecord3D& record = dstar_session_.records_[node];
  const double minimum = std::min(record.g, record.rhs);
  return DStarLiteKey3D{
      minimum + heuristic(start_, node) + dstar_session_.key_modifier_, minimum};
}

void PersistentDStarLitePlanner3DImpl::enqueue(const PersistentPlannerNode3D node,
                                               DStarLiteRecord3D& record) {
  ++dstar_session_.queue_token_;
  if (dstar_session_.queue_token_ == 0U) {
    dstar_session_.queue_token_ = 1U;
  }
  ++dstar_session_.queue_sequence_;
  if (dstar_session_.queue_sequence_ == 0U) {
    dstar_session_.queue_sequence_ = 1U;
  }
  record.open_token = dstar_session_.queue_token_;
  dstar_session_.open_.push(DStarLiteQueueEntry3D{
      .key = calculateKey(node),
      .node = node,
      .token = record.open_token,
      .sequence = dstar_session_.queue_sequence_,
  });
}

void PersistentDStarLitePlanner3DImpl::updateVertex(
    const PersistentPlannerNode3D node) {
  DStarLiteRecord3D& record = dstar_session_.records_[node];
  if (node != goal_) {
    double best = std::numeric_limits<double>::infinity();
    for (const PersistentPlannerNode3D successor : adjacentNodes(node)) {
      const double edge_cost = rawEdgeCost(node, successor);
      if (!std::isfinite(edge_cost)) {
        continue;
      }
      const auto found = dstar_session_.records_.find(successor);
      const double successor_cost = found != dstar_session_.records_.end()
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

void PersistentDStarLitePlanner3DImpl::scheduleAffectedVertices(
    const std::vector<GridIndex3D>& changed_cells, std::size_t& affected_states) {
  std::unordered_set<PersistentPlannerNode3D, PersistentPlannerNode3DHash>
      resident_candidates;
  const double raw_half_diagonal =
      0.5 * std::numbers::sqrt3 * lattice_.raw_bounds_.resolution_m;
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
                                        dstar_session_.records_.contains(candidate))) {
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
  for (const PersistentPlannerNode3D node : resident_candidates) {
    for (const PersistentPlannerNode3D neighbor : adjacentNodes(node)) {
      const PersistentPlannerEdge3D edge = canonicalEdge(node, neighbor);
      const auto cached = dstar_session_.edge_cost_cache_.find(edge);
      if (cached == dstar_session_.edge_cost_cache_.end()) {
        continue;
      }
      dstar_session_.edge_cost_cache_.erase(cached);
      affected.insert(edge.first);
      affected.insert(edge.second);
    }
  }
  std::vector<PersistentPlannerNode3D> ordered{affected.begin(), affected.end()};
  std::ranges::sort(ordered, nodeLess);
  for (const PersistentPlannerNode3D node : ordered) {
    if (dstar_session_.pending_repair_members_.insert(node).second) {
      dstar_session_.pending_repair_nodes_.push_back(node);
    }
  }
  affected_states = ordered.size();
}

bool PersistentDStarLitePlanner3DImpl::continueAffectedVertexRepair(
    const std::chrono::steady_clock::time_point deadline,
    const std::size_t maximum_vertices, std::size_t& processed_vertices) {
  processed_vertices = 0U;
  while (!dstar_session_.pending_repair_nodes_.empty() &&
         processed_vertices < maximum_vertices &&
         std::chrono::steady_clock::now() < deadline) {
    const PersistentPlannerNode3D node = dstar_session_.pending_repair_nodes_.front();
    dstar_session_.pending_repair_nodes_.pop_front();
    dstar_session_.pending_repair_members_.erase(node);
    updateVertex(node);
    ++processed_vertices;
  }
  return dstar_session_.pending_repair_nodes_.empty();
}

std::optional<DStarLiteQueueEntry3D> PersistentDStarLitePlanner3DImpl::currentTop() {
  while (!dstar_session_.open_.empty()) {
    const DStarLiteQueueEntry3D& entry = dstar_session_.open_.top();
    const auto record = dstar_session_.records_.find(entry.node);
    if (record != dstar_session_.records_.end() &&
        record->second.open_token == entry.token && entry.token != 0U) {
      return entry;
    }
    dstar_session_.open_.pop();
  }
  return std::nullopt;
}

bool PersistentDStarLitePlanner3DImpl::shortestPathComplete() {
  const auto start_record = dstar_session_.records_.find(start_);
  const double start_g = start_record != dstar_session_.records_.end()
                             ? start_record->second.g
                             : std::numeric_limits<double>::infinity();
  const double start_rhs = start_record != dstar_session_.records_.end()
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
    dstar_session_.open_.pop();
    DStarLiteRecord3D& record = dstar_session_.records_.at(next->node);
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

std::optional<std::vector<Point3>> PersistentDStarLitePlanner3DImpl::findFeasiblePath(
    const std::chrono::steady_clock::time_point deadline,
    const std::size_t maximum_expansions, std::size_t& expansions) {
  expansions = 0U;
  if (!feasibility_search_.initialized_) {
    initializeFeasibilitySearch();
  }

  const auto reconstruct = [&](const PersistentPlannerNode3D terminal)
      -> std::optional<std::vector<Point3>> {
    std::vector<PersistentPlannerNode3D> nodes;
    nodes.reserve(std::min(config_.maximum_extracted_path_nodes,
                           feasibility_search_.costs_.size()));
    PersistentPlannerNode3D current = terminal;
    while (true) {
      nodes.push_back(current);
      if (current == start_) {
        break;
      }
      if (nodes.size() >= config_.maximum_extracted_path_nodes) {
        return std::nullopt;
      }
      const auto parent = feasibility_search_.parents_.find(current);
      if (parent == feasibility_search_.parents_.end()) {
        return std::nullopt;
      }
      current = parent->second;
    }
    std::ranges::reverse(nodes);
    std::vector<Point3> path;
    path.reserve(nodes.size() + 2U);
    path.push_back(exact_start_);
    for (const PersistentPlannerNode3D node : nodes) {
      const Point3 point = pointFor(node);
      if (distance3D(path.back(), point) > kCostTolerance) {
        path.push_back(point);
      }
    }
    if (distance3D(path.back(), exact_goal_) > kCostTolerance) {
      path.push_back(exact_goal_);
    }
    return path.size() >= 2U ? std::optional<std::vector<Point3>>{std::move(path)}
                             : std::nullopt;
  };

  while (!feasibility_search_.open_.empty() && expansions < maximum_expansions &&
         std::chrono::steady_clock::now() < deadline) {
    const FeasibilityQueueEntry3D current = feasibility_search_.open_.top();
    feasibility_search_.open_.pop();
    const auto current_cost = feasibility_search_.costs_.find(current.node);
    if (current_cost == feasibility_search_.costs_.end() ||
        !approximatelyEqual(current.cost_from_start_s, current_cost->second)) {
      continue;
    }
    ++expansions;

    const Point3 current_point = pointFor(current.node);
    const bool direct_departure =
        current.node == start_ &&
        distance3D(exact_start_, current_point) <= kCostTolerance;
    const bool goal_connector_valid =
        direct_departure ? departureSegmentValid(exact_start_, exact_goal_)
                         : rawSegmentValid(current_point, exact_goal_);
    if (goal_connector_valid) {
      std::optional<std::vector<Point3>> candidate = reconstruct(current.node);
      if (candidate.has_value() && pathRawValid(*candidate)) {
        return candidate;
      }
    }

    for (const PersistentPlannerNode3D neighbor : adjacentNodes(current.node)) {
      ++lattice_edge_queries_;
      const std::size_t level = latticeLevel(current.node, neighbor);
      if (level > 0U) {
        ++adaptive_edge_queries_;
        maximum_queried_lattice_level_ =
            std::max(maximum_queried_lattice_level_, level);
      }
      ++raw_edge_validation_checks_;
      if (!rawSegmentValid(pointFor(current.node), pointFor(neighbor))) {
        continue;
      }
      const double transition_cost = minimumFlightTranslationTime3D(
          pointFor(current.node), pointFor(neighbor), config_.time_model);
      const double candidate_cost = current.cost_from_start_s + transition_cost;
      const auto existing = feasibility_search_.costs_.find(neighbor);
      if (existing != feasibility_search_.costs_.end() &&
          (existing->second < candidate_cost ||
           approximatelyEqual(existing->second, candidate_cost))) {
        continue;
      }
      feasibility_search_.costs_[neighbor] = candidate_cost;
      feasibility_search_.parents_.insert_or_assign(neighbor, current.node);
      const std::size_t depth = current.depth + 1U;
      ++feasibility_search_.queue_sequence_;
      if (feasibility_search_.queue_sequence_ == 0U) {
        feasibility_search_.queue_sequence_ = 1U;
      }
      feasibility_search_.open_.push(FeasibilityQueueEntry3D{
          .estimated_total_s = candidate_cost + heuristic(neighbor, goal_),
          .cost_from_start_s = candidate_cost,
          .depth = depth,
          .node = neighbor,
          .sequence = feasibility_search_.queue_sequence_,
      });
    }
  }
  return std::nullopt;
}

void PersistentDStarLitePlanner3DImpl::resetFeasibilitySearch() noexcept {
  feasibility_search_.reset();
}

void PersistentDStarLitePlanner3DImpl::initializeFeasibilitySearch() {
  resetFeasibilitySearch();
  feasibility_search_.initialized_ = true;
  feasibility_search_.queue_sequence_ = 1U;
  feasibility_search_.costs_.emplace(start_, 0.0);
  feasibility_search_.open_.push(FeasibilityQueueEntry3D{
      .estimated_total_s = heuristic(start_, goal_),
      .cost_from_start_s = 0.0,
      .depth = 0U,
      .node = start_,
      .sequence = feasibility_search_.queue_sequence_,
  });
}

std::vector<Point3> PersistentDStarLitePlanner3DImpl::extractPath() {
  const auto start_record = dstar_session_.records_.find(start_);
  if (start_record == dstar_session_.records_.end() ||
      !std::isfinite(start_record->second.g)) {
    return {};
  }
  std::vector<Point3> path;
  path.reserve(std::min(
      config_.maximum_extracted_path_nodes,
      static_cast<std::size_t>(lattice_.width_ + lattice_.height_ + lattice_.depth_)));
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
      const auto successor_record = dstar_session_.records_.find(successor);
      if (!std::isfinite(edge_cost) ||
          successor_record == dstar_session_.records_.end() ||
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
    if (!departureSegmentValid(start, incumbent[candidate])) {
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

std::optional<SpatialRouteCandidate3D> PersistentDStarLitePlanner3DImpl::makeCandidate(
    std::vector<Point3> path, const SpatialRouteCandidateSource3D source,
    const Vec3& initial_velocity) const {
  if (path.size() < 2U || !pathRawValid(path)) {
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

} // namespace drone_city_nav::detail
