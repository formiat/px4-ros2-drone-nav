#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

namespace drone_city_nav::detail {
namespace {

constexpr double kDirectionEpsilon{1.0e-9};
constexpr double kTimeCostTolerance{1.0e-12};

[[nodiscard]] bool costLess(const double first, const double second) noexcept {
  return first < second - kTimeCostTolerance *
                              std::max({1.0, std::abs(first), std::abs(second)});
}

[[nodiscard]] int sign(const double value) noexcept {
  if (value > kDirectionEpsilon) {
    return 1;
  }
  if (value < -kDirectionEpsilon) {
    return -1;
  }
  return 0;
}

} // namespace

void PersistentDStarLitePlanner3DImpl::resetExecutionTimeSearch() noexcept {
  execution_time_search_initialized_ = false;
  execution_time_search_complete_ = false;
  execution_time_start_from_rest_ = false;
  execution_time_start_ = {};
  execution_time_goal_.reset();
  execution_time_spatial_incumbent_.clear();
  execution_time_goal_cost_s_ = std::numeric_limits<double>::infinity();
  execution_time_queue_sequence_ = 0U;
  execution_time_open_ = {};
  execution_time_costs_.clear();
  execution_time_parents_.clear();
}

PersistentPlannerDirection3D PersistentDStarLitePlanner3DImpl::directionForVector(
    const Vec3& vector) const noexcept {
  return PersistentPlannerDirection3D{
      .x = static_cast<std::int8_t>(sign(vector.x)),
      .y = static_cast<std::int8_t>(sign(vector.y)),
      .z = static_cast<std::int8_t>(sign(vector.z)),
  };
}

PersistentPlannerDirection3D PersistentDStarLitePlanner3DImpl::directionForEdge(
    const PersistentPlannerNode3D first,
    const PersistentPlannerNode3D second) const noexcept {
  return PersistentPlannerDirection3D{
      .x = static_cast<std::int8_t>(sign(static_cast<double>(second.x - first.x))),
      .y = static_cast<std::int8_t>(sign(static_cast<double>(second.y - first.y))),
      .z = static_cast<std::int8_t>(sign(static_cast<double>(second.z - first.z))),
  };
}

Vec3 PersistentDStarLitePlanner3DImpl::directionVector(
    const PersistentPlannerDirection3D direction) const noexcept {
  return Vec3{
      static_cast<double>(direction.x) * config_.minimum_horizontal_step_m,
      static_cast<double>(direction.y) * config_.minimum_horizontal_step_m,
      static_cast<double>(direction.z) * config_.minimum_vertical_step_m,
  };
}

PersistentPlannerTimeState3D PersistentDStarLitePlanner3DImpl::executionTimeStartState(
    const PersistentPlannerRequest3D& request,
    const PersistentPlannerNode3D start_anchor) const noexcept {
  const Point3 anchor = pointFor(start_anchor);
  Vec3 incoming{
      anchor.x - request.start.x,
      anchor.y - request.start.y,
      anchor.z - request.start.z,
  };
  if (std::hypot(std::hypot(incoming.x, incoming.y), incoming.z) <= kDirectionEpsilon) {
    incoming = request.velocity;
  }
  return PersistentPlannerTimeState3D{
      .position = start_anchor,
      .incoming = directionForVector(incoming),
  };
}

void PersistentDStarLitePlanner3DImpl::initializeExecutionTimeSearch(
    const PersistentPlannerRequest3D& request,
    const PersistentPlannerTimeState3D start_state) {
  resetExecutionTimeSearch();
  execution_time_search_initialized_ = true;
  execution_time_start_ = start_state;
  execution_time_start_from_rest_ =
      std::hypot(std::hypot(request.velocity.x, request.velocity.y),
                 request.velocity.z) <= kDirectionEpsilon;
  execution_time_costs_.emplace(start_state, 0.0);
  execution_time_open_.push(PersistentPlannerTimeQueueEntry3D{
      .estimated_total_s = executionTimeHeuristic(start_state),
      .cost_from_start_s = 0.0,
      .state = start_state,
      .sequence = ++execution_time_queue_sequence_,
  });
  seedExecutionTimeIncumbent();
}

void PersistentDStarLitePlanner3DImpl::seedExecutionTimeIncumbent() {
  const std::vector<Point3> spatial_path = extractPath();
  if (spatial_path.size() < 2U || !pathRawValid(spatial_path)) {
    return;
  }
  PersistentPlannerTimeState3D current = execution_time_start_;
  double cost = 0.0;
  for (std::size_t index = 1U; index < spatial_path.size(); ++index) {
    if (current.position == goal_) {
      break;
    }
    const PersistentPlannerNode3D position = nearestNode(spatial_path[index]);
    if (position == current.position) {
      continue;
    }
    const PersistentPlannerTimeState3D successor{
        .position = position,
        .incoming = directionForEdge(current.position, position),
    };
    const double transition = executionTimeTransitionCost(current, successor);
    if (!std::isfinite(transition)) {
      return;
    }
    cost += transition;
    current = successor;
  }
  if (current.position != goal_) {
    return;
  }
  cost += executionTimeTerminalCost(current);
  if (!std::isfinite(cost)) {
    return;
  }
  execution_time_spatial_incumbent_ = spatial_path;
  execution_time_goal_cost_s_ = cost;
}

bool PersistentDStarLitePlanner3DImpl::hasExecutionTimeIncumbent() const noexcept {
  return execution_time_goal_.has_value() || !execution_time_spatial_incumbent_.empty();
}

std::vector<Point3> PersistentDStarLitePlanner3DImpl::bestExecutionTimePath() {
  if (execution_time_goal_.has_value()) {
    return extractExecutionTimePath();
  }
  std::vector<Point3> path = execution_time_spatial_incumbent_;
  if (path.empty()) {
    return path;
  }
  path.front() = exact_start_;
  path.back() = exact_goal_;
  adaptive_edges_in_extracted_path_ = 0U;
  PersistentPlannerNode3D previous = execution_time_start_.position;
  for (std::size_t index = 1U; index < path.size() && previous != goal_; ++index) {
    const PersistentPlannerNode3D current = nearestNode(path[index]);
    if (current == previous) {
      continue;
    }
    adaptive_edges_in_extracted_path_ += latticeLevel(previous, current) > 0U ? 1U : 0U;
    previous = current;
  }
  return path;
}

double PersistentDStarLitePlanner3DImpl::executionTimeHeuristic(
    const PersistentPlannerTimeState3D& state) const noexcept {
  const double euclidean = minimumFlightTranslationTime3D(
      pointFor(state.position), exact_goal_, config_.time_model);
  if (!dstar_cost_to_goal_heuristic_admissible_) {
    return euclidean;
  }
  const auto record = records_.find(state.position);
  return record != records_.end() && std::isfinite(record->second.g)
             ? std::max(euclidean, record->second.g)
             : euclidean;
}

double PersistentDStarLitePlanner3DImpl::executionTimeTransitionCost(
    const PersistentPlannerTimeState3D& first,
    const PersistentPlannerTimeState3D& second) {
  if (second.incoming != directionForEdge(first.position, second.position)) {
    return std::numeric_limits<double>::infinity();
  }
  const double translation_cost = rawEdgeCost(first.position, second.position);
  if (!std::isfinite(translation_cost)) {
    return translation_cost;
  }
  const Point3 first_point = pointFor(first.position);
  const Point3 second_point = pointFor(second.position);
  const Vec3 outgoing{second_point.x - first_point.x, second_point.y - first_point.y,
                      second_point.z - first_point.z};
  if (first.incoming.empty()) {
    if (first == execution_time_start_ && execution_time_start_from_rest_) {
      return translation_cost +
             estimatedFlightRestTransitionDelay3D(outgoing, config_.time_model);
    }
    return translation_cost;
  }
  const Vec3 incoming = directionVector(first.incoming);
  if (!requiresFlightStopAndTurn3D(incoming, outgoing,
                                   config_.minimum_continuous_turn_alignment)) {
    return translation_cost;
  }
  return translation_cost +
         estimatedFlightStopAndTurnDelay3D(incoming, outgoing, config_.time_model);
}

double PersistentDStarLitePlanner3DImpl::executionTimeTerminalCost(
    const PersistentPlannerTimeState3D& state) const noexcept {
  if (state.position != goal_) {
    return std::numeric_limits<double>::infinity();
  }
  const Point3 anchor = pointFor(goal_);
  const Vec3 connector{exact_goal_.x - anchor.x, exact_goal_.y - anchor.y,
                       exact_goal_.z - anchor.z};
  const double connector_length =
      std::hypot(std::hypot(connector.x, connector.y), connector.z);
  if (connector_length > kDirectionEpsilon) {
    double cost =
        minimumFlightTranslationTime3D(anchor, exact_goal_, config_.time_model) +
        estimatedFlightRestTransitionDelay3D(connector, config_.time_model);
    if (!state.incoming.empty()) {
      const Vec3 incoming = directionVector(state.incoming);
      if (requiresFlightStopAndTurn3D(incoming, connector,
                                      config_.minimum_continuous_turn_alignment)) {
        cost +=
            estimatedFlightStopAndTurnDelay3D(incoming, connector, config_.time_model);
      }
    }
    return cost;
  }
  return state.incoming.empty()
             ? 0.0
             : estimatedFlightRestTransitionDelay3D(directionVector(state.incoming),
                                                    config_.time_model);
}

std::optional<std::vector<Point3>>
PersistentDStarLitePlanner3DImpl::continueExecutionTimeSearch(
    const std::chrono::steady_clock::time_point deadline,
    const std::size_t maximum_expansions, std::size_t& expansions) {
  if (!execution_time_search_initialized_) {
    return std::nullopt;
  }
  if (execution_time_search_complete_) {
    return hasExecutionTimeIncumbent()
               ? std::optional<std::vector<Point3>>{bestExecutionTimePath()}
               : std::nullopt;
  }

  while (true) {
    while (!execution_time_open_.empty()) {
      const PersistentPlannerTimeQueueEntry3D& entry = execution_time_open_.top();
      const auto found = execution_time_costs_.find(entry.state);
      if (found != execution_time_costs_.end() &&
          !costLess(found->second, entry.cost_from_start_s) &&
          !costLess(entry.cost_from_start_s, found->second)) {
        break;
      }
      execution_time_open_.pop();
    }

    if (execution_time_open_.empty() ||
        (hasExecutionTimeIncumbent() &&
         !costLess(execution_time_open_.top().estimated_total_s,
                   execution_time_goal_cost_s_))) {
      execution_time_search_complete_ = true;
      return hasExecutionTimeIncumbent()
                 ? std::optional<std::vector<Point3>>{bestExecutionTimePath()}
                 : std::nullopt;
    }
    if (expansions >= maximum_expansions ||
        std::chrono::steady_clock::now() >= deadline) {
      return std::nullopt;
    }

    const PersistentPlannerTimeQueueEntry3D current = execution_time_open_.top();
    execution_time_open_.pop();
    ++expansions;
    if (current.state.position == goal_) {
      const double terminal_cost = executionTimeTerminalCost(current.state);
      const double total_cost = current.cost_from_start_s + terminal_cost;
      if (std::isfinite(total_cost) &&
          (!hasExecutionTimeIncumbent() ||
           costLess(total_cost, execution_time_goal_cost_s_))) {
        execution_time_goal_ = current.state;
        execution_time_goal_cost_s_ = total_cost;
      }
      continue;
    }

    for (const PersistentPlannerNode3D successor_position :
         adjacentNodes(current.state.position)) {
      const PersistentPlannerTimeState3D successor{
          .position = successor_position,
          .incoming = directionForEdge(current.state.position, successor_position),
      };
      const double transition_cost =
          executionTimeTransitionCost(current.state, successor);
      const double candidate_cost = current.cost_from_start_s + transition_cost;
      if (!std::isfinite(candidate_cost)) {
        continue;
      }
      const auto existing = execution_time_costs_.find(successor);
      if (existing != execution_time_costs_.end() &&
          !costLess(candidate_cost, existing->second)) {
        continue;
      }
      execution_time_costs_[successor] = candidate_cost;
      execution_time_parents_[successor] = current.state;
      ++execution_time_queue_sequence_;
      if (execution_time_queue_sequence_ == 0U) {
        execution_time_queue_sequence_ = 1U;
      }
      const double estimated_total = candidate_cost + executionTimeHeuristic(successor);
      if (hasExecutionTimeIncumbent() &&
          !costLess(estimated_total, execution_time_goal_cost_s_)) {
        continue;
      }
      execution_time_open_.push(PersistentPlannerTimeQueueEntry3D{
          .estimated_total_s = estimated_total,
          .cost_from_start_s = candidate_cost,
          .state = successor,
          .sequence = execution_time_queue_sequence_,
      });
    }
  }
}

std::vector<Point3> PersistentDStarLitePlanner3DImpl::extractExecutionTimePath() {
  if (!execution_time_goal_.has_value()) {
    return {};
  }
  std::vector<PersistentPlannerTimeState3D> states;
  states.reserve(config_.maximum_extracted_path_nodes);
  PersistentPlannerTimeState3D current = *execution_time_goal_;
  while (true) {
    states.push_back(current);
    if (current == execution_time_start_) {
      break;
    }
    if (states.size() >= config_.maximum_extracted_path_nodes) {
      return {};
    }
    const auto parent = execution_time_parents_.find(current);
    if (parent == execution_time_parents_.end()) {
      return {};
    }
    current = parent->second;
  }
  std::ranges::reverse(states);
  adaptive_edges_in_extracted_path_ = 0U;
  for (std::size_t index = 1U; index < states.size(); ++index) {
    adaptive_edges_in_extracted_path_ +=
        latticeLevel(states[index - 1U].position, states[index].position) > 0U ? 1U
                                                                               : 0U;
  }

  std::vector<Point3> path;
  path.reserve(states.size() + 2U);
  path.push_back(exact_start_);
  for (const PersistentPlannerTimeState3D& state : states) {
    const Point3 point = pointFor(state.position);
    if (distance3D(path.back(), point) > kDirectionEpsilon) {
      path.push_back(point);
    }
  }
  if (distance3D(path.back(), exact_goal_) > kDirectionEpsilon) {
    path.push_back(exact_goal_);
  }
  return path;
}

} // namespace drone_city_nav::detail
