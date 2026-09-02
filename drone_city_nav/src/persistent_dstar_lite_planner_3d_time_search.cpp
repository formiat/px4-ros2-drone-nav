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

void ExecutionTimeRefiner3D::reset() noexcept {
  request_ = {};
  initialized_ = false;
  complete_ = false;
  goal_.reset();
  spatial_incumbent_.clear();
  goal_cost_s_ = std::numeric_limits<double>::infinity();
  queue_sequence_ = 0U;
  adaptive_edges_in_extracted_path_ = 0U;
  open_ = {};
  costs_.clear();
  parents_.clear();
}

bool ExecutionTimeRefiner3D::initialized() const noexcept {
  return initialized_;
}

bool ExecutionTimeRefiner3D::complete() const noexcept {
  return complete_;
}

bool ExecutionTimeRefiner3D::startChanged(const PersistentPlannerTimeState3D& start,
                                          const bool start_from_rest) const noexcept {
  return initialized_ &&
         (request_.start != start || request_.start_from_rest != start_from_rest);
}

bool ExecutionTimeRefiner3D::goalChanged(const Point3& exact_goal) const noexcept {
  return initialized_ && distance3D(request_.exact_goal, exact_goal) > 1.0e-9;
}

std::size_t ExecutionTimeRefiner3D::records() const noexcept {
  return costs_.size();
}

std::size_t ExecutionTimeRefiner3D::openEntries() const noexcept {
  return open_.size();
}

double ExecutionTimeRefiner3D::objectiveSeconds() const noexcept {
  return std::isfinite(goal_cost_s_) ? goal_cost_s_ : 0.0;
}

std::size_t ExecutionTimeRefiner3D::adaptiveEdgesInExtractedPath() const noexcept {
  return adaptive_edges_in_extracted_path_;
}

PersistentPlannerDirection3D directionForVector3D(const Vec3& vector) noexcept {
  return PersistentPlannerDirection3D{
      .x = static_cast<std::int8_t>(sign(vector.x)),
      .y = static_cast<std::int8_t>(sign(vector.y)),
      .z = static_cast<std::int8_t>(sign(vector.z)),
  };
}

PersistentPlannerDirection3D
directionForEdge3D(const PersistentPlannerNode3D first,
                   const PersistentPlannerNode3D second) noexcept {
  return PersistentPlannerDirection3D{
      .x = static_cast<std::int8_t>(sign(static_cast<double>(second.x - first.x))),
      .y = static_cast<std::int8_t>(sign(static_cast<double>(second.y - first.y))),
      .z = static_cast<std::int8_t>(sign(static_cast<double>(second.z - first.z))),
  };
}

Vec3 ExecutionTimeRefiner3D::directionVector(
    const PersistentPlannerDirection3D direction) const noexcept {
  return Vec3{
      static_cast<double>(direction.x) * config_->minimum_horizontal_step_m,
      static_cast<double>(direction.y) * config_->minimum_horizontal_step_m,
      static_cast<double>(direction.z) * config_->minimum_vertical_step_m,
  };
}

ExecutionTimeRefiner3D::Request3D PersistentDStarLitePlanner3DImpl::refinementRequest(
    const PersistentPlannerRequest3D& request,
    const PersistentPlannerNode3D start_anchor,
    const PersistentPlannerNode3D goal_anchor) const noexcept {
  const Point3 anchor = lattice_.pointFor(start_anchor);
  Vec3 incoming{
      anchor.x - request.start.x,
      anchor.y - request.start.y,
      anchor.z - request.start.z,
  };
  if (std::hypot(std::hypot(incoming.x, incoming.y), incoming.z) <= kDirectionEpsilon) {
    incoming = request.velocity;
  }
  return ExecutionTimeRefiner3D::Request3D{
      .start =
          PersistentPlannerTimeState3D{
              .position = start_anchor,
              .incoming = directionForVector3D(incoming),
          },
      .goal_anchor = goal_anchor,
      .exact_start = request.start,
      .exact_goal = request.mission_goal,
      .start_from_rest = std::hypot(std::hypot(request.velocity.x, request.velocity.y),
                                    request.velocity.z) <= kDirectionEpsilon,
  };
}

void ExecutionTimeRefiner3D::begin(const Request3D& request,
                                   const std::vector<Point3>& spatial_route) {
  reset();
  initialized_ = true;
  request_ = request;
  costs_.emplace(request_.start, 0.0);
  open_.push(PersistentPlannerTimeQueueEntry3D{
      .estimated_total_s = heuristic(request_.start),
      .cost_from_start_s = 0.0,
      .state = request_.start,
      .sequence = ++queue_sequence_,
  });
  seedIncumbent(spatial_route);
}

void ExecutionTimeRefiner3D::seedIncumbent(const std::vector<Point3>& spatial_route) {
  if (spatial_route.size() < 2U || !lattice_->pathTraversable(spatial_route)) {
    return;
  }
  const std::vector<Point3>& spatial_path = spatial_route;
  PersistentPlannerTimeState3D current = request_.start;
  double cost = 0.0;
  for (std::size_t index = 1U; index < spatial_path.size(); ++index) {
    if (current.position == request_.goal_anchor) {
      break;
    }
    const PersistentPlannerNode3D position = lattice_->nearestNode(spatial_path[index]);
    if (position == current.position) {
      continue;
    }
    const PersistentPlannerTimeState3D successor{
        .position = position,
        .incoming = directionForEdge3D(current.position, position),
    };
    const double transition = transitionCost(current, successor);
    if (!std::isfinite(transition)) {
      return;
    }
    cost += transition;
    current = successor;
  }
  if (current.position != request_.goal_anchor) {
    return;
  }
  cost += terminalCost(current);
  if (!std::isfinite(cost)) {
    return;
  }
  spatial_incumbent_ = spatial_route;
  goal_cost_s_ = cost;
}

bool ExecutionTimeRefiner3D::hasIncumbent() const noexcept {
  return goal_.has_value() || !spatial_incumbent_.empty();
}

std::vector<Point3> ExecutionTimeRefiner3D::bestPath() {
  if (goal_.has_value()) {
    return extractPath();
  }
  std::vector<Point3> path = spatial_incumbent_;
  if (path.empty()) {
    return path;
  }
  path.front() = request_.exact_start;
  path.back() = request_.exact_goal;
  adaptive_edges_in_extracted_path_ = 0U;
  PersistentPlannerNode3D previous = request_.start.position;
  for (std::size_t index = 1U; index < path.size() && previous != request_.goal_anchor;
       ++index) {
    const PersistentPlannerNode3D current = lattice_->nearestNode(path[index]);
    if (current == previous) {
      continue;
    }
    adaptive_edges_in_extracted_path_ +=
        lattice_->level(previous, current) > 0U ? 1U : 0U;
    previous = current;
  }
  return path;
}

double ExecutionTimeRefiner3D::heuristic(
    const PersistentPlannerTimeState3D& state) const noexcept {
  const double euclidean = minimumFlightTranslationTime3D(
      lattice_->pointFor(state.position), request_.exact_goal, config_->time_model);
  const std::optional<double> cost_to_goal = session_->costToGoal(state.position);
  return cost_to_goal.has_value() ? std::max(euclidean, *cost_to_goal) : euclidean;
}

double
ExecutionTimeRefiner3D::transitionCost(const PersistentPlannerTimeState3D& first,
                                       const PersistentPlannerTimeState3D& second) {
  if (second.incoming != directionForEdge3D(first.position, second.position)) {
    return std::numeric_limits<double>::infinity();
  }
  const double translation_cost =
      lattice_->rawEdgeCost(first.position, second.position);
  if (!std::isfinite(translation_cost)) {
    return translation_cost;
  }
  const Point3 first_point = lattice_->pointFor(first.position);
  const Point3 second_point = lattice_->pointFor(second.position);
  const Vec3 outgoing{second_point.x - first_point.x, second_point.y - first_point.y,
                      second_point.z - first_point.z};
  if (first.incoming.empty()) {
    if (first == request_.start && request_.start_from_rest) {
      return translation_cost +
             estimatedFlightRestTransitionDelay3D(outgoing, config_->time_model);
    }
    return translation_cost;
  }
  const Vec3 incoming = directionVector(first.incoming);
  if (!requiresFlightStopAndTurn3D(incoming, outgoing,
                                   config_->minimum_continuous_turn_alignment)) {
    return translation_cost;
  }
  return translation_cost +
         estimatedFlightStopAndTurnDelay3D(incoming, outgoing, config_->time_model);
}

double ExecutionTimeRefiner3D::terminalCost(
    const PersistentPlannerTimeState3D& state) const noexcept {
  if (state.position != request_.goal_anchor) {
    return std::numeric_limits<double>::infinity();
  }
  const Point3 anchor = lattice_->pointFor(request_.goal_anchor);
  const Vec3 connector{request_.exact_goal.x - anchor.x,
                       request_.exact_goal.y - anchor.y,
                       request_.exact_goal.z - anchor.z};
  const double connector_length =
      std::hypot(std::hypot(connector.x, connector.y), connector.z);
  if (connector_length > kDirectionEpsilon) {
    double cost = minimumFlightTranslationTime3D(anchor, request_.exact_goal,
                                                 config_->time_model) +
                  estimatedFlightRestTransitionDelay3D(connector, config_->time_model);
    if (!state.incoming.empty()) {
      const Vec3 incoming = directionVector(state.incoming);
      if (requiresFlightStopAndTurn3D(incoming, connector,
                                      config_->minimum_continuous_turn_alignment)) {
        cost +=
            estimatedFlightStopAndTurnDelay3D(incoming, connector, config_->time_model);
      }
    }
    return cost;
  }
  return state.incoming.empty()
             ? 0.0
             : estimatedFlightRestTransitionDelay3D(directionVector(state.incoming),
                                                    config_->time_model);
}

std::optional<std::vector<Point3>>
ExecutionTimeRefiner3D::advance(const std::chrono::steady_clock::time_point deadline,
                                const std::size_t maximum_expansions,
                                std::size_t& expansions) {
  if (!initialized_) {
    return std::nullopt;
  }
  if (complete_) {
    return hasIncumbent() ? std::optional<std::vector<Point3>>{bestPath()}
                          : std::nullopt;
  }

  while (true) {
    while (!open_.empty()) {
      const PersistentPlannerTimeQueueEntry3D& entry = open_.top();
      const auto found = costs_.find(entry.state);
      if (found != costs_.end() && !costLess(found->second, entry.cost_from_start_s) &&
          !costLess(entry.cost_from_start_s, found->second)) {
        break;
      }
      open_.pop();
    }

    if (open_.empty() ||
        (hasIncumbent() && !costLess(open_.top().estimated_total_s, goal_cost_s_))) {
      complete_ = true;
      return hasIncumbent() ? std::optional<std::vector<Point3>>{bestPath()}
                            : std::nullopt;
    }
    if (expansions >= maximum_expansions ||
        std::chrono::steady_clock::now() >= deadline) {
      // The spatial incumbent is already a complete raw-valid route. Publish
      // it as the anytime result while the direction-aware search continues
      // refining the execution-time objective on later calls.
      return hasIncumbent() ? std::optional<std::vector<Point3>>{bestPath()}
                            : std::nullopt;
    }

    const PersistentPlannerTimeQueueEntry3D current = open_.top();
    open_.pop();
    ++expansions;
    if (current.state.position == request_.goal_anchor) {
      const double terminal_cost = terminalCost(current.state);
      const double total_cost = current.cost_from_start_s + terminal_cost;
      if (std::isfinite(total_cost) &&
          (!hasIncumbent() || costLess(total_cost, goal_cost_s_))) {
        goal_ = current.state;
        goal_cost_s_ = total_cost;
      }
      continue;
    }

    lattice_->forEachAdjacentNode(
        current.state.position, [&](const PersistentPlannerNode3D successor_position) {
          const PersistentPlannerTimeState3D successor{
              .position = successor_position,
              .incoming =
                  directionForEdge3D(current.state.position, successor_position),
          };
          const double transition_cost = transitionCost(current.state, successor);
          const double candidate_cost = current.cost_from_start_s + transition_cost;
          if (!std::isfinite(candidate_cost)) {
            return;
          }
          const auto existing = costs_.find(successor);
          if (existing != costs_.end() && !costLess(candidate_cost, existing->second)) {
            return;
          }
          costs_[successor] = candidate_cost;
          parents_[successor] = current.state;
          ++queue_sequence_;
          if (queue_sequence_ == 0U) {
            queue_sequence_ = 1U;
          }
          const double estimated_total = candidate_cost + heuristic(successor);
          if (hasIncumbent() && !costLess(estimated_total, goal_cost_s_)) {
            return;
          }
          open_.push(PersistentPlannerTimeQueueEntry3D{
              .estimated_total_s = estimated_total,
              .cost_from_start_s = candidate_cost,
              .state = successor,
              .sequence = queue_sequence_,
          });
        });
  }
}

std::vector<Point3> ExecutionTimeRefiner3D::extractPath() {
  if (!goal_.has_value()) {
    return {};
  }
  std::vector<PersistentPlannerTimeState3D> states;
  states.reserve(config_->maximum_extracted_path_nodes);
  PersistentPlannerTimeState3D current = *goal_;
  while (true) {
    states.push_back(current);
    if (current == request_.start) {
      break;
    }
    if (states.size() >= config_->maximum_extracted_path_nodes) {
      return {};
    }
    const auto parent = parents_.find(current);
    if (parent == parents_.end()) {
      return {};
    }
    current = parent->second;
  }
  std::ranges::reverse(states);
  adaptive_edges_in_extracted_path_ = 0U;
  for (std::size_t index = 1U; index < states.size(); ++index) {
    adaptive_edges_in_extracted_path_ +=
        lattice_->level(states[index - 1U].position, states[index].position) > 0U ? 1U
                                                                                  : 0U;
  }

  std::vector<Point3> path;
  path.reserve(states.size() + 2U);
  path.push_back(request_.exact_start);
  for (const PersistentPlannerTimeState3D& state : states) {
    const Point3 point = lattice_->pointFor(state.position);
    if (distance3D(path.back(), point) > kDirectionEpsilon) {
      path.push_back(point);
    }
  }
  if (distance3D(path.back(), request_.exact_goal) > kDirectionEpsilon) {
    path.push_back(request_.exact_goal);
  }
  return path;
}

} // namespace drone_city_nav::detail
