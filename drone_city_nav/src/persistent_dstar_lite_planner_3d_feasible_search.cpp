#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <ranges>
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

} // namespace

std::optional<std::vector<Point3>>
FeasiblePathSearch3D::reconstruct(const Endpoints3D& endpoints,
                                  const PersistentPlannerNode3D terminal) const {
  std::vector<PersistentPlannerNode3D> nodes;
  nodes.reserve(std::min(config_->maximum_extracted_path_nodes, costs_.size()));
  PersistentPlannerNode3D current = terminal;
  while (true) {
    nodes.push_back(current);
    if (current == endpoints.start) {
      break;
    }
    if (nodes.size() >= config_->maximum_extracted_path_nodes) {
      return std::nullopt;
    }
    const auto parent = parents_.find(current);
    if (parent == parents_.end()) {
      return std::nullopt;
    }
    current = parent->second;
  }
  std::ranges::reverse(nodes);
  std::vector<Point3> path;
  path.reserve(nodes.size() + 2U);
  path.push_back(endpoints.exact_start);
  for (const PersistentPlannerNode3D node : nodes) {
    const Point3 point = lattice_->pointFor(node);
    if (distance3D(path.back(), point) > kCostTolerance) {
      path.push_back(point);
    }
  }
  if (distance3D(path.back(), endpoints.exact_goal) > kCostTolerance) {
    path.push_back(endpoints.exact_goal);
  }
  return path.size() >= 2U ? std::optional<std::vector<Point3>>{std::move(path)}
                           : std::nullopt;
}

std::optional<std::vector<Point3>> FeasiblePathSearch3D::advance(
    const Endpoints3D& endpoints, const std::chrono::steady_clock::time_point deadline,
    const std::size_t maximum_expansions, std::size_t& expansions) {
  expansions = 0U;
  if (!initialized_) {
    initialize(endpoints);
  }

  while (!open_.empty() && expansions < maximum_expansions &&
         std::chrono::steady_clock::now() < deadline) {
    const FeasibilityQueueEntry3D current = open_.top();
    open_.pop();
    const auto current_cost = costs_.find(current.node);
    if (current_cost == costs_.end() ||
        !approximatelyEqual(current.cost_from_start_s, current_cost->second)) {
      continue;
    }
    ++expansions;

    const Point3 current_point = lattice_->pointFor(current.node);
    const bool direct_departure =
        current.node == endpoints.start &&
        distance3D(endpoints.exact_start, current_point) <= kCostTolerance;
    const bool goal_connector_valid =
        direct_departure
            ? lattice_->departureSegmentValid(endpoints.exact_start,
                                              endpoints.exact_goal)
            : lattice_->rawSegmentValid(current_point, endpoints.exact_goal);
    if (goal_connector_valid) {
      std::optional<std::vector<Point3>> candidate =
          reconstruct(endpoints, current.node);
      if (candidate.has_value() && lattice_->pathTraversable(*candidate)) {
        return candidate;
      }
    }

    for (const PersistentPlannerNode3D neighbor :
         lattice_->adjacentNodes(current.node)) {
      if (!lattice_->edgeTraversable(current.node, neighbor)) {
        continue;
      }
      const double transition_cost = minimumFlightTranslationTime3D(
          lattice_->pointFor(current.node), lattice_->pointFor(neighbor),
          config_->time_model);
      const double candidate_cost = current.cost_from_start_s + transition_cost;
      const auto existing = costs_.find(neighbor);
      if (existing != costs_.end() &&
          (existing->second < candidate_cost ||
           approximatelyEqual(existing->second, candidate_cost))) {
        continue;
      }
      costs_[neighbor] = candidate_cost;
      parents_.insert_or_assign(neighbor, current.node);
      const std::size_t depth = current.depth + 1U;
      ++queue_sequence_;
      if (queue_sequence_ == 0U) {
        queue_sequence_ = 1U;
      }
      open_.push(FeasibilityQueueEntry3D{
          .estimated_total_s =
              candidate_cost + lattice_->heuristic(neighbor, endpoints.goal),
          .cost_from_start_s = candidate_cost,
          .depth = depth,
          .node = neighbor,
          .sequence = queue_sequence_,
      });
    }
  }
  return std::nullopt;
}

void FeasiblePathSearch3D::initialize(const Endpoints3D& endpoints) {
  reset();
  initialized_ = true;
  queue_sequence_ = 1U;
  costs_.emplace(endpoints.start, 0.0);
  open_.push(FeasibilityQueueEntry3D{
      .estimated_total_s = lattice_->heuristic(endpoints.start, endpoints.goal),
      .cost_from_start_s = 0.0,
      .depth = 0U,
      .node = endpoints.start,
      .sequence = queue_sequence_,
  });
}

void FeasiblePathSearch3D::reset() noexcept {
  initialized_ = false;
  queue_sequence_ = 0U;
  open_ = {};
  costs_.clear();
  parents_.clear();
}

bool FeasiblePathSearch3D::initialized() const noexcept {
  return initialized_;
}

} // namespace drone_city_nav::detail
