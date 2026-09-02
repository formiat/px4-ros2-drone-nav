#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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

std::vector<PersistentPlannerNode3D>
FeasiblePathSearch3D::reconstructNodes(const PersistentPlannerNode3D terminal) const {
  std::vector<PersistentPlannerNode3D> nodes;
  nodes.reserve(std::min(config_->maximum_extracted_path_nodes, explored_));
  std::size_t current = lattice_->linearIndex(terminal);
  while (true) {
    if (!labelled(current)) {
      return {};
    }
    const PersistentPlannerNode3D node = lattice_->nodeAt(current);
    nodes.push_back(node);
    if (node == anchor_) {
      break;
    }
    if (nodes.size() >= config_->maximum_extracted_path_nodes ||
        parent_index_[current] == kNoParent) {
      return {};
    }
    current = parent_index_[current];
  }
  std::ranges::reverse(nodes);
  return nodes;
}

std::optional<std::vector<Point3>>
FeasiblePathSearch3D::pathFromNodes(const Endpoints3D& endpoints,
                                    std::vector<PersistentPlannerNode3D>& nodes) const {
  // The vehicle may have drifted from the anchor since the labels were seeded;
  // the departure joins the first of the leading nodes it still reaches.
  constexpr std::size_t kDepartureCandidates{8U};
  std::size_t first = nodes.size();
  for (std::size_t index = 0U; index < std::min(nodes.size(), kDepartureCandidates);
       ++index) {
    const Point3 point = lattice_->pointFor(nodes[index]);
    if (distance3D(endpoints.exact_start, point) <= kCostTolerance ||
        lattice_->departureSegmentValid(endpoints.exact_start, point)) {
      first = index;
      break;
    }
  }
  if (first >= nodes.size()) {
    return std::nullopt;
  }
  nodes.erase(nodes.begin(),
              std::next(nodes.begin(), static_cast<std::ptrdiff_t>(first)));
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

void FeasiblePathSearch3D::reseedFromPrefix(
    const Endpoints3D& endpoints, const std::vector<PersistentPlannerNode3D>& prefix) {
  std::vector<double> costs;
  costs.reserve(prefix.size());
  for (const PersistentPlannerNode3D node : prefix) {
    costs.push_back(cost_s_[lattice_->linearIndex(node)]);
  }
  reset();
  ensureLabelStorage();
  initialized_ = true;
  queue_sequence_ = 1U;
  std::uint32_t parent = kNoParent;
  for (std::size_t index = 0U; index < prefix.size(); ++index) {
    const std::size_t node_index = lattice_->linearIndex(prefix[index]);
    label_generation_[node_index] = generation_;
    cost_s_[node_index] = costs[index];
    parent_index_[node_index] = parent;
    parent = static_cast<std::uint32_t>(node_index);
    ++explored_;
    ++queue_sequence_;
    open_.push(FeasibilityQueueEntry3D{
        .estimated_total_s =
            costs[index] + lattice_->heuristic(prefix[index], endpoints.goal),
        .cost_from_start_s = costs[index],
        .depth = index,
        .node = prefix[index],
        .sequence = queue_sequence_,
    });
  }
}

std::optional<std::vector<Point3>> FeasiblePathSearch3D::advance(
    const Endpoints3D& endpoints, const std::chrono::steady_clock::time_point deadline,
    const std::size_t maximum_expansions, std::size_t& expansions) {
  expansions = 0U;
  if (!initialized_) {
    initialize(endpoints);
  }

  // Labels survive occupied changes; a candidate that fails raw validation,
  // or a frontier exhausted without a valid candidate, restarts the search on
  // the resident world from the current start anchor once per call.
  bool restarted{false};
  frontier_exhausted_ = false;
  while (expansions < maximum_expansions &&
         std::chrono::steady_clock::now() < deadline) {
    if (open_.empty()) {
      frontier_exhausted_ = true;
      if (restarted) {
        break;
      }
      restarted = true;
      ++restart_count_;
      initialize(endpoints);
      continue;
    }
    const FeasibilityQueueEntry3D current = open_.top();
    open_.pop();
    const std::size_t current_index = lattice_->linearIndex(current.node);
    if (!labelled(current_index) ||
        !approximatelyEqual(current.cost_from_start_s, cost_s_[current_index])) {
      continue;
    }
    ++expansions;

    const Point3 current_point = lattice_->pointFor(current.node);
    closest_goal_distance_m_ = std::min(
        closest_goal_distance_m_, distance3D(current_point, endpoints.exact_goal));
    const bool direct_departure =
        current.node == anchor_ &&
        distance3D(endpoints.exact_start, current_point) <= kCostTolerance;
    const bool goal_connector_in_reach =
        distance3D(current_point, endpoints.exact_goal) <=
        config_->feasibility_goal_connector_reach_m;
    const bool goal_connector_valid =
        goal_connector_in_reach &&
        (direct_departure
             ? lattice_->departureSegmentValid(endpoints.exact_start,
                                               endpoints.exact_goal)
             : lattice_->rawSegmentValid(current_point, endpoints.exact_goal));
    if (goal_connector_valid) {
      std::vector<PersistentPlannerNode3D> nodes = reconstructNodes(current.node);
      std::optional<std::vector<Point3>> candidate =
          nodes.empty() ? std::nullopt : pathFromNodes(endpoints, nodes);
      const std::optional<std::size_t> invalid_segment =
          candidate.has_value() ? lattice_->firstInvalidSegment(*candidate)
                                : std::optional<std::size_t>{0U};
      if (!invalid_segment.has_value()) {
        return candidate;
      }
      last_invalid_segment_ = *invalid_segment;
      // Segment s joins candidate[s-1] and candidate[s]; with the exact start
      // in front, segment s >= 2 leaves nodes[s-2]. Its edge was traversable
      // when labelled and is not on the resident world any more: forget it and
      // continue from the validated prefix instead of the whole closed set.
      if (*invalid_segment >= 2U && *invalid_segment - 1U < nodes.size()) {
        const std::size_t prefix_end = *invalid_segment - 2U;
        if (*invalid_segment < nodes.size() + 1U) {
          static_cast<void>(lattice_->forgetEdgeCost(
              PersistentPlannerEdge3D{nodes[prefix_end], nodes[prefix_end + 1U]}));
        }
        nodes.resize(prefix_end + 1U);
        ++prefix_reseed_count_;
        reseedFromPrefix(endpoints, nodes);
        continue;
      }
      // The departure no longer reaches any leading node, or the candidate is
      // degenerate: start over from the current anchor on the resident world.
      if (restarted) {
        break;
      }
      restarted = true;
      ++restart_count_;
      initialize(endpoints);
      continue;
    }

    lattice_->forEachAdjacentNode(
        current.node, [&](const PersistentPlannerNode3D neighbor) {
          const double transition_cost = lattice_->rawEdgeCost(current.node, neighbor);
          if (!std::isfinite(transition_cost)) {
            return;
          }
          const double candidate_cost = current.cost_from_start_s + transition_cost;
          const std::size_t neighbor_index = lattice_->linearIndex(neighbor);
          if (labelled(neighbor_index)) {
            const double existing = cost_s_[neighbor_index];
            if (existing < candidate_cost ||
                approximatelyEqual(existing, candidate_cost)) {
              return;
            }
          } else {
            label_generation_[neighbor_index] = generation_;
            ++explored_;
          }
          cost_s_[neighbor_index] = candidate_cost;
          parent_index_[neighbor_index] = static_cast<std::uint32_t>(current_index);
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
        });
  }
  return std::nullopt;
}

void FeasiblePathSearch3D::ensureLabelStorage() {
  const std::size_t count = lattice_->nodeCount();
  if (cost_s_.size() == count) {
    return;
  }
  cost_s_.assign(count, std::numeric_limits<double>::infinity());
  label_generation_.assign(count, 0U);
  parent_index_.assign(count, kNoParent);
  generation_ = 1U;
}

bool FeasiblePathSearch3D::labelled(const std::size_t index) const noexcept {
  return label_generation_[index] == generation_;
}

void FeasiblePathSearch3D::initialize(const Endpoints3D& endpoints) {
  reset();
  ensureLabelStorage();
  initialized_ = true;
  anchor_ = endpoints.start;
  queue_sequence_ = 1U;
  const std::size_t anchor_index = lattice_->linearIndex(anchor_);
  label_generation_[anchor_index] = generation_;
  cost_s_[anchor_index] = 0.0;
  parent_index_[anchor_index] = kNoParent;
  explored_ = 1U;
  open_.push(FeasibilityQueueEntry3D{
      .estimated_total_s = lattice_->heuristic(anchor_, endpoints.goal),
      .cost_from_start_s = 0.0,
      .depth = 0U,
      .node = anchor_,
      .sequence = queue_sequence_,
  });
}

void FeasiblePathSearch3D::reset() noexcept {
  initialized_ = false;
  closest_goal_distance_m_ = std::numeric_limits<double>::infinity();
  frontier_exhausted_ = false;
  open_ = FeasibilityOpenQueue3D{};
  explored_ = 0U;
  if (++generation_ == 0U) {
    std::ranges::fill(label_generation_, 0U);
    generation_ = 1U;
  }
}

bool FeasiblePathSearch3D::initialized() const noexcept {
  return initialized_;
}

PersistentPlannerNode3D FeasiblePathSearch3D::anchor() const noexcept {
  return anchor_;
}

bool FeasiblePathSearch3D::frontierExhausted() const noexcept {
  return frontier_exhausted_;
}

std::size_t FeasiblePathSearch3D::exploredNodes() const noexcept {
  return explored_;
}

double FeasiblePathSearch3D::closestGoalDistanceM() const noexcept {
  return closest_goal_distance_m_;
}

std::size_t FeasiblePathSearch3D::restartCount() const noexcept {
  return restart_count_;
}

std::size_t FeasiblePathSearch3D::prefixReseedCount() const noexcept {
  return prefix_reseed_count_;
}

std::size_t FeasiblePathSearch3D::lastInvalidSegment() const noexcept {
  return last_invalid_segment_;
}

} // namespace drone_city_nav::detail
