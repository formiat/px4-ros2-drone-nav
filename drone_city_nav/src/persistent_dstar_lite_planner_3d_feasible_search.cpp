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

// Feasibility-first search: a resumable best-first search whose labels survive
// occupied changes. A label is trusted only while the chain of lattice edges
// that reached it survives the resident world; the chain is re-validated
// lazily against the lattice edge cache, so a change drops exactly the labels
// behind the edges it moved and the rest of the frontier keeps its progress.

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

std::optional<std::vector<Point3>> FeasiblePathSearch3D::pathFromNodes(
    const Endpoints3D& endpoints,
    const std::vector<PersistentPlannerNode3D>& nodes) const {
  // The vehicle may have drifted from the anchor since the labels were seeded;
  // the departure joins the first of the leading nodes it still reaches.
  constexpr std::size_t kDepartureCandidates{8U};
  std::size_t first = nodes.size();
  for (std::size_t index = 0U; index < std::min(nodes.size(), kDepartureCandidates);
       ++index) {
    const Point3 point = lattice_->pointFor(nodes[index]);
    if (distance3D(endpoints.exact_start, point) <= kCostTolerance ||
        lattice_->departureReachable(endpoints.exact_start,
                                     endpoints.departure_waypoints, point)) {
      first = index;
      break;
    }
  }
  if (first >= nodes.size()) {
    return std::nullopt;
  }
  std::vector<Point3> path;
  path.reserve(nodes.size() - first + 2U);
  path.push_back(endpoints.exact_start);
  for (const Point3& waypoint : endpoints.departure_waypoints) {
    if (distance3D(path.back(), waypoint) > kCostTolerance) {
      path.push_back(waypoint);
    }
  }
  for (std::size_t index = first; index < nodes.size(); ++index) {
    // An edge whose straight segment does not clear the body is traversable
    // through a waypoint; the path has to carry it.
    if (index > first) {
      if (const std::optional<Point3> waypoint =
              lattice_->edgeWaypoint(nodes[index - 1U], nodes[index]);
          waypoint.has_value() && distance3D(path.back(), *waypoint) > kCostTolerance) {
        path.push_back(*waypoint);
      }
    }
    const Point3 point = lattice_->pointFor(nodes[index]);
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

void FeasiblePathSearch3D::noteWorldChanged() noexcept {
  rejected_edges_.clear();
  advanceValidationEpoch();
}

void FeasiblePathSearch3D::advanceValidationEpoch() noexcept {
  if (++validation_epoch_ == 0U) {
    std::ranges::fill(validated_epoch_, 0U);
    validation_epoch_ = 1U;
  }
}

bool FeasiblePathSearch3D::label(const std::size_t index,
                                 const double cost_from_start_s,
                                 const std::uint32_t parent,
                                 const std::uint32_t depth) {
  // Adopted labels leave costs non-monotone along a chain, so a cheaper
  // parent may sit below the label; closing that loop would make every
  // chain walk endless.
  if (parent != kNoParent && descendsFrom(parent, index)) {
    return false;
  }
  if (!labelled(index)) {
    label_generation_[index] = generation_;
    ++explored_;
  }
  cost_s_[index] = cost_from_start_s;
  parent_index_[index] = parent;
  depth_[index] = depth;
  validated_epoch_[index] = validation_epoch_;
  return true;
}

void FeasiblePathSearch3D::push(const std::size_t index,
                                const PersistentPlannerNode3D node) {
  if (queued_[index] != 0U) {
    return;
  }
  queued_[index] = 1U;
  ++queue_sequence_;
  if (queue_sequence_ == 0U) {
    queue_sequence_ = 1U;
  }
  open_.push(FeasibilityQueueEntry3D{
      .estimated_total_s = cost_s_[index] + lattice_->heuristic(node, goal_),
      .cost_from_start_s = cost_s_[index],
      .depth = depth_[index],
      .node = node,
      .sequence = queue_sequence_,
  });
}

bool FeasiblePathSearch3D::chainValid(const std::size_t index) {
  if (!labelled(index)) {
    return false;
  }
  constexpr std::size_t kMaximumAdoptions{32U};
  for (std::size_t adoption = 0U; adoption < kMaximumAdoptions; ++adoption) {
    if (validated_epoch_[index] == validation_epoch_) {
      return true;
    }
    // Climb to the nearest ancestor validated on this epoch. The anchor has no
    // parent edge and is valid by construction; an ancestor that lost its
    // label leaves the label below it to adoption.
    chain_.clear();
    std::size_t current = index;
    bool parent_missing{false};
    while (true) {
      if (chain_.size() >= kMaximumChainWalk) {
        // No chain is this long on a bounded lattice: a loop of parent links
        // would be, and it must not hold the planning thread.
        for (const std::uint32_t stale : chain_) {
          invalidateLabel(stale);
        }
        return false;
      }
      chain_.push_back(static_cast<std::uint32_t>(current));
      const std::uint32_t parent = parent_index_[current];
      if (parent == kNoParent) {
        break;
      }
      if (!labelled(parent)) {
        parent_missing = true;
        break;
      }
      if (validated_epoch_[parent] == validation_epoch_) {
        break;
      }
      current = parent;
    }
    // Validate top-down. A validated entry is a sound parent for the entry
    // below it. The first broken entry is re-parented and the walk starts
    // over from the label; an entry no neighbour adopts takes the rest of
    // this chain with it.
    bool adopted{false};
    for (std::size_t position = chain_.size(); position-- > 0U;) {
      const std::size_t child = chain_[position];
      const std::uint32_t parent = parent_index_[child];
      bool intact = parent == kNoParent;
      if (!intact && !(position + 1U == chain_.size() && parent_missing)) {
        const PersistentPlannerNode3D parent_node = lattice_->nodeAt(parent);
        const PersistentPlannerNode3D child_node = lattice_->nodeAt(child);
        intact = (rejected_edges_.empty() ||
                  !rejected_edges_.contains(canonicalEdge(parent_node, child_node))) &&
                 lattice_->edgeTraversable(parent_node, child_node);
      }
      if (intact) {
        validated_epoch_[child] = validation_epoch_;
        continue;
      }
      if (adoptLabel(child)) {
        adopted = true;
        break;
      }
      for (std::size_t stale = 0U; stale <= position; ++stale) {
        invalidateLabel(chain_[stale]);
      }
      return false;
    }
    if (!adopted) {
      return true;
    }
  }
  // A chain that keeps breaking above every adopter is not worth more walks.
  invalidateLabel(index);
  return false;
}

bool FeasiblePathSearch3D::descendsFrom(std::size_t index,
                                        const std::size_t ancestor) const noexcept {
  // Parent links of dropped labels are followed too: a dropped label that is
  // re-labelled below one of its former descendants would close a cycle.
  for (std::size_t step = 0U; step < kMaximumChainWalk; ++step) {
    if (index == ancestor) {
      return true;
    }
    const std::uint32_t parent = parent_index_[index];
    if (parent == kNoParent) {
      return false;
    }
    index = parent;
  }
  return true;
}

bool FeasiblePathSearch3D::adoptLabel(const std::size_t index) {
  // A neighbour validated on this epoch cannot descend from the label: within
  // an epoch its chain would have failed on the same broken edge. Any other
  // labelled neighbour outside the label's own subtree is a plausible parent
  // whose chain the next walk validates in turn.
  const PersistentPlannerNode3D node = lattice_->nodeAt(index);
  double validated_cost_s = std::numeric_limits<double>::infinity();
  std::uint32_t validated_parent = kNoParent;
  double plausible_cost_s = std::numeric_limits<double>::infinity();
  std::uint32_t plausible_parent = kNoParent;
  lattice_->forEachAdjacentNode(node, [&](const PersistentPlannerNode3D neighbor) {
    const std::size_t neighbor_index = lattice_->linearIndex(neighbor);
    if (!labelled(neighbor_index)) {
      return;
    }
    if (!rejected_edges_.empty() &&
        rejected_edges_.contains(canonicalEdge(neighbor, node))) {
      return;
    }
    const bool validated = validated_epoch_[neighbor_index] == validation_epoch_;
    if (!validated && (cost_s_[neighbor_index] >= plausible_cost_s ||
                       descendsFrom(neighbor_index, index))) {
      return;
    }
    const double transition_cost = lattice_->rankedEdgeCost(
        neighbor, node, config_->feasibility_clearance_ranking_distance_m);
    if (!std::isfinite(transition_cost)) {
      return;
    }
    const double candidate_cost = cost_s_[neighbor_index] + transition_cost;
    if (validated) {
      if (candidate_cost < validated_cost_s) {
        validated_cost_s = candidate_cost;
        validated_parent = static_cast<std::uint32_t>(neighbor_index);
      }
    } else if (candidate_cost < plausible_cost_s) {
      plausible_cost_s = candidate_cost;
      plausible_parent = static_cast<std::uint32_t>(neighbor_index);
    }
  });
  const bool validated = validated_parent != kNoParent;
  const std::uint32_t parent = validated ? validated_parent : plausible_parent;
  if (parent == kNoParent) {
    return false;
  }
  cost_s_[index] = validated ? validated_cost_s : plausible_cost_s;
  parent_index_[index] = parent;
  depth_[index] = depth_[parent] + 1U;
  if (validated) {
    validated_epoch_[index] = validation_epoch_;
  }
  ++adopted_label_count_;
  if (queued_[index] != 0U) {
    // The queued entry carries the old cost and drops on pop; queue the label
    // again at its adopted cost so the frontier keeps it.
    queued_[index] = 0U;
    push(index, node);
  }
  return true;
}

void FeasiblePathSearch3D::invalidateLabel(const std::size_t index) {
  if (!labelled(index)) {
    return;
  }
  label_generation_[index] = 0U;
  --explored_;
  ++invalidated_label_count_;
  invalidation_queue_.push_back(static_cast<std::uint32_t>(index));
}

void FeasiblePathSearch3D::drainInvalidations(
    const std::chrono::steady_clock::time_point deadline) {
  std::size_t drained{0U};
  while (!invalidation_queue_.empty()) {
    if ((++drained & 63U) == 0U && std::chrono::steady_clock::now() >= deadline) {
      return;
    }
    const std::size_t index = invalidation_queue_.back();
    invalidation_queue_.pop_back();
    lattice_->forEachAdjacentNode(
        lattice_->nodeAt(index), [&](const PersistentPlannerNode3D neighbor) {
          const std::size_t neighbor_index = lattice_->linearIndex(neighbor);
          // A neighbour whose own chain holds is frontier again; one whose
          // chain is broken joins the queue through its validation.
          if (!labelled(neighbor_index) || !chainValid(neighbor_index)) {
            return;
          }
          push(neighbor_index, neighbor);
        });
  }
}

std::optional<std::vector<Point3>> FeasiblePathSearch3D::advance(
    const Endpoints3D& endpoints, const std::chrono::steady_clock::time_point deadline,
    const std::size_t maximum_expansions, std::size_t& expansions) {
  // The restart inside advanceFrontier re-seeds the labels through reset(),
  // which clears the exhaustion flag; the flag has to report what happened in
  // this call, or an exhaustion followed by a restart looks like an ordinary
  // update and nothing above the search ever learns that the start's
  // component is closed.
  bool exhausted = false;
  std::optional<std::vector<Point3>> candidate =
      advanceFrontier(endpoints, deadline, maximum_expansions, expansions, exhausted);
  frontier_exhausted_ = exhausted;
  return candidate;
}

std::optional<std::vector<Point3>> FeasiblePathSearch3D::advanceFrontier(
    const Endpoints3D& endpoints, const std::chrono::steady_clock::time_point deadline,
    const std::size_t maximum_expansions, std::size_t& expansions, bool& exhausted) {
  expansions = 0U;
  if (!initialized_) {
    initialize(endpoints);
  }

  // Labels survive occupied changes behind a lazily validated chain; a
  // candidate that still fails the raw sweep drops the labels behind the
  // failed edge, and a frontier exhausted without a valid candidate restarts
  // the search on the resident world from the current start anchor once per
  // call.
  bool restarted{false};
  while (expansions < maximum_expansions &&
         std::chrono::steady_clock::now() < deadline) {
    if (!invalidation_queue_.empty()) {
      // A re-entry cascade the previous call could not finish.
      drainInvalidations(deadline);
      continue;
    }
    if (open_.empty()) {
      exhausted = true;
      markClosedComponent();
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
    if (!chainValid(current_index)) {
      drainInvalidations(deadline);
      continue;
    }
    if (!current.goal_connector) {
      queued_[current_index] = 0U;
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
    if (goal_connector_valid && !current.goal_connector) {
      // The connector is one more priced edge: it competes in the queue with
      // the labelled frontier instead of ending the search on the first
      // raw-valid straight line, so a connector that hugs the floor or a wall
      // yields to a route that climbs clear of it first.
      ++queue_sequence_;
      if (queue_sequence_ == 0U) {
        queue_sequence_ = 1U;
      }
      open_.push(FeasibilityQueueEntry3D{
          .estimated_total_s = current.cost_from_start_s +
                               lattice_->rankedSegmentTimeS(
                                   current_point, endpoints.exact_goal,
                                   config_->feasibility_clearance_ranking_distance_m),
          .cost_from_start_s = current.cost_from_start_s,
          .depth = current.depth,
          .node = current.node,
          .sequence = queue_sequence_,
          .goal_connector = true,
      });
    }
    if (current.goal_connector && goal_connector_valid) {
      const std::vector<PersistentPlannerNode3D> nodes = reconstructNodes(current.node);
      std::optional<std::vector<Point3>> candidate =
          nodes.empty() ? std::nullopt : pathFromNodes(endpoints, nodes);
      const std::optional<std::size_t> invalid_segment =
          candidate.has_value() ? lattice_->firstInvalidSegment(*candidate)
                                : std::optional<std::size_t>{0U};
      if (!invalid_segment.has_value()) {
        return candidate;
      }
      last_invalid_segment_ = *invalid_segment;
      // Segment s joins candidate[s-1] and candidate[s]. When the lattice
      // priced that segment — a straight edge, or a leg of a refined edge
      // through its waypoint — and the sweep rejects it on the resident world,
      // the sweep is the authority: the edge is forgotten for re-derivation,
      // withheld from this search on this world, and the labels behind it are
      // dropped and re-entered from the intact labels around them. Restarting
      // instead would find the same edge again from the same cache and reject
      // it again, forever.
      if (const std::optional<PlannerLattice3D::PathSegmentEdge3D> priced =
              lattice_->pricedEdgeForSegment(*candidate, *invalid_segment);
          priced.has_value()) {
        const PersistentPlannerEdge3D edge = canonicalEdge(priced->from, priced->to);
        static_cast<void>(lattice_->rejectEdgeBySweep(edge));
        rejected_edges_.insert(edge);
        advanceValidationEpoch();
        invalidateLabel(lattice_->linearIndex(priced->to));
        drainInvalidations(deadline);
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
          if (!rejected_edges_.empty() &&
              rejected_edges_.contains(canonicalEdge(current.node, neighbor))) {
            return;
          }
          // Priced with the soft clearance ranking within the feasibility
          // reach: the first route already keeps its body out of the
          // critical band instead of hugging the nearest floor or wall.
          const double transition_cost = lattice_->rankedEdgeCost(
              current.node, neighbor,
              config_->feasibility_clearance_ranking_distance_m);
          if (!std::isfinite(transition_cost)) {
            return;
          }
          const double candidate_cost = current.cost_from_start_s + transition_cost;
          const std::size_t neighbor_index = lattice_->linearIndex(neighbor);
          // A label whose chain broke is dropped here and re-labelled below.
          if (labelled(neighbor_index) && chainValid(neighbor_index)) {
            const double existing = cost_s_[neighbor_index];
            if (existing < candidate_cost ||
                approximatelyEqual(existing, candidate_cost)) {
              return;
            }
          }
          if (!label(neighbor_index, candidate_cost,
                     static_cast<std::uint32_t>(current_index),
                     static_cast<std::uint32_t>(current.depth + 1U))) {
            return;
          }
          queued_[neighbor_index] = 0U;
          push(neighbor_index, neighbor);
        });
    drainInvalidations(deadline);
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
  depth_.assign(count, 0U);
  validated_epoch_.assign(count, 0U);
  queued_.assign(count, 0U);
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
  goal_ = endpoints.goal;
  queue_sequence_ = 1U;
  const std::size_t anchor_index = lattice_->linearIndex(anchor_);
  static_cast<void>(label(anchor_index, 0.0, kNoParent, 0U));
  push(anchor_index, anchor_);
}

void FeasiblePathSearch3D::reset() noexcept {
  initialized_ = false;
  closest_goal_distance_m_ = std::numeric_limits<double>::infinity();
  frontier_exhausted_ = false;
  open_ = FeasibilityOpenQueue3D{};
  explored_ = 0U;
  chain_.clear();
  invalidation_queue_.clear();
  rejected_edges_.clear();
  std::ranges::fill(queued_, 0U);
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

void FeasiblePathSearch3D::markClosedComponent() {
  if (closed_component_.size() != label_generation_.size()) {
    closed_component_.assign(label_generation_.size(), 0U);
  }
  for (std::size_t index = 0U; index < label_generation_.size(); ++index) {
    if (label_generation_[index] == generation_) {
      closed_component_[index] = 1U;
    }
  }
  closed_component_marked_ = true;
}

bool FeasiblePathSearch3D::closedComponentMarked() const noexcept {
  return closed_component_marked_;
}

bool FeasiblePathSearch3D::inClosedComponent(
    const PersistentPlannerNode3D node) const noexcept {
  if (!closed_component_marked_ || !lattice_->nodeInside(node)) {
    return false;
  }
  const std::size_t index = lattice_->linearIndex(node);
  return index < closed_component_.size() && closed_component_[index] != 0U;
}

void FeasiblePathSearch3D::clearClosedComponent() noexcept {
  std::ranges::fill(closed_component_, 0U);
  closed_component_marked_ = false;
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

std::size_t FeasiblePathSearch3D::invalidatedLabelCount() const noexcept {
  return invalidated_label_count_;
}

std::size_t FeasiblePathSearch3D::adoptedLabelCount() const noexcept {
  return adopted_label_count_;
}

std::size_t FeasiblePathSearch3D::lastInvalidSegment() const noexcept {
  return last_invalid_segment_;
}

} // namespace drone_city_nav::detail
