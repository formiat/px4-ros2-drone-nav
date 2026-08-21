#include "drone_city_nav/topological_exploration_memory_3d.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace drone_city_nav {
namespace {

void hashCombine(std::size_t& seed, const std::size_t value) noexcept {
  seed ^= value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
}

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] double revisionFreshness(const std::uint64_t current_revision,
                                       const std::uint64_t evidence_revision,
                                       const double decay) noexcept {
  if (evidence_revision == 0U || current_revision < evidence_revision) {
    return 0.0;
  }
  const double age = static_cast<double>(current_revision - evidence_revision);
  return 1.0 / (1.0 + decay * age);
}

} // namespace

std::size_t DirectedTopologyEdge3DHash::operator()(
    const DirectedTopologyEdge3D& edge) const noexcept {
  std::size_t seed = std::hash<std::uint64_t>{}(edge.edge_id.value);
  hashCombine(seed, std::hash<std::uint64_t>{}(edge.from.value));
  hashCombine(seed, std::hash<std::uint64_t>{}(edge.to.value));
  return seed;
}

std::size_t TopologicalExplorationMemory3D::SparseCoverageIndex3DHash::operator()(
    const SparseCoverageIndex3D& index) const noexcept {
  std::size_t seed = std::hash<int>{}(index.x);
  hashCombine(seed, std::hash<int>{}(index.y));
  hashCombine(seed, std::hash<int>{}(index.z));
  return seed;
}

bool topologicalExplorationMemory3DConfigIsValid(
    const TopologicalExplorationMemory3DConfig& config) noexcept {
  return std::isfinite(config.coverage_resolution_m) &&
         config.coverage_resolution_m > 0.0 &&
         std::isfinite(config.visit_penalty_weight) &&
         config.visit_penalty_weight >= 0.0 &&
         std::isfinite(config.observation_penalty_weight) &&
         config.observation_penalty_weight >= 0.0 &&
         std::isfinite(config.revision_decay) && config.revision_decay >= 0.0 &&
         config.maximum_trail_nodes > 0U;
}

TopologicalExplorationMemory3D::TopologicalExplorationMemory3D(
    const TopologicalExplorationMemory3DConfig& config)
    : config_{config} {
  if (!topologicalExplorationMemory3DConfigIsValid(config_)) {
    throw std::invalid_argument{"invalid topological exploration memory configuration"};
  }
}

void TopologicalExplorationMemory3D::recordTraversal(
    const DirectedTopologyEdge3D& edge, const std::uint64_t supporting_revision,
    const double distance_m) {
  if (edge.edge_id.value == 0U || edge.from.value == 0U || edge.to.value == 0U ||
      edge.from == edge.to || supporting_revision == 0U || !std::isfinite(distance_m) ||
      distance_m < 0.0) {
    return;
  }
  DirectedTopologyEdgeEvidence3D& state = edge_evidence_[edge];
  ++state.traversal_count;
  state.traversed_distance_m += distance_m;
  state.last_traversal_revision = supporting_revision;
  state.conclusion_revision = supporting_revision;
  state.result = TopologicalExplorationResult3D::kTraversed;
  recordTrailTransition(edge.from, edge.to);
}

void TopologicalExplorationMemory3D::recordDeadEnd(
    const DirectedTopologyEdge3D& edge, const std::uint64_t supporting_revision) {
  if (edge.edge_id.value == 0U || edge.from.value == 0U || edge.to.value == 0U ||
      edge.from == edge.to || supporting_revision == 0U) {
    return;
  }
  DirectedTopologyEdgeEvidence3D& state = edge_evidence_[edge];
  state.conclusion_revision = supporting_revision;
  state.result = TopologicalExplorationResult3D::kDeadEnd;
}

DirectedTopologyEdgeEvidence3D TopologicalExplorationMemory3D::evidence(
    const DirectedTopologyEdge3D& edge,
    const std::uint64_t current_supporting_revision) const noexcept {
  const auto found = edge_evidence_.find(edge);
  if (found == edge_evidence_.end()) {
    return {};
  }
  DirectedTopologyEdgeEvidence3D result = found->second;
  if (result.result == TopologicalExplorationResult3D::kDeadEnd &&
      current_supporting_revision > result.conclusion_revision) {
    result.result = TopologicalExplorationResult3D::kUnknown;
    result.conclusion_revision = 0U;
  }
  return result;
}

TopologicalExplorationMemory3D::SparseCoverageIndex3D
TopologicalExplorationMemory3D::coverageIndex(const Point3& position) const noexcept {
  return SparseCoverageIndex3D{
      .x = static_cast<int>(std::floor(position.x / config_.coverage_resolution_m)),
      .y = static_cast<int>(std::floor(position.y / config_.coverage_resolution_m)),
      .z = static_cast<int>(std::floor(position.z / config_.coverage_resolution_m)),
  };
}

void TopologicalExplorationMemory3D::recordVisited(const Point3& position,
                                                   const std::uint64_t revision) {
  if (!finitePoint(position) || revision == 0U) {
    return;
  }
  SparseCoverageCell3D& cell = coverage_[coverageIndex(position)];
  ++cell.visit_count;
  cell.last_visit_revision = revision;
}

void TopologicalExplorationMemory3D::recordObserved(const Point3& position,
                                                    const std::uint64_t revision) {
  if (!finitePoint(position) || revision == 0U) {
    return;
  }
  SparseCoverageCell3D& cell = coverage_[coverageIndex(position)];
  ++cell.observation_count;
  cell.last_observation_revision = revision;
}

void TopologicalExplorationMemory3D::recordVisitedPath(
    const std::span<const Point3> points, const std::uint64_t revision,
    const double sample_spacing_m) {
  if (points.empty() || revision == 0U || !(sample_spacing_m > 0.0) ||
      !std::isfinite(sample_spacing_m)) {
    return;
  }
  recordVisited(points.front(), revision);
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const Point3& first = points[index - 1U];
    const Point3& second = points[index];
    if (!finitePoint(first) || !finitePoint(second)) {
      continue;
    }
    const double length_m = distance3D(first, second);
    const std::size_t sample_count =
        static_cast<std::size_t>(std::max(1.0, std::ceil(length_m / sample_spacing_m)));
    for (std::size_t sample = 1U; sample <= sample_count; ++sample) {
      const double ratio =
          static_cast<double>(sample) / static_cast<double>(sample_count);
      recordVisited(Point3{.x = std::lerp(first.x, second.x, ratio),
                           .y = std::lerp(first.y, second.y, ratio),
                           .z = std::lerp(first.z, second.z, ratio)},
                    revision);
    }
  }
}

double TopologicalExplorationMemory3D::softCoveragePenalty(
    const Point3& position, const std::uint64_t current_revision) const noexcept {
  if (!finitePoint(position) || current_revision == 0U) {
    return 0.0;
  }
  const auto found = coverage_.find(coverageIndex(position));
  if (found == coverage_.end()) {
    return 0.0;
  }
  const SparseCoverageCell3D& cell = found->second;
  const double visit_penalty =
      config_.visit_penalty_weight * std::log1p(static_cast<double>(cell.visit_count)) *
      revisionFreshness(current_revision, cell.last_visit_revision,
                        config_.revision_decay);
  const double observation_penalty =
      config_.observation_penalty_weight *
      std::log1p(static_cast<double>(cell.observation_count)) *
      revisionFreshness(current_revision, cell.last_observation_revision,
                        config_.revision_decay);
  const double total = visit_penalty + observation_penalty;
  return std::isfinite(total) ? total : std::numeric_limits<double>::max();
}

std::size_t TopologicalExplorationMemory3D::coverageCellCount() const noexcept {
  return coverage_.size();
}

void TopologicalExplorationMemory3D::recordFrontierSelection(
    const ObservationFrontierId frontier_id) {
  if (frontier_id.value != 0U) {
    ++frontier_selection_counts_[frontier_id.value];
  }
}

std::size_t TopologicalExplorationMemory3D::frontierSelectionCount(
    const ObservationFrontierId frontier_id) const noexcept {
  const auto found = frontier_selection_counts_.find(frontier_id.value);
  return found == frontier_selection_counts_.end() ? 0U : found->second;
}

void TopologicalExplorationMemory3D::resetTrail(const IncrementalTopologyNodeId node) {
  trail_.clear();
  if (node.value != 0U) {
    trail_.push_back(node);
  }
}

void TopologicalExplorationMemory3D::recordTrailTransition(
    const IncrementalTopologyNodeId from, const IncrementalTopologyNodeId to) {
  if (from.value == 0U || to.value == 0U || from == to) {
    return;
  }
  if (trail_.empty()) {
    trail_.push_back(from);
  } else if (trail_.back() != from) {
    const auto found = std::find(trail_.rbegin(), trail_.rend(), from);
    if (found == trail_.rend()) {
      trail_.clear();
      trail_.push_back(from);
    } else {
      trail_.erase(found.base(), trail_.end());
    }
  }
  if (trail_.size() >= 2U && trail_[trail_.size() - 2U] == to) {
    trail_.pop_back();
    return;
  }
  trail_.push_back(to);
  if (trail_.size() > config_.maximum_trail_nodes) {
    const std::size_t excess = trail_.size() - config_.maximum_trail_nodes;
    trail_.erase(trail_.begin(), trail_.begin() + static_cast<std::ptrdiff_t>(excess));
  }
}

std::span<const IncrementalTopologyNodeId>
TopologicalExplorationMemory3D::trail() const noexcept {
  return trail_;
}

void TopologicalExplorationMemory3D::beginMissionLeg() {
  for (auto evidence = edge_evidence_.begin(); evidence != edge_evidence_.end();) {
    if (evidence->second.result != TopologicalExplorationResult3D::kDeadEnd) {
      evidence = edge_evidence_.erase(evidence);
      continue;
    }
    evidence->second.traversal_count = 0U;
    evidence->second.traversed_distance_m = 0.0;
    evidence->second.last_traversal_revision = 0U;
    ++evidence;
  }
  coverage_.clear();
  frontier_selection_counts_.clear();
  trail_.clear();
}

void TopologicalExplorationMemory3D::clear() {
  edge_evidence_.clear();
  coverage_.clear();
  frontier_selection_counts_.clear();
  trail_.clear();
}

const TopologicalExplorationMemory3DConfig&
TopologicalExplorationMemory3D::config() const noexcept {
  return config_;
}

const char* topologicalExplorationResult3DName(
    const TopologicalExplorationResult3D result) noexcept {
  switch (result) {
    case TopologicalExplorationResult3D::kUnknown:
      return "unknown";
    case TopologicalExplorationResult3D::kTraversed:
      return "traversed";
    case TopologicalExplorationResult3D::kDeadEnd:
      return "dead_end";
  }
  return "unknown";
}

} // namespace drone_city_nav
