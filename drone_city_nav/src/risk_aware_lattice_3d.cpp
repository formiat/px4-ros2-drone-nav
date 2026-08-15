#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include "drone_city_nav/bounded_worker_pool.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <compare>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <ranges>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "risk_aware_lattice_3d_continuation.hpp"
#include "risk_aware_lattice_3d_geometry.hpp"
#include "risk_aware_lattice_3d_passage_index.hpp"
#include "risk_aware_lattice_3d_search.hpp"

namespace drone_city_nav {
namespace {

enum class NodeKind : std::uint8_t {
  kLattice,
  kPassageExit
};

enum class TopologyProgress : std::uint8_t {
  kPassageNotTraversed,
  kPassageTraversed,
};

struct Key {
  NodeKind kind{NodeKind::kLattice};
  int x{0};
  int y{0};
  int z{0};
  int passage_index{-1};
  bool reversed{false};
  TopologyProgress topology_progress{TopologyProgress::kPassageNotTraversed};

  [[nodiscard]] bool operator==(const Key&) const noexcept = default;
};

struct KeyHash {
  [[nodiscard]] std::size_t operator()(const Key& key) const noexcept {
    std::size_t seed = std::hash<int>{}(key.x);
    const auto combine = [&seed](const std::size_t value) {
      seed ^= value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    combine(std::hash<int>{}(key.y));
    combine(std::hash<int>{}(key.z));
    combine(std::hash<int>{}(key.passage_index));
    combine(std::hash<unsigned>{}(static_cast<unsigned>(key.kind)));
    combine(std::hash<bool>{}(key.reversed));
    combine(std::hash<unsigned>{}(static_cast<unsigned>(key.topology_progress)));
    return seed;
  }
};

struct CostMetrics {
  double objective_cost{0.0};
  double route_length_m{0.0};
  double travel_time_s{0.0};
  double vertical_alignment_time_s{0.0};
  double planning_exposure_m{0.0};
  double critical_exposure_m{0.0};
  double turn_cost{0.0};
};

struct PassageTransition {
  std::size_t passage_index{0U};
  bool reversed{false};
};

struct Record {
  double g{std::numeric_limits<double>::infinity()};
  Key parent{};
  bool has_parent{false};
  std::optional<PassageTransition> passage_transition;
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};
  Vec3 incoming_direction{};
  CostMetrics metrics{};
};

struct QueueEntry {
  double f{0.0};
  double g_at_insert{0.0};
  bool topology_satisfied{true};
  std::uint64_t sequence{0U};
  Key key{};
};

struct Greater {
  [[nodiscard]] bool operator()(const QueueEntry& lhs,
                                const QueueEntry& rhs) const noexcept {
    if (lhs.topology_satisfied != rhs.topology_satisfied) {
      return !lhs.topology_satisfied && rhs.topology_satisfied;
    }
    return lhs.f == rhs.f ? lhs.sequence > rhs.sequence : lhs.f > rhs.f;
  }
};

using detail::Lattice3DEdgeEvaluation;
using detail::Lattice3DEdgeEvaluationStatus;
using detail::OrientedPassageCandidate;
using detail::PassageEntrySpatialIndex;

struct ReconstructedPath {
  std::vector<Point3> points;
  std::vector<SelectedPassageTraversal> traversals;
};

[[nodiscard]] Point3 latticePoint(const Key& key, const Point3& origin,
                                  const RiskAwareLattice3DConfig& config) noexcept {
  return Point3{origin.x + static_cast<double>(key.x) * config.horizontal_step_m,
                origin.y + static_cast<double>(key.y) * config.horizontal_step_m,
                origin.z + static_cast<double>(key.z) * config.vertical_step_m};
}

[[nodiscard]] Point3 passageEntry(const PassageTraversalEdge& edge,
                                  const bool reversed) noexcept {
  return reversed ? edge.exit : edge.entry;
}

[[nodiscard]] Point3 passageExit(const PassageTraversalEdge& edge,
                                 const bool reversed) noexcept {
  return reversed ? edge.entry : edge.exit;
}

[[nodiscard]] bool
passageInsideFlightEnvelope(const PassageTraversalEdge& passage,
                            const FlightEnvelopeConfig& envelope) noexcept {
  return insideFlightEnvelope(passage.entry, envelope) &&
         insideFlightEnvelope(passage.exit, envelope) &&
         std::ranges::all_of(passage.centerline, [&](const RouteSample3D& sample) {
           return insideFlightEnvelope(sample.position, envelope);
         });
}

[[nodiscard]] Point3 pointFor(const Key& key, const Point3& origin,
                              const std::span<const PassageTraversalEdge> passages,
                              const RiskAwareLattice3DConfig& config) noexcept {
  if (key.kind == NodeKind::kLattice) {
    return latticePoint(key, origin, config);
  }
  const auto index = static_cast<std::size_t>(key.passage_index);
  return passageExit(passages[index], key.reversed);
}

[[nodiscard]] Key latticeKeyNear(const Point3& point, const Point3& origin,
                                 const RiskAwareLattice3DConfig& config,
                                 const TopologyProgress topology_progress) noexcept {
  return Key{.kind = NodeKind::kLattice,
             .x = static_cast<int>(
                 std::llround((point.x - origin.x) / config.horizontal_step_m)),
             .y = static_cast<int>(
                 std::llround((point.y - origin.y) / config.horizontal_step_m)),
             .z = static_cast<int>(
                 std::llround((point.z - origin.z) / config.vertical_step_m)),
             .topology_progress = topology_progress};
}

[[nodiscard]] Vec3 directionBetween(const Point3& first,
                                    const Point3& second) noexcept {
  const double length = distance3D(first, second);
  return length > 1.0e-9
             ? Vec3{(second.x - first.x) / length, (second.y - first.y) / length,
                    (second.z - first.z) / length}
             : Vec3{};
}

[[nodiscard]] double horizontalAngle(const Vec3& first, const Vec3& second) noexcept {
  const double first_norm = std::hypot(first.x, first.y);
  const double second_norm = std::hypot(second.x, second.y);
  if (!(first_norm > 1.0e-9) || !(second_norm > 1.0e-9)) {
    return 0.0;
  }
  const double cosine =
      std::clamp((first.x * second.x + first.y * second.y) / (first_norm * second_norm),
                 -1.0, 1.0);
  return std::acos(cosine);
}

[[nodiscard]] CostMetrics edgeCost(const Point3& first, const Point3& second,
                                   const Vec3& incoming_direction,
                                   const Vec3& preferred_direction,
                                   const Lattice3DEdgeEvaluation& exposure,
                                   const RiskAwareLattice3DConfig& config,
                                   const bool charge_shape_turn = true) noexcept {
  CostMetrics result;
  result.route_length_m = distance3D(first, second);
  const double horizontal_m = std::hypot(second.x - first.x, second.y - first.y);
  const double vertical_m = std::abs(second.z - first.z);
  const double horizontal_time_s =
      horizontal_m / std::max(1.0e-6, config.nominal_horizontal_speed_mps);
  result.vertical_alignment_time_s =
      vertical_m / std::max(1.0e-6, config.nominal_vertical_speed_mps);
  result.travel_time_s = std::max(horizontal_time_s, result.vertical_alignment_time_s);
  result.planning_exposure_m = exposure.planning_exposure_m;
  result.critical_exposure_m = exposure.critical_exposure_m;
  const Vec3 outgoing = directionBetween(first, second);
  result.turn_cost = charge_shape_turn
                         ? config.route_shape_turn_cost_per_rad *
                               horizontalAngle(incoming_direction, outgoing)
                         : 0.0;
  const bool first_maneuver =
      std::hypot(incoming_direction.x, incoming_direction.y) <= 1.0e-9;
  const double heading_cost = first_maneuver
                                  ? config.heading_bias_cost_per_rad *
                                        horizontalAngle(preferred_direction, outgoing)
                                  : 0.0;
  result.objective_cost =
      result.travel_time_s +
      config.vertical_alignment_cost_weight * result.vertical_alignment_time_s +
      config.planning_exposure_cost_per_m * result.planning_exposure_m +
      config.critical_exposure_cost_per_m * result.critical_exposure_m +
      result.turn_cost + heading_cost;
  return result;
}

void accumulate(CostMetrics& target, const CostMetrics& addition) noexcept {
  target.objective_cost += addition.objective_cost;
  target.route_length_m += addition.route_length_m;
  target.travel_time_s += addition.travel_time_s;
  target.vertical_alignment_time_s += addition.vertical_alignment_time_s;
  target.planning_exposure_m += addition.planning_exposure_m;
  target.critical_exposure_m += addition.critical_exposure_m;
  target.turn_cost += addition.turn_cost;
}

[[nodiscard]] double heuristicCost(const Point3& point, const Point3& goal,
                                   const RiskAwareLattice3DConfig& config) noexcept {
  const double horizontal_m = std::hypot(goal.x - point.x, goal.y - point.y);
  const double vertical_m = std::abs(goal.z - point.z);
  return std::max(horizontal_m / std::max(1.0e-6, config.nominal_horizontal_speed_mps),
                  vertical_m / std::max(1.0e-6, config.nominal_vertical_speed_mps));
}

[[nodiscard]] double
requiredPassageHeuristic(const Point3& point, const Point3& goal,
                         const std::span<const PassageTraversalEdge> passages,
                         const RiskAwareLattice3DConfig& config) noexcept {
  double best = std::numeric_limits<double>::infinity();
  for (const PassageTraversalEdge& passage : passages) {
    for (const bool reversed : {false, true}) {
      const Point3 entry = passageEntry(passage, reversed);
      const Point3 exit = passageExit(passage, reversed);
      best = std::min(best, heuristicCost(point, entry, config) +
                                config.passage_topology_transition_cost +
                                heuristicCost(entry, exit, config) +
                                heuristicCost(exit, goal, config));
    }
  }
  return best;
}

[[nodiscard]] double
searchHeuristic(const Point3& point, const Point3& goal,
                const TopologyProgress topology_progress,
                const std::span<const PassageTraversalEdge> passages,
                const detail::Lattice3DTopologyRequirement topology_requirement,
                const RiskAwareLattice3DConfig& config) noexcept {
  if (topology_requirement == detail::Lattice3DTopologyRequirement::kUnconstrained ||
      topology_progress == TopologyProgress::kPassageTraversed) {
    return heuristicCost(point, goal, config);
  }
  return requiredPassageHeuristic(point, goal, passages, config);
}

[[nodiscard]] bool queueTopologySatisfied(
    const TopologyProgress topology_progress,
    const detail::Lattice3DTopologyRequirement topology_requirement) noexcept {
  return topology_requirement == detail::Lattice3DTopologyRequirement::kUnconstrained ||
         topology_progress == TopologyProgress::kPassageTraversed;
}

[[nodiscard]] bool
appendEvaluatedSegment(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
                       const Point3& first, const Point3& second,
                       const Lattice3DRiskStage stage, const Vec3& preferred_direction,
                       const RiskAwareLattice3DConfig& config, Vec3& incoming_direction,
                       CostMetrics& metrics, double& minimum_clearance_m,
                       Lattice3DEdgeEvaluationStatus& evaluation_status,
                       const bool charge_shape_turn = true) {
  const Lattice3DEdgeEvaluation evaluation =
      detail::evaluateLattice3DEdge(grid, esdf_m, first, second, stage, config);
  evaluation_status = evaluation.status;
  if (evaluation.status != Lattice3DEdgeEvaluationStatus::kValid) {
    return false;
  }
  accumulate(metrics, edgeCost(first, second, incoming_direction, preferred_direction,
                               evaluation, config, charge_shape_turn));
  minimum_clearance_m = std::min(minimum_clearance_m, evaluation.minimum_clearance_m);
  if (distance3D(first, second) > 1.0e-9) {
    incoming_direction = directionBetween(first, second);
  }
  return true;
}

[[nodiscard]] std::optional<Record>
passageSuccessorRecord(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
                       const Point3& current, const Record& current_record,
                       const PassageTraversalEdge& passage,
                       const std::size_t passage_index, const bool reversed,
                       const Lattice3DRiskStage stage, const Vec3& preferred_direction,
                       const RiskAwareLattice3DConfig& config,
                       const double maximum_connection_distance_m,
                       Lattice3DSuccessorDiagnostics& diagnostics) {
  if (!passageInsideFlightEnvelope(passage, config.flight_envelope)) {
    ++diagnostics.passage_rejected_flight_envelope;
    return std::nullopt;
  }
  const Point3 entry = passageEntry(passage, reversed);
  if (distance3D(current, entry) > maximum_connection_distance_m) {
    ++diagnostics.passage_rejected_connection_distance;
    return std::nullopt;
  }
  Record candidate = current_record;
  candidate.has_parent = true;
  candidate.passage_transition =
      PassageTransition{.passage_index = passage_index, .reversed = reversed};
  candidate.metrics.objective_cost += config.passage_topology_transition_cost;
  Vec3 incoming = current_record.incoming_direction;
  Lattice3DEdgeEvaluationStatus evaluation_status{
      Lattice3DEdgeEvaluationStatus::kValid};
  if (!appendEvaluatedSegment(grid, esdf_m, current, entry, stage, preferred_direction,
                              config, incoming, candidate.metrics,
                              candidate.minimum_clearance_m, evaluation_status)) {
    detail::recordLattice3DRejectedEdge(diagnostics, evaluation_status, true);
    return std::nullopt;
  }
  if (reversed) {
    for (std::size_t index = passage.centerline.size(); index > 1U; --index) {
      const Point3 first = passage.centerline[index - 1U].position;
      const Point3 second = passage.centerline[index - 2U].position;
      if (!appendEvaluatedSegment(grid, esdf_m, first, second, stage,
                                  preferred_direction, config, incoming,
                                  candidate.metrics, candidate.minimum_clearance_m,
                                  evaluation_status, false)) {
        detail::recordLattice3DRejectedEdge(diagnostics, evaluation_status, true);
        return std::nullopt;
      }
    }
  } else {
    for (std::size_t index = 0U; index + 1U < passage.centerline.size(); ++index) {
      if (!appendEvaluatedSegment(grid, esdf_m, passage.centerline[index].position,
                                  passage.centerline[index + 1U].position, stage,
                                  preferred_direction, config, incoming,
                                  candidate.metrics, candidate.minimum_clearance_m,
                                  evaluation_status, false)) {
        detail::recordLattice3DRejectedEdge(diagnostics, evaluation_status, true);
        return std::nullopt;
      }
    }
  }
  candidate.g = candidate.metrics.objective_cost;
  candidate.incoming_direction = incoming;
  return candidate;
}

void appendUnique(std::vector<Point3>& points, const Point3& point, double& station_m) {
  if (!points.empty()) {
    const double segment_m = distance3D(points.back(), point);
    if (!(segment_m > 1.0e-9)) {
      return;
    }
    station_m += segment_m;
  }
  points.push_back(point);
}

[[nodiscard]] ReconstructedPath
reconstruct(const Key& terminal, const Point3& origin,
            const std::span<const PassageTraversalEdge> passages,
            const RiskAwareLattice3DConfig& config,
            const std::unordered_map<Key, Record, KeyHash>& records) {
  std::vector<Key> chain;
  Key current = terminal;
  while (true) {
    chain.push_back(current);
    const auto found = records.find(current);
    if (found == records.end() || !found->second.has_parent) {
      break;
    }
    current = found->second.parent;
  }
  std::ranges::reverse(chain);
  ReconstructedPath result;
  result.points.reserve(chain.size());
  double station_m = 0.0;
  appendUnique(result.points, origin, station_m);
  for (std::size_t index = 1U; index < chain.size(); ++index) {
    const Key& child = chain[index];
    const Record& record = records.at(child);
    if (!record.passage_transition.has_value()) {
      appendUnique(result.points, pointFor(child, origin, passages, config), station_m);
      continue;
    }
    const PassageTransition transition = *record.passage_transition;
    const PassageTraversalEdge& passage = passages[transition.passage_index];
    appendUnique(result.points, passageEntry(passage, transition.reversed), station_m);
    const double begin_station_m = station_m;
    if (transition.reversed) {
      for (const RouteSample3D& sample : std::views::reverse(passage.centerline)) {
        appendUnique(result.points, sample.position, station_m);
      }
    } else {
      for (const RouteSample3D& sample : passage.centerline) {
        appendUnique(result.points, sample.position, station_m);
      }
    }
    std::vector<PassageTraversalSegmentSpan> segment_spans;
    segment_spans.reserve(passage.segment_spans.size());
    const double passage_length_m =
        passage.centerline.empty() ? 0.0 : passage.centerline.back().station_m;
    if (transition.reversed) {
      for (const PassageTraversalSegmentSpan& segment :
           std::views::reverse(passage.segment_spans)) {
        segment_spans.push_back(PassageTraversalSegmentSpan{
            .passage_segment_id = segment.passage_segment_id,
            .begin_station_m =
                begin_station_m + passage_length_m - segment.end_station_m,
            .end_station_m =
                begin_station_m + passage_length_m - segment.begin_station_m,
        });
      }
    } else {
      for (const PassageTraversalSegmentSpan& segment : passage.segment_spans) {
        segment_spans.push_back(PassageTraversalSegmentSpan{
            .passage_segment_id = segment.passage_segment_id,
            .begin_station_m = begin_station_m + segment.begin_station_m,
            .end_station_m = begin_station_m + segment.end_station_m,
        });
      }
    }
    result.traversals.push_back(SelectedPassageTraversal{
        .passage_traversal_id = passage.id,
        .direction_sign = transition.reversed ? -1 : 1,
        .begin_station_m = begin_station_m,
        .end_station_m = station_m,
        .min_z_m = passage.min_z_m,
        .max_z_m = passage.max_z_m,
        .width_m = passage.width_m,
        .height_m = passage.height_m,
        .minimum_clearance_m = passage.minimum_clearance_m,
        .speed_limit_mps = passage.speed_limit_mps,
        .segment_spans = std::move(segment_spans),
    });
  }
  return result;
}

[[nodiscard]] RiskAwareLattice3DResult searchStage(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point3& start, const Vec3& preferred_direction, const Point3& planning_goal,
    const Point3& mission_goal, const std::span<const PassageTraversalEdge> passages,
    const Lattice3DRiskStage stage,
    const detail::Lattice3DTopologyRequirement topology_requirement,
    const RiskAwareLattice3DConfig& config, BoundedWorkerPool* const worker_pool) {
  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now() + std::chrono::duration<double, std::milli>(
                                           config.maximum_search_time_ms / 3.0);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, Greater> open;
  std::unordered_map<Key, Record, KeyHash> records;
  const PassageEntrySpatialIndex passage_entry_index{
      passages, config.passage_connection_distance_m};
  const Key root{};
  records[root].g = 0.0;
  std::uint64_t sequence = 0U;
  open.push(
      QueueEntry{.f = searchHeuristic(start, planning_goal, root.topology_progress,
                                      passages, topology_requirement, config),
                 .g_at_insert = 0.0,
                 .topology_satisfied = queueTopologySatisfied(root.topology_progress,
                                                              topology_requirement),
                 .sequence = sequence++,
                 .key = root});
  Lattice3DSuccessorDiagnostics successor_diagnostics;
  Lattice3DSuccessorProfiling successor_profiling;
  const auto root_successors_started = Clock::now();

  struct PassageEvaluation {
    std::optional<Record> candidate;
    Lattice3DSuccessorDiagnostics diagnostics{};
  };

  const std::size_t root_candidate_count = passages.size() * 2U;
  std::vector<PassageEvaluation> root_evaluations(root_candidate_count);
  const auto evaluate_root_passage = [&](const std::size_t candidate_index) {
    const std::size_t passage_index = candidate_index / 2U;
    const bool reversed = candidate_index % 2U != 0U;
    PassageEvaluation evaluation;
    evaluation.candidate = passageSuccessorRecord(
        grid, esdf_m, start, records.at(root), passages[passage_index], passage_index,
        reversed, stage, preferred_direction, config,
        std::numeric_limits<double>::infinity(), evaluation.diagnostics);
    root_evaluations[candidate_index] = evaluation;
  };
  const bool root_parallel = worker_pool != nullptr &&
                             worker_pool->canParallelizeFromCurrentThread() &&
                             root_candidate_count > 1U;
  if (root_parallel) {
    worker_pool->parallelFor(root_candidate_count, evaluate_root_passage);
  } else {
    for (std::size_t candidate_index = 0U; candidate_index < root_candidate_count;
         ++candidate_index) {
      evaluate_root_passage(candidate_index);
    }
  }
  for (std::size_t candidate_index = 0U; candidate_index < root_candidate_count;
       ++candidate_index) {
    const std::size_t passage_index = candidate_index / 2U;
    const bool reversed = candidate_index % 2U != 0U;
    PassageEvaluation& evaluation = root_evaluations[candidate_index];
    ++successor_diagnostics.passage_generated;
    detail::accumulateLattice3DSuccessorDiagnostics(successor_diagnostics,
                                                    evaluation.diagnostics);
    if (!evaluation.candidate.has_value()) {
      ++successor_diagnostics.passage_rejected;
      continue;
    }
    const Key next{.kind = NodeKind::kPassageExit,
                   .passage_index = static_cast<int>(passage_index),
                   .reversed = reversed,
                   .topology_progress = TopologyProgress::kPassageTraversed};
    records[next] = *evaluation.candidate;
    records[next].parent = root;
    const Point3 successor = passageExit(passages[passage_index], reversed);
    open.push(QueueEntry{.f = evaluation.candidate->g +
                              1.5 * heuristicCost(successor, planning_goal, config),
                         .g_at_insert = evaluation.candidate->g,
                         .topology_satisfied = true,
                         .sequence = sequence++,
                         .key = next});
    ++successor_diagnostics.passage_accepted;
  }
  if (root_candidate_count > 0U) {
    ++successor_profiling.search.collection_calls;
    successor_profiling.search.candidates += root_candidate_count;
    if (root_parallel) {
      ++successor_profiling.search.parallel_collection_calls;
      successor_profiling.search.parallel_candidates += root_candidate_count;
    }
    successor_profiling.search.maximum_candidates =
        std::max(successor_profiling.search.maximum_candidates, root_candidate_count);
    successor_profiling.search.worker_ms += std::chrono::duration<double, std::milli>(
                                                Clock::now() - root_successors_started)
                                                .count();
  }
  Key best = root;
  const bool unconstrained =
      topology_requirement == detail::Lattice3DTopologyRequirement::kUnconstrained;
  bool best_satisfies_topology = unconstrained;
  double best_remaining = unconstrained ? distance3D(start, planning_goal)
                                        : std::numeric_limits<double>::infinity();
  std::size_t expansions = 0U;
  std::size_t stale = 0U;
  std::size_t open_peak = open.size();
  std::size_t records_peak = records.size();
  bool reached = false;
  Lattice3DSearchTermination termination{Lattice3DSearchTermination::kOpenSetExhausted};
  std::optional<CostMetrics> goal_connector_metrics;
  double goal_connector_clearance = std::numeric_limits<double>::infinity();
  constexpr std::array<int, 3> kHorizontalOffsets{-1, 0, 1};
  constexpr std::array<int, 3> kVerticalOffsets{0, 1, -1};
  while (!open.empty()) {
    if (expansions >= config.maximum_expansions) {
      termination = Lattice3DSearchTermination::kExpansionBudgetExhausted;
      break;
    }
    if (Clock::now() >= deadline) {
      termination = Lattice3DSearchTermination::kDeadlineReached;
      break;
    }
    const QueueEntry entry = open.top();
    open.pop();
    const auto found = records.find(entry.key);
    if (found == records.end() || entry.g_at_insert > found->second.g + 1.0e-9) {
      ++stale;
      continue;
    }
    ++expansions;
    const Point3 current = pointFor(entry.key, start, passages, config);
    const double remaining = distance3D(current, planning_goal);
    const bool satisfies_topology =
        unconstrained ||
        entry.key.topology_progress == TopologyProgress::kPassageTraversed;
    if (satisfies_topology &&
        (!best_satisfies_topology || remaining < best_remaining)) {
      best_remaining = remaining;
      best = entry.key;
      best_satisfies_topology = true;
    }
    if (satisfies_topology && remaining <= config.goal_tolerance_m) {
      CostMetrics connector;
      Vec3 incoming = found->second.incoming_direction;
      double clearance = found->second.minimum_clearance_m;
      Lattice3DEdgeEvaluationStatus connector_status{
          Lattice3DEdgeEvaluationStatus::kValid};
      if (appendEvaluatedSegment(grid, esdf_m, current, planning_goal, stage,
                                 preferred_direction, config, incoming, connector,
                                 clearance, connector_status)) {
        best = entry.key;
        reached = true;
        termination = Lattice3DSearchTermination::kPlanningGoalReached;
        goal_connector_metrics = connector;
        goal_connector_clearance = clearance;
        break;
      }
    }

    const auto successors_started = Clock::now();
    const std::size_t lattice_generated_before =
        successor_diagnostics.lattice_generated;
    const std::size_t passage_generated_before =
        successor_diagnostics.passage_generated;

    const Key lattice_base =
        entry.key.kind == NodeKind::kLattice
            ? entry.key
            : latticeKeyNear(current, start, config, entry.key.topology_progress);

    struct LatticeEvaluation {
      Key next{};
      Point3 successor{};
      Lattice3DEdgeEvaluation edge{};
      bool zero_length{false};
    };

    std::vector<LatticeEvaluation> lattice_evaluations;
    lattice_evaluations.reserve(26U);
    for (const int dx : kHorizontalOffsets) {
      for (const int dy : kHorizontalOffsets) {
        for (const int dz : kVerticalOffsets) {
          if (entry.key.kind == NodeKind::kLattice && dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const Key next{.kind = NodeKind::kLattice,
                         .x = lattice_base.x + dx,
                         .y = lattice_base.y + dy,
                         .z = lattice_base.z + dz,
                         .topology_progress = entry.key.topology_progress};
          lattice_evaluations.push_back(LatticeEvaluation{
              .next = next,
              .successor = latticePoint(next, start, config),
          });
        }
      }
    }
    const auto evaluate_lattice = [&](const std::size_t candidate_index) {
      LatticeEvaluation& evaluation = lattice_evaluations[candidate_index];
      evaluation.edge = detail::evaluateLattice3DEdge(
          grid, esdf_m, current, evaluation.successor, stage, config);
      evaluation.zero_length = distance3D(current, evaluation.successor) <= 1.0e-9;
    };
    const bool lattice_parallel = worker_pool != nullptr &&
                                  worker_pool->canParallelizeFromCurrentThread() &&
                                  lattice_evaluations.size() > 1U;
    if (lattice_parallel) {
      worker_pool->parallelFor(lattice_evaluations.size(), evaluate_lattice);
    } else {
      for (std::size_t candidate_index = 0U;
           candidate_index < lattice_evaluations.size(); ++candidate_index) {
        evaluate_lattice(candidate_index);
      }
    }
    for (const LatticeEvaluation& evaluation : lattice_evaluations) {
      ++successor_diagnostics.lattice_generated;
      if (evaluation.zero_length) {
        ++successor_diagnostics.lattice_rejected_edge;
        ++successor_diagnostics.lattice_rejected_zero_length;
        continue;
      }
      if (evaluation.edge.status != Lattice3DEdgeEvaluationStatus::kValid) {
        ++successor_diagnostics.lattice_rejected_edge;
        detail::recordLattice3DRejectedEdge(successor_diagnostics,
                                            evaluation.edge.status, false);
        continue;
      }
      Record candidate = found->second;
      candidate.parent = entry.key;
      candidate.has_parent = true;
      candidate.passage_transition.reset();
      const CostMetrics addition =
          edgeCost(current, evaluation.successor, found->second.incoming_direction,
                   preferred_direction, evaluation.edge, config);
      accumulate(candidate.metrics, addition);
      candidate.g = candidate.metrics.objective_cost;
      candidate.minimum_clearance_m = std::min(found->second.minimum_clearance_m,
                                               evaluation.edge.minimum_clearance_m);
      candidate.incoming_direction = directionBetween(current, evaluation.successor);
      Record& stored = records[evaluation.next];
      if (!(candidate.g + 1.0e-9 < stored.g)) {
        ++successor_diagnostics.lattice_rejected_no_cost_improvement;
        continue;
      }
      stored = candidate;
      open.push(
          QueueEntry{.f = candidate.g +
                          1.5 * searchHeuristic(evaluation.successor, planning_goal,
                                                evaluation.next.topology_progress,
                                                passages, topology_requirement, config),
                     .g_at_insert = candidate.g,
                     .topology_satisfied = queueTopologySatisfied(
                         evaluation.next.topology_progress, topology_requirement),
                     .sequence = sequence++,
                     .key = evaluation.next});
      ++successor_diagnostics.lattice_accepted;
      open_peak = std::max(open_peak, open.size());
      records_peak = std::max(records_peak, records.size());
    }

    const std::vector<OrientedPassageCandidate> passage_candidates =
        passage_entry_index.near(current);
    const std::size_t passage_candidate_count = passage_candidates.size();
    std::vector<PassageEvaluation> passage_evaluations(passage_candidate_count);
    const auto evaluate_passage = [&](const std::size_t candidate_index) {
      const OrientedPassageCandidate& passage_candidate =
          passage_candidates[candidate_index];
      PassageEvaluation evaluation;
      evaluation.candidate = passageSuccessorRecord(
          grid, esdf_m, current, found->second,
          passages[passage_candidate.passage_index], passage_candidate.passage_index,
          passage_candidate.reversed, stage, preferred_direction, config,
          config.passage_connection_distance_m, evaluation.diagnostics);
      passage_evaluations[candidate_index] = evaluation;
    };
    const bool passage_parallel = worker_pool != nullptr &&
                                  worker_pool->canParallelizeFromCurrentThread() &&
                                  passage_candidate_count > 1U;
    if (passage_parallel) {
      worker_pool->parallelFor(passage_candidate_count, evaluate_passage);
    } else {
      for (std::size_t candidate_index = 0U; candidate_index < passage_candidate_count;
           ++candidate_index) {
        evaluate_passage(candidate_index);
      }
    }
    for (std::size_t candidate_index = 0U; candidate_index < passage_candidate_count;
         ++candidate_index) {
      const OrientedPassageCandidate& passage_candidate =
          passage_candidates[candidate_index];
      const std::size_t passage_index = passage_candidate.passage_index;
      const bool reversed = passage_candidate.reversed;
      PassageEvaluation& evaluation = passage_evaluations[candidate_index];
      ++successor_diagnostics.passage_generated;
      detail::accumulateLattice3DSuccessorDiagnostics(successor_diagnostics,
                                                      evaluation.diagnostics);
      if (!evaluation.candidate.has_value()) {
        ++successor_diagnostics.passage_rejected;
        continue;
      }
      const Key next{.kind = NodeKind::kPassageExit,
                     .passage_index = static_cast<int>(passage_index),
                     .reversed = reversed,
                     .topology_progress = TopologyProgress::kPassageTraversed};
      Record& stored = records[next];
      if (!(evaluation.candidate->g + 1.0e-9 < stored.g)) {
        ++successor_diagnostics.passage_rejected_no_cost_improvement;
        continue;
      }
      stored = *evaluation.candidate;
      stored.parent = entry.key;
      const Point3 successor = passageExit(passages[passage_index], reversed);
      open.push(QueueEntry{.f = stored.g +
                                1.5 * heuristicCost(successor, planning_goal, config),
                           .g_at_insert = stored.g,
                           .topology_satisfied = true,
                           .sequence = sequence++,
                           .key = next});
      ++successor_diagnostics.passage_accepted;
      open_peak = std::max(open_peak, open.size());
      records_peak = std::max(records_peak, records.size());
    }
    const std::size_t candidate_count =
        successor_diagnostics.lattice_generated - lattice_generated_before +
        successor_diagnostics.passage_generated - passage_generated_before;
    ++successor_profiling.search.collection_calls;
    successor_profiling.search.candidates += candidate_count;
    if (lattice_parallel || passage_parallel) {
      ++successor_profiling.search.parallel_collection_calls;
      successor_profiling.search.parallel_candidates +=
          (lattice_parallel ? lattice_evaluations.size() : 0U) +
          (passage_parallel ? passage_candidate_count : 0U);
    }
    successor_profiling.search.maximum_candidates =
        std::max(successor_profiling.search.maximum_candidates, candidate_count);
    successor_profiling.search.worker_ms +=
        std::chrono::duration<double, std::milli>(Clock::now() - successors_started)
            .count();
  }

  RiskAwareLattice3DResult result;
  result.risk_stage = stage;
  result.termination = termination;
  result.planning_goal = planning_goal;
  result.expansions = expansions;
  result.stale_queue_pops = stale;
  result.open_peak = open_peak;
  result.records_peak = records_peak;
  result.successor_diagnostics = successor_diagnostics;
  result.successor_profiling = successor_profiling;
  ReconstructedPath reconstructed = reconstruct(best, start, passages, config, records);
  result.points = std::move(reconstructed.points);
  result.selected_passage_traversals = std::move(reconstructed.traversals);
  result.achieved_progress_m =
      best_satisfies_topology ? distance3D(start, planning_goal) - best_remaining : 0.0;
  result.minimum_clearance_m = records.at(best).minimum_clearance_m;
  CostMetrics metrics = records.at(best).metrics;
  if (reached && goal_connector_metrics.has_value()) {
    accumulate(metrics, *goal_connector_metrics);
    result.minimum_clearance_m =
        std::min(result.minimum_clearance_m, goal_connector_clearance);
    double unused_station_m = 0.0;
    for (std::size_t index = 1U; index < result.points.size(); ++index) {
      unused_station_m += distance3D(result.points[index - 1U], result.points[index]);
    }
    appendUnique(result.points, planning_goal, unused_station_m);
    result.reached_mission_goal = distance3D(planning_goal, mission_goal) <= 1.0e-6;
    result.status = Lattice3DStatus::kReachedPlanningGoal;
  } else if (best_satisfies_topology && result.points.size() >= 3U &&
             result.achieved_progress_m >= 4.0) {
    result.status = Lattice3DStatus::kViableFrontier;
  } else {
    result.status =
        termination == Lattice3DSearchTermination::kDeadlineReached ||
                termination == Lattice3DSearchTermination::kExpansionBudgetExhausted
            ? Lattice3DStatus::kSearchIncomplete
            : Lattice3DStatus::kMotionGraphExhausted;
  }
  result.objective_cost = metrics.objective_cost;
  result.route_length_m = metrics.route_length_m;
  result.estimated_travel_time_s = metrics.travel_time_s;
  result.vertical_alignment_time_s = metrics.vertical_alignment_time_s;
  result.planning_exposure_m = metrics.planning_exposure_m;
  result.critical_exposure_m = metrics.critical_exposure_m;
  result.turn_cost = metrics.turn_cost;
  if (!reached && best_satisfies_topology) {
    const auto continuation_started = std::chrono::steady_clock::now();
    const detail::Lattice3DContinuationMetrics continuation =
        detail::evaluateLattice3DContinuation(
            grid, esdf_m, pointFor(best, start, passages, config),
            records.at(best).incoming_direction, planning_goal, stage, config,
            worker_pool);
    result.terminal_successor_count = continuation.immediate_successors;
    result.continuation_reachable_states = continuation.reachable_states;
    result.continuation_reachable_depth_m = continuation.reachable_depth_m;
    result.successor_profiling.continuation = continuation.successor_profile;
    result.continuation_validation_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  continuation_started)
            .count();
    const bool continuation_viable = continuation.immediate_successors > 0U &&
                                     continuation.reachable_depth_m + 1.0e-9 >=
                                         config.frontier_minimum_reachable_depth_m &&
                                     continuation.path.size() >= 2U;
    const bool continuation_materializable =
        continuation_viable &&
        termination == Lattice3DSearchTermination::kDeadlineReached;
    bool continuation_route_accepted = false;
    if (result.status == Lattice3DStatus::kViableFrontier && !continuation_viable) {
      result.status =
          termination == Lattice3DSearchTermination::kDeadlineReached ||
                  termination == Lattice3DSearchTermination::kExpansionBudgetExhausted
              ? Lattice3DStatus::kSearchIncomplete
              : Lattice3DStatus::kMotionGraphExhausted;
    }
    if (continuation_materializable) {
      CostMetrics continuation_metrics;
      Vec3 incoming_direction = records.at(best).incoming_direction;
      double continuation_clearance_m = records.at(best).minimum_clearance_m;
      Lattice3DEdgeEvaluationStatus continuation_status{
          Lattice3DEdgeEvaluationStatus::kValid};
      bool continuation_accepted = true;
      for (std::size_t index = 1U; index < continuation.path.size(); ++index) {
        if (!appendEvaluatedSegment(
                grid, esdf_m, continuation.path[index - 1U], continuation.path[index],
                stage, preferred_direction, config, incoming_direction,
                continuation_metrics, continuation_clearance_m, continuation_status)) {
          continuation_accepted = false;
          break;
        }
      }
      if (continuation_accepted) {
        accumulate(metrics, continuation_metrics);
        result.minimum_clearance_m =
            std::min(result.minimum_clearance_m, continuation_clearance_m);
        result.achieved_progress_m =
            std::max(result.achieved_progress_m,
                     distance3D(start, planning_goal) -
                         distance3D(continuation.path.back(), planning_goal));
        result.status = Lattice3DStatus::kViableFrontier;
        continuation_route_accepted = true;
      }
    }
    if (result.status == Lattice3DStatus::kViableFrontier &&
        continuation_route_accepted) {
      double station_m = result.route_length_m;
      for (std::size_t index = 1U; index < continuation.path.size(); ++index) {
        appendUnique(result.points, continuation.path[index], station_m);
      }
    }
  }
  result.objective_cost = metrics.objective_cost;
  result.route_length_m = metrics.route_length_m;
  result.estimated_travel_time_s = metrics.travel_time_s;
  result.vertical_alignment_time_s = metrics.vertical_alignment_time_s;
  result.planning_exposure_m = metrics.planning_exposure_m;
  result.critical_exposure_m = metrics.critical_exposure_m;
  result.turn_cost = metrics.turn_cost;
  result.route = sampleRoute3D(result.points, config.sample_step_m,
                               config.nominal_horizontal_speed_mps);
  result.topology_candidates.reserve(passages.size());
  for (std::size_t passage_index = 0U; passage_index < passages.size();
       ++passage_index) {
    const Key forward{.kind = NodeKind::kPassageExit,
                      .passage_index = static_cast<int>(passage_index),
                      .reversed = false,
                      .topology_progress = TopologyProgress::kPassageTraversed};
    const Key reverse{.kind = NodeKind::kPassageExit,
                      .passage_index = static_cast<int>(passage_index),
                      .reversed = true,
                      .topology_progress = TopologyProgress::kPassageTraversed};
    const auto forward_record = records.find(forward);
    const auto reverse_record = records.find(reverse);
    const Record* best_record = nullptr;
    if (forward_record != records.end()) {
      best_record = &forward_record->second;
    }
    if (reverse_record != records.end() &&
        (best_record == nullptr || reverse_record->second.g < best_record->g)) {
      best_record = &reverse_record->second;
    }
    if (best_record == nullptr) {
      const bool search_incomplete =
          termination == Lattice3DSearchTermination::kDeadlineReached ||
          termination == Lattice3DSearchTermination::kExpansionBudgetExhausted;
      result.topology_candidates.push_back(Lattice3DTopologyCandidate{
          .topology = "passage:" + passages[passage_index].id.value(),
          .risk_stage = stage,
          .status = search_incomplete ? Lattice3DStatus::kSearchIncomplete
                                      : Lattice3DStatus::kMotionGraphExhausted,
          .decision_reason = search_incomplete
                                 ? "not_reached_before_search_termination"
                                 : "entry_unreachable_or_stage_rejected"});
      continue;
    }
    result.topology_candidates.push_back(Lattice3DTopologyCandidate{
        .topology = "passage:" + passages[passage_index].id.value(),
        .risk_stage = stage,
        .status = Lattice3DStatus::kViableFrontier,
        .objective_cost = best_record->metrics.objective_cost,
        .route_length_m = best_record->metrics.route_length_m,
        .estimated_travel_time_s = best_record->metrics.travel_time_s,
        .vertical_alignment_time_s = best_record->metrics.vertical_alignment_time_s,
        .planning_exposure_m = best_record->metrics.planning_exposure_m,
        .critical_exposure_m = best_record->metrics.critical_exposure_m,
        .turn_cost = best_record->metrics.turn_cost,
        .decision_reason = "reachable_passage_transition"});
  }
  for (Lattice3DTopologyCandidate& candidate : result.topology_candidates) {
    candidate.termination = result.termination;
    candidate.achieved_progress_m = result.achieved_progress_m;
    candidate.minimum_clearance_m = result.minimum_clearance_m;
    candidate.expansions = result.expansions;
    candidate.stale_queue_pops = result.stale_queue_pops;
    candidate.open_peak = result.open_peak;
    candidate.records_peak = result.records_peak;
    candidate.terminal_successor_count = result.terminal_successor_count;
    candidate.continuation_reachable_states = result.continuation_reachable_states;
    candidate.continuation_reachable_depth_m = result.continuation_reachable_depth_m;
  }
  return result;
}

} // namespace

namespace detail {

RiskAwareLattice3DResult searchRiskAwareLattice3DStage(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point3& start, const Vec3& preferred_direction, const Point3& planning_goal,
    const Point3& mission_goal,
    const std::span<const PassageTraversalEdge> passage_traversals,
    const Lattice3DRiskStage stage,
    const Lattice3DTopologyRequirement topology_requirement,
    const RiskAwareLattice3DConfig& config, BoundedWorkerPool* const worker_pool) {
  return searchStage(grid, esdf_m, start, preferred_direction, planning_goal,
                     mission_goal, passage_traversals, stage, topology_requirement,
                     config, worker_pool);
}

} // namespace detail

} // namespace drone_city_nav
