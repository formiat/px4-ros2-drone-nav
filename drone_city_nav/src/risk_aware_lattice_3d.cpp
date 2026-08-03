#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include "drone_city_nav/bounded_worker_pool.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <ranges>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "risk_aware_lattice_3d_geometry.hpp"
#include "risk_aware_lattice_3d_result.hpp"

namespace drone_city_nav {
namespace {

enum class NodeKind : std::uint8_t {
  kLattice,
  kChannelExit
};

struct Key {
  NodeKind kind{NodeKind::kLattice};
  int x{0};
  int y{0};
  int z{0};
  int channel_index{-1};
  bool reversed{false};

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
    combine(std::hash<int>{}(key.channel_index));
    combine(std::hash<unsigned>{}(static_cast<unsigned>(key.kind)));
    combine(std::hash<bool>{}(key.reversed));
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

struct ChannelTransition {
  std::size_t channel_index{0U};
  bool reversed{false};
};

struct Record {
  double g{std::numeric_limits<double>::infinity()};
  Key parent{};
  bool has_parent{false};
  std::optional<ChannelTransition> channel_transition;
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};
  Vec3 incoming_direction{};
  CostMetrics metrics{};
};

struct QueueEntry {
  double f{0.0};
  double g_at_insert{0.0};
  std::uint64_t sequence{0U};
  Key key{};
};

struct Greater {
  [[nodiscard]] bool operator()(const QueueEntry& lhs,
                                const QueueEntry& rhs) const noexcept {
    return lhs.f == rhs.f ? lhs.sequence > rhs.sequence : lhs.f > rhs.f;
  }
};

using detail::Lattice3DEdgeEvaluation;
using detail::Lattice3DEdgeEvaluationStatus;

struct ContinuationMetrics {
  std::size_t immediate_successors{0U};
  std::size_t reachable_states{0U};
  double reachable_depth_m{0.0};
  Lattice3DSuccessorBatchProfile successor_profile{};
};

struct ReconstructedPath {
  std::vector<Point3> points;
  std::vector<SelectedChannelTraversal> traversals;
};

[[nodiscard]] Point3 latticePoint(const Key& key, const Point3& origin,
                                  const RiskAwareLattice3DConfig& config) noexcept {
  return Point3{origin.x + static_cast<double>(key.x) * config.horizontal_step_m,
                origin.y + static_cast<double>(key.y) * config.horizontal_step_m,
                origin.z + static_cast<double>(key.z) * config.vertical_step_m};
}

[[nodiscard]] Point3 channelEntry(const ConstrainedFreeSpaceEdge& edge,
                                  const bool reversed) noexcept {
  return reversed ? edge.exit : edge.entry;
}

[[nodiscard]] Point3 channelExit(const ConstrainedFreeSpaceEdge& edge,
                                 const bool reversed) noexcept {
  return reversed ? edge.entry : edge.exit;
}

[[nodiscard]] Point3 pointFor(const Key& key, const Point3& origin,
                              const std::span<const ConstrainedFreeSpaceEdge> channels,
                              const RiskAwareLattice3DConfig& config) noexcept {
  if (key.kind == NodeKind::kLattice) {
    return latticePoint(key, origin, config);
  }
  const auto index = static_cast<std::size_t>(key.channel_index);
  return channelExit(channels[index], key.reversed);
}

[[nodiscard]] Key latticeKeyNear(const Point3& point, const Point3& origin,
                                 const RiskAwareLattice3DConfig& config) noexcept {
  return Key{.kind = NodeKind::kLattice,
             .x = static_cast<int>(
                 std::llround((point.x - origin.x) / config.horizontal_step_m)),
             .y = static_cast<int>(
                 std::llround((point.y - origin.y) / config.horizontal_step_m)),
             .z = static_cast<int>(
                 std::llround((point.z - origin.z) / config.vertical_step_m))};
}

[[nodiscard]] ContinuationMetrics
evaluateContinuation(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
                     const Point3& terminal, const Vec3& incoming_direction,
                     const Lattice3DRiskStage stage,
                     const RiskAwareLattice3DConfig& config,
                     BoundedWorkerPool* const worker_pool) {
  struct Candidate {
    Key key{};
    Point3 point{};
  };

  constexpr std::array<int, 3> kOffsets{-1, 0, 1};
  const std::size_t maximum_states =
      std::max<std::size_t>(1U, config.frontier_validation_maximum_states);
  std::queue<Candidate> pending;
  std::unordered_set<Key, KeyHash> visited;
  const Key origin{};
  pending.push(Candidate{.key = origin, .point = terminal});
  visited.insert(origin);
  ContinuationMetrics result;
  while (!pending.empty() && result.reachable_states < maximum_states &&
         result.reachable_depth_m + 1.0e-9 <
             config.frontier_minimum_reachable_depth_m) {
    const Candidate current = pending.front();
    pending.pop();
    const auto collection_started = std::chrono::steady_clock::now();

    struct NeighborEvaluation {
      Candidate candidate{};
      Lattice3DEdgeEvaluation edge{};
    };

    std::vector<NeighborEvaluation> evaluations;
    evaluations.reserve(26U);
    for (const int dx : kOffsets) {
      for (const int dy : kOffsets) {
        for (const int dz : kOffsets) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const Key next{.kind = NodeKind::kLattice,
                         .x = current.key.x + dx,
                         .y = current.key.y + dy,
                         .z = current.key.z + dz};
          if (visited.contains(next)) {
            continue;
          }
          visited.insert(next);
          const Point3 successor{
              terminal.x + static_cast<double>(next.x) * config.horizontal_step_m,
              terminal.y + static_cast<double>(next.y) * config.horizontal_step_m,
              terminal.z + static_cast<double>(next.z) * config.vertical_step_m};
          if (current.key == origin) {
            const double departure_length = distance3D(terminal, successor);
            const Vec3 departure{(successor.x - terminal.x) / departure_length,
                                 (successor.y - terminal.y) / departure_length,
                                 (successor.z - terminal.z) / departure_length};
            const double alignment = departure.x * incoming_direction.x +
                                     departure.y * incoming_direction.y +
                                     departure.z * incoming_direction.z;
            if (alignment < -1.0e-6) {
              continue;
            }
          }
          evaluations.push_back(NeighborEvaluation{
              .candidate = Candidate{.key = next, .point = successor}});
        }
      }
    }
    const auto evaluate_neighbor = [&](const std::size_t candidate_index) {
      NeighborEvaluation& evaluation = evaluations[candidate_index];
      evaluation.edge = detail::evaluateLattice3DEdge(
          grid, esdf_m, current.point, evaluation.candidate.point, stage, config);
    };
    const bool parallel = worker_pool != nullptr &&
                          worker_pool->canParallelizeFromCurrentThread() &&
                          evaluations.size() > 1U;
    if (parallel) {
      worker_pool->parallelFor(evaluations.size(), evaluate_neighbor);
    } else {
      for (std::size_t candidate_index = 0U; candidate_index < evaluations.size();
           ++candidate_index) {
        evaluate_neighbor(candidate_index);
      }
    }
    for (const NeighborEvaluation& evaluation : evaluations) {
      if (evaluation.edge.status != Lattice3DEdgeEvaluationStatus::kValid) {
        continue;
      }
      if (current.key == origin) {
        ++result.immediate_successors;
      }
      ++result.reachable_states;
      result.reachable_depth_m = std::max(
          result.reachable_depth_m, distance3D(terminal, evaluation.candidate.point));
      pending.push(evaluation.candidate);
    }
    ++result.successor_profile.collection_calls;
    result.successor_profile.candidates += evaluations.size();
    if (parallel) {
      ++result.successor_profile.parallel_collection_calls;
      result.successor_profile.parallel_candidates += evaluations.size();
    }
    result.successor_profile.maximum_candidates =
        std::max(result.successor_profile.maximum_candidates, evaluations.size());
    result.successor_profile.worker_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  collection_started)
            .count();
  }
  return result;
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
channelSuccessorRecord(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
                       const Point3& current, const Record& current_record,
                       const ConstrainedFreeSpaceEdge& channel,
                       const std::size_t channel_index, const bool reversed,
                       const Lattice3DRiskStage stage, const Vec3& preferred_direction,
                       const RiskAwareLattice3DConfig& config,
                       const double maximum_connection_distance_m,
                       Lattice3DSuccessorDiagnostics& diagnostics) {
  const Point3 entry = channelEntry(channel, reversed);
  if (distance3D(current, entry) > maximum_connection_distance_m) {
    ++diagnostics.channel_rejected_connection_distance;
    return std::nullopt;
  }
  Record candidate = current_record;
  candidate.has_parent = true;
  candidate.channel_transition =
      ChannelTransition{.channel_index = channel_index, .reversed = reversed};
  candidate.metrics.objective_cost += config.channel_topology_transition_cost;
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
    for (std::size_t index = channel.centerline.size(); index > 1U; --index) {
      const Point3 first = channel.centerline[index - 1U].position;
      const Point3 second = channel.centerline[index - 2U].position;
      if (!appendEvaluatedSegment(grid, esdf_m, first, second, stage,
                                  preferred_direction, config, incoming,
                                  candidate.metrics, candidate.minimum_clearance_m,
                                  evaluation_status, false)) {
        detail::recordLattice3DRejectedEdge(diagnostics, evaluation_status, true);
        return std::nullopt;
      }
    }
  } else {
    for (std::size_t index = 0U; index + 1U < channel.centerline.size(); ++index) {
      if (!appendEvaluatedSegment(grid, esdf_m, channel.centerline[index].position,
                                  channel.centerline[index + 1U].position, stage,
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
            const std::span<const ConstrainedFreeSpaceEdge> channels,
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
    if (!record.channel_transition.has_value()) {
      appendUnique(result.points, pointFor(child, origin, channels, config), station_m);
      continue;
    }
    const ChannelTransition transition = *record.channel_transition;
    const ConstrainedFreeSpaceEdge& channel = channels[transition.channel_index];
    appendUnique(result.points, channelEntry(channel, transition.reversed), station_m);
    const double begin_station_m = station_m;
    if (transition.reversed) {
      for (const RouteSample3D& sample : std::views::reverse(channel.centerline)) {
        appendUnique(result.points, sample.position, station_m);
      }
    } else {
      for (const RouteSample3D& sample : channel.centerline) {
        appendUnique(result.points, sample.position, station_m);
      }
    }
    result.traversals.push_back(SelectedChannelTraversal{
        .channel_id = channel.id,
        .begin_station_m = begin_station_m,
        .end_station_m = station_m,
        .min_z_m = channel.min_z_m,
        .max_z_m = channel.max_z_m,
        .minimum_clearance_m = channel.minimum_clearance_m,
        .speed_limit_mps = channel.speed_limit_mps,
    });
  }
  return result;
}

[[nodiscard]] RiskAwareLattice3DResult
searchStage(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
            const Point3& start, const Vec3& preferred_direction,
            const Point3& planning_goal, const Point3& mission_goal,
            const std::span<const ConstrainedFreeSpaceEdge> channels,
            const Lattice3DRiskStage stage, const RiskAwareLattice3DConfig& config,
            BoundedWorkerPool* const worker_pool) {
  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now() + std::chrono::duration<double, std::milli>(
                                           config.maximum_search_time_ms / 3.0);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, Greater> open;
  std::unordered_map<Key, Record, KeyHash> records;
  const Key root{};
  records[root].g = 0.0;
  std::uint64_t sequence = 0U;
  open.push(QueueEntry{.f = heuristicCost(start, planning_goal, config),
                       .g_at_insert = 0.0,
                       .sequence = sequence++,
                       .key = root});
  Lattice3DSuccessorDiagnostics successor_diagnostics;
  Lattice3DSuccessorProfiling successor_profiling;
  const auto root_successors_started = Clock::now();

  struct ChannelEvaluation {
    std::optional<Record> candidate;
    Lattice3DSuccessorDiagnostics diagnostics{};
  };

  const std::size_t root_candidate_count = channels.size() * 2U;
  std::vector<ChannelEvaluation> root_evaluations(root_candidate_count);
  const auto evaluate_root_channel = [&](const std::size_t candidate_index) {
    const std::size_t channel_index = candidate_index / 2U;
    const bool reversed = candidate_index % 2U != 0U;
    ChannelEvaluation evaluation;
    evaluation.candidate = channelSuccessorRecord(
        grid, esdf_m, start, records.at(root), channels[channel_index], channel_index,
        reversed, stage, preferred_direction, config,
        std::numeric_limits<double>::infinity(), evaluation.diagnostics);
    root_evaluations[candidate_index] = evaluation;
  };
  const bool root_parallel = worker_pool != nullptr &&
                             worker_pool->canParallelizeFromCurrentThread() &&
                             root_candidate_count > 1U;
  if (root_parallel) {
    worker_pool->parallelFor(root_candidate_count, evaluate_root_channel);
  } else {
    for (std::size_t candidate_index = 0U; candidate_index < root_candidate_count;
         ++candidate_index) {
      evaluate_root_channel(candidate_index);
    }
  }
  for (std::size_t candidate_index = 0U; candidate_index < root_candidate_count;
       ++candidate_index) {
    const std::size_t channel_index = candidate_index / 2U;
    const bool reversed = candidate_index % 2U != 0U;
    ChannelEvaluation& evaluation = root_evaluations[candidate_index];
    ++successor_diagnostics.channel_generated;
    detail::accumulateLattice3DSuccessorDiagnostics(successor_diagnostics,
                                                    evaluation.diagnostics);
    if (!evaluation.candidate.has_value()) {
      ++successor_diagnostics.channel_rejected;
      continue;
    }
    const Key next{.kind = NodeKind::kChannelExit,
                   .channel_index = static_cast<int>(channel_index),
                   .reversed = reversed};
    records[next] = *evaluation.candidate;
    records[next].parent = root;
    const Point3 successor = channelExit(channels[channel_index], reversed);
    open.push(QueueEntry{.f = evaluation.candidate->g +
                              1.5 * heuristicCost(successor, planning_goal, config),
                         .g_at_insert = evaluation.candidate->g,
                         .sequence = sequence++,
                         .key = next});
    ++successor_diagnostics.channel_accepted;
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
  double best_remaining = distance3D(start, planning_goal);
  std::size_t expansions = 0U;
  std::size_t stale = 0U;
  std::size_t open_peak = open.size();
  std::size_t records_peak = records.size();
  bool reached = false;
  Lattice3DSearchTermination termination{Lattice3DSearchTermination::kOpenSetExhausted};
  std::optional<CostMetrics> goal_connector_metrics;
  double goal_connector_clearance = std::numeric_limits<double>::infinity();
  constexpr std::array<int, 3> kOffsets{-1, 0, 1};
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
    const Point3 current = pointFor(entry.key, start, channels, config);
    const double remaining = distance3D(current, planning_goal);
    if (remaining < best_remaining) {
      best_remaining = remaining;
      best = entry.key;
    }
    if (remaining <= config.goal_tolerance_m) {
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
    const std::size_t channel_generated_before =
        successor_diagnostics.channel_generated;

    const Key lattice_base = entry.key.kind == NodeKind::kLattice
                                 ? entry.key
                                 : latticeKeyNear(current, start, config);

    struct LatticeEvaluation {
      Key next{};
      Point3 successor{};
      Lattice3DEdgeEvaluation edge{};
      bool zero_length{false};
    };

    std::vector<LatticeEvaluation> lattice_evaluations;
    lattice_evaluations.reserve(26U);
    for (const int dx : kOffsets) {
      for (const int dy : kOffsets) {
        for (const int dz : kOffsets) {
          if (entry.key.kind == NodeKind::kLattice && dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const Key next{.kind = NodeKind::kLattice,
                         .x = lattice_base.x + dx,
                         .y = lattice_base.y + dy,
                         .z = lattice_base.z + dz};
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
      candidate.channel_transition.reset();
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
          QueueEntry{.f = candidate.g + 1.5 * heuristicCost(evaluation.successor,
                                                            planning_goal, config),
                     .g_at_insert = candidate.g,
                     .sequence = sequence++,
                     .key = evaluation.next});
      ++successor_diagnostics.lattice_accepted;
      open_peak = std::max(open_peak, open.size());
      records_peak = std::max(records_peak, records.size());
    }

    const std::size_t channel_candidate_count = channels.size() * 2U;
    std::vector<ChannelEvaluation> channel_evaluations(channel_candidate_count);
    const auto evaluate_channel = [&](const std::size_t candidate_index) {
      const std::size_t channel_index = candidate_index / 2U;
      const bool reversed = candidate_index % 2U != 0U;
      ChannelEvaluation evaluation;
      evaluation.candidate = channelSuccessorRecord(
          grid, esdf_m, current, found->second, channels[channel_index], channel_index,
          reversed, stage, preferred_direction, config,
          config.channel_connection_distance_m, evaluation.diagnostics);
      channel_evaluations[candidate_index] = evaluation;
    };
    const bool channel_parallel = worker_pool != nullptr &&
                                  worker_pool->canParallelizeFromCurrentThread() &&
                                  channel_candidate_count > 1U;
    if (channel_parallel) {
      worker_pool->parallelFor(channel_candidate_count, evaluate_channel);
    } else {
      for (std::size_t candidate_index = 0U; candidate_index < channel_candidate_count;
           ++candidate_index) {
        evaluate_channel(candidate_index);
      }
    }
    for (std::size_t candidate_index = 0U; candidate_index < channel_candidate_count;
         ++candidate_index) {
      const std::size_t channel_index = candidate_index / 2U;
      const bool reversed = candidate_index % 2U != 0U;
      ChannelEvaluation& evaluation = channel_evaluations[candidate_index];
      ++successor_diagnostics.channel_generated;
      detail::accumulateLattice3DSuccessorDiagnostics(successor_diagnostics,
                                                      evaluation.diagnostics);
      if (!evaluation.candidate.has_value()) {
        ++successor_diagnostics.channel_rejected;
        continue;
      }
      const Key next{.kind = NodeKind::kChannelExit,
                     .channel_index = static_cast<int>(channel_index),
                     .reversed = reversed};
      Record& stored = records[next];
      if (!(evaluation.candidate->g + 1.0e-9 < stored.g)) {
        ++successor_diagnostics.channel_rejected_no_cost_improvement;
        continue;
      }
      stored = *evaluation.candidate;
      stored.parent = entry.key;
      const Point3 successor = channelExit(channels[channel_index], reversed);
      open.push(QueueEntry{.f = stored.g +
                                1.5 * heuristicCost(successor, planning_goal, config),
                           .g_at_insert = stored.g,
                           .sequence = sequence++,
                           .key = next});
      ++successor_diagnostics.channel_accepted;
      open_peak = std::max(open_peak, open.size());
      records_peak = std::max(records_peak, records.size());
    }
    const std::size_t candidate_count =
        successor_diagnostics.lattice_generated - lattice_generated_before +
        successor_diagnostics.channel_generated - channel_generated_before;
    ++successor_profiling.search.collection_calls;
    successor_profiling.search.candidates += candidate_count;
    if (lattice_parallel || channel_parallel) {
      ++successor_profiling.search.parallel_collection_calls;
      successor_profiling.search.parallel_candidates +=
          (lattice_parallel ? lattice_evaluations.size() : 0U) +
          (channel_parallel ? channel_candidate_count : 0U);
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
  ReconstructedPath reconstructed = reconstruct(best, start, channels, config, records);
  result.points = std::move(reconstructed.points);
  result.selected_channels = std::move(reconstructed.traversals);
  result.achieved_progress_m = distance3D(start, planning_goal) - best_remaining;
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
  } else if (result.points.size() >= 3U && result.achieved_progress_m >= 4.0) {
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
  if (!reached) {
    const auto continuation_started = std::chrono::steady_clock::now();
    const ContinuationMetrics continuation = evaluateContinuation(
        grid, esdf_m, pointFor(best, start, channels, config),
        records.at(best).incoming_direction, stage, config, worker_pool);
    result.terminal_successor_count = continuation.immediate_successors;
    result.continuation_reachable_states = continuation.reachable_states;
    result.continuation_reachable_depth_m = continuation.reachable_depth_m;
    result.successor_profiling.continuation = continuation.successor_profile;
    result.continuation_validation_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  continuation_started)
            .count();
    if (result.status == Lattice3DStatus::kViableFrontier &&
        (continuation.immediate_successors == 0U ||
         continuation.reachable_depth_m + 1.0e-9 <
             config.frontier_minimum_reachable_depth_m)) {
      result.status =
          termination == Lattice3DSearchTermination::kDeadlineReached ||
                  termination == Lattice3DSearchTermination::kExpansionBudgetExhausted
              ? Lattice3DStatus::kSearchIncomplete
              : Lattice3DStatus::kMotionGraphExhausted;
    }
  }
  result.route = sampleRoute3D(result.points, config.sample_step_m,
                               config.nominal_horizontal_speed_mps);
  result.topology_candidates.reserve(channels.size());
  for (std::size_t channel_index = 0U; channel_index < channels.size();
       ++channel_index) {
    const Key forward{.kind = NodeKind::kChannelExit,
                      .channel_index = static_cast<int>(channel_index),
                      .reversed = false};
    const Key reverse{.kind = NodeKind::kChannelExit,
                      .channel_index = static_cast<int>(channel_index),
                      .reversed = true};
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
          .topology = "channel:" + channels[channel_index].id,
          .risk_stage = stage,
          .status = search_incomplete ? Lattice3DStatus::kSearchIncomplete
                                      : Lattice3DStatus::kMotionGraphExhausted,
          .decision_reason = search_incomplete
                                 ? "not_reached_before_search_termination"
                                 : "entry_unreachable_or_stage_rejected"});
      continue;
    }
    result.topology_candidates.push_back(Lattice3DTopologyCandidate{
        .topology = "channel:" + channels[channel_index].id,
        .risk_stage = stage,
        .status = Lattice3DStatus::kViableFrontier,
        .objective_cost = best_record->metrics.objective_cost,
        .route_length_m = best_record->metrics.route_length_m,
        .estimated_travel_time_s = best_record->metrics.travel_time_s,
        .vertical_alignment_time_s = best_record->metrics.vertical_alignment_time_s,
        .planning_exposure_m = best_record->metrics.planning_exposure_m,
        .critical_exposure_m = best_record->metrics.critical_exposure_m,
        .turn_cost = best_record->metrics.turn_cost,
        .decision_reason = "reachable_channel_transition"});
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

RiskAwareLattice3DResult planRiskAwareLattice3D(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point3& start, const Vec3& preferred_direction, const Point3& mission_goal,
    const std::span<const ConstrainedFreeSpaceEdge> channel_edges,
    const RiskAwareLattice3DConfig& config, BoundedWorkerPool* const worker_pool) {
  const auto search_started = std::chrono::steady_clock::now();
  if (grid.depth <= 1 ||
      esdf_m.size() != static_cast<std::size_t>(grid.width) *
                           static_cast<std::size_t>(grid.height) *
                           static_cast<std::size_t>(grid.depth) ||
      !(config.horizontal_step_m > 0.0) || !(config.vertical_step_m > 0.0) ||
      !(config.sample_step_m > 0.0) || !(config.nominal_horizontal_speed_mps > 0.0) ||
      !(config.nominal_vertical_speed_mps > 0.0) ||
      !(config.channel_connection_distance_m > 0.0)) {
    return {};
  }
  const double full_distance = distance3D(start, mission_goal);
  const double ratio = full_distance > config.planning_goal_distance_m
                           ? config.planning_goal_distance_m / full_distance
                           : 1.0;
  const Point3 planning_goal{std::lerp(start.x, mission_goal.x, ratio),
                             std::lerp(start.y, mission_goal.y, ratio), mission_goal.z};
  std::vector<RiskAwareLattice3DResult> stage_results;
  stage_results.reserve(3U);
  for (const Lattice3DRiskStage stage :
       {Lattice3DRiskStage::kPreferredOnly, Lattice3DRiskStage::kPlanningAllowed,
        Lattice3DRiskStage::kCriticalAllowed}) {
    stage_results.push_back(searchStage(grid, esdf_m, start, preferred_direction,
                                        planning_goal, mission_goal, channel_edges,
                                        stage, config, worker_pool));
  }

  detail::Lattice3DStageSelection selection =
      detail::selectLattice3DStageResult(stage_results, channel_edges.size());
  Lattice3DSuccessorProfiling aggregate_successor_profiling;
  double aggregate_continuation_validation_ms = 0.0;
  for (const RiskAwareLattice3DResult& stage_result : stage_results) {
    detail::accumulateLattice3DSuccessorProfile(
        aggregate_successor_profiling.search, stage_result.successor_profiling.search);
    detail::accumulateLattice3DSuccessorProfile(
        aggregate_successor_profiling.continuation,
        stage_result.successor_profiling.continuation);
    aggregate_continuation_validation_ms += stage_result.continuation_validation_ms;
  }
  RiskAwareLattice3DResult result = std::move(stage_results[selection.selected_index]);
  result.successor_profiling = aggregate_successor_profiling;
  result.continuation_validation_ms = aggregate_continuation_validation_ms;
  result.planning_goal = planning_goal;
  result.topology_candidates = std::move(selection.diagnostics);
  result.route_fingerprint = routeFingerprint(result.route, result.selected_channels);
  result.search_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - search_started)
                         .count();
  return result;
}

} // namespace drone_city_nav
