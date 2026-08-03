#include "drone_city_nav/risk_aware_lattice.hpp"

#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/esdf_query.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <ranges>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "risk_aware_lattice_geometry.hpp"

namespace drone_city_nav {
namespace {

using detail::SegmentEvaluation;

struct LatticeKey {
  int x{0};
  int y{0};
  int heading{0};

  bool operator==(const LatticeKey&) const = default;
};

struct LatticeKeyHash {
  std::size_t operator()(const LatticeKey& key) const noexcept {
    const std::uint64_t x = static_cast<std::uint32_t>(key.x);
    const std::uint64_t y = static_cast<std::uint32_t>(key.y);
    const std::uint64_t heading = static_cast<std::uint32_t>(key.heading);
    return static_cast<std::size_t>((x << 32U) ^ (y << 8U) ^ heading);
  }
};

struct Record {
  double cost{std::numeric_limits<double>::infinity()};
  double path_length_m{0.0};
  std::optional<LatticeKey> parent;
  std::array<Point2, 4U> edge_points{};
  std::size_t edge_point_count{0U};
};

struct QueueEntry {
  LatticeKey key{};
  double g_at_insert{0.0};
  double f_score{0.0};
  std::uint64_t insertion_sequence{0U};

  bool operator>(const QueueEntry& other) const noexcept {
    if (f_score != other.f_score) {
      return f_score > other.f_score;
    }
    return insertion_sequence > other.insertion_sequence;
  }
};

struct Successor {
  LatticeKey key{};
  Point2 endpoint{};
  double length_m{0.0};
  double edge_cost{0.0};
  std::array<Point2, 4U> edge_points{};
  std::size_t edge_point_count{0U};
};

struct FrontierEvaluation {
  LatticeKey key{};
  std::vector<Point2> guide;
  double guide_length_m{0.0};
  double progress_m{0.0};
  double remaining_m{0.0};
  double endpoint_displacement_m{0.0};
  double selection_score{0.0};
  double cost{0.0};
  std::size_t immediate_successors{0U};
  std::size_t continuation_states{0U};
  double reachable_depth_m{0.0};
  LatticeRiskStage stage{LatticeRiskStage::kPreferredOnly};
  LatticeSuccessorBatchProfile successor_profile{};
};

struct ContinuationEvaluation {
  std::size_t immediate_successors{0U};
  std::size_t reachable_states{0U};
  double reachable_depth_m{0.0};
  std::vector<Point2> extension;
  LatticeSuccessorBatchProfile successor_profile{};
};

void accumulateSuccessorProfile(LatticeSuccessorBatchProfile& target,
                                const LatticeSuccessorBatchProfile& addition) noexcept {
  target.collection_calls += addition.collection_calls;
  target.parallel_collection_calls += addition.parallel_collection_calls;
  target.candidates += addition.candidates;
  target.parallel_candidates += addition.parallel_candidates;
  target.maximum_candidates =
      std::max(target.maximum_candidates, addition.maximum_candidates);
  target.worker_ms += addition.worker_ms;
}

struct ContinuationQueueEntry {
  LatticeKey key{};
  double path_depth_m{0.0};
  double endpoint_displacement_m{0.0};
  double remaining_goal_distance_m{0.0};

  bool operator<(const ContinuationQueueEntry& other) const noexcept {
    return std::tuple{endpoint_displacement_m,
                      -remaining_goal_distance_m,
                      path_depth_m,
                      -key.x,
                      -key.y,
                      -key.heading} < std::tuple{other.endpoint_displacement_m,
                                                 -other.remaining_goal_distance_m,
                                                 other.path_depth_m,
                                                 -other.key.x,
                                                 -other.key.y,
                                                 -other.key.heading};
  }
};

struct ContinuationRecord {
  LatticeKey key{};
  std::optional<std::uint64_t> parent;
};

struct FrontierPoolCandidate {
  LatticeKey key{};
  double path_length_m{0.0};
  double endpoint_displacement_m{0.0};
  double remaining_m{0.0};
  double selection_score{0.0};
};

struct StageOutcome {
  bool initialized{false};
  bool goal_reached{false};
  bool budget_exhausted{false};
  bool deadline_reached{false};
  bool roi_boundary_seen{false};
  LatticeRiskStage stage{LatticeRiskStage::kPreferredOnly};
  LatticeSearchTermination termination{LatticeSearchTermination::kOpenSetExhausted};
  std::unordered_map<LatticeKey, Record, LatticeKeyHash> records;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open;
  std::optional<LatticeKey> terminal;
  double terminal_connector_cost{0.0};
  std::uint64_t insertion_sequence{0U};
  std::size_t expansions{0U};
  std::size_t stale_pops{0U};
  std::size_t open_peak{0U};
  std::size_t records_peak{0U};
  std::size_t last_frontier_validation_expansions{0U};
  bool frontier_validation_started{false};
  LatticeSuccessorDiagnostics successor_diagnostics{};
};

[[nodiscard]] Point2 cellCenter(const mppi::EsdfGrid& grid, const LatticeKey& key) {
  return Point2{
      grid.origin_x_m + (static_cast<double>(key.x) + 0.5) * grid.resolution_m,
      grid.origin_y_m + (static_cast<double>(key.y) + 0.5) * grid.resolution_m};
}

} // namespace

struct RiskAwareLatticeSearchSession::Impl {
  bool initialized{false};
  mppi::EsdfGrid grid{};
  const float* esdf_data{nullptr};
  std::size_t esdf_size{0U};
  Point2 start{};
  Point2 mission_goal{};
  double preferred_heading_rad{0.0};
  const LatticeFrontierBlacklistEntry* blacklist_data{nullptr};
  std::size_t blacklist_size{0U};
  std::uint64_t failure_memory_fingerprint{0U};
  std::array<StageOutcome, 3U> outcomes{};

  [[nodiscard]] bool
  matches(const mppi::EsdfGrid& candidate_grid,
          const std::span<const float> candidate_esdf, const Point2 candidate_start,
          const double candidate_heading, const Point2 candidate_goal,
          const std::span<const LatticeFrontierBlacklistEntry> candidate_blacklist)
      const noexcept {
    return initialized && grid.width == candidate_grid.width &&
           grid.height == candidate_grid.height &&
           grid.resolution_m == candidate_grid.resolution_m &&
           grid.origin_x_m == candidate_grid.origin_x_m &&
           grid.origin_y_m == candidate_grid.origin_y_m &&
           esdf_data == candidate_esdf.data() && esdf_size == candidate_esdf.size() &&
           start.x == candidate_start.x && start.y == candidate_start.y &&
           mission_goal.x == candidate_goal.x && mission_goal.y == candidate_goal.y &&
           preferred_heading_rad == candidate_heading &&
           blacklist_data == candidate_blacklist.data() &&
           blacklist_size == candidate_blacklist.size() &&
           failure_memory_fingerprint ==
               detail::latticeFailureMemoryFingerprint(candidate_blacklist);
  }

  void
  initialize(const mppi::EsdfGrid& candidate_grid,
             const std::span<const float> candidate_esdf, const Point2 candidate_start,
             const double candidate_heading, const Point2 candidate_goal,
             const std::span<const LatticeFrontierBlacklistEntry> candidate_blacklist) {
    initialized = true;
    grid = candidate_grid;
    esdf_data = candidate_esdf.data();
    esdf_size = candidate_esdf.size();
    start = candidate_start;
    mission_goal = candidate_goal;
    preferred_heading_rad = candidate_heading;
    blacklist_data = candidate_blacklist.data();
    blacklist_size = candidate_blacklist.size();
    failure_memory_fingerprint =
        detail::latticeFailureMemoryFingerprint(candidate_blacklist);
    outcomes = {};
  }
};

RiskAwareLatticeSearchSession::RiskAwareLatticeSearchSession()
    : impl_{std::make_unique<Impl>()} {
}

RiskAwareLatticeSearchSession::~RiskAwareLatticeSearchSession() = default;

RiskAwareLatticeSearchSession::RiskAwareLatticeSearchSession(
    RiskAwareLatticeSearchSession&&) noexcept = default;

RiskAwareLatticeSearchSession& RiskAwareLatticeSearchSession::operator=(
    RiskAwareLatticeSearchSession&&) noexcept = default;

void RiskAwareLatticeSearchSession::reset() {
  impl_ = std::make_unique<Impl>();
}

RiskAwareLatticeResult planRiskAwareMotionPrimitiveGuide(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m, const Point2 start,
    const double preferred_heading_rad, const Point2 mission_goal,
    const RiskAwareLatticeConfig& config,
    const std::span<const LatticeFrontierBlacklistEntry> frontier_blacklist,
    RiskAwareLatticeSearchSession* session, BoundedWorkerPool* const worker_pool) {
  RiskAwareLatticeResult result;
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution_m <= 0.0F ||
      esdf_m.size() != static_cast<std::size_t>(grid.width) *
                           static_cast<std::size_t>(grid.height) ||
      config.heading_bins < 4 || !(config.short_primitive_length_m > 0.0) ||
      !(config.primitive_length_m >= config.short_primitive_length_m) ||
      !(config.maximum_search_roi_halo_m > 0.0) ||
      !(config.maximum_search_time_ms > 0.0) ||
      config.maximum_frontier_candidates == 0U ||
      !(config.minimum_frontier_endpoint_displacement_m >= 0.0) ||
      !(config.minimum_frontier_reachable_depth_m > 0.0) ||
      config.frontier_validation_maximum_states == 0U ||
      config.frontier_validation_expansion_interval == 0U ||
      !(config.frontier_goal_distance_weight >= 0.0) ||
      !std::isfinite(config.frontier_goal_distance_weight) ||
      !(config.frontier_blacklist_radius_m >= 0.0) ||
      config.frontier_blacklist_heading_tolerance_bins < 0 ||
      config.frontier_blacklist_heading_tolerance_bins > config.heading_bins / 2) {
    return result;
  }
  result.planning_goal = detail::recedingLatticeGoal(start, mission_goal, config,
                                                     result.reached_mission_goal);
  const auto makeKey = [&grid, &config](const Point2 point, const int heading) {
    return LatticeKey{
        static_cast<int>(std::floor((point.x - grid.origin_x_m) / grid.resolution_m)),
        static_cast<int>(std::floor((point.y - grid.origin_y_m) / grid.resolution_m)),
        detail::wrapLatticeHeading(heading, config.heading_bins)};
  };
  const LatticeKey start_key =
      makeKey(start, detail::nearestLatticeHeadingBin(preferred_heading_rad,
                                                      config.heading_bins));
  if (detail::queryLatticeEsdf(grid, esdf_m, start).status ==
      EsdfQueryStatus::kOutsideGrid) {
    return result;
  }
  std::unique_ptr<RiskAwareLatticeSearchSession> local_session;
  if (session == nullptr) {
    local_session = std::make_unique<RiskAwareLatticeSearchSession>();
    session = local_session.get();
  }
  RiskAwareLatticeSearchSession& active_session = *session;
  result.search_session_resumed = active_session.impl_->matches(
      grid, esdf_m, start, preferred_heading_rad, mission_goal, frontier_blacklist);
  if (!result.search_session_resumed) {
    active_session.impl_->initialize(grid, esdf_m, start, preferred_heading_rad,
                                     mission_goal, frontier_blacklist);
  }
  std::array<StageOutcome, 3U>& outcomes = active_session.impl_->outcomes;
  const double roi_min_x =
      std::min(start.x, result.planning_goal.x) - config.maximum_search_roi_halo_m;
  const double roi_max_x =
      std::max(start.x, result.planning_goal.x) + config.maximum_search_roi_halo_m;
  const double roi_min_y =
      std::min(start.y, result.planning_goal.y) - config.maximum_search_roi_halo_m;
  const double roi_max_y =
      std::max(start.y, result.planning_goal.y) + config.maximum_search_roi_halo_m;
  const auto inside_roi = [&](const Point2 point) {
    return point.x >= roi_min_x && point.x <= roi_max_x && point.y >= roi_min_y &&
           point.y <= roi_max_y;
  };

  const auto collect_successors = [&](const Point2 current,
                                      const LatticeKey& current_key,
                                      const LatticeRiskStage stage,
                                      bool& roi_boundary_seen,
                                      LatticeSuccessorDiagnostics* diagnostics,
                                      LatticeSuccessorBatchProfile* profile) {
    const auto collection_started = std::chrono::steady_clock::now();
    struct CandidateSpec {
      int next_heading{0};
      double length_m{0.0};
    };
    struct CandidateEvaluation {
      Point2 endpoint{};
      LatticeKey key{};
      SegmentEvaluation segment{};
      detail::FailureMemoryEvaluation failure_memory{};
      bool inside_roi{false};
    };

    constexpr std::array normal_heading_offsets{-4, -2, -1, 0, 1, 2, 4, 8};
    std::vector<CandidateSpec> candidate_specs;
    candidate_specs.reserve(static_cast<std::size_t>(config.heading_bins) +
                            normal_heading_offsets.size());
    for (int next_heading = 0; next_heading < config.heading_bins; ++next_heading) {
      candidate_specs.push_back(CandidateSpec{
          .next_heading = next_heading,
          .length_m = config.short_primitive_length_m,
      });
    }
    for (const int heading_offset : normal_heading_offsets) {
      candidate_specs.push_back(CandidateSpec{
          .next_heading = detail::wrapLatticeHeading(
              current_key.heading + heading_offset, config.heading_bins),
          .length_m = config.primitive_length_m,
      });
    }

    std::vector<CandidateEvaluation> evaluations(candidate_specs.size());
    const auto evaluate_candidate = [&](const std::size_t candidate_index) {
      const CandidateSpec& spec = candidate_specs[candidate_index];
      const double heading =
          detail::latticeHeadingForBin(spec.next_heading, config.heading_bins);
      CandidateEvaluation evaluation;
      evaluation.endpoint = Point2{current.x + std::cos(heading) * spec.length_m,
                                   current.y + std::sin(heading) * spec.length_m};
      evaluation.inside_roi = inside_roi(evaluation.endpoint);
      if (!evaluation.inside_roi) {
        evaluations[candidate_index] = evaluation;
        return;
      }
      evaluation.segment = detail::evaluateLatticeSegment(
          grid, esdf_m, current, evaluation.endpoint, config, stage);
      if (!evaluation.segment.valid) {
        evaluations[candidate_index] = evaluation;
        return;
      }
      const std::array candidate_segment{current, evaluation.endpoint};
      evaluation.failure_memory = detail::evaluateLatticeFailureMemory(
          candidate_segment, frontier_blacklist, config);
      evaluation.key = makeKey(evaluation.endpoint, spec.next_heading);
      evaluations[candidate_index] = evaluation;
    };
    const bool parallel = worker_pool != nullptr &&
                          worker_pool->canParallelizeFromCurrentThread() &&
                          candidate_specs.size() > 1U;
    if (parallel) {
      worker_pool->parallelFor(candidate_specs.size(), evaluate_candidate);
    } else {
      for (std::size_t candidate_index = 0U; candidate_index < candidate_specs.size();
           ++candidate_index) {
        evaluate_candidate(candidate_index);
      }
    }

    std::vector<Successor> successors;
    successors.reserve(candidate_specs.size());
    std::unordered_set<LatticeKey, LatticeKeyHash> emitted;
    for (std::size_t candidate_index = 0U; candidate_index < candidate_specs.size();
         ++candidate_index) {
      const CandidateSpec& spec = candidate_specs[candidate_index];
      const CandidateEvaluation& evaluation = evaluations[candidate_index];
      if (diagnostics != nullptr) {
        ++diagnostics->generated;
      }
      if (!evaluation.inside_roi) {
        roi_boundary_seen = true;
        if (diagnostics != nullptr) {
          ++diagnostics->rejected_outside_roi;
        }
        continue;
      }
      if (!evaluation.segment.valid) {
        if (diagnostics != nullptr) {
          detail::recordLatticeSegmentRejection(evaluation.segment, *diagnostics);
        }
        continue;
      }
      if (evaluation.failure_memory.hard_rejected) {
        if (diagnostics != nullptr) {
          ++diagnostics->rejected_blacklisted_failure;
        }
        continue;
      }
      if (!emitted.insert(evaluation.key).second) {
        continue;
      }
      const int heading_change = detail::latticeHeadingBinDistance(
          current_key.heading, spec.next_heading, config.heading_bins);
      successors.push_back(Successor{
          .key = evaluation.key,
          .endpoint = evaluation.endpoint,
          .length_m = spec.length_m,
          .edge_cost = spec.length_m + evaluation.segment.risk_cost +
                       evaluation.failure_memory.soft_penalty_cost +
                       config.turn_cost * static_cast<double>(heading_change),
      });
      if (diagnostics != nullptr && evaluation.failure_memory.soft_penalty_cost > 0.0) {
        ++diagnostics->soft_tabu_penalties_applied;
      }
      if (diagnostics != nullptr) {
        ++diagnostics->accepted;
      }
    }
    if (profile != nullptr) {
      ++profile->collection_calls;
      const std::size_t candidate_count = candidate_specs.size();
      profile->candidates += candidate_count;
      if (parallel) {
        ++profile->parallel_collection_calls;
        profile->parallel_candidates += candidate_count;
      }
      profile->maximum_candidates =
          std::max(profile->maximum_candidates, candidate_count);
      profile->worker_ms += std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - collection_started)
                                .count();
    }
    return successors;
  };

  const auto run_stage = [&](StageOutcome& outcome, const LatticeRiskStage stage,
                             const std::size_t expansion_budget,
                             const double stage_time_ms) {
    if (!outcome.initialized) {
      outcome.initialized = true;
      outcome.stage = stage;
      outcome.records[start_key].cost = 0.0;
      outcome.open.push(QueueEntry{.key = start_key,
                                   .g_at_insert = 0.0,
                                   .f_score = distance(start, result.planning_goal),
                                   .insertion_sequence = outcome.insertion_sequence++});
      outcome.open_peak = 1U;
      outcome.records_peak = 1U;
    }
    outcome.budget_exhausted = false;
    outcome.deadline_reached = false;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double, std::milli>{stage_time_ms};
    const std::size_t slice_start_expansions = outcome.expansions;
    while (!outcome.open.empty() &&
           outcome.expansions - slice_start_expansions < expansion_budget &&
           std::chrono::steady_clock::now() < deadline) {
      const QueueEntry entry = outcome.open.top();
      outcome.open.pop();
      const auto record = outcome.records.find(entry.key);
      if (record == outcome.records.end() ||
          entry.g_at_insert > record->second.cost + 1.0e-9) {
        ++outcome.stale_pops;
        continue;
      }
      const Point2 current = cellCenter(grid, entry.key);
      const double goal_distance = distance(current, result.planning_goal);
      if (goal_distance <= config.goal_tolerance_m) {
        const SegmentEvaluation connector = detail::evaluateLatticeSegment(
            grid, esdf_m, current, result.planning_goal, config, stage);
        if (connector.valid) {
          outcome.goal_reached = true;
          outcome.terminal = entry.key;
          outcome.terminal_connector_cost = goal_distance + connector.risk_cost;
          outcome.termination = LatticeSearchTermination::kPlanningGoalReached;
          break;
        }
      }
      ++outcome.expansions;
      const std::vector<Successor> successors = collect_successors(
          current, entry.key, stage, outcome.roi_boundary_seen,
          &outcome.successor_diagnostics, &result.successor_profiling.search);
      for (const Successor& successor : successors) {
        const double next_cost = record->second.cost + successor.edge_cost;
        Record& next_record = outcome.records[successor.key];
        if (next_cost >= next_record.cost) {
          ++outcome.successor_diagnostics.rejected_no_cost_improvement;
          continue;
        }
        next_record.cost = next_cost;
        next_record.path_length_m = record->second.path_length_m + successor.length_m;
        next_record.parent = entry.key;
        next_record.edge_points = successor.edge_points;
        next_record.edge_point_count = successor.edge_point_count;
        const double heuristic = distance(result.planning_goal, successor.endpoint);
        outcome.open.push(QueueEntry{
            .key = successor.key,
            .g_at_insert = next_cost,
            .f_score = next_cost + config.heuristic_weight * heuristic,
            .insertion_sequence = outcome.insertion_sequence++,
        });
      }
      outcome.open_peak = std::max(outcome.open_peak, outcome.open.size());
      outcome.records_peak = std::max(outcome.records_peak, outcome.records.size());
    }
    if (!outcome.goal_reached) {
      outcome.deadline_reached =
          !outcome.open.empty() && std::chrono::steady_clock::now() >= deadline;
      outcome.budget_exhausted =
          !outcome.open.empty() &&
          outcome.expansions - slice_start_expansions >= expansion_budget;
      if (outcome.deadline_reached) {
        outcome.termination = LatticeSearchTermination::kDeadlineReached;
      } else if (outcome.budget_exhausted) {
        outcome.termination = LatticeSearchTermination::kExpansionBudgetExhausted;
      } else {
        outcome.termination = LatticeSearchTermination::kOpenSetExhausted;
      }
    }
  };

  const auto reconstruct = [&](const StageOutcome& outcome,
                               const LatticeKey& terminal) {
    std::vector<LatticeKey> chain;
    for (std::optional<LatticeKey> key = terminal; key.has_value();
         key = outcome.records.at(*key).parent) {
      chain.push_back(*key);
    }
    std::ranges::reverse(chain);
    std::vector<Point2> guide;
    guide.reserve(chain.size() + 4U);
    const auto append_distinct = [&guide](const Point2 point) {
      if (guide.empty() || distance(guide.back(), point) > 1.0e-6) {
        guide.push_back(point);
      }
    };
    append_distinct(cellCenter(grid, chain.front()));
    for (std::size_t chain_index = 1U; chain_index < chain.size(); ++chain_index) {
      const Record& record = outcome.records.at(chain[chain_index]);
      for (std::size_t edge_index = 0U; edge_index < record.edge_point_count;
           ++edge_index) {
        append_distinct(record.edge_points.at(edge_index));
      }
      append_distinct(cellCenter(grid, chain[chain_index]));
    }
    return guide;
  };

  const auto collect_frontier_pool = [&](const StageOutcome& outcome) {
    const auto objective_rank = [](const FrontierPoolCandidate& candidate) {
      return std::tuple{candidate.selection_score, -candidate.path_length_m,
                        candidate.remaining_m,     candidate.key.x,
                        candidate.key.y,           candidate.key.heading};
    };
    const auto cell_id = [](const LatticeKey& key) {
      const std::uint64_t x = static_cast<std::uint32_t>(key.x);
      const std::uint64_t y = static_cast<std::uint32_t>(key.y);
      return (x << 32U) | y;
    };

    std::unordered_map<std::uint64_t, FrontierPoolCandidate> best_by_cell;
    best_by_cell.reserve(outcome.records.size());
    for (const auto& [key, record] : outcome.records) {
      if (!record.parent.has_value()) {
        continue;
      }
      const Point2 endpoint = cellCenter(grid, key);
      const double endpoint_displacement_m = distance(start, endpoint);
      const double remaining_m = distance(endpoint, result.planning_goal);
      FrontierPoolCandidate candidate{
          .key = key,
          .path_length_m = record.path_length_m,
          .endpoint_displacement_m = endpoint_displacement_m,
          .remaining_m = remaining_m,
          .selection_score =
              record.cost + config.frontier_goal_distance_weight * remaining_m,
      };
      const std::uint64_t id = cell_id(key);
      const auto existing = best_by_cell.find(id);
      if (existing == best_by_cell.end() ||
          objective_rank(candidate) < objective_rank(existing->second)) {
        best_by_cell.insert_or_assign(id, candidate);
      }
    }

    const std::size_t sector_count =
        std::min(config.maximum_frontier_candidates,
                 static_cast<std::size_t>(config.heading_bins));
    std::vector<std::vector<FrontierPoolCandidate>> sectors(sector_count);
    std::vector<FrontierPoolCandidate> all_candidates;
    all_candidates.reserve(best_by_cell.size());
    for (const auto& [unused_id, candidate] : best_by_cell) {
      static_cast<void>(unused_id);
      const Point2 endpoint = cellCenter(grid, candidate.key);
      const double bearing = std::atan2(endpoint.y - start.y, endpoint.x - start.x);
      const std::size_t sector = static_cast<std::size_t>(
          detail::nearestLatticeHeadingBin(bearing, static_cast<int>(sector_count)));
      sectors.at(sector).push_back(candidate);
      all_candidates.push_back(candidate);
    }

    std::vector<FrontierPoolCandidate> selected;
    selected.reserve(
        std::min(config.maximum_frontier_candidates, all_candidates.size()));
    std::unordered_set<LatticeKey, LatticeKeyHash> selected_keys;
    const auto append_candidate = [&](const FrontierPoolCandidate& candidate) {
      if (selected.size() < config.maximum_frontier_candidates &&
          selected_keys.insert(candidate.key).second) {
        selected.push_back(candidate);
      }
    };
    const std::size_t base_quota = config.maximum_frontier_candidates / sector_count;
    const std::size_t extra_sectors = config.maximum_frontier_candidates % sector_count;
    for (std::size_t sector_index = 0U; sector_index < sectors.size(); ++sector_index) {
      std::vector<FrontierPoolCandidate>& sector = sectors[sector_index];
      if (sector.empty()) {
        continue;
      }
      std::ranges::sort(sector, {}, objective_rank);
      const std::size_t quota = base_quota + (sector_index < extra_sectors ? 1U : 0U);
      const std::size_t selected_before_sector = selected.size();
      append_candidate(sector.front());
      if (quota > 1U && sector.size() > 1U) {
        const auto furthest =
            std::ranges::max_element(sector, {}, &FrontierPoolCandidate::path_length_m);
        append_candidate(*furthest);
      }
      for (const FrontierPoolCandidate& candidate : sector) {
        if (selected.size() - selected_before_sector >= quota) {
          break;
        }
        append_candidate(candidate);
      }
    }
    if (selected.size() < config.maximum_frontier_candidates) {
      std::ranges::sort(all_candidates, {}, objective_rank);
      for (const FrontierPoolCandidate& candidate : all_candidates) {
        append_candidate(candidate);
      }
    }
    std::ranges::sort(selected, {}, objective_rank);
    return selected;
  };

  const auto evaluate_continuation = [&](const Point2 terminal,
                                         const LatticeKey terminal_key,
                                         const LatticeRiskStage stage) {
    ContinuationEvaluation evaluation;
    std::priority_queue<ContinuationQueueEntry> open;
    const auto cell_id = [](const LatticeKey& key) {
      const std::uint64_t x = static_cast<std::uint32_t>(key.x);
      const std::uint64_t y = static_cast<std::uint32_t>(key.y);
      return (x << 32U) | y;
    };
    const std::uint64_t start_id = cell_id(terminal_key);
    std::unordered_map<std::uint64_t, ContinuationRecord> records;
    records.emplace(start_id,
                    ContinuationRecord{.key = terminal_key, .parent = std::nullopt});
    open.push(ContinuationQueueEntry{
        .key = terminal_key,
        .path_depth_m = 0.0,
        .endpoint_displacement_m = 0.0,
        .remaining_goal_distance_m = distance(terminal, result.planning_goal),
    });
    std::uint64_t best_id = start_id;
    double best_remaining_goal_distance_m = distance(terminal, result.planning_goal);
    double best_path_depth_m = 0.0;
    std::size_t expanded_states = 0U;
    while (!open.empty() &&
           expanded_states < config.frontier_validation_maximum_states &&
           evaluation.reachable_depth_m + 1.0e-9 <
               config.minimum_frontier_reachable_depth_m) {
      const ContinuationQueueEntry current = open.top();
      open.pop();
      ++expanded_states;
      bool roi_boundary_seen = false;
      const Point2 current_point =
          current.path_depth_m > 0.0 ? cellCenter(grid, current.key) : terminal;
      const std::vector<Successor> successors =
          collect_successors(current_point, current.key, stage, roi_boundary_seen,
                             nullptr, &evaluation.successor_profile);
      static_cast<void>(roi_boundary_seen);
      if (current.path_depth_m == 0.0) {
        evaluation.immediate_successors = successors.size();
      }
      const std::uint64_t current_id = cell_id(current.key);
      for (const Successor& successor : successors) {
        const std::uint64_t id = cell_id(successor.key);
        if (records.contains(id)) {
          continue;
        }
        records.emplace(id,
                        ContinuationRecord{.key = successor.key, .parent = current_id});
        const double path_depth_m = current.path_depth_m + successor.length_m;
        const double endpoint_displacement_m = distance(terminal, successor.endpoint);
        const double remaining_goal_distance_m =
            distance(successor.endpoint, result.planning_goal);
        if (std::tuple{endpoint_displacement_m, -remaining_goal_distance_m,
                       path_depth_m} > std::tuple{evaluation.reachable_depth_m,
                                                  -best_remaining_goal_distance_m,
                                                  best_path_depth_m}) {
          evaluation.reachable_depth_m = endpoint_displacement_m;
          best_remaining_goal_distance_m = remaining_goal_distance_m;
          best_path_depth_m = path_depth_m;
          best_id = id;
        }
        open.push(ContinuationQueueEntry{
            .key = successor.key,
            .path_depth_m = path_depth_m,
            .endpoint_displacement_m = endpoint_displacement_m,
            .remaining_goal_distance_m = remaining_goal_distance_m,
        });
      }
    }
    evaluation.reachable_states = records.size() - 1U;
    for (std::uint64_t id = best_id; id != start_id;) {
      const ContinuationRecord& record = records.at(id);
      evaluation.extension.push_back(cellCenter(grid, record.key));
      id = *record.parent;
    }
    std::ranges::reverse(evaluation.extension);
    return evaluation;
  };

  constexpr std::array stages{
      LatticeRiskStage::kPreferredOnly,
      LatticeRiskStage::kPlanningAllowed,
      LatticeRiskStage::kCriticalAllowed,
  };
  const std::size_t stage_budget =
      std::max<std::size_t>(1U, config.maximum_expansions / stages.size());
  const double stage_time_ms =
      config.maximum_search_time_ms / static_cast<double>(stages.size());
  std::optional<FrontierEvaluation> best_frontier;
  std::optional<FrontierEvaluation> best_fallback;
  const auto fallback_rank = [](const FrontierEvaluation& value) {
    return std::tuple{
        value.immediate_successors == 0U,
        -value.reachable_depth_m,
        -static_cast<double>(value.continuation_states),
        static_cast<int>(value.stage),
        value.selection_score,
        -value.endpoint_displacement_m,
    };
  };
  bool search_incomplete = false;
  bool frontier_validation_performed = false;
  LatticeSearchTermination incomplete_termination{
      LatticeSearchTermination::kOpenSetExhausted};

  for (std::size_t stage_index = 0U; stage_index < stages.size(); ++stage_index) {
    const LatticeRiskStage stage = stages.at(stage_index);
    StageOutcome& outcome = outcomes.at(stage_index);
    run_stage(outcome, stage, stage_budget, stage_time_ms);
    result.expansions += outcome.expansions;
    result.stale_queue_pops += outcome.stale_pops;
    result.open_peak = std::max(result.open_peak, outcome.open_peak);
    result.records_peak = std::max(result.records_peak, outcome.records_peak);
    result.successor_diagnostics.generated += outcome.successor_diagnostics.generated;
    result.successor_diagnostics.accepted += outcome.successor_diagnostics.accepted;
    result.successor_diagnostics.rejected_outside_roi +=
        outcome.successor_diagnostics.rejected_outside_roi;
    result.successor_diagnostics.rejected_outside_grid +=
        outcome.successor_diagnostics.rejected_outside_grid;
    result.successor_diagnostics.rejected_invalid_clearance +=
        outcome.successor_diagnostics.rejected_invalid_clearance;
    result.successor_diagnostics.rejected_raw_collision +=
        outcome.successor_diagnostics.rejected_raw_collision;
    result.successor_diagnostics.rejected_risk_stage +=
        outcome.successor_diagnostics.rejected_risk_stage;
    result.successor_diagnostics.rejected_blacklisted_failure +=
        outcome.successor_diagnostics.rejected_blacklisted_failure;
    result.successor_diagnostics.soft_tabu_penalties_applied +=
        outcome.successor_diagnostics.soft_tabu_penalties_applied;
    result.successor_diagnostics.rejected_no_cost_improvement +=
        outcome.successor_diagnostics.rejected_no_cost_improvement;
    if (outcome.goal_reached && outcome.terminal.has_value()) {
      result.guide = reconstruct(outcome, *outcome.terminal);
      if (result.guide.empty() ||
          distance(result.guide.back(), result.planning_goal) > 1.0e-6) {
        result.guide.push_back(result.planning_goal);
      }
      result.cost =
          outcome.records.at(*outcome.terminal).cost + outcome.terminal_connector_cost;
      result.status = LatticePlanStatus::kReachedPlanningGoal;
      result.termination = LatticeSearchTermination::kPlanningGoalReached;
      result.risk_stage = stage;
      result.planning_goal_reached = true;
      result.exact_terminal_connector = true;
      result.guide_length_m = detail::latticeGuideLength(result.guide);
      result.achieved_progress_m = distance(start, result.planning_goal);
      result.remaining_goal_distance_m = 0.0;
      result.valid = true;
      return result;
    }
    if (outcome.deadline_reached || outcome.budget_exhausted ||
        outcome.roi_boundary_seen) {
      search_incomplete = true;
      if (outcome.deadline_reached) {
        incomplete_termination = LatticeSearchTermination::kDeadlineReached;
      } else if (outcome.budget_exhausted) {
        incomplete_termination = LatticeSearchTermination::kExpansionBudgetExhausted;
      } else {
        incomplete_termination = LatticeSearchTermination::kRoiBoundaryReached;
      }
    }
    const bool validate_frontiers =
        !outcome.frontier_validation_started || outcome.open.empty() ||
        outcome.expansions - outcome.last_frontier_validation_expansions >=
            config.frontier_validation_expansion_interval;
    if (!validate_frontiers) {
      continue;
    }
    outcome.frontier_validation_started = true;
    outcome.last_frontier_validation_expansions = outcome.expansions;
    frontier_validation_performed = true;
    const std::vector<FrontierPoolCandidate> frontier_pool =
        collect_frontier_pool(outcome);
    result.frontier_candidates_considered += frontier_pool.size();
    std::vector<std::optional<FrontierEvaluation>> evaluated(frontier_pool.size());
    const auto continuation_started = std::chrono::steady_clock::now();
    const auto evaluate_candidate = [&](const std::size_t candidate_index) {
      const FrontierPoolCandidate& pooled = frontier_pool[candidate_index];
      const LatticeKey& key = pooled.key;
      const Point2 terminal = cellCenter(grid, key);
      std::vector<Point2> guide = reconstruct(outcome, key);
      if (detail::evaluateLatticeFailureMemory(guide, frontier_blacklist, config)
              .hard_rejected) {
        return;
      }
      const ContinuationEvaluation continuation =
          evaluate_continuation(terminal, key, stage);
      for (const Point2 point : continuation.extension) {
        if (guide.empty() || distance(guide.back(), point) > 1.0e-6) {
          guide.push_back(point);
        }
      }
      const double guide_length_m = detail::latticeGuideLength(guide);
      const Point2 endpoint = guide.empty() ? terminal : guide.back();
      const double remaining_m = distance(endpoint, result.planning_goal);
      const double progress_m = distance(start, result.planning_goal) - remaining_m;
      const double endpoint_displacement_m = distance(start, endpoint);
      const double extension_length_m = guide_length_m - pooled.path_length_m;
      FrontierEvaluation candidate{
          .key = key,
          .guide = std::move(guide),
          .guide_length_m = guide_length_m,
          .progress_m = progress_m,
          .remaining_m = remaining_m,
          .endpoint_displacement_m = endpoint_displacement_m,
          .selection_score = pooled.selection_score + extension_length_m,
          .cost = outcome.records.at(key).cost + extension_length_m,
          .immediate_successors = continuation.immediate_successors,
          .continuation_states = continuation.reachable_states,
          .reachable_depth_m = continuation.reachable_depth_m,
          .stage = stage,
          .successor_profile = continuation.successor_profile,
      };
      evaluated[candidate_index] = std::move(candidate);
    };
    if (worker_pool != nullptr && frontier_pool.size() > 1U) {
      worker_pool->parallelFor(frontier_pool.size(), evaluate_candidate);
    } else {
      for (std::size_t candidate_index = 0U; candidate_index < frontier_pool.size();
           ++candidate_index) {
        evaluate_candidate(candidate_index);
      }
    }
    result.continuation_validation_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  continuation_started)
            .count();
    for (std::optional<FrontierEvaluation>& evaluated_candidate : evaluated) {
      if (!evaluated_candidate.has_value()) {
        continue;
      }
      FrontierEvaluation candidate = std::move(*evaluated_candidate);
      accumulateSuccessorProfile(result.successor_profiling.continuation,
                                 candidate.successor_profile);
      const double guide_length_m = candidate.guide_length_m;
      const double endpoint_displacement_m = candidate.endpoint_displacement_m;
      if (candidate.guide.size() >= 2U && endpoint_displacement_m > 1.0e-9 &&
          (!best_fallback.has_value() ||
           fallback_rank(candidate) < fallback_rank(*best_fallback))) {
        best_fallback = candidate;
      }
      if (candidate.guide.size() < config.minimum_frontier_guide_points ||
          guide_length_m < config.minimum_frontier_guide_length_m ||
          endpoint_displacement_m + 1.0e-9 <
              config.minimum_frontier_endpoint_displacement_m ||
          candidate.immediate_successors == 0U || candidate.continuation_states == 0U ||
          candidate.reachable_depth_m + 1.0e-9 <
              config.minimum_frontier_reachable_depth_m) {
        continue;
      }
      const auto rank = [](const FrontierEvaluation& value) {
        return std::tuple{-value.reachable_depth_m,
                          -static_cast<double>(value.continuation_states),
                          static_cast<int>(value.stage),
                          value.selection_score,
                          -value.endpoint_displacement_m,
                          value.remaining_m};
      };
      if (!best_frontier.has_value() || rank(candidate) < rank(*best_frontier)) {
        best_frontier = std::move(candidate);
      }
    }
  }

  if (best_frontier.has_value()) {
    result.guide = std::move(best_frontier->guide);
    result.cost = best_frontier->cost;
    result.status = LatticePlanStatus::kViableFrontier;
    result.termination = search_incomplete
                             ? incomplete_termination
                             : LatticeSearchTermination::kOpenSetExhausted;
    result.risk_stage = best_frontier->stage;
    result.guide_length_m = best_frontier->guide_length_m;
    result.achieved_progress_m = best_frontier->progress_m;
    result.remaining_goal_distance_m = best_frontier->remaining_m;
    result.frontier_endpoint_displacement_m = best_frontier->endpoint_displacement_m;
    result.frontier_selection_score = best_frontier->selection_score;
    result.terminal_successor_count = best_frontier->immediate_successors;
    result.continuation_reachable_states = best_frontier->continuation_states;
    result.reachable_depth_m = best_frontier->reachable_depth_m;
    result.search_session_complete = !search_incomplete;
    result.valid = true;
    return result;
  }

  if (search_incomplete && frontier_validation_performed &&
      !best_fallback.has_value()) {
    bool roi_boundary_seen = false;
    const Point2 search_start = cellCenter(grid, start_key);
    const std::vector<Successor> immediate = collect_successors(
        search_start, start_key, LatticeRiskStage::kCriticalAllowed, roi_boundary_seen,
        nullptr, &result.successor_profiling.continuation);
    static_cast<void>(roi_boundary_seen);
    for (const Successor& successor : immediate) {
      const SegmentEvaluation from_actual_start =
          detail::evaluateLatticeSegment(grid, esdf_m, start, successor.endpoint,
                                         config, LatticeRiskStage::kCriticalAllowed);
      const std::array validation_guide{start, successor.endpoint};
      if (!from_actual_start.valid || detail::evaluateLatticeFailureMemory(
                                          validation_guide, frontier_blacklist, config)
                                          .hard_rejected) {
        continue;
      }
      const ContinuationEvaluation continuation = evaluate_continuation(
          successor.endpoint, successor.key, LatticeRiskStage::kCriticalAllowed);
      std::vector<Point2> fallback_guide{start, successor.endpoint};
      fallback_guide.insert(fallback_guide.end(), continuation.extension.begin(),
                            continuation.extension.end());
      const Point2 endpoint = fallback_guide.back();
      const double guide_length_m = detail::latticeGuideLength(fallback_guide);
      const double remaining_m = distance(endpoint, result.planning_goal);
      FrontierEvaluation candidate{
          .key = successor.key,
          .guide = std::move(fallback_guide),
          .guide_length_m = guide_length_m,
          .progress_m = distance(start, result.planning_goal) - remaining_m,
          .remaining_m = remaining_m,
          .endpoint_displacement_m = distance(start, endpoint),
          .selection_score =
              guide_length_m + config.frontier_goal_distance_weight * remaining_m,
          .cost = guide_length_m,
          .immediate_successors = continuation.immediate_successors,
          .continuation_states = continuation.reachable_states,
          .reachable_depth_m = continuation.reachable_depth_m,
          .stage = LatticeRiskStage::kCriticalAllowed,
      };
      if (!best_fallback.has_value() ||
          fallback_rank(candidate) < fallback_rank(*best_fallback)) {
        best_fallback = std::move(candidate);
      }
    }
  }

  if (search_incomplete && best_fallback.has_value()) {
    result.guide = std::move(best_fallback->guide);
    result.cost = best_fallback->cost;
    result.status = LatticePlanStatus::kRawSafeDetourPrefix;
    result.termination = incomplete_termination;
    result.risk_stage = best_fallback->stage;
    result.guide_length_m = best_fallback->guide_length_m;
    result.achieved_progress_m = best_fallback->progress_m;
    result.remaining_goal_distance_m = best_fallback->remaining_m;
    result.frontier_endpoint_displacement_m = best_fallback->endpoint_displacement_m;
    result.frontier_selection_score = best_fallback->selection_score;
    result.terminal_successor_count = best_fallback->immediate_successors;
    result.continuation_reachable_states = best_fallback->continuation_states;
    result.reachable_depth_m = best_fallback->reachable_depth_m;
    result.search_session_complete = false;
    result.valid = true;
    return result;
  }

  result.status = search_incomplete ? LatticePlanStatus::kSearchIncomplete
                                    : LatticePlanStatus::kMotionGraphExhausted;
  result.termination = search_incomplete ? incomplete_termination
                                         : LatticeSearchTermination::kOpenSetExhausted;
  result.search_session_complete = !search_incomplete;
  return result;
}

} // namespace drone_city_nav
