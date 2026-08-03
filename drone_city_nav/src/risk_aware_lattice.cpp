#include "drone_city_nav/risk_aware_lattice.hpp"

#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <queue>
#include <ranges>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace drone_city_nav {
namespace {

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
};

struct ContinuationEvaluation {
  std::size_t immediate_successors{0U};
  std::size_t reachable_states{0U};
  double reachable_depth_m{0.0};
};

struct ContinuationQueueEntry {
  LatticeKey key{};
  double depth_m{0.0};

  bool operator<(const ContinuationQueueEntry& other) const noexcept {
    return depth_m < other.depth_m;
  }
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
  LatticeSuccessorDiagnostics successor_diagnostics{};
};

[[nodiscard]] Point2 cellCenter(const mppi::EsdfGrid& grid, const LatticeKey& key) {
  return Point2{
      grid.origin_x_m + (static_cast<double>(key.x) + 0.5) * grid.resolution_m,
      grid.origin_y_m + (static_cast<double>(key.y) + 0.5) * grid.resolution_m};
}

[[nodiscard]] EsdfQueryResult queryAt(const mppi::EsdfGrid& grid,
                                      const std::span<const float> esdf_m,
                                      const Point2 point) {
  return queryConservativeEsdf(grid, esdf_m, static_cast<float>(point.x),
                               static_cast<float>(point.y));
}

[[nodiscard]] double headingForBin(const int bin, const int bins) {
  return 2.0 * std::numbers::pi * static_cast<double>(bin) / static_cast<double>(bins);
}

[[nodiscard]] int wrapHeading(const int heading, const int bins) {
  const int wrapped = heading % bins;
  return wrapped < 0 ? wrapped + bins : wrapped;
}

[[nodiscard]] int nearestHeadingBin(const double heading_rad, const int bins) {
  return wrapHeading(
      static_cast<int>(std::lround(heading_rad * static_cast<double>(bins) /
                                   (2.0 * std::numbers::pi))),
      bins);
}

[[nodiscard]] int headingBinDistance(const int first, const int second,
                                     const int bins) noexcept {
  const int direct = std::abs(first - second);
  return std::min(direct, bins - direct);
}

struct SegmentEvaluation {
  enum class RejectionReason : std::uint8_t {
    kNone,
    kOutsideGrid,
    kInvalidClearance,
    kRawCollision,
    kRiskStage,
  };

  bool valid{false};
  RejectionReason rejection_reason{RejectionReason::kNone};
  double critical_exposure_m{0.0};
  double planning_exposure_m{0.0};
  mppi::RiskTier worst_tier{mppi::RiskTier::kPreferred};
  double risk_cost{0.0};
};

[[nodiscard]] bool riskAllowed(const mppi::RiskTier tier,
                               const LatticeRiskStage stage) noexcept {
  switch (stage) {
    case LatticeRiskStage::kPreferredOnly:
      return tier == mppi::RiskTier::kPreferred;
    case LatticeRiskStage::kPlanningAllowed:
      return tier <= mppi::RiskTier::kPlanning;
    case LatticeRiskStage::kCriticalAllowed:
      return tier <= mppi::RiskTier::kCritical;
  }
  return false;
}

[[nodiscard]] SegmentEvaluation
evaluateSegment(const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
                const Point2 start, const Point2 endpoint,
                const RiskAwareLatticeConfig& config, const LatticeRiskStage stage) {
  SegmentEvaluation result{.valid = true};
  const SweptFootprintResult footprint = validateSweptFootprint(
      grid, esdf_m, Point3{start.x, start.y, 0.0}, Point3{endpoint.x, endpoint.y, 0.0},
      SweptFootprintConfig{.radius_m = config.physical_footprint_radius_m,
                           .perimeter_samples = config.physical_footprint_samples,
                           .sweep_step_m = config.primitive_sample_step_m});
  if (!footprint.accepted()) {
    result.valid = false;
    switch (footprint.status) {
      case SweptFootprintStatus::kOutsideGrid:
        result.rejection_reason = SegmentEvaluation::RejectionReason::kOutsideGrid;
        break;
      case SweptFootprintStatus::kInvalidEsdf:
        result.rejection_reason = SegmentEvaluation::RejectionReason::kInvalidClearance;
        break;
      case SweptFootprintStatus::kRawCollision:
        result.rejection_reason = SegmentEvaluation::RejectionReason::kRawCollision;
        break;
      case SweptFootprintStatus::kValid:
        break;
    }
    return result;
  }
  const double length_m = distance(start, endpoint);
  const int sample_count =
      static_cast<int>(std::ceil(length_m / config.primitive_sample_step_m));
  for (int sample_index = 1; sample_index <= sample_count; ++sample_index) {
    const double sample_distance = std::min(
        length_m, static_cast<double>(sample_index) * config.primitive_sample_step_m);
    const double ratio = length_m > 0.0 ? sample_distance / length_m : 1.0;
    const Point2 sample{std::lerp(start.x, endpoint.x, ratio),
                        std::lerp(start.y, endpoint.y, ratio)};
    const EsdfQueryResult query = queryAt(grid, esdf_m, sample);
    if (query.status == EsdfQueryStatus::kOutsideGrid) {
      result.valid = false;
      result.rejection_reason = SegmentEvaluation::RejectionReason::kOutsideGrid;
      return result;
    }
    if (query.status != EsdfQueryStatus::kValid) {
      result.valid = false;
      result.rejection_reason = SegmentEvaluation::RejectionReason::kInvalidClearance;
      return result;
    }
    if (query.raw_occupied) {
      result.valid = false;
      result.rejection_reason = SegmentEvaluation::RejectionReason::kRawCollision;
      return result;
    }
    const float clearance = query.clearance_m;
    if (std::isinf(clearance) && clearance > 0.0F) {
      continue;
    }
    if (clearance < config.critical_distance_m) {
      result.worst_tier = mppi::RiskTier::kCritical;
      result.critical_exposure_m += config.primitive_sample_step_m;
    } else if (clearance < config.preferred_distance_m) {
      result.worst_tier = std::max(result.worst_tier, mppi::RiskTier::kPlanning);
      result.planning_exposure_m += config.primitive_sample_step_m;
    }
  }
  if (!riskAllowed(result.worst_tier, stage)) {
    result.valid = false;
    result.rejection_reason = SegmentEvaluation::RejectionReason::kRiskStage;
    return result;
  }
  result.risk_cost =
      result.critical_exposure_m * config.critical_exposure_tie_break_per_m +
      result.planning_exposure_m * config.planning_exposure_tie_break_per_m;
  return result;
}

void recordSegmentRejection(const SegmentEvaluation& segment,
                            LatticeSuccessorDiagnostics& diagnostics) noexcept {
  switch (segment.rejection_reason) {
    case SegmentEvaluation::RejectionReason::kNone:
      break;
    case SegmentEvaluation::RejectionReason::kOutsideGrid:
      ++diagnostics.rejected_outside_grid;
      break;
    case SegmentEvaluation::RejectionReason::kInvalidClearance:
      ++diagnostics.rejected_invalid_clearance;
      break;
    case SegmentEvaluation::RejectionReason::kRawCollision:
      ++diagnostics.rejected_raw_collision;
      break;
    case SegmentEvaluation::RejectionReason::kRiskStage:
      ++diagnostics.rejected_risk_stage;
      break;
  }
}

[[nodiscard]] Point2 recedingGoal(const Point2 start, const Point2 mission_goal,
                                  const RiskAwareLatticeConfig& config,
                                  bool& reached_mission_goal) {
  const double distance =
      std::hypot(mission_goal.x - start.x, mission_goal.y - start.y);
  reached_mission_goal = distance <= config.receding_goal_distance_m;
  if (reached_mission_goal || distance <= 1.0e-6) {
    return mission_goal;
  }
  const double ratio = config.receding_goal_distance_m / distance;
  return Point2{start.x + (mission_goal.x - start.x) * ratio,
                start.y + (mission_goal.y - start.y) * ratio};
}

[[nodiscard]] double guideLength(const std::span<const Point2> guide) noexcept {
  double length_m = 0.0;
  for (std::size_t index = 1U; index < guide.size(); ++index) {
    length_m += std::hypot(guide[index].x - guide[index - 1U].x,
                           guide[index].y - guide[index - 1U].y);
  }
  return length_m;
}

struct FailureMemoryEvaluation {
  bool hard_rejected{false};
  double soft_penalty_cost{0.0};
};

[[nodiscard]] std::uint64_t failureMemoryFingerprint(
    const std::span<const LatticeFrontierBlacklistEntry> entries) noexcept {
  std::uint64_t hash = 14695981039346656037ULL;
  const auto append = [&hash](const std::uint64_t value) {
    hash ^= value;
    hash *= 1099511628211ULL;
  };
  for (const LatticeFrontierBlacklistEntry& entry : entries) {
    append(std::bit_cast<std::uint64_t>(entry.failure_point.x));
    append(std::bit_cast<std::uint64_t>(entry.failure_point.y));
    append(std::bit_cast<std::uint64_t>(entry.approach_heading_rad));
    append(static_cast<std::uint64_t>(entry.expires_at_ns));
    append(std::bit_cast<std::uint64_t>(entry.soft_penalty_cost));
  }
  return hash;
}

[[nodiscard]] FailureMemoryEvaluation
evaluateFailureMemory(const std::span<const Point2> guide,
                      const std::span<const LatticeFrontierBlacklistEntry> blacklist,
                      const RiskAwareLatticeConfig& config) {
  FailureMemoryEvaluation result;
  for (std::size_t index = 1U; index < guide.size(); ++index) {
    const Point2& from = guide[index - 1U];
    const Point2& to = guide[index];
    const double segment_heading = std::atan2(to.y - from.y, to.x - from.x);
    const int segment_heading_bin =
        nearestHeadingBin(segment_heading, config.heading_bins);
    for (const LatticeFrontierBlacklistEntry& entry : blacklist) {
      const int failed_heading_bin =
          nearestHeadingBin(entry.approach_heading_rad, config.heading_bins);
      if (headingBinDistance(segment_heading_bin, failed_heading_bin,
                             config.heading_bins) >
          config.frontier_blacklist_heading_tolerance_bins) {
        continue;
      }
      const double segment_length = distance(from, to);
      const int sample_count = std::max(
          1,
          static_cast<int>(std::ceil(segment_length / config.primitive_sample_step_m)));
      for (int sample = 1; sample <= sample_count; ++sample) {
        const double ratio =
            static_cast<double>(sample) / static_cast<double>(sample_count);
        const Point2 point{std::lerp(from.x, to.x, ratio),
                           std::lerp(from.y, to.y, ratio)};
        if (distance(point, entry.failure_point) <=
            config.frontier_blacklist_radius_m) {
          if (entry.soft_penalty_cost > 0.0) {
            result.soft_penalty_cost =
                std::max(result.soft_penalty_cost, entry.soft_penalty_cost);
          } else {
            result.hard_rejected = true;
          }
          break;
        }
      }
    }
  }
  return result;
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
           failure_memory_fingerprint == failureMemoryFingerprint(candidate_blacklist);
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
    failure_memory_fingerprint = failureMemoryFingerprint(candidate_blacklist);
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
    RiskAwareLatticeSearchSession* session) {
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
      !(config.frontier_goal_distance_weight >= 0.0) ||
      !std::isfinite(config.frontier_goal_distance_weight) ||
      !(config.frontier_blacklist_radius_m >= 0.0) ||
      config.frontier_blacklist_heading_tolerance_bins < 0 ||
      config.frontier_blacklist_heading_tolerance_bins > config.heading_bins / 2) {
    return result;
  }
  result.planning_goal =
      recedingGoal(start, mission_goal, config, result.reached_mission_goal);
  const auto makeKey = [&grid, &config](const Point2 point, const int heading) {
    return LatticeKey{
        static_cast<int>(std::floor((point.x - grid.origin_x_m) / grid.resolution_m)),
        static_cast<int>(std::floor((point.y - grid.origin_y_m) / grid.resolution_m)),
        wrapHeading(heading, config.heading_bins)};
  };
  const LatticeKey start_key =
      makeKey(start, nearestHeadingBin(preferred_heading_rad, config.heading_bins));
  if (queryAt(grid, esdf_m, start).status == EsdfQueryStatus::kOutsideGrid) {
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
                                      LatticeSuccessorDiagnostics* diagnostics) {
    std::vector<Successor> successors;
    successors.reserve(static_cast<std::size_t>(config.heading_bins) + 8U);
    std::unordered_set<LatticeKey, LatticeKeyHash> emitted;
    const auto add_motion_successor = [&](const int next_heading,
                                          const double length_m) {
      if (diagnostics != nullptr) {
        ++diagnostics->generated;
      }
      const double heading = headingForBin(next_heading, config.heading_bins);
      const Point2 endpoint{current.x + std::cos(heading) * length_m,
                            current.y + std::sin(heading) * length_m};
      if (!inside_roi(endpoint)) {
        roi_boundary_seen = true;
        if (diagnostics != nullptr) {
          ++diagnostics->rejected_outside_roi;
        }
        return;
      }
      const SegmentEvaluation segment =
          evaluateSegment(grid, esdf_m, current, endpoint, config, stage);
      if (!segment.valid) {
        if (diagnostics != nullptr) {
          recordSegmentRejection(segment, *diagnostics);
        }
        return;
      }
      const std::array candidate_segment{current, endpoint};
      const FailureMemoryEvaluation failure_memory =
          evaluateFailureMemory(candidate_segment, frontier_blacklist, config);
      if (failure_memory.hard_rejected) {
        if (diagnostics != nullptr) {
          ++diagnostics->rejected_blacklisted_failure;
        }
        return;
      }
      const LatticeKey key = makeKey(endpoint, next_heading);
      if (!emitted.insert(key).second) {
        return;
      }
      const int heading_change =
          headingBinDistance(current_key.heading, next_heading, config.heading_bins);
      successors.push_back(Successor{
          .key = key,
          .endpoint = endpoint,
          .length_m = length_m,
          .edge_cost = length_m + segment.risk_cost + failure_memory.soft_penalty_cost +
                       config.turn_cost * static_cast<double>(heading_change),
      });
      if (diagnostics != nullptr && failure_memory.soft_penalty_cost > 0.0) {
        ++diagnostics->soft_tabu_penalties_applied;
      }
      if (diagnostics != nullptr) {
        ++diagnostics->accepted;
      }
    };

    for (int next_heading = 0; next_heading < config.heading_bins; ++next_heading) {
      add_motion_successor(next_heading, config.short_primitive_length_m);
    }
    constexpr std::array normal_heading_offsets{-4, -2, -1, 0, 1, 2, 4, 8};
    for (const int heading_offset : normal_heading_offsets) {
      add_motion_successor(
          wrapHeading(current_key.heading + heading_offset, config.heading_bins),
          config.primitive_length_m);
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
        const SegmentEvaluation connector =
            evaluateSegment(grid, esdf_m, current, result.planning_goal, config, stage);
        if (connector.valid) {
          outcome.goal_reached = true;
          outcome.terminal = entry.key;
          outcome.terminal_connector_cost = goal_distance + connector.risk_cost;
          outcome.termination = LatticeSearchTermination::kPlanningGoalReached;
          break;
        }
      }
      ++outcome.expansions;
      const std::vector<Successor> successors =
          collect_successors(current, entry.key, stage, outcome.roi_boundary_seen,
                             &outcome.successor_diagnostics);
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
          nearestHeadingBin(bearing, static_cast<int>(sector_count)));
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
    std::unordered_set<std::uint64_t> visited_cells;
    const auto cell_id = [](const LatticeKey& key) {
      const std::uint64_t x = static_cast<std::uint32_t>(key.x);
      const std::uint64_t y = static_cast<std::uint32_t>(key.y);
      return (x << 32U) | y;
    };
    visited_cells.insert(cell_id(terminal_key));
    open.push(ContinuationQueueEntry{terminal_key, 0.0});
    std::size_t expanded_states = 0U;
    while (!open.empty() &&
           expanded_states < config.frontier_validation_maximum_states) {
      const ContinuationQueueEntry current = open.top();
      open.pop();
      if (current.depth_m + 1.0e-9 >= config.minimum_frontier_reachable_depth_m) {
        break;
      }
      ++expanded_states;
      bool roi_boundary_seen = false;
      const Point2 current_point =
          current.depth_m > 0.0 ? cellCenter(grid, current.key) : terminal;
      const std::vector<Successor> successors = collect_successors(
          current_point, current.key, stage, roi_boundary_seen, nullptr);
      static_cast<void>(roi_boundary_seen);
      if (current.depth_m == 0.0) {
        evaluation.immediate_successors = successors.size();
      }
      for (const Successor& successor : successors) {
        const std::uint64_t id = cell_id(successor.key);
        if (!visited_cells.insert(id).second) {
          continue;
        }
        const double reachable_depth_m = current.depth_m + successor.length_m;
        evaluation.reachable_depth_m =
            std::max(evaluation.reachable_depth_m, reachable_depth_m);
        open.push(ContinuationQueueEntry{successor.key, reachable_depth_m});
      }
    }
    evaluation.reachable_states = visited_cells.size() - 1U;
    return evaluation;
  };

  const std::array stage_runs{
      std::pair{LatticeRiskStage::kPreferredOnly, &outcomes.at(0U)},
      std::pair{LatticeRiskStage::kPlanningAllowed, &outcomes.at(1U)},
      std::pair{LatticeRiskStage::kCriticalAllowed, &outcomes.at(2U)},
  };
  const std::size_t stage_budget =
      std::max<std::size_t>(1U, config.maximum_expansions / stage_runs.size());
  const double stage_time_ms =
      config.maximum_search_time_ms / static_cast<double>(stage_runs.size());
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
  LatticeSearchTermination incomplete_termination{
      LatticeSearchTermination::kOpenSetExhausted};

  for (const auto& [stage, outcome_pointer] : stage_runs) {
    StageOutcome& outcome = *outcome_pointer;
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
      result.guide_length_m = guideLength(result.guide);
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
    for (const FrontierPoolCandidate& pooled : collect_frontier_pool(outcome)) {
      ++result.frontier_candidates_considered;
      const LatticeKey& key = pooled.key;
      const Point2 terminal = cellCenter(grid, key);
      std::vector<Point2> guide = reconstruct(outcome, key);
      if (evaluateFailureMemory(guide, frontier_blacklist, config).hard_rejected) {
        continue;
      }
      const ContinuationEvaluation continuation =
          evaluate_continuation(terminal, key, stage);
      const double guide_length_m = guideLength(guide);
      const double remaining_m = distance(terminal, result.planning_goal);
      const double progress_m = distance(start, result.planning_goal) - remaining_m;
      const double endpoint_displacement_m = distance(start, terminal);
      FrontierEvaluation candidate{
          .key = key,
          .guide = std::move(guide),
          .guide_length_m = guide_length_m,
          .progress_m = progress_m,
          .remaining_m = remaining_m,
          .endpoint_displacement_m = endpoint_displacement_m,
          .selection_score = pooled.selection_score,
          .cost = outcome.records.at(key).cost,
          .immediate_successors = continuation.immediate_successors,
          .continuation_states = continuation.reachable_states,
          .reachable_depth_m = continuation.reachable_depth_m,
          .stage = stage,
      };
      if (candidate.guide.size() >= 2U && endpoint_displacement_m > 1.0e-9 &&
          (!best_fallback.has_value() ||
           fallback_rank(candidate) < fallback_rank(*best_fallback))) {
        best_fallback = candidate;
      }
      if (candidate.guide.size() < config.minimum_frontier_guide_points ||
          guide_length_m < config.minimum_frontier_guide_length_m ||
          endpoint_displacement_m + 1.0e-9 <
              config.minimum_frontier_endpoint_displacement_m ||
          continuation.immediate_successors == 0U ||
          continuation.reachable_states == 0U ||
          continuation.reachable_depth_m + 1.0e-9 <
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

  if (search_incomplete && !best_fallback.has_value()) {
    bool roi_boundary_seen = false;
    const Point2 search_start = cellCenter(grid, start_key);
    const std::vector<Successor> immediate =
        collect_successors(search_start, start_key, LatticeRiskStage::kCriticalAllowed,
                           roi_boundary_seen, nullptr);
    static_cast<void>(roi_boundary_seen);
    for (const Successor& successor : immediate) {
      const SegmentEvaluation from_actual_start =
          evaluateSegment(grid, esdf_m, start, successor.endpoint, config,
                          LatticeRiskStage::kCriticalAllowed);
      const std::array fallback_guide{start, successor.endpoint};
      if (!from_actual_start.valid ||
          evaluateFailureMemory(fallback_guide, frontier_blacklist, config)
              .hard_rejected) {
        continue;
      }
      const ContinuationEvaluation continuation = evaluate_continuation(
          successor.endpoint, successor.key, LatticeRiskStage::kCriticalAllowed);
      const double remaining_m = distance(successor.endpoint, result.planning_goal);
      FrontierEvaluation candidate{
          .key = successor.key,
          .guide = {start, successor.endpoint},
          .guide_length_m = distance(start, successor.endpoint),
          .progress_m = distance(start, result.planning_goal) - remaining_m,
          .remaining_m = remaining_m,
          .endpoint_displacement_m = distance(start, successor.endpoint),
          .selection_score =
              successor.edge_cost + config.frontier_goal_distance_weight * remaining_m,
          .cost = successor.edge_cost,
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
