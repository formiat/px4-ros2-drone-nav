#include "drone_city_nav/risk_aware_lattice.hpp"

#include <algorithm>
#include <array>
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
  std::optional<LatticeKey> parent;
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

[[nodiscard]] Point2 cellCenter(const mppi::EsdfGrid& grid, const LatticeKey& key) {
  return Point2{
      grid.origin_x_m + (static_cast<double>(key.x) + 0.5) * grid.resolution_m,
      grid.origin_y_m + (static_cast<double>(key.y) + 0.5) * grid.resolution_m};
}

[[nodiscard]] std::optional<float> clearanceAt(const mppi::EsdfGrid& grid,
                                               const std::span<const float> esdf_m,
                                               const Point2 point) {
  const int x =
      static_cast<int>(std::floor((point.x - grid.origin_x_m) / grid.resolution_m));
  const int y =
      static_cast<int>(std::floor((point.y - grid.origin_y_m) / grid.resolution_m));
  if (x < 0 || y < 0 || x >= grid.width || y >= grid.height) {
    return std::nullopt;
  }
  return esdf_m[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) +
                static_cast<std::size_t>(x)];
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

[[nodiscard]] bool
pointInsidePortalFootprint(const Point2 point,
                           const SemanticPortalPrimitive& portal) noexcept {
  const Point2 offset{point.x - portal.center.x, point.y - portal.center.y};
  const Point2 lateral_axis{-portal.normal_xy.y, portal.normal_xy.x};
  const double longitudinal =
      offset.x * portal.normal_xy.x + offset.y * portal.normal_xy.y;
  const double lateral = offset.x * lateral_axis.x + offset.y * lateral_axis.y;
  return std::abs(longitudinal) <= 0.5 * portal.depth_m &&
         std::abs(lateral) <= 0.5 * portal.width_m;
}

[[nodiscard]] bool pointInsideAnyPortalFootprint(
    const Point2 point,
    const std::span<const SemanticPortalPrimitive> portals) noexcept {
  return std::ranges::any_of(portals, [&](const SemanticPortalPrimitive& portal) {
    return pointInsidePortalFootprint(point, portal);
  });
}

struct SegmentEvaluation {
  enum class RejectionReason : std::uint8_t {
    kNone,
    kOutsideGrid,
    kInvalidClearance,
    kRawCollision,
    kPortalFootprint,
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
                const RiskAwareLatticeConfig& config,
                const std::span<const SemanticPortalPrimitive> portals,
                const bool allow_portal_footprint, const LatticeRiskStage stage) {
  SegmentEvaluation result{.valid = true};
  const double length_m = distance(start, endpoint);
  const int sample_count =
      static_cast<int>(std::ceil(length_m / config.primitive_sample_step_m));
  for (int sample_index = 1; sample_index <= sample_count; ++sample_index) {
    const double sample_distance = std::min(
        length_m, static_cast<double>(sample_index) * config.primitive_sample_step_m);
    const double ratio = length_m > 0.0 ? sample_distance / length_m : 1.0;
    const Point2 sample{std::lerp(start.x, endpoint.x, ratio),
                        std::lerp(start.y, endpoint.y, ratio)};
    const std::optional<float> clearance = clearanceAt(grid, esdf_m, sample);
    if (!clearance.has_value()) {
      result.valid = false;
      result.rejection_reason = SegmentEvaluation::RejectionReason::kOutsideGrid;
      return result;
    }
    if (!std::isfinite(*clearance)) {
      if (*clearance > 0.0F) {
        continue;
      }
      result.valid = false;
      result.rejection_reason = SegmentEvaluation::RejectionReason::kInvalidClearance;
      return result;
    }
    if (*clearance <= config.collision_radius_m) {
      result.valid = false;
      result.rejection_reason = SegmentEvaluation::RejectionReason::kRawCollision;
      return result;
    }
    if (!allow_portal_footprint && pointInsideAnyPortalFootprint(sample, portals)) {
      result.valid = false;
      result.rejection_reason = SegmentEvaluation::RejectionReason::kPortalFootprint;
      return result;
    }
    if (*clearance < config.critical_distance_m) {
      result.worst_tier = mppi::RiskTier::kCritical;
      result.critical_exposure_m += config.primitive_sample_step_m;
    } else if (*clearance < config.preferred_distance_m) {
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
    case SegmentEvaluation::RejectionReason::kPortalFootprint:
      ++diagnostics.rejected_portal_footprint;
      break;
    case SegmentEvaluation::RejectionReason::kRiskStage:
      ++diagnostics.rejected_risk_stage;
      break;
  }
}

struct PortalSuccessor {
  Point2 endpoint{};
  int heading_bin{0};
  double length_m{0.0};
};

[[nodiscard]] std::optional<PortalSuccessor>
portalSuccessor(const Point2 current, const int current_heading,
                const SemanticPortalPrimitive& portal, const int direction,
                const RiskAwareLatticeConfig& config) {
  const Point2 lateral_axis{-portal.normal_xy.y, portal.normal_xy.x};
  const Point2 offset{current.x - portal.center.x, current.y - portal.center.y};
  const double longitudinal =
      offset.x * portal.normal_xy.x + offset.y * portal.normal_xy.y;
  const double lateral = offset.x * lateral_axis.x + offset.y * lateral_axis.y;
  const double directed_longitudinal = static_cast<double>(direction) * longitudinal;
  const double half_depth_m = 0.5 * portal.depth_m;
  const double usable_half_width_m =
      0.5 * portal.width_m - config.portal_lateral_margin_m;
  if (!(usable_half_width_m > 0.0) || std::abs(lateral) > usable_half_width_m ||
      directed_longitudinal < -half_depth_m - config.portal_entry_capture_distance_m ||
      directed_longitudinal >= half_depth_m) {
    return std::nullopt;
  }
  const double heading =
      std::atan2(static_cast<double>(direction) * portal.normal_xy.y,
                 static_cast<double>(direction) * portal.normal_xy.x);
  const int heading_bin = nearestHeadingBin(heading, config.heading_bins);
  if (headingBinDistance(current_heading, heading_bin, config.heading_bins) >
      config.portal_maximum_heading_delta_bins) {
    return std::nullopt;
  }
  const double exit_longitudinal =
      static_cast<double>(direction) * (half_depth_m + config.portal_exit_extension_m);
  const Point2 endpoint{
      portal.center.x + exit_longitudinal * portal.normal_xy.x,
      portal.center.y + exit_longitudinal * portal.normal_xy.y,
  };
  return PortalSuccessor{
      .endpoint = endpoint,
      .heading_bin = heading_bin,
      .length_m = distance(current, endpoint),
  };
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

[[nodiscard]] bool repeatsBlacklistedFailure(
    const std::span<const Point2> guide,
    const std::span<const LatticeFrontierBlacklistEntry> blacklist,
    const RiskAwareLatticeConfig& config) {
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
          return true;
        }
      }
    }
  }
  return false;
}

} // namespace

RiskAwareLatticeResult planRiskAwareMotionPrimitiveGuide(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m, const Point2 start,
    const double preferred_heading_rad, const Point2 mission_goal,
    const RiskAwareLatticeConfig& config,
    const std::span<const SemanticPortalPrimitive> portals,
    const std::span<const LatticeFrontierBlacklistEntry> frontier_blacklist) {
  RiskAwareLatticeResult result;
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution_m <= 0.0F ||
      esdf_m.size() != static_cast<std::size_t>(grid.width) *
                           static_cast<std::size_t>(grid.height) ||
      config.heading_bins < 4 || !(config.short_primitive_length_m > 0.0) ||
      !(config.primitive_length_m >= config.short_primitive_length_m) ||
      !(config.portal_lateral_margin_m >= 0.0) ||
      !(config.portal_entry_capture_distance_m > 0.0) ||
      !(config.portal_exit_extension_m > 0.0) ||
      !(config.maximum_search_roi_halo_m > 0.0) ||
      !(config.maximum_search_time_ms > 0.0) ||
      config.maximum_frontier_candidates == 0U ||
      !(config.minimum_frontier_reachable_depth_m > 0.0) ||
      !(config.frontier_blacklist_radius_m >= 0.0) ||
      config.frontier_blacklist_heading_tolerance_bins < 0 ||
      config.portal_maximum_heading_delta_bins < 0 ||
      config.portal_maximum_heading_delta_bins > config.heading_bins / 2) {
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
  if (!clearanceAt(grid, esdf_m, start).has_value()) {
    return result;
  }
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

  struct Successor {
    LatticeKey key{};
    Point2 endpoint{};
    double length_m{0.0};
    double edge_cost{0.0};
  };

  struct FrontierEvaluation {
    LatticeKey key{};
    std::vector<Point2> guide;
    double guide_length_m{0.0};
    double progress_m{0.0};
    double remaining_m{0.0};
    double cost{0.0};
    std::size_t immediate_successors{0U};
    std::size_t two_step_states{0U};
    double reachable_depth_m{0.0};
    LatticeRiskStage stage{LatticeRiskStage::kPreferredOnly};
  };

  struct StageOutcome {
    bool goal_reached{false};
    bool budget_exhausted{false};
    bool deadline_reached{false};
    bool roi_boundary_seen{false};
    LatticeRiskStage stage{LatticeRiskStage::kPreferredOnly};
    LatticeSearchTermination termination{LatticeSearchTermination::kOpenSetExhausted};
    std::unordered_map<LatticeKey, Record, LatticeKeyHash> records;
    std::optional<LatticeKey> terminal;
    std::vector<LatticeKey> frontier_keys;
    double terminal_connector_cost{0.0};
    std::size_t expansions{0U};
    std::size_t stale_pops{0U};
    std::size_t open_peak{0U};
    std::size_t records_peak{0U};
    LatticeSuccessorDiagnostics successor_diagnostics{};
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
      const SegmentEvaluation segment = evaluateSegment(grid, esdf_m, current, endpoint,
                                                        config, portals, false, stage);
      if (!segment.valid) {
        if (diagnostics != nullptr) {
          recordSegmentRejection(segment, *diagnostics);
        }
        return;
      }
      const std::array candidate_segment{current, endpoint};
      if (repeatsBlacklistedFailure(candidate_segment, frontier_blacklist, config)) {
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
          .edge_cost = length_m + segment.risk_cost +
                       config.turn_cost * static_cast<double>(heading_change),
      });
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
    for (const SemanticPortalPrimitive& portal : portals) {
      for (const int direction : {-1, 1}) {
        const std::optional<PortalSuccessor> portal_successor =
            portalSuccessor(current, current_key.heading, portal, direction, config);
        if (!portal_successor.has_value()) {
          continue;
        }
        if (!inside_roi(portal_successor->endpoint)) {
          roi_boundary_seen = true;
          if (diagnostics != nullptr) {
            ++diagnostics->generated;
            ++diagnostics->rejected_outside_roi;
          }
          continue;
        }
        if (diagnostics != nullptr) {
          ++diagnostics->generated;
        }
        const SegmentEvaluation segment =
            evaluateSegment(grid, esdf_m, current, portal_successor->endpoint, config,
                            portals, true, stage);
        if (!segment.valid) {
          if (diagnostics != nullptr) {
            recordSegmentRejection(segment, *diagnostics);
          }
          continue;
        }
        const std::array candidate_segment{current, portal_successor->endpoint};
        if (repeatsBlacklistedFailure(candidate_segment, frontier_blacklist, config)) {
          if (diagnostics != nullptr) {
            ++diagnostics->rejected_blacklisted_failure;
          }
          continue;
        }
        const int heading_delta = headingBinDistance(
            current_key.heading, portal_successor->heading_bin, config.heading_bins);
        successors.push_back(Successor{
            .key = makeKey(portal_successor->endpoint, portal_successor->heading_bin),
            .endpoint = portal_successor->endpoint,
            .length_m = portal_successor->length_m,
            .edge_cost = portal_successor->length_m + segment.risk_cost +
                         config.turn_cost * static_cast<double>(heading_delta),
        });
        if (diagnostics != nullptr) {
          ++diagnostics->accepted;
        }
      }
    }
    return successors;
  };

  const auto run_stage = [&](const LatticeRiskStage stage,
                             const std::size_t expansion_budget,
                             const double stage_time_ms) {
    StageOutcome outcome;
    outcome.stage = stage;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open;
    outcome.records[start_key].cost = 0.0;
    std::uint64_t insertion_sequence = 0U;
    open.push(QueueEntry{.key = start_key,
                         .g_at_insert = 0.0,
                         .f_score = distance(start, result.planning_goal),
                         .insertion_sequence = insertion_sequence++});
    outcome.open_peak = 1U;
    outcome.records_peak = 1U;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double, std::milli>{stage_time_ms};
    std::vector<std::pair<double, LatticeKey>> frontier_pool;
    frontier_pool.reserve(config.maximum_frontier_candidates + 1U);
    while (!open.empty() && outcome.expansions < expansion_budget &&
           std::chrono::steady_clock::now() < deadline) {
      const QueueEntry entry = open.top();
      open.pop();
      const auto record = outcome.records.find(entry.key);
      if (record == outcome.records.end() ||
          entry.g_at_insert > record->second.cost + 1.0e-9) {
        ++outcome.stale_pops;
        continue;
      }
      const Point2 current = cellCenter(grid, entry.key);
      const double goal_distance = distance(current, result.planning_goal);
      const double progress = distance(start, result.planning_goal) - goal_distance;
      if (progress >= config.minimum_frontier_progress_m) {
        frontier_pool.emplace_back(goal_distance, entry.key);
        std::ranges::sort(frontier_pool, {}, &std::pair<double, LatticeKey>::first);
        if (frontier_pool.size() > config.maximum_frontier_candidates) {
          frontier_pool.pop_back();
        }
      }
      if (goal_distance <= config.goal_tolerance_m) {
        const SegmentEvaluation connector = evaluateSegment(
            grid, esdf_m, current, result.planning_goal, config, portals, false, stage);
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
        next_record.parent = entry.key;
        const double heuristic = distance(result.planning_goal, successor.endpoint);
        open.push(QueueEntry{
            .key = successor.key,
            .g_at_insert = next_cost,
            .f_score = next_cost + config.heuristic_weight * heuristic,
            .insertion_sequence = insertion_sequence++,
        });
      }
      outcome.open_peak = std::max(outcome.open_peak, open.size());
      outcome.records_peak = std::max(outcome.records_peak, outcome.records.size());
    }
    if (!outcome.goal_reached) {
      outcome.deadline_reached =
          !open.empty() && std::chrono::steady_clock::now() >= deadline;
      outcome.budget_exhausted =
          !open.empty() && outcome.expansions >= expansion_budget;
      if (outcome.deadline_reached) {
        outcome.termination = LatticeSearchTermination::kDeadlineReached;
      } else if (outcome.budget_exhausted) {
        outcome.termination = LatticeSearchTermination::kExpansionBudgetExhausted;
      } else {
        outcome.termination = LatticeSearchTermination::kOpenSetExhausted;
      }
    }
    outcome.frontier_keys.reserve(frontier_pool.size());
    for (const auto& [unused_distance, key] : frontier_pool) {
      static_cast<void>(unused_distance);
      outcome.frontier_keys.push_back(key);
    }
    return outcome;
  };

  const auto reconstruct = [&](const StageOutcome& outcome,
                               const LatticeKey& terminal) {
    std::vector<Point2> guide;
    for (std::optional<LatticeKey> key = terminal; key.has_value();) {
      guide.push_back(cellCenter(grid, *key));
      key = outcome.records.at(*key).parent;
    }
    std::ranges::reverse(guide);
    return guide;
  };

  const std::array stages{LatticeRiskStage::kPreferredOnly,
                          LatticeRiskStage::kPlanningAllowed,
                          LatticeRiskStage::kCriticalAllowed};
  const std::size_t stage_budget =
      std::max<std::size_t>(1U, config.maximum_expansions / stages.size());
  const double stage_time_ms =
      config.maximum_search_time_ms / static_cast<double>(stages.size());
  std::vector<StageOutcome> outcomes;
  outcomes.reserve(stages.size());
  std::optional<FrontierEvaluation> best_frontier;
  bool search_incomplete = false;
  LatticeSearchTermination incomplete_termination{
      LatticeSearchTermination::kOpenSetExhausted};

  for (const LatticeRiskStage stage : stages) {
    outcomes.push_back(run_stage(stage, stage_budget, stage_time_ms));
    const StageOutcome& outcome = outcomes.back();
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
    result.successor_diagnostics.rejected_portal_footprint +=
        outcome.successor_diagnostics.rejected_portal_footprint;
    result.successor_diagnostics.rejected_risk_stage +=
        outcome.successor_diagnostics.rejected_risk_stage;
    result.successor_diagnostics.rejected_blacklisted_failure +=
        outcome.successor_diagnostics.rejected_blacklisted_failure;
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
    for (const LatticeKey& key : outcome.frontier_keys) {
      ++result.frontier_candidates_considered;
      const Point2 terminal = cellCenter(grid, key);
      std::vector<Point2> guide = reconstruct(outcome, key);
      if (repeatsBlacklistedFailure(guide, frontier_blacklist, config)) {
        continue;
      }
      bool roi_boundary_seen = false;
      const std::vector<Successor> immediate =
          collect_successors(terminal, key, stage, roi_boundary_seen, nullptr);
      static_cast<void>(roi_boundary_seen);
      std::size_t two_step_states = 0U;
      double reachable_depth_m = 0.0;
      for (const Successor& successor : immediate) {
        bool second_roi_boundary_seen = false;
        const std::vector<Successor> second =
            collect_successors(successor.endpoint, successor.key, stage,
                               second_roi_boundary_seen, nullptr);
        static_cast<void>(second_roi_boundary_seen);
        two_step_states += second.size();
        if (!second.empty()) {
          const double deepest_second =
              std::ranges::max(second, {}, &Successor::length_m).length_m;
          reachable_depth_m =
              std::max(reachable_depth_m, successor.length_m + deepest_second);
        }
      }
      const double guide_length_m = guideLength(guide);
      const double remaining_m = distance(terminal, result.planning_goal);
      const double progress_m = distance(start, result.planning_goal) - remaining_m;
      if (guide.size() < config.minimum_frontier_guide_points ||
          guide_length_m < config.minimum_frontier_guide_length_m ||
          progress_m < config.minimum_frontier_progress_m || immediate.empty() ||
          two_step_states == 0U ||
          reachable_depth_m < config.minimum_frontier_reachable_depth_m) {
        continue;
      }
      FrontierEvaluation candidate{
          .key = key,
          .guide = std::move(guide),
          .guide_length_m = guide_length_m,
          .progress_m = progress_m,
          .remaining_m = remaining_m,
          .cost = outcome.records.at(key).cost,
          .immediate_successors = immediate.size(),
          .two_step_states = two_step_states,
          .reachable_depth_m = reachable_depth_m,
          .stage = stage,
      };
      const auto rank = [](const FrontierEvaluation& value) {
        return std::tuple{-value.reachable_depth_m,
                          -value.progress_m,
                          static_cast<int>(value.stage),
                          -static_cast<double>(value.two_step_states),
                          value.remaining_m,
                          value.cost};
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
    result.terminal_successor_count = best_frontier->immediate_successors;
    result.two_step_reachable_states = best_frontier->two_step_states;
    result.reachable_depth_m = best_frontier->reachable_depth_m;
    result.valid = true;
    return result;
  }

  result.status = search_incomplete ? LatticePlanStatus::kSearchIncomplete
                                    : LatticePlanStatus::kMotionGraphExhausted;
  result.termination = search_incomplete ? incomplete_termination
                                         : LatticeSearchTermination::kOpenSetExhausted;
  return result;
}

const char* latticePlanStatusName(const LatticePlanStatus status) noexcept {
  switch (status) {
    case LatticePlanStatus::kInvalidInput:
      return "invalid_input";
    case LatticePlanStatus::kReachedPlanningGoal:
      return "reached_planning_goal";
    case LatticePlanStatus::kViableFrontier:
      return "viable_frontier";
    case LatticePlanStatus::kSearchIncomplete:
      return "search_incomplete";
    case LatticePlanStatus::kMotionGraphExhausted:
      return "motion_graph_exhausted";
  }
  return "unknown";
}

const char* latticeRiskStageName(const LatticeRiskStage stage) noexcept {
  switch (stage) {
    case LatticeRiskStage::kPreferredOnly:
      return "preferred";
    case LatticeRiskStage::kPlanningAllowed:
      return "planning";
    case LatticeRiskStage::kCriticalAllowed:
      return "critical";
  }
  return "unknown";
}

const char*
latticeSearchTerminationName(const LatticeSearchTermination termination) noexcept {
  switch (termination) {
    case LatticeSearchTermination::kInvalidInput:
      return "invalid_input";
    case LatticeSearchTermination::kPlanningGoalReached:
      return "planning_goal_reached";
    case LatticeSearchTermination::kOpenSetExhausted:
      return "open_set_exhausted";
    case LatticeSearchTermination::kExpansionBudgetExhausted:
      return "expansion_budget_exhausted";
    case LatticeSearchTermination::kDeadlineReached:
      return "deadline_reached";
    case LatticeSearchTermination::kRoiBoundaryReached:
      return "roi_boundary_reached";
  }
  return "unknown";
}

} // namespace drone_city_nav
