#include "risk_aware_lattice_geometry.hpp"

#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <numbers>

namespace drone_city_nav::detail {
namespace {

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

} // namespace

double latticeHeadingForBin(const int bin, const int bins) {
  return 2.0 * std::numbers::pi * static_cast<double>(bin) / static_cast<double>(bins);
}

int wrapLatticeHeading(const int heading, const int bins) {
  const int wrapped = heading % bins;
  return wrapped < 0 ? wrapped + bins : wrapped;
}

int nearestLatticeHeadingBin(const double heading_rad, const int bins) {
  return wrapLatticeHeading(
      static_cast<int>(std::lround(heading_rad * static_cast<double>(bins) /
                                   (2.0 * std::numbers::pi))),
      bins);
}

int latticeHeadingBinDistance(const int first, const int second,
                              const int bins) noexcept {
  const int direct = std::abs(first - second);
  return std::min(direct, bins - direct);
}

EsdfQueryResult queryLatticeEsdf(const mppi::EsdfGrid& grid,
                                 const std::span<const float> esdf_m,
                                 const Point2 point) {
  return queryConservativeEsdf(grid, esdf_m, static_cast<float>(point.x),
                               static_cast<float>(point.y));
}

SegmentEvaluation evaluateLatticeSegment(const mppi::EsdfGrid& grid,
                                         const std::span<const float> esdf_m,
                                         const Point2 start, const Point2 endpoint,
                                         const RiskAwareLatticeConfig& config,
                                         const LatticeRiskStage stage) {
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
    const EsdfQueryResult query = queryLatticeEsdf(grid, esdf_m, sample);
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

void recordLatticeSegmentRejection(const SegmentEvaluation& segment,
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

Point2 recedingLatticeGoal(const Point2 start, const Point2 mission_goal,
                           const RiskAwareLatticeConfig& config,
                           bool& reached_mission_goal) {
  const double distance_to_goal =
      std::hypot(mission_goal.x - start.x, mission_goal.y - start.y);
  reached_mission_goal = distance_to_goal <= config.receding_goal_distance_m;
  if (reached_mission_goal || distance_to_goal <= 1.0e-6) {
    return mission_goal;
  }
  const double ratio = config.receding_goal_distance_m / distance_to_goal;
  return Point2{start.x + (mission_goal.x - start.x) * ratio,
                start.y + (mission_goal.y - start.y) * ratio};
}

double latticeGuideLength(const std::span<const Point2> guide) noexcept {
  double length_m = 0.0;
  for (std::size_t index = 1U; index < guide.size(); ++index) {
    length_m += std::hypot(guide[index].x - guide[index - 1U].x,
                           guide[index].y - guide[index - 1U].y);
  }
  return length_m;
}

std::uint64_t latticeFailureMemoryFingerprint(
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

FailureMemoryEvaluation evaluateLatticeFailureMemory(
    const std::span<const Point2> guide,
    const std::span<const LatticeFrontierBlacklistEntry> blacklist,
    const RiskAwareLatticeConfig& config) {
  FailureMemoryEvaluation result;
  for (std::size_t index = 1U; index < guide.size(); ++index) {
    const Point2& from = guide[index - 1U];
    const Point2& to = guide[index];
    const double segment_heading = std::atan2(to.y - from.y, to.x - from.x);
    const int segment_heading_bin =
        nearestLatticeHeadingBin(segment_heading, config.heading_bins);
    for (const LatticeFrontierBlacklistEntry& entry : blacklist) {
      const int failed_heading_bin =
          nearestLatticeHeadingBin(entry.approach_heading_rad, config.heading_bins);
      if (latticeHeadingBinDistance(segment_heading_bin, failed_heading_bin,
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

} // namespace drone_city_nav::detail
