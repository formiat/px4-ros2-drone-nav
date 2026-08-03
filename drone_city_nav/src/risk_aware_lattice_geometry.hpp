#pragma once

#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/risk_aware_lattice.hpp"

#include <cstdint>
#include <span>

namespace drone_city_nav::detail {

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

struct FailureMemoryEvaluation {
  bool hard_rejected{false};
  double soft_penalty_cost{0.0};
};

[[nodiscard]] double latticeHeadingForBin(int bin, int bins);

[[nodiscard]] int wrapLatticeHeading(int heading, int bins);

[[nodiscard]] int nearestLatticeHeadingBin(double heading_rad, int bins);

[[nodiscard]] int latticeHeadingBinDistance(int first, int second, int bins) noexcept;

[[nodiscard]] EsdfQueryResult queryLatticeEsdf(const mppi::EsdfGrid& grid,
                                               std::span<const float> esdf_m,
                                               Point2 point);

[[nodiscard]] SegmentEvaluation
evaluateLatticeSegment(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                       Point2 start, Point2 endpoint,
                       const RiskAwareLatticeConfig& config, LatticeRiskStage stage);

void recordLatticeSegmentRejection(const SegmentEvaluation& segment,
                                   LatticeSuccessorDiagnostics& diagnostics) noexcept;

[[nodiscard]] Point2 recedingLatticeGoal(Point2 start, Point2 mission_goal,
                                         const RiskAwareLatticeConfig& config,
                                         bool& reached_mission_goal);

[[nodiscard]] double latticeGuideLength(std::span<const Point2> guide) noexcept;

[[nodiscard]] std::uint64_t latticeFailureMemoryFingerprint(
    std::span<const LatticeFrontierBlacklistEntry> entries) noexcept;

[[nodiscard]] FailureMemoryEvaluation
evaluateLatticeFailureMemory(std::span<const Point2> guide,
                             std::span<const LatticeFrontierBlacklistEntry> blacklist,
                             const RiskAwareLatticeConfig& config);

} // namespace drone_city_nav::detail
