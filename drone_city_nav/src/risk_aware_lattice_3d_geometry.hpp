#pragma once

#include "drone_city_nav/risk_aware_lattice_3d.hpp"

#include <cstdint>
#include <limits>
#include <span>

namespace drone_city_nav::detail {

enum class Lattice3DEdgeEvaluationStatus : std::uint8_t {
  kValid,
  kOutsideFlightEnvelope,
  kOutsideGrid,
  kInvalidEsdf,
  kRawCollision,
  kRiskStageRejected,
};

struct Lattice3DEdgeEvaluation {
  Lattice3DEdgeEvaluationStatus status{Lattice3DEdgeEvaluationStatus::kInvalidEsdf};
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};
  double planning_exposure_m{0.0};
  double critical_exposure_m{0.0};
};

[[nodiscard]] Lattice3DEdgeEvaluation
evaluateLattice3DEdge(const mppi::EsdfGrid& grid, std::span<const float> esdf_m,
                      const Point3& first, const Point3& second,
                      Lattice3DRiskStage stage, const RiskAwareLattice3DConfig& config);

void accumulateLattice3DSuccessorProfile(
    Lattice3DSuccessorBatchProfile& target,
    const Lattice3DSuccessorBatchProfile& addition) noexcept;

void accumulateLattice3DSuccessorDiagnostics(
    Lattice3DSuccessorDiagnostics& target,
    const Lattice3DSuccessorDiagnostics& addition) noexcept;

void recordLattice3DRejectedEdge(Lattice3DSuccessorDiagnostics& diagnostics,
                                 Lattice3DEdgeEvaluationStatus status,
                                 bool channel) noexcept;

} // namespace drone_city_nav::detail
