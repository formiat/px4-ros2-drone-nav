#include "risk_aware_lattice_3d_geometry.hpp"

#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav::detail {
namespace {

[[nodiscard]] bool stageAllows(const Lattice3DRiskStage stage, const double clearance_m,
                               const RiskAwareLattice3DConfig& config) noexcept {
  switch (stage) {
    case Lattice3DRiskStage::kPreferredOnly:
      return clearance_m >= config.preferred_distance_m;
    case Lattice3DRiskStage::kPlanningAllowed:
      return clearance_m >= config.critical_distance_m;
    case Lattice3DRiskStage::kCriticalAllowed:
      return true;
  }
  return false;
}

} // namespace

Lattice3DEdgeEvaluation evaluateLattice3DEdge(const mppi::EsdfGrid& grid,
                                              const std::span<const float> esdf_m,
                                              const Point3& first, const Point3& second,
                                              const Lattice3DRiskStage stage,
                                              const RiskAwareLattice3DConfig& config) {
  const double length = distance3D(first, second);
  if (!(length > 1.0e-9)) {
    return Lattice3DEdgeEvaluation{.status = Lattice3DEdgeEvaluationStatus::kValid};
  }
  const SweptFootprintResult footprint = validateSweptFootprint(
      grid, esdf_m, first, second,
      SweptFootprintConfig{.radius_m = config.physical_footprint_radius_m,
                           .perimeter_samples = config.physical_footprint_samples,
                           .sweep_step_m = config.sample_step_m});
  if (!footprint.accepted()) {
    switch (footprint.status) {
      case SweptFootprintStatus::kOutsideGrid:
        return Lattice3DEdgeEvaluation{.status =
                                           Lattice3DEdgeEvaluationStatus::kOutsideGrid};
      case SweptFootprintStatus::kInvalidEsdf:
        return Lattice3DEdgeEvaluation{.status =
                                           Lattice3DEdgeEvaluationStatus::kInvalidEsdf};
      case SweptFootprintStatus::kRawCollision:
        return Lattice3DEdgeEvaluation{
            .status = Lattice3DEdgeEvaluationStatus::kRawCollision};
      case SweptFootprintStatus::kValid:
        break;
    }
  }
  const std::size_t samples = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(length / config.sample_step_m)));
  Lattice3DEdgeEvaluation result{.status = Lattice3DEdgeEvaluationStatus::kValid};
  const double exposure_per_sample = length / static_cast<double>(samples);
  for (std::size_t sample = 1U; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
    const Point3 point{std::lerp(first.x, second.x, ratio),
                       std::lerp(first.y, second.y, ratio),
                       std::lerp(first.z, second.z, ratio)};
    const EsdfQueryResult query = queryConservativeEsdf3D(
        grid, esdf_m, static_cast<float>(point.x), static_cast<float>(point.y),
        static_cast<float>(point.z));
    if (query.status == EsdfQueryStatus::kOutsideGrid) {
      return Lattice3DEdgeEvaluation{.status =
                                         Lattice3DEdgeEvaluationStatus::kOutsideGrid};
    }
    if (query.status != EsdfQueryStatus::kValid) {
      return Lattice3DEdgeEvaluation{.status =
                                         Lattice3DEdgeEvaluationStatus::kInvalidEsdf};
    }
    if (query.raw_occupied) {
      return Lattice3DEdgeEvaluation{.status =
                                         Lattice3DEdgeEvaluationStatus::kRawCollision};
    }
    if (!stageAllows(stage, query.clearance_m, config)) {
      return Lattice3DEdgeEvaluation{
          .status = Lattice3DEdgeEvaluationStatus::kRiskStageRejected};
    }
    result.minimum_clearance_m =
        std::min(result.minimum_clearance_m, static_cast<double>(query.clearance_m));
    if (query.clearance_m < config.critical_distance_m) {
      result.critical_exposure_m += exposure_per_sample;
    } else if (query.clearance_m < config.preferred_distance_m) {
      result.planning_exposure_m += exposure_per_sample;
    }
  }
  return result;
}

void accumulateLattice3DSuccessorProfile(
    Lattice3DSuccessorBatchProfile& target,
    const Lattice3DSuccessorBatchProfile& addition) noexcept {
  target.collection_calls += addition.collection_calls;
  target.parallel_collection_calls += addition.parallel_collection_calls;
  target.candidates += addition.candidates;
  target.parallel_candidates += addition.parallel_candidates;
  target.maximum_candidates =
      std::max(target.maximum_candidates, addition.maximum_candidates);
  target.worker_ms += addition.worker_ms;
}

void accumulateLattice3DSuccessorDiagnostics(
    Lattice3DSuccessorDiagnostics& target,
    const Lattice3DSuccessorDiagnostics& addition) noexcept {
  target.lattice_generated += addition.lattice_generated;
  target.lattice_accepted += addition.lattice_accepted;
  target.lattice_rejected_edge += addition.lattice_rejected_edge;
  target.lattice_rejected_zero_length += addition.lattice_rejected_zero_length;
  target.lattice_rejected_outside_grid += addition.lattice_rejected_outside_grid;
  target.lattice_rejected_invalid_esdf += addition.lattice_rejected_invalid_esdf;
  target.lattice_rejected_raw_collision += addition.lattice_rejected_raw_collision;
  target.lattice_rejected_risk_stage += addition.lattice_rejected_risk_stage;
  target.lattice_rejected_no_cost_improvement +=
      addition.lattice_rejected_no_cost_improvement;
  target.channel_generated += addition.channel_generated;
  target.channel_accepted += addition.channel_accepted;
  target.channel_rejected += addition.channel_rejected;
  target.channel_rejected_connection_distance +=
      addition.channel_rejected_connection_distance;
  target.channel_rejected_outside_grid += addition.channel_rejected_outside_grid;
  target.channel_rejected_invalid_esdf += addition.channel_rejected_invalid_esdf;
  target.channel_rejected_raw_collision += addition.channel_rejected_raw_collision;
  target.channel_rejected_risk_stage += addition.channel_rejected_risk_stage;
  target.channel_rejected_no_cost_improvement +=
      addition.channel_rejected_no_cost_improvement;
}

void recordLattice3DRejectedEdge(Lattice3DSuccessorDiagnostics& diagnostics,
                                 const Lattice3DEdgeEvaluationStatus status,
                                 const bool channel) noexcept {
  std::size_t* counter = nullptr;
  switch (status) {
    case Lattice3DEdgeEvaluationStatus::kValid:
      return;
    case Lattice3DEdgeEvaluationStatus::kOutsideGrid:
      counter = channel ? &diagnostics.channel_rejected_outside_grid
                        : &diagnostics.lattice_rejected_outside_grid;
      break;
    case Lattice3DEdgeEvaluationStatus::kInvalidEsdf:
      counter = channel ? &diagnostics.channel_rejected_invalid_esdf
                        : &diagnostics.lattice_rejected_invalid_esdf;
      break;
    case Lattice3DEdgeEvaluationStatus::kRawCollision:
      counter = channel ? &diagnostics.channel_rejected_raw_collision
                        : &diagnostics.lattice_rejected_raw_collision;
      break;
    case Lattice3DEdgeEvaluationStatus::kRiskStageRejected:
      counter = channel ? &diagnostics.channel_rejected_risk_stage
                        : &diagnostics.lattice_rejected_risk_stage;
      break;
  }
  ++(*counter);
}

} // namespace drone_city_nav::detail
