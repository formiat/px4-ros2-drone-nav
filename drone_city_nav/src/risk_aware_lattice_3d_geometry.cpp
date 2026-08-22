#include "risk_aware_lattice_3d_geometry.hpp"

#include "drone_city_nav/esdf_query.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

[[nodiscard]] bool
footprintSegmentInsideGrid(const mppi::EsdfGrid& grid, const Point3& first,
                           const Point3& second,
                           const RiskAwareLattice3DConfig& config) noexcept {
  if (grid.width <= 0 || grid.height <= 0 || grid.depth <= 0 ||
      !(grid.resolution_m > 0.0F)) {
    return false;
  }
  const double minimum_x = static_cast<double>(grid.origin_x_m);
  const double minimum_y = static_cast<double>(grid.origin_y_m);
  const double minimum_z = static_cast<double>(grid.origin_z_m);
  const double maximum_x =
      minimum_x + static_cast<double>(grid.width) * grid.resolution_m;
  const double maximum_y =
      minimum_y + static_cast<double>(grid.height) * grid.resolution_m;
  const double maximum_z =
      minimum_z + static_cast<double>(grid.depth) * grid.resolution_m;
  return std::min(first.x, second.x) - config.physical_footprint_radius_m >=
             minimum_x &&
         std::max(first.x, second.x) + config.physical_footprint_radius_m <=
             maximum_x &&
         std::min(first.y, second.y) - config.physical_footprint_radius_m >=
             minimum_y &&
         std::max(first.y, second.y) + config.physical_footprint_radius_m <=
             maximum_y &&
         std::min(first.z, second.z) - config.physical_footprint_lower_extent_m >=
             minimum_z &&
         std::max(first.z, second.z) + config.physical_footprint_upper_extent_m <=
             maximum_z;
}

} // namespace

Lattice3DEdgeEvaluation evaluateLattice3DEdge(const mppi::EsdfGrid& grid,
                                              const std::span<const float> esdf_m,
                                              const Point3& first, const Point3& second,
                                              const Lattice3DRiskStage stage,
                                              const RiskAwareLattice3DConfig& config) {
  const double length = distance3D(first, second);
  if (!segmentInsideFlightEnvelope(first, second, config.flight_envelope)) {
    return Lattice3DEdgeEvaluation{
        .status = Lattice3DEdgeEvaluationStatus::kOutsideFlightEnvelope};
  }
  if (!footprintSegmentInsideGrid(grid, first, second, config)) {
    return Lattice3DEdgeEvaluation{.status =
                                       Lattice3DEdgeEvaluationStatus::kOutsideGrid};
  }
  if (!(length > 1.0e-9)) {
    return Lattice3DEdgeEvaluation{.status = Lattice3DEdgeEvaluationStatus::kValid};
  }
  const SweptFootprintClearanceProfile profile = profileSweptFootprintClearance(
      grid, esdf_m, first, second,
      SweptFootprintConfig{.radius_m = config.physical_footprint_radius_m,
                           .lower_extent_m = config.physical_footprint_lower_extent_m,
                           .upper_extent_m = config.physical_footprint_upper_extent_m,
                           .perimeter_samples = config.physical_footprint_samples,
                           .radial_rings = config.physical_footprint_radial_rings,
                           .axial_samples = config.physical_footprint_axial_samples,
                           .sweep_step_m = config.physical_footprint_sweep_step_m},
      config.critical_distance_m, config.preferred_distance_m);
  const SweptFootprintResult& footprint = profile.validation;
  if (!footprint.accepted()) {
    switch (footprint.status) {
      case SweptFootprintStatus::kOutsideGrid:
        return Lattice3DEdgeEvaluation{.status =
                                           Lattice3DEdgeEvaluationStatus::kOutsideGrid,
                                       .evidence = footprint.evidence};
      case SweptFootprintStatus::kUnknownSpace:
        if (config.require_known_free_space) {
          return Lattice3DEdgeEvaluation{
              .status = Lattice3DEdgeEvaluationStatus::kUnknownSpace,
              .evidence = footprint.evidence};
        }
        break;
      case SweptFootprintStatus::kInvalidEsdf:
        return Lattice3DEdgeEvaluation{.status =
                                           Lattice3DEdgeEvaluationStatus::kInvalidEsdf,
                                       .evidence = footprint.evidence};
      case SweptFootprintStatus::kRawCollision:
        return Lattice3DEdgeEvaluation{.status =
                                           Lattice3DEdgeEvaluationStatus::kRawCollision,
                                       .evidence = footprint.evidence};
      case SweptFootprintStatus::kValid:
        break;
    }
  }
  const double minimum_known_clearance_m =
      footprint.evidence.known_clearance_observed
          ? footprint.evidence.minimum_known_clearance_m
          : std::numeric_limits<double>::infinity();
  if (!stageAllows(stage, minimum_known_clearance_m, config)) {
    return Lattice3DEdgeEvaluation{
        .status = Lattice3DEdgeEvaluationStatus::kRiskStageRejected,
        .evidence = footprint.evidence};
  }
  return Lattice3DEdgeEvaluation{.status = Lattice3DEdgeEvaluationStatus::kValid,
                                 .evidence = footprint.evidence,
                                 .minimum_clearance_m = minimum_known_clearance_m,
                                 .planning_exposure_m = profile.planning_exposure_m,
                                 .critical_exposure_m = profile.critical_exposure_m};
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
  target.lattice_rejected_unknown_space += addition.lattice_rejected_unknown_space;
  target.lattice_rejected_flight_envelope += addition.lattice_rejected_flight_envelope;
  target.lattice_rejected_invalid_esdf += addition.lattice_rejected_invalid_esdf;
  target.lattice_rejected_raw_collision += addition.lattice_rejected_raw_collision;
  target.lattice_rejected_risk_stage += addition.lattice_rejected_risk_stage;
  target.lattice_rejected_no_cost_improvement +=
      addition.lattice_rejected_no_cost_improvement;
  target.passage_generated += addition.passage_generated;
  target.passage_accepted += addition.passage_accepted;
  target.passage_rejected += addition.passage_rejected;
  target.passage_rejected_connection_distance +=
      addition.passage_rejected_connection_distance;
  target.passage_rejected_outside_grid += addition.passage_rejected_outside_grid;
  target.passage_rejected_unknown_space += addition.passage_rejected_unknown_space;
  target.passage_rejected_flight_envelope += addition.passage_rejected_flight_envelope;
  target.passage_rejected_invalid_esdf += addition.passage_rejected_invalid_esdf;
  target.passage_rejected_raw_collision += addition.passage_rejected_raw_collision;
  target.passage_rejected_risk_stage += addition.passage_rejected_risk_stage;
  target.passage_rejected_no_cost_improvement +=
      addition.passage_rejected_no_cost_improvement;
}

void recordLattice3DRejectedEdge(Lattice3DSuccessorDiagnostics& diagnostics,
                                 const Lattice3DEdgeEvaluationStatus status,
                                 const bool passage) noexcept {
  std::size_t* counter = nullptr;
  switch (status) {
    case Lattice3DEdgeEvaluationStatus::kValid:
      return;
    case Lattice3DEdgeEvaluationStatus::kOutsideFlightEnvelope:
      counter = passage ? &diagnostics.passage_rejected_flight_envelope
                        : &diagnostics.lattice_rejected_flight_envelope;
      break;
    case Lattice3DEdgeEvaluationStatus::kOutsideGrid:
      counter = passage ? &diagnostics.passage_rejected_outside_grid
                        : &diagnostics.lattice_rejected_outside_grid;
      break;
    case Lattice3DEdgeEvaluationStatus::kUnknownSpace:
      counter = passage ? &diagnostics.passage_rejected_unknown_space
                        : &diagnostics.lattice_rejected_unknown_space;
      break;
    case Lattice3DEdgeEvaluationStatus::kInvalidEsdf:
      counter = passage ? &diagnostics.passage_rejected_invalid_esdf
                        : &diagnostics.lattice_rejected_invalid_esdf;
      break;
    case Lattice3DEdgeEvaluationStatus::kRawCollision:
      counter = passage ? &diagnostics.passage_rejected_raw_collision
                        : &diagnostics.lattice_rejected_raw_collision;
      break;
    case Lattice3DEdgeEvaluationStatus::kRiskStageRejected:
      counter = passage ? &diagnostics.passage_rejected_risk_stage
                        : &diagnostics.lattice_rejected_risk_stage;
      break;
  }
  ++(*counter);
}

} // namespace drone_city_nav::detail
