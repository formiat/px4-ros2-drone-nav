#pragma once

#include <cmath>
#include <span>
#include <sstream>
#include <string>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double finiteOrNegative(const double value) noexcept {
  return std::isfinite(value) ? value : -1.0;
}

[[nodiscard]] const char*
planningStatusName(const ProductionMppiPreparedEsdf& esdf) noexcept {
  return esdf.planning_search_kind ==
                 ProductionPlanningSearchKind::kPersistentDStarLite3D
             ? lattice3DStatusName(esdf.lattice_3d_status)
             : latticePlanStatusName(esdf.lattice_status);
}

[[nodiscard]] const char*
planningTerminationName(const ProductionMppiPreparedEsdf& esdf) noexcept {
  return esdf.planning_search_kind ==
                 ProductionPlanningSearchKind::kPersistentDStarLite3D
             ? lattice3DSearchTerminationName(esdf.lattice_3d_termination)
             : latticeSearchTerminationName(esdf.lattice_termination);
}

[[nodiscard]] const char*
planningRiskStageName(const ProductionMppiPreparedEsdf& esdf) noexcept {
  return esdf.planning_search_kind ==
                 ProductionPlanningSearchKind::kPersistentDStarLite3D
             ? lattice3DRiskStageName(esdf.lattice_3d_risk_stage)
             : latticeRiskStageName(esdf.lattice_risk_stage);
}

[[nodiscard]] const char*
planningRoutePurposeName(const ProductionMppiPreparedEsdf& esdf) noexcept {
  return esdf.planning_search_kind ==
                 ProductionPlanningSearchKind::kPersistentDStarLite3D
             ? lattice3DRoutePurposeName(esdf.lattice_3d_route_purpose)
             : "mission_transit";
}

[[nodiscard]] ConstrainedRouteObservation
diagnosticRouteConstraint(const ProductionMppiDiagnosticsSnapshot& snapshot,
                          const RouteEnvelopeConfig& route_envelope_config,
                          const double diagnostics_distance_m) {
  const ProductionMppiPreparedEsdf& esdf = snapshot.esdf;
  const mppi::MppiTickInput& input = snapshot.input;
  const std::span<const RouteSample3D> route =
      snapshot.route_projection_valid && esdf.route_3d
          ? std::span<const RouteSample3D>{*esdf.route_3d}
          : std::span<const RouteSample3D>{};
  const std::span<const ConstrainedRouteSpan> spans =
      esdf.constrained_spans
          ? std::span<const ConstrainedRouteSpan>{*esdf.constrained_spans}
          : std::span<const ConstrainedRouteSpan>{};
  return observeConstrainedRoute(
      route, spans, esdf.global_guide_generation, snapshot.route_station_m,
      Point3{input.initial_state.x, input.initial_state.y, input.initial_state.z},
      Vec3{input.initial_state.vx, input.initial_state.vy, input.initial_state.vz},
      route_envelope_config, diagnostics_distance_m);
}

[[nodiscard]] std::string
certifiedRouteReserveInfoFields(const ProductionMppiPreparedEsdf& esdf) {
  std::ostringstream fields;
  fields << " certified_route_reserve="
         << certifiedRouteReserveStatus3DName(esdf.certified_route_reserve_status)
         << " certified_route_reserve_available_m="
         << esdf.certified_route_reserve_available_m
         << " certified_route_reserve_required_m="
         << esdf.certified_route_reserve_required_m
         << " certified_route_reserve_shortfall_m="
         << esdf.certified_route_reserve_shortfall_m;
  return fields.str();
}

[[nodiscard]] std::string
certifiedRouteReserveJsonFields(const ProductionMppiPreparedEsdf& esdf) {
  std::ostringstream fields;
  fields << ",\"certified_route_reserve\":\""
         << certifiedRouteReserveStatus3DName(esdf.certified_route_reserve_status)
         << '"' << ",\"certified_route_reserve_available_m\":"
         << esdf.certified_route_reserve_available_m
         << ",\"certified_route_reserve_required_m\":"
         << esdf.certified_route_reserve_required_m
         << ",\"certified_route_reserve_shortfall_m\":"
         << esdf.certified_route_reserve_shortfall_m;
  return fields.str();
}

[[nodiscard]] std::string
trackingErrorTubeInfoFields(const ProductionMppiPreparedEsdf& esdf) {
  const TrackingErrorTubeProfile3D* const tube =
      esdf.compiled_route_geometry != nullptr
          ? esdf.compiled_route_geometry->tracking_error_tube.get()
          : nullptr;
  std::ostringstream fields;
  fields << " tracking_tube_obstacle_evidence="
         << (tube != nullptr && tube->obstacle_evidence_available ? "true" : "false")
         << " tracking_tube_minimum_speed_limit_mps="
         << (tube != nullptr ? tube->minimum_speed_limit_mps : 0.0)
         << " tracking_tube_maximum_error_m="
         << (tube != nullptr ? tube->maximum_tracking_error_m : 0.0)
         << " tracking_tube_constrained_segments="
         << (tube != nullptr ? tube->constrained_segment_count : 0U);
  return fields.str();
}

[[nodiscard]] std::string
trackingErrorTubeJsonFields(const ProductionMppiPreparedEsdf& esdf) {
  const TrackingErrorTubeProfile3D* const tube =
      esdf.compiled_route_geometry != nullptr
          ? esdf.compiled_route_geometry->tracking_error_tube.get()
          : nullptr;
  std::ostringstream fields;
  fields << ",\"tracking_tube_obstacle_evidence\":"
         << (tube != nullptr && tube->obstacle_evidence_available ? "true" : "false")
         << ",\"tracking_tube_minimum_speed_limit_mps\":"
         << (tube != nullptr ? tube->minimum_speed_limit_mps : 0.0)
         << ",\"tracking_tube_maximum_error_m\":"
         << (tube != nullptr ? tube->maximum_tracking_error_m : 0.0)
         << ",\"tracking_tube_constrained_segments\":"
         << (tube != nullptr ? tube->constrained_segment_count : 0U);
  return fields.str();
}

} // namespace
} // namespace drone_city_nav
