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
      route, spans, esdf.route_generation, snapshot.route_station_m,
      Point3{input.initial_state.x, input.initial_state.y, input.initial_state.z},
      Vec3{input.initial_state.vx, input.initial_state.vy, input.initial_state.vz},
      route_envelope_config, diagnostics_distance_m);
}

[[nodiscard]] std::string
persistentPlannerInfoFields(const ProductionMppiPreparedEsdf& esdf) {
  const ProductionPersistentPlannerTelemetry3D& planner = esdf.planner;
  std::ostringstream fields;
  fields << " planner=persistent_dstar_lite_3d"
         << " planner_invoked=" << (planner.invoked ? "true" : "false")
         << " planner_status=" << persistentPlannerStatus3DName(planner.status)
         << " planner_executable=" << (planner.executable ? "true" : "false")
         << " planner_search_complete=" << (planner.search_complete ? "true" : "false")
         << " planner_search_state_reused="
         << (planner.search_state_reused ? "true" : "false")
         << " planner_occupied_world_unchanged="
         << (planner.occupied_world_unchanged ? "true" : "false")
         << " planner_incumbent_retained="
         << (planner.incumbent_retained ? "true" : "false")
         << " planner_mission_epoch=" << planner.mission_epoch
         << " planner_planned_on_revision=" << planner.planned_on_revision
         << " planner_occupied_fingerprint=" << planner.occupied_fingerprint
         << " planner_search_generation=" << planner.search_generation
         << " planner_repair_generation=" << planner.repair_generation
         << " planner_expansions=" << planner.expansions
         << " planner_changed_occupied_voxels=" << planner.changed_occupied_voxels
         << " planner_affected_lattice_states=" << planner.affected_lattice_states
         << " planner_records=" << planner.records
         << " planner_open_entries=" << planner.open_entries
         << " planner_shortcut_checks=" << planner.shortcut_checks
         << " planner_shortcuts_applied=" << planner.shortcuts_applied
         << " planner_lattice_edge_queries=" << planner.lattice_edge_queries
         << " planner_raw_edge_validation_checks=" << planner.raw_edge_validation_checks
         << " planner_adaptive_edge_queries=" << planner.adaptive_edge_queries
         << " planner_adaptive_edges_in_extracted_path="
         << planner.adaptive_edges_in_extracted_path
         << " planner_maximum_queried_lattice_level="
         << planner.maximum_queried_lattice_level
         << " planner_path_length_m=" << planner.path_length_m
         << " planner_remaining_goal_distance_m=" << planner.remaining_goal_distance_m
         << " planner_estimated_execution_time_s=" << planner.estimated_execution_time_s
         << " planner_estimated_translation_time_s="
         << planner.estimated_translation_time_s
         << " planner_estimated_stationary_turn_time_s="
         << planner.estimated_stationary_turn_time_s
         << " planner_world_update_ms=" << planner.world_update_ms
         << " planner_search_ms=" << planner.search_ms;
  return fields.str();
}

[[nodiscard]] std::string
persistentPlannerJsonFields(const ProductionMppiPreparedEsdf& esdf) {
  const ProductionPersistentPlannerTelemetry3D& planner = esdf.planner;
  std::ostringstream fields;
  fields << ",\"planner\":\"persistent_dstar_lite_3d\""
         << ",\"planner_invoked\":" << (planner.invoked ? "true" : "false")
         << ",\"planner_status\":\"" << persistentPlannerStatus3DName(planner.status)
         << '"' << ",\"planner_executable\":" << (planner.executable ? "true" : "false")
         << ",\"planner_search_complete\":"
         << (planner.search_complete ? "true" : "false")
         << ",\"planner_search_state_reused\":"
         << (planner.search_state_reused ? "true" : "false")
         << ",\"planner_occupied_world_unchanged\":"
         << (planner.occupied_world_unchanged ? "true" : "false")
         << ",\"planner_incumbent_retained\":"
         << (planner.incumbent_retained ? "true" : "false")
         << ",\"planner_mission_epoch\":" << planner.mission_epoch
         << ",\"planner_planned_on_revision\":" << planner.planned_on_revision
         << ",\"planner_occupied_fingerprint\":" << planner.occupied_fingerprint
         << ",\"planner_search_generation\":" << planner.search_generation
         << ",\"planner_repair_generation\":" << planner.repair_generation
         << ",\"planner_expansions\":" << planner.expansions
         << ",\"planner_changed_occupied_voxels\":" << planner.changed_occupied_voxels
         << ",\"planner_affected_lattice_states\":" << planner.affected_lattice_states
         << ",\"planner_records\":" << planner.records
         << ",\"planner_open_entries\":" << planner.open_entries
         << ",\"planner_shortcut_checks\":" << planner.shortcut_checks
         << ",\"planner_shortcuts_applied\":" << planner.shortcuts_applied
         << ",\"planner_lattice_edge_queries\":" << planner.lattice_edge_queries
         << ",\"planner_raw_edge_validation_checks\":"
         << planner.raw_edge_validation_checks
         << ",\"planner_adaptive_edge_queries\":" << planner.adaptive_edge_queries
         << ",\"planner_adaptive_edges_in_extracted_path\":"
         << planner.adaptive_edges_in_extracted_path
         << ",\"planner_maximum_queried_lattice_level\":"
         << planner.maximum_queried_lattice_level
         << ",\"planner_path_length_m\":" << planner.path_length_m
         << ",\"planner_remaining_goal_distance_m\":"
         << planner.remaining_goal_distance_m
         << ",\"planner_estimated_execution_time_s\":"
         << planner.estimated_execution_time_s
         << ",\"planner_estimated_translation_time_s\":"
         << planner.estimated_translation_time_s
         << ",\"planner_estimated_stationary_turn_time_s\":"
         << planner.estimated_stationary_turn_time_s
         << ",\"planner_world_update_ms\":" << planner.world_update_ms
         << ",\"planner_search_ms\":" << planner.search_ms;
  return fields.str();
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
