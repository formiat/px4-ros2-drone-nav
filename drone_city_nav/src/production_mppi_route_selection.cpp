#include "production_mppi_route_selection.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] SegmentEvidenceWorld3D
evidenceWorld(const WorldSnapshot3D& world,
              const ObservedOccupancyGrid3D* const route_search_occupancy,
              const std::uint64_t route_search_raw_revision,
              const OccupancyGrid3D* const static_occupancy,
              const SweptFootprintConfig& physical_footprint,
              const FlightEnvelopeConfig& flight_envelope) noexcept {
  return SegmentEvidenceWorld3D{
      .grid = &world.grid,
      .esdf_m = world.distances_m ? std::span<const float>{*world.distances_m}
                                  : std::span<const float>{},
      .collision =
          OccupiedCollisionWorld3D{
              .observed_occupancy = route_search_occupancy,
              .static_occupancy = static_occupancy,
              .planar_occupancy = nullptr,
              .raw_point_cloud = {},
              .launch_support_contact =
                  world.launch_support_contact
                      ? std::addressof(*world.launch_support_contact)
                      : nullptr,
              .footprint = physical_footprint,
              .flight_envelope = flight_envelope,
          },
      .validated_through_revision = route_search_raw_revision,
  };
}

[[nodiscard]] Vec3 velocityAtStitch(const RouteSample3D& stitch,
                                    const double cruise_speed_mps) noexcept {
  const double speed = std::clamp(stitch.reference_speed_mps, 0.0, cruise_speed_mps);
  return Vec3{stitch.tangent.x * speed, stitch.tangent.y * speed,
              stitch.tangent.z * speed};
}

} // namespace

ProductionPlannerUpdate3D ProductionMppiNode::generatePlannerUpdate3D(
    const ProductionMppiPreparedEsdf& world, const ProductionMppiNavigation& navigation,
    const Point3& mission_goal, const CertifiedRouteSuffix3D* const active_route,
    std::shared_ptr<const ProductionPlannerSession3D> continuation_session) {
  const auto search_started = std::chrono::steady_clock::now();
  ProductionPlannerUpdate3D result;
  if (!continuation_session) {
    Point3 search_start{navigation.state.x, navigation.state.y, navigation.state.z};
    Vec3 search_velocity{static_cast<double>(navigation.state.vx),
                         static_cast<double>(navigation.state.vy),
                         static_cast<double>(navigation.state.vz)};
    RouteInstanceId3D search_base_route_instance_id{};
    std::optional<double> search_base_stitch_station_m;

    const bool certified_stitch_required =
        productionRouteSearchContinuity3D(world.static_route_extension_request,
                                          world.static_route_replan_request) ==
        ProductionRouteSearchContinuity3D::kCertifiedStitch;
    const bool certified_stitch_base_available =
        active_route != nullptr && active_route->route_instance_id.valid() &&
        active_route->route_instance_id == world.bound_route_instance_id &&
        active_route->geometry != nullptr && active_route->geometry->route != nullptr &&
        active_route->progress.valid();
    if (certified_stitch_required) {
      if (!certified_stitch_base_available) {
        RCLCPP_INFO(get_logger(),
                    "PERSISTENT_PLANNER3D stage=deferred "
                    "reason=stitch_base_unavailable revision=%" PRIu64,
                    world.world->revision);
        result.search_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - search_started)
                               .count();
        return result;
      }
      const std::vector<RouteSample3D>& active_geometry =
          *active_route->geometry->route;
      const RouteProjection3D navigation_projection =
          projectOntoRoute3DWithinStationWindow(active_geometry, search_start,
                                                active_route->progress.station_m,
                                                active_geometry.back().station_m);
      const double stitch_station_m =
          std::max({active_route->progress.station_m,
                    world.route_projection.valid ? world.route_projection.station_m
                                                 : active_route->progress.station_m,
                    navigation_projection.valid ? navigation_projection.station_m
                                                : active_route->progress.station_m}) +
          static_route_extension_config_.required_certified_overlap_m;
      if (stitch_station_m > active_geometry.back().station_m) {
        RCLCPP_INFO(get_logger(),
                    "PERSISTENT_PLANNER3D stage=deferred "
                    "reason=future_stitch_beyond_certified_route revision=%" PRIu64
                    " stitch_station_m=%.3f route_end_station_m=%.3f",
                    world.world->revision, stitch_station_m,
                    active_geometry.back().station_m);
        result.search_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - search_started)
                               .count();
        return result;
      }
      const RouteSample3D stitch =
          sampleRoute3DAtStation(active_geometry, stitch_station_m);
      search_start = stitch.position;
      search_velocity = velocityAtStitch(stitch, speed_policy_config_.cruise_speed_mps);
      search_base_route_instance_id = active_route->route_instance_id;
      search_base_stitch_station_m = stitch_station_m;
    }

    PersistentPlannerWorld3D planner_world;
    if (use_static_map_) {
      if (static_occupancy_3d_ == nullptr) {
        result.search_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - search_started)
                               .count();
        return result;
      }
      planner_world.static_occupancy = static_occupancy_3d_;
      planner_world.producer_instance_id = static_occupancy_3d_->fingerprint();
      planner_world.revision = 1U;
      planner_world.occupied_fingerprint = static_occupancy_3d_->contentFingerprint();
    } else {
      const std::shared_ptr<const PersistentPlannerWorld3D> search_world =
          routeSearchPlannerWorld3D(world.observed_planner_world,
                                    world.route_search_planner_world,
                                    world.static_route_replan_request);
      if (search_world == nullptr || !search_world->valid()) {
        RCLCPP_INFO(get_logger(),
                    "PERSISTENT_PLANNER3D stage=deferred reason=raw_world_unavailable "
                    "revision=%" PRIu64,
                    world.world->revision);
        result.search_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - search_started)
                               .count();
        return result;
      }
      planner_world = *search_world;
    }
    planner_world.proprioceptive_free_space_seed =
        world.world->proprioceptive_free_space_seed;
    planner_world.launch_support_contact = world.world->launch_support_contact;
    if (persistent_planner_3d_ == nullptr || !world.search_objective.available ||
        world.search_objective.mission_epoch == 0U || !planner_world.valid()) {
      result.search_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - search_started)
                             .count();
      return result;
    }
    RouteIntent3D intent{
        .planned_on_revision = planner_world.revision,
        .mission_target = mission_goal,
        .valid = true,
    };
    intent.id =
        makeRouteIntentId3D(intent.mission_target, world.search_objective.mission_epoch,
                            world.search_objective.assignment_generation,
                            world.search_objective.target_detection_id,
                            world.search_objective.target_track_id);
    continuation_session =
        std::make_shared<const ProductionPlannerSession3D>(ProductionPlannerSession3D{
            .request =
                PersistentPlannerRequest3D{
                    .start = search_start,
                    .velocity = search_velocity,
                    .mission_goal = mission_goal,
                    .mission_epoch = world.search_objective.mission_epoch,
                    .world = std::move(planner_world),
                },
            .mission_goal = mission_goal,
            .search_start = search_start,
            .search_velocity = search_velocity,
            .search_base_route_instance_id = search_base_route_instance_id,
            .search_base_stitch_station_m = search_base_stitch_station_m,
            .intent = intent,
        });
  }

  if (persistent_planner_3d_ == nullptr || !continuation_session ||
      !continuation_session->request.world.valid() ||
      !world.search_objective.available ||
      world.search_objective.mission_epoch !=
          continuation_session->request.mission_epoch ||
      distance3D(mission_goal, continuation_session->mission_goal) > 1.0e-9) {
    result.search_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - search_started)
                           .count();
    return result;
  }
  const ObservedOccupancyGrid3D* const route_search_occupancy =
      continuation_session->request.world.observed_occupancy.get();
  const std::uint64_t route_search_raw_revision =
      continuation_session->request.world.revision;

  result.planner_invoked = true;
  PlannerUpdate3D planner_update =
      persistent_planner_3d_->plan(continuation_session->request);
  result.planner_input_status = planner_update.input_status;
  result.planner_progress = planner_update.progress;
  result.planner_telemetry = planner_update.telemetry;
  result.dispatch = coordinatePlannerUpdate3D(planner_update);
  result.planner_session = continuation_session;
  const PlannerTelemetry3D& telemetry = planner_update.telemetry;
  const SpatialRouteCandidate3D* const improved =
      planner_update.improved_incumbent
          ? std::addressof(*planner_update.improved_incumbent)
          : nullptr;
  RCLCPP_INFO(get_logger(),
              "PERSISTENT_PLANNER3D stage=complete raw_revision=%" PRIu64
              " mission_epoch=%" PRIu64 " input=%s progress=%s publishable=%s "
              "source=%s reused=%s occupied_unchanged=%s incumbent_retained=%s "
              "time_search_complete=%s points=%zu expansions=%zu time_expansions=%zu "
              "changed_occupied=%zu affected_states=%zu repair_processed=%zu "
              "repair_pending=%zu repair_in_progress=%s feasibility_attempted=%s "
              "feasibility_found=%s feasibility_expansions=%zu records=%zu open=%zu "
              "time_records=%zu time_open=%zu shortcuts=%zu/%zu edge_queries=%zu "
              "raw_edge_checks=%zu adaptive_edge_queries=%zu adaptive_path_edges=%zu "
              "maximum_adaptive_level=%zu time_objective_s=%.3f eta_s=%.3f "
              "translation_s=%.3f turn_s=%.3f world_update_ms=%.3f search_ms=%.3f",
              telemetry.planned_on_revision, telemetry.mission_epoch,
              plannerInputStatus3DName(planner_update.input_status),
              searchProgress3DName(planner_update.progress),
              planner_update.publishable() ? "true" : "false",
              improved ? spatialRouteCandidateSource3DName(improved->source) : "none",
              telemetry.search_state_reused ? "true" : "false",
              telemetry.occupied_world_unchanged ? "true" : "false",
              telemetry.incumbent_retained ? "true" : "false",
              telemetry.execution_time_search_complete ? "true" : "false",
              improved ? improved->points.size() : 0U, telemetry.expansions,
              telemetry.execution_time_search_expansions,
              telemetry.changed_occupied_voxels, telemetry.affected_lattice_states,
              telemetry.repair_lattice_states_processed,
              telemetry.repair_lattice_states_pending,
              telemetry.repair_pending ? "true" : "false",
              telemetry.feasibility_attempted ? "true" : "false",
              telemetry.feasibility_route_found ? "true" : "false",
              telemetry.feasibility_expansions, telemetry.records,
              telemetry.open_entries, telemetry.execution_time_search_records,
              telemetry.execution_time_search_open_entries, telemetry.shortcuts_applied,
              telemetry.shortcut_checks, telemetry.lattice_edge_queries,
              telemetry.raw_edge_validation_checks, telemetry.adaptive_edge_queries,
              telemetry.adaptive_edges_in_extracted_path,
              telemetry.maximum_queried_lattice_level,
              telemetry.execution_time_search_objective_s,
              improved ? improved->estimated_execution_time_s : 0.0,
              improved ? improved->estimated_translation_time_s : 0.0,
              improved ? improved->estimated_stationary_turn_time_s : 0.0,
              telemetry.world_update_ms, telemetry.search_ms);

  if (planner_update.improved_incumbent.has_value()) {
    SpatialRouteCandidate3D spatial_route =
        std::move(*planner_update.improved_incumbent);
    std::vector<RouteSample3D> route =
        sampleRoute3D(spatial_route.points, route_sampling_step_m_,
                      speed_policy_config_.cruise_speed_mps);
    RouteIntent3D intent = continuation_session->intent;
    intent.planned_on_revision = telemetry.planned_on_revision;
    const SegmentEvidenceWorld3D evidence_world =
        evidenceWorld(*world.world, route_search_occupancy, route_search_raw_revision,
                      static_occupancy_3d_.get(), physical_footprint_config_,
                      flight_envelope_config_);
    SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
        intent, route, continuation_session->search_start, true, true,
        spatial_route.estimated_execution_time_s, evidence_world);
    result.improved_incumbent = ProductionRouteSearchCandidate3D{
        .search_start = continuation_session->search_start,
        .search_velocity = continuation_session->search_velocity,
        .search_base_route_instance_id =
            continuation_session->search_base_route_instance_id,
        .search_base_stitch_station_m =
            continuation_session->search_base_stitch_station_m,
        .intent = intent,
        .evidence = evidence,
        .spatial_route = std::move(spatial_route),
        .planner_input_status = planner_update.input_status,
        .planner_progress = planner_update.progress,
        .planner_telemetry = telemetry,
        .route = std::move(route),
    };
  }
  result.search_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - search_started)
                         .count();
  return result;
}

} // namespace drone_city_nav
