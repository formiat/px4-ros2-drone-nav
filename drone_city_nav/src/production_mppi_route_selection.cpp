#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] SegmentEvidenceWorld3D
evidenceWorld(const ProductionMppiPreparedEsdf& world,
              const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
              const OccupancyGrid3D* const static_occupancy,
              const SweptFootprintConfig& physical_footprint,
              const FlightEnvelopeConfig& flight_envelope) noexcept {
  return SegmentEvidenceWorld3D{
      .grid = &world.grid,
      .esdf_m = world.distances_m ? std::span<const float>{*world.distances_m}
                                  : std::span<const float>{},
      .latest_observed_occupancy = latest_raw_world && latest_raw_world->occupancy
                                       ? latest_raw_world->occupancy.get()
                                       : nullptr,
      .static_occupancy = static_occupancy,
      .proprioceptive_free_space_seed =
          world.proprioceptive_free_space_seed
              ? std::addressof(*world.proprioceptive_free_space_seed)
              : nullptr,
      .launch_support_contact = world.launch_support_contact
                                    ? std::addressof(*world.launch_support_contact)
                                    : nullptr,
      .footprint = physical_footprint,
      .flight_envelope = flight_envelope,
      .validated_through_revision = latest_raw_world
                                        ? latest_raw_world->version.revision
                                        : world.source_raw_revision,
      .require_known_free_space = false,
      .reject_invalid_esdf = false,
  };
}

[[nodiscard]] bool observedPlannerWorldValid(
    const std::shared_ptr<const ProductionMppiRawWorld3D>& raw_world) noexcept {
  return raw_world != nullptr && raw_world->version.valid() &&
         raw_world->occupancy != nullptr && raw_world->execution_owner != nullptr &&
         raw_world->execution_owner->valid() &&
         std::addressof(raw_world->execution_owner->occupancy()) ==
             raw_world->occupancy.get() &&
         raw_world->execution_owner->version().sameLineage(raw_world->version) &&
         raw_world->execution_owner->version().revision ==
             raw_world->version.revision &&
         raw_world->execution_owner->occupiedContentFingerprint() != 0U;
}

[[nodiscard]] Vec3 velocityAtStitch(const RouteSample3D& stitch,
                                    const double cruise_speed_mps) noexcept {
  const double speed = std::clamp(stitch.reference_speed_mps, 0.0, cruise_speed_mps);
  return Vec3{stitch.tangent.x * speed, stitch.tangent.y * speed,
              stitch.tangent.z * speed};
}

} // namespace

ProductionRouteCandidateSet3D ProductionMppiNode::generateRouteCandidates3D(
    const ProductionMppiPreparedEsdf& world, const ProductionMppiNavigation& navigation,
    const Point3& mission_goal,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
    const CertifiedRouteSuffix3D* const active_route) {
  const auto search_started = std::chrono::steady_clock::now();
  ProductionRouteCandidateSet3D result;
  Point3 search_start{navigation.state.x, navigation.state.y, navigation.state.z};
  Vec3 search_velocity{static_cast<double>(navigation.state.vx),
                       static_cast<double>(navigation.state.vy),
                       static_cast<double>(navigation.state.vz)};
  RouteInstanceId3D search_base_route_instance_id{};
  std::optional<double> search_base_stitch_station_m;

  const bool successor_search =
      world.static_route_extension_request || world.static_route_replan_request;
  const bool certified_stitch_base_available =
      active_route != nullptr && active_route->route_instance_id.valid() &&
      active_route->route_instance_id == world.bound_route_instance_id &&
      active_route->geometry != nullptr && active_route->geometry->route != nullptr &&
      active_route->progress.valid();
  if (successor_search && active_route != nullptr) {
    if (!certified_stitch_base_available) {
      RCLCPP_INFO(get_logger(),
                  "PERSISTENT_PLANNER3D stage=deferred reason=stitch_base_unavailable "
                  "revision=%" PRIu64,
                  world.revision);
      result.search_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - search_started)
                             .count();
      return result;
    }
    const std::vector<RouteSample3D>& active_geometry = *active_route->geometry->route;
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
                  world.revision, stitch_station_m, active_geometry.back().station_m);
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
    if (!observedPlannerWorldValid(latest_raw_world)) {
      RCLCPP_INFO(get_logger(),
                  "PERSISTENT_PLANNER3D stage=deferred reason=raw_world_unavailable "
                  "revision=%" PRIu64,
                  world.revision);
      result.search_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - search_started)
                             .count();
      return result;
    }
    planner_world.observed_occupancy = latest_raw_world->occupancy;
    planner_world.dirty_chunks = latest_raw_world->dirty_chunks;
    planner_world.producer_instance_id = latest_raw_world->version.producer_instance_id;
    planner_world.revision = latest_raw_world->version.revision;
    planner_world.occupied_fingerprint =
        latest_raw_world->execution_owner->occupiedContentFingerprint();
    planner_world.full_reset = latest_raw_world->full_reset;
  }
  planner_world.proprioceptive_free_space_seed = world.proprioceptive_free_space_seed;
  planner_world.launch_support_contact = world.launch_support_contact;

  if (persistent_planner_3d_ == nullptr || !world.search_objective.available ||
      world.search_objective.mission_epoch == 0U || !planner_world.valid()) {
    result.search_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - search_started)
                           .count();
    return result;
  }

  result.planner_invoked = true;
  result.planner_result = persistent_planner_3d_->plan(PersistentPlannerRequest3D{
      .start = search_start,
      .velocity = search_velocity,
      .mission_goal = mission_goal,
      .mission_epoch = world.search_objective.mission_epoch,
      .world = std::move(planner_world),
  });
  const PersistentPlannerResult3D& plan = result.planner_result;
  RCLCPP_INFO(
      get_logger(),
      "PERSISTENT_PLANNER3D stage=complete raw_revision=%" PRIu64
      " mission_epoch=%" PRIu64 " status=%s executable=%s reused=%s "
      "occupied_unchanged=%s incumbent_retained=%s search_complete=%s "
      "points=%zu expansions=%zu changed_occupied=%zu affected_states=%zu "
      "records=%zu open=%zu shortcuts=%zu/%zu edge_queries=%zu "
      "raw_edge_checks=%zu adaptive_edge_queries=%zu adaptive_path_edges=%zu "
      "maximum_adaptive_level=%zu eta_s=%.3f "
      "translation_s=%.3f turn_s=%.3f world_update_ms=%.3f search_ms=%.3f",
      plan.planned_on_revision, plan.mission_epoch,
      persistentPlannerStatus3DName(plan.status), plan.executable() ? "true" : "false",
      plan.search_state_reused ? "true" : "false",
      plan.occupied_world_unchanged ? "true" : "false",
      plan.incumbent_retained ? "true" : "false",
      plan.search_complete ? "true" : "false", plan.points.size(), plan.expansions,
      plan.changed_occupied_voxels, plan.affected_lattice_states, plan.records,
      plan.open_entries, plan.shortcuts_applied, plan.shortcut_checks,
      plan.lattice_edge_queries, plan.raw_edge_validation_checks,
      plan.adaptive_edge_queries, plan.adaptive_edges_in_extracted_path,
      plan.maximum_queried_lattice_level, plan.estimated_execution_time_s,
      plan.estimated_translation_time_s, plan.estimated_stationary_turn_time_s,
      plan.world_update_ms, plan.search_ms);

  if (plan.executable()) {
    std::vector<RouteSample3D> route = sampleRoute3D(
        plan.points, route_sampling_step_m_, speed_policy_config_.cruise_speed_mps);
    RouteIntent3D intent{
        .planned_on_revision = plan.planned_on_revision,
        .mission_target = mission_goal,
        .valid = true,
    };
    intent.id =
        makeRouteIntentId3D(intent.mission_target, world.search_objective.mission_epoch,
                            world.search_objective.assignment_generation,
                            world.search_objective.target_detection_id,
                            world.search_objective.target_track_id);
    const SegmentEvidenceWorld3D evidence_world =
        evidenceWorld(world, latest_raw_world, static_occupancy_3d_.get(),
                      physical_footprint_config_, flight_envelope_config_);
    SegmentEvidence3D evidence =
        evaluateSegmentEvidence3D(intent, route, search_start, true, true,
                                  plan.estimated_execution_time_s, evidence_world);
    result.candidates.push_back(ProductionRouteSearchCandidate3D{
        .search_start = search_start,
        .search_velocity = search_velocity,
        .search_base_route_instance_id = search_base_route_instance_id,
        .search_base_stitch_station_m = search_base_stitch_station_m,
        .intent = intent,
        .evidence = evidence,
        .plan = plan,
        .route = std::move(route),
    });
  }
  result.search_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - search_started)
                         .count();
  return result;
}

} // namespace drone_city_nav
