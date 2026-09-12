#include "route_planner_3d.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <span>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finiteVector(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] RoutePlannerConfig3D validatedConfig(const RoutePlannerConfig3D& config) {
  if (!std::isfinite(config.route_sampling_step_m) ||
      config.route_sampling_step_m <= 0.0 || !std::isfinite(config.cruise_speed_mps) ||
      config.cruise_speed_mps <= 0.0 ||
      !std::isfinite(config.extension.required_certified_overlap_m) ||
      config.extension.required_certified_overlap_m < 0.0) {
    throw std::invalid_argument{"invalid route planner configuration"};
  }
  return config;
}

[[nodiscard]] double
elapsedMilliseconds(const std::chrono::steady_clock::time_point started) noexcept {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                   started)
      .count();
}

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
              .proprioceptive_free_space_seed =
                  world.proprioceptive_free_space_seed
                      ? std::addressof(*world.proprioceptive_free_space_seed)
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

std::string_view
routePlannerUpdateStatus3DName(const RoutePlannerUpdateStatus3D status) noexcept {
  switch (status) {
    case RoutePlannerUpdateStatus3D::kUpdated:
      return "updated";
    case RoutePlannerUpdateStatus3D::kInvalidRequest:
      return "invalid_request";
    case RoutePlannerUpdateStatus3D::kStitchBaseUnavailable:
      return "stitch_base_unavailable";
    case RoutePlannerUpdateStatus3D::kStitchBeyondCertifiedRoute:
      return "stitch_beyond_certified_route";
  }
  return "unknown";
}

RoutePlanner3D::RoutePlanner3D(const RoutePlannerConfig3D& config)
    : config_{validatedConfig(config)},
      planner_{config_.planner} {
}

RoutePlannerUpdate3D RoutePlanner3D::update(
    const PlannerSearchTransaction3D& transaction,
    const RoutePlannerVehicleState3D& vehicle_state,
    std::shared_ptr<const RoutePlannerSession3D> continuation_session,
    const bool renew_consumer_session) {
  const auto search_started = std::chrono::steady_clock::now();
  RoutePlannerUpdate3D result;
  const auto finish = [search_started](RoutePlannerUpdate3D update) {
    update.search_ms = elapsedMilliseconds(search_started);
    return update;
  };
  if (!transaction.valid() || !vehicle_state.valid ||
      !finitePoint(vehicle_state.position) || !finiteVector(vehicle_state.velocity)) {
    return finish(std::move(result));
  }
  const Point3 mission_goal = transaction.objective.goal;

  if (continuation_session == nullptr) {
    Point3 search_start = vehicle_state.position;
    Vec3 search_velocity = vehicle_state.velocity;
    RouteInstanceId3D search_base_route_instance_id{};
    std::optional<double> search_base_stitch_station_m;

    const PlannerSearchContinuityBase3D* const continuity_base =
        transaction.continuity_base ? std::addressof(*transaction.continuity_base)
                                    : nullptr;
    // An extension always stitches. A replacement of a route blocked ahead
    // stitches onto the certified prefix before the block, and falls back to
    // a search from the vehicle when the overlap would reach the block.
    const bool certified_stitch_required =
        transaction.extension() ||
        (transaction.replacement() && continuity_base != nullptr);
    const CertifiedRouteSuffix3D* const active_route =
        continuity_base != nullptr ? continuity_base->route.get() : nullptr;
    const bool certified_stitch_base_available =
        active_route != nullptr && active_route->route_instance_id.valid() &&
        active_route->geometry != nullptr && active_route->geometry->route != nullptr &&
        active_route->progress.valid();
    if (certified_stitch_required) {
      if (!certified_stitch_base_available) {
        result.status = RoutePlannerUpdateStatus3D::kStitchBaseUnavailable;
        return finish(std::move(result));
      }
      const std::vector<RouteSample3D>& active_geometry =
          *active_route->geometry->route;
      const RouteProjection3D navigation_projection =
          projectOntoRoute3DWithinStationWindow(active_geometry, search_start,
                                                active_route->progress.station_m,
                                                active_geometry.back().station_m);
      const double stitch_station_m = std::max(
          std::max({active_route->progress.station_m,
                    continuity_base->request_projection.valid
                        ? continuity_base->request_projection.station_m
                        : active_route->progress.station_m,
                    navigation_projection.valid ? navigation_projection.station_m
                                                : active_route->progress.station_m}) +
              config_.extension.required_certified_overlap_m,
          continuity_base->minimum_stitch_station_m.value_or(0.0));
      result.attempted_stitch_station_m = stitch_station_m;
      result.certified_route_end_station_m = active_geometry.back().station_m;
      const bool stitch_beyond_limit =
          continuity_base->stitch_limit_station_m.has_value() &&
          stitch_station_m > *continuity_base->stitch_limit_station_m;
      if (stitch_station_m > active_geometry.back().station_m || stitch_beyond_limit) {
        if (transaction.extension()) {
          result.status = RoutePlannerUpdateStatus3D::kStitchBeyondCertifiedRoute;
          return finish(std::move(result));
        }
        // The block is inside the overlap: nothing certified is left to keep,
        // so the replacement is searched from the vehicle after all.
        result.stitch_fallback_to_vehicle = true;
      } else {
        const RouteSample3D stitch =
            sampleRoute3DAtStation(active_geometry, stitch_station_m);
        search_start = stitch.position;
        search_velocity = velocityAtStitch(stitch, config_.cruise_speed_mps);
        search_base_route_instance_id = active_route->route_instance_id;
        search_base_stitch_station_m = stitch_station_m;
      }
    }

    PersistentPlannerWorld3D planner_world = *transaction.planner_world;
    if (!transaction.objective.available || transaction.objective.mission_epoch == 0U ||
        !planner_world.valid()) {
      return finish(std::move(result));
    }
    RouteIntent3D intent{
        .planned_on_revision = planner_world.revision,
        .mission_target = mission_goal,
        .valid = true,
    };
    intent.id =
        makeRouteIntentId3D(intent.mission_target, transaction.objective.mission_epoch,
                            transaction.objective.assignment_generation,
                            transaction.objective.target_detection_id,
                            transaction.objective.target_track_id);
    continuation_session =
        std::make_shared<const RoutePlannerSession3D>(RoutePlannerSession3D{
            .request =
                PersistentPlannerRequest3D{
                    .start = search_start,
                    .velocity = search_velocity,
                    .mission_goal = mission_goal,
                    .mission_epoch = transaction.objective.mission_epoch,
                    .world = std::move(planner_world),
                    .session_id = ++next_session_id_,
                },
            .mission_goal = mission_goal,
            .search_start = search_start,
            .search_velocity = search_velocity,
            .search_base_route_instance_id = search_base_route_instance_id,
            .search_base_stitch_station_m = search_base_stitch_station_m,
            .intent = intent,
        });
  }

  if (continuation_session == nullptr || !continuation_session->request.world.valid() ||
      transaction.objective.mission_epoch !=
          continuation_session->request.mission_epoch ||
      distance3D(mission_goal, continuation_session->mission_goal) > 1.0e-9) {
    return finish(std::move(result));
  }
  if (renew_consumer_session) {
    auto renewed = std::make_shared<RoutePlannerSession3D>(*continuation_session);
    renewed->request.session_id = ++next_session_id_;
    continuation_session = std::move(renewed);
  }
  const ObservedOccupancyGrid3D* const route_search_occupancy =
      continuation_session->request.world.observed_occupancy.get();
  const std::uint64_t route_search_raw_revision =
      continuation_session->request.world.revision;

  result.planner_invoked = true;
  PersistentPlannerRequest3D planner_request = continuation_session->request;
  planner_request.vehicle_route_available = vehicle_state.route_available;
  PlannerUpdate3D planner_update = planner_.plan(planner_request);
  result.planner_input_status = planner_update.input_status;
  result.planner_progress = planner_update.progress;
  result.planner_telemetry = planner_update.telemetry;
  result.dispatch = coordinatePlannerUpdate3D(planner_update);
  result.planner_session = continuation_session;
  result.status = RoutePlannerUpdateStatus3D::kUpdated;

  if (planner_update.improved_incumbent.has_value()) {
    SpatialRouteCandidate3D spatial_route =
        std::move(*planner_update.improved_incumbent);
    std::vector<RouteSample3D> route = sampleRoute3D(
        spatial_route.points, config_.route_sampling_step_m, config_.cruise_speed_mps);
    RouteIntent3D intent = continuation_session->intent;
    intent.planned_on_revision = planner_update.telemetry.planned_on_revision;
    const SegmentEvidenceWorld3D evidence_world = evidenceWorld(
        *transaction.world, route_search_occupancy, route_search_raw_revision,
        transaction.world->static_occupancy.get(), config_.planner.physical_footprint,
        config_.planner.flight_envelope);
    SegmentEvidence3D evidence = evaluateSegmentEvidence3D(
        intent, route, continuation_session->search_start, true, true,
        spatial_route.estimated_execution_time_s, evidence_world);
    result.improved_incumbent = RouteSearchCandidate3D{
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
        .planner_telemetry = planner_update.telemetry,
        .route = std::move(route),
    };
  }
  return finish(std::move(result));
}

void RoutePlanner3D::reset() noexcept {
  planner_.reset();
}

} // namespace drone_city_nav
