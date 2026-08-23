#include <algorithm>
#include <cinttypes>
#include <memory>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

ProductionRouteExecutionSelection3D ProductionMppiNode::resolveRouteExecution3D(
    const ProductionMppiPreparedEsdf& world,
    const ProductionNavigationObjective* const objective,
    const ProductionMppiNavigation& navigation,
    const std::shared_ptr<const ProductionMppiRawWorld3D>& latest_raw_world,
    const std::uint64_t minimum_tracking_sample_sequence, const bool direct_tracking,
    const bool observed_3d_world) {
  ProductionRouteExecutionSelection3D result{
      .route = world.activated_route_3d,
      .lifecycle_event = std::nullopt,
      .hold_position =
          Point3{navigation.state.x, navigation.state.y,
                 clampToFlightEnvelope(navigation.state.z, flight_envelope_config_)
                     .value_or(flight_envelope_config_.minimum_target_z_m)},
  };
  if (direct_tracking) {
    return result;
  }

  const ProductionActivatedRoute3D* const active_route = result.route.get();
  if (active_route == nullptr) {
    return result;
  }
  const std::span<const RouteSample3D> route =
      active_route->geometry.route
          ? std::span<const RouteSample3D>{*active_route->geometry.route}
          : std::span<const RouteSample3D>{};
  const StaticRouteObjective current_objective =
      objective != nullptr ? makeStaticRouteObjective(*objective)
                           : StaticRouteObjective{};
  const ObservedOccupancyGrid3D* const latest_occupancy =
      observed_3d_world && latest_raw_world && latest_raw_world->occupancy
          ? latest_raw_world->occupancy.get()
          : nullptr;
  const SweptFootprintConfig footprint{
      .radius_m = lattice_3d_config_.physical_footprint_radius_m,
      .lower_extent_m = lattice_3d_config_.physical_footprint_lower_extent_m,
      .upper_extent_m = lattice_3d_config_.physical_footprint_upper_extent_m,
      .perimeter_samples = physical_footprint_config_.perimeter_samples,
      .radial_rings = physical_footprint_config_.radial_rings,
      .axial_samples = physical_footprint_config_.axial_samples,
      .sweep_step_m = physical_footprint_config_.sweep_step_m,
  };
  RouteExecutionAssessment3D assessment;
  RouteExecutionState3D supervised_state;
  {
    const std::scoped_lock lock{route_supervisor_mutex_};
    const ActivatedRouteIdentity3D* const supervised_route =
        route_supervisor_.activeRoute();
    if (supervised_route == nullptr ||
        supervised_route->generation != active_route->identity.generation) {
      result.route.reset();
      return result;
    }
    assessment = route_supervisor_.assessExecution(
        route,
        RouteExecutionObservation3D{
            .current_objective = current_objective,
            .minimum_tracking_sample_sequence = minimum_tracking_sample_sequence,
            .position = {navigation.state.x, navigation.state.y, navigation.state.z},
            .maximum_cross_track_m = active_guide_config_.maximum_cross_track_m,
            .latest_raw_occupancy = latest_occupancy,
            .latest_raw_producer_instance_id =
                latest_raw_world ? latest_raw_world->producer_instance_id : 0U,
            .latest_raw_revision = latest_raw_world ? latest_raw_world->revision : 0U,
            .footprint = footprint,
            .proprioceptive_free_space_seed =
                world.proprioceptive_free_space_seed
                    ? std::addressof(*world.proprioceptive_free_space_seed)
                    : nullptr,
            .launch_support_contact =
                world.launch_support_contact
                    ? std::addressof(*world.launch_support_contact)
                    : nullptr,
        });
    supervised_state = route_supervisor_.executionState();
  }
  result.status = assessment.status;
  result.route_usable = assessment.usable();
  if (assessment.projection.valid) {
    result.projection = GlobalGuideProjection{
        .valid = true,
        .station_m = supervised_state.station_m,
        .total_length_m = route.empty() ? 0.0 : route.back().station_m,
        .remaining_m = route.empty() ? 0.0
                                     : std::max(0.0, route.back().station_m -
                                                         supervised_state.station_m),
        .cross_track_m = assessment.projection.distance_m,
        .point = {assessment.projection.point.x, assessment.projection.point.y},
    };
  }
  result.station_m = supervised_state.station_m;

  const std::uint64_t generation = active_route->identity.generation;
  if (assessment.status == RouteExecutionStatus3D::kRawCollision) {
    result.lifecycle_event = RouteLifecycleEvent3D{
        .kind = RouteLifecycleEventKind3D::kRawInvalidated,
        .generation = generation,
    };
    const std::uint64_t raw_revision =
        latest_raw_world ? latest_raw_world->revision : 0U;
    std::uint64_t no_blocked_revision{0U};
    observed_route_blocked_raw_revision_.compare_exchange_strong(
        no_blocked_revision, raw_revision, std::memory_order_release,
        std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ROUTE_EXECUTION3D status=raw_collision route_generation=%" PRIu64
        " validated_through_raw_revision=%" PRIu64 " latest_raw_revision=%" PRIu64
        " first_suffix_segment=%zu "
        "failure_segment=%zu failure=(%.2f,%.2f,%.2f) action=replan",
        generation, assessment.validated_through_raw_revision, raw_revision,
        assessment.raw_validation.first_validated_route_segment,
        assessment.raw_validation.failure_route_segment,
        assessment.raw_validation.failure_point.x,
        assessment.raw_validation.failure_point.y,
        assessment.raw_validation.failure_point.z);
    requestGuideRelease(GlobalGuideReleaseReason::kBlocked, generation);
  } else if (assessment.replacementRequired()) {
    const RouteLifecycleEventKind3D event_kind =
        assessment.status == RouteExecutionStatus3D::kObjectiveMismatch
            ? RouteLifecycleEventKind3D::kObjectiveSuperseded
            : RouteLifecycleEventKind3D::kCrossTrackExceeded;
    result.lifecycle_event =
        RouteLifecycleEvent3D{.kind = event_kind, .generation = generation};
    const StaticRouteObjective& route_objective =
        active_route->identity.proposal.objective;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ROUTE_EXECUTION3D status=%.*s route_generation=%" PRIu64
        " route_epoch=%" PRIu64 " current_epoch=%" PRIu64 " route_sample=%" PRIu64
        " current_sample=%" PRIu64 " required_sample=%" PRIu64
        " route_assignment_generation=%" PRIu64
        " current_assignment_generation=%" PRIu64 " route_target_track_id=%" PRIu64
        " current_target_track_id=%" PRIu64
        " cross_track_m=%.2f maximum_m=%.2f action=replan",
        static_cast<int>(routeExecutionStatus3DName(result.status).size()),
        routeExecutionStatus3DName(result.status).data(), generation,
        route_objective.mission_epoch,
        objective != nullptr ? objective->mission_epoch : 0U,
        route_objective.sample_sequence,
        objective != nullptr ? objective->sample_sequence : 0U,
        minimum_tracking_sample_sequence, route_objective.assignment_generation,
        objective != nullptr ? objective->assignment_generation : 0U,
        route_objective.target_track_id,
        objective != nullptr ? objective->target_track_id : 0U,
        assessment.projection.distance_m, active_guide_config_.maximum_cross_track_m);
    requestGuideRelease(event_kind == RouteLifecycleEventKind3D::kObjectiveSuperseded
                            ? GlobalGuideReleaseReason::kObjectiveChanged
                            : GlobalGuideReleaseReason::kDiverged,
                        generation);
  }
  if (result.lifecycle_event.has_value()) {
    static_cast<void>(execution_arbiter_.observe(*result.lifecycle_event));
    const std::scoped_lock lock{route_supervisor_mutex_};
    static_cast<void>(route_supervisor_.applyEvent(*result.lifecycle_event));
  }
  return result;
}

} // namespace drone_city_nav
