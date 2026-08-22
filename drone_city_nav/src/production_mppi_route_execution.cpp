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
    route_execution_state_3d_ = {};
    return result;
  }
  if (route_execution_state_3d_.generation != active_route->identity.generation) {
    route_execution_state_3d_ = RouteExecutionState3D{
        .generation = active_route->identity.generation,
        .raw_validated_through_revision =
            active_route->identity.proposal.validated_world
                .raw_validated_through_revision,
        .station_m = 0.0,
    };
  }
  const std::span<const RouteSample3D> route =
      active_route != nullptr && active_route->geometry.route
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
  const RouteExecutionAssessment3D assessment = assessRouteExecution3D(
      active_route != nullptr ? &active_route->identity : nullptr, route,
      RouteExecutionObservation3D{
          .current_objective = current_objective,
          .minimum_tracking_sample_sequence = minimum_tracking_sample_sequence,
          .previously_validated_through_raw_revision =
              route_execution_state_3d_.raw_validated_through_revision,
          .position = {navigation.state.x, navigation.state.y, navigation.state.z},
          .minimum_station_m = route_execution_state_3d_.station_m,
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
          .launch_support_contact = world.launch_support_contact
                                        ? std::addressof(*world.launch_support_contact)
                                        : nullptr,
      });
  result.status = assessment.status;
  result.route_usable = assessment.usable();
  route_execution_state_3d_.raw_validated_through_revision =
      std::max(route_execution_state_3d_.raw_validated_through_revision,
               assessment.validated_through_raw_revision);
  if (assessment.projection.valid) {
    route_execution_state_3d_.station_m =
        std::max(route_execution_state_3d_.station_m, assessment.projection.station_m);
    result.projection = GlobalGuideProjection{
        .valid = true,
        .station_m = route_execution_state_3d_.station_m,
        .total_length_m = route.empty() ? 0.0 : route.back().station_m,
        .remaining_m = route.empty()
                           ? 0.0
                           : std::max(0.0, route.back().station_m -
                                               route_execution_state_3d_.station_m),
        .cross_track_m = assessment.projection.distance_m,
        .point = {assessment.projection.point.x, assessment.projection.point.y},
    };
  }
  result.station_m = route_execution_state_3d_.station_m;

  const std::uint64_t generation =
      active_route != nullptr ? active_route->identity.generation : 0U;
  if (assessment.status == RouteExecutionStatus3D::kRawCollision) {
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
  } else if (assessment.status == RouteExecutionStatus3D::kWorldLineageMismatch ||
             assessment.status == RouteExecutionStatus3D::kExcessiveCrossTrack ||
             assessment.status == RouteExecutionStatus3D::kInvalidProjection) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ROUTE_EXECUTION3D status=%.*s route_generation=%" PRIu64
        " cross_track_m=%.2f maximum_m=%.2f action=replan",
        static_cast<int>(routeExecutionStatus3DName(result.status).size()),
        routeExecutionStatus3DName(result.status).data(), generation,
        assessment.projection.distance_m, active_guide_config_.maximum_cross_track_m);
    requestGuideRelease(GlobalGuideReleaseReason::kObjectiveChanged, generation);
  } else if (result.status == RouteExecutionStatus3D::kObjectiveMismatch &&
             objective != nullptr && objective->continuous_tracking) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ROUTE_EXECUTION3D status=objective_mismatch route_generation=%" PRIu64
        " current_epoch=%" PRIu64 " current_sample=%" PRIu64
        " required_sample=%" PRIu64,
        generation, objective->mission_epoch, objective->sample_sequence,
        minimum_tracking_sample_sequence);
  }
  return result;
}

} // namespace drone_city_nav
