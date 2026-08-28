#include "drone_city_nav/execution_route_snapshot_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {

using namespace execution_route_snapshot_3d_internal;

namespace execution_route_snapshot_3d_internal {

bool certifiedTrackingTubeHandoffPending(
    const ExecutionRouteSnapshot3D& current,
    const CertifiedRouteSuffix3D& target_route) noexcept {
  const FiniteExecutionState3D* const execution =
      current.finite_execution.has_value() ? &*current.finite_execution : nullptr;
  // Retention recertifies the same published handoff while the vehicle is still
  // converging to the route. The connector provenance ends naturally once the
  // resident execution begins inside the route corridor; emergency tails do not
  // inherit it.
  if (execution == nullptr ||
      (execution->kind != FiniteExecutionKind3D::kNominal &&
       execution->kind != FiniteExecutionKind3D::kRetained) ||
      execution->source_route_instance_id != target_route.route_instance_id ||
      execution->source_route_generation != target_route.identity.generation ||
      execution->source_geometry_revision !=
          target_route.geometry->executable_geometry_revision ||
      execution->horizon == nullptr || execution->horizon->states.empty() ||
      target_route.progress.execution_input != execution->execution_input) {
    return false;
  }
  const mppi::State& initial_state = execution->horizon->states.front();
  const RouteProjection3D initial_projection = projectOntoRoute3DWithinStationWindow(
      *target_route.geometry->route,
      Point3{initial_state.x, initial_state.y, initial_state.z},
      std::max(certificateView(target_route.certificate).suffix_start_station_m,
               execution->begin_route_station_m - kExecutionBindingToleranceM),
      std::min(target_route.endStationM(),
               execution->begin_route_station_m + kExecutionBindingToleranceM));
  if (!initial_projection.valid) {
    return false;
  }
  const TrackingErrorTubeExecutionAssessment3D tube =
      assessTrackingErrorTubeExecution3D(
          *target_route.geometry->route, *target_route.geometry->tracking_error_tube,
          TrackingErrorTubeExecutionObservation3D{
              .station_m = initial_projection.station_m,
              .cross_track_error_m = initial_projection.distance_m,
              .speed_mps = std::hypot(std::hypot(static_cast<double>(initial_state.vx),
                                                 static_cast<double>(initial_state.vy)),
                                      static_cast<double>(initial_state.vz)),
          });
  if (tube.status != TrackingErrorTubeExecutionStatus3D::kSpeedLimitExceeded &&
      tube.status != TrackingErrorTubeExecutionStatus3D::kCrossTrackExceeded) {
    return false;
  }
  const mppi::State& terminal_state = execution->horizon->states.back();
  const RouteProjection3D terminal_projection = projectOntoRoute3DWithinStationWindow(
      *target_route.geometry->route,
      Point3{terminal_state.x, terminal_state.y, terminal_state.z},
      execution->begin_route_station_m, target_route.endStationM());
  return terminal_projection.valid &&
         assessTrackingErrorTubeExecution3D(
             *target_route.geometry->route, *target_route.geometry->tracking_error_tube,
             TrackingErrorTubeExecutionObservation3D{
                 .station_m = terminal_projection.station_m,
                 .cross_track_error_m = terminal_projection.distance_m,
                 .speed_mps =
                     std::hypot(std::hypot(static_cast<double>(terminal_state.vx),
                                           static_cast<double>(terminal_state.vy)),
                                static_cast<double>(terminal_state.vz)),
             })
             .accepted();
}

bool validateTrackingTubeHandoffClearance(
    const CertifiedRouteSuffix3D& route, const mppi::FiniteHorizon& horizon,
    const double begin_route_station_m, const mppi::Control& previous_control,
    const mppi::FiniteExecutionPathWorld& world) noexcept {
  if (route.geometry == nullptr || route.geometry->route == nullptr ||
      route.geometry->tracking_error_tube == nullptr || horizon.states.size() < 2U ||
      horizon.controls.size() + 1U != horizon.states.size() ||
      world.footprint == nullptr ||
      (world.observed_occupancy == nullptr) == (world.static_occupancy == nullptr)) {
    return false;
  }
  const std::span<const RouteSample3D> route_samples{*route.geometry->route};
  const TrackingErrorTubeProfile3D& profile = *route.geometry->tracking_error_tube;
  const auto speed_mps = [](const mppi::State& state) {
    return std::hypot(
        std::hypot(static_cast<double>(state.vx), static_cast<double>(state.vy)),
        static_cast<double>(state.vz));
  };
  const auto assessment = [&](const mppi::State& state,
                              const RouteProjection3D& projection) {
    return assessTrackingErrorTubeExecution3D(
        route_samples, profile,
        TrackingErrorTubeExecutionObservation3D{
            .station_m = projection.station_m,
            .cross_track_error_m = projection.distance_m,
            .speed_mps = speed_mps(state),
        });
  };

  Point3 previous_position{horizon.states.front().x, horizon.states.front().y,
                           horizon.states.front().z};
  RouteProjection3D previous_projection = projectOntoRoute3DWithinStationWindow(
      route_samples, previous_position,
      std::max(route_samples.front().station_m,
               begin_route_station_m - kExecutionBindingToleranceM),
      std::min(route.endStationM(),
               begin_route_station_m + kExecutionBindingToleranceM));
  if (!previous_projection.valid) {
    return false;
  }
  if (assessment(horizon.states.front(), previous_projection).accepted()) {
    return true;
  }

  for (std::size_t index = 1U; index < horizon.states.size(); ++index) {
    const mppi::State& previous_state = horizon.states[index - 1U];
    const mppi::State& state = horizon.states[index];
    const Point3 position{state.x, state.y, state.z};
    const double travel_m = distance3D(previous_position, position);
    if (!std::isfinite(travel_m)) {
      return false;
    }
    const RouteProjection3D projection = projectOntoRoute3DWithinStationWindow(
        route_samples, position,
        std::max(route_samples.front().station_m,
                 previous_projection.station_m - kStationToleranceM),
        std::min(route.endStationM(),
                 previous_projection.station_m + travel_m + kStationToleranceM));
    if (!projection.valid ||
        projection.station_m + kStationToleranceM < previous_projection.station_m) {
      return false;
    }

    const double reference_speed_limit_mps =
        std::max(speed_mps(previous_state), speed_mps(state));
    const double tracking_radius_m =
        trackingErrorTubeRadiusM(profile.config, reference_speed_limit_mps);
    if (!std::isfinite(tracking_radius_m)) {
      return false;
    }
    SweptFootprintConfig inflated = *world.footprint;
    inflated.radius_m += tracking_radius_m;
    inflated.lower_extent_m += tracking_radius_m;
    inflated.upper_extent_m += tracking_radius_m;
    if (tracking_radius_m > 0.0) {
      inflated.perimeter_samples =
          std::max<std::size_t>(8U, inflated.perimeter_samples);
      inflated.radial_rings = std::max<std::size_t>(1U, inflated.radial_rings);
      inflated.axial_samples = std::max<std::size_t>(1U, inflated.axial_samples);
    }
    const mppi::Control& start_control =
        index == 1U ? previous_control : horizon.controls[index - 2U];
    const mppi::Control& stop_control = horizon.controls[index - 1U];
    const FootprintBodyAxis start_axis = bodyAxisFromWorldAcceleration(
        Vec3{start_control.ax, start_control.ay, start_control.az});
    const FootprintBodyAxis stop_axis = bodyAxisFromWorldAcceleration(
        Vec3{stop_control.ax, stop_control.ay, stop_control.az});
    const bool occupancy_safe =
        world.observed_occupancy != nullptr
            ? validateObservedSweptFootprint(
                  *world.observed_occupancy, previous_position, start_axis, position,
                  stop_axis, inflated, ObservedSpaceValidationPolicy::kAllowUnknown,
                  world.proprioceptive_free_space_seed, world.launch_support_contact)
                  .accepted()
            : validateKnownStaticSweptFootprint(*world.static_occupancy,
                                                previous_position, start_axis, position,
                                                stop_axis, inflated)
                  .accepted();
    const bool lidar_safe =
        world.latest_lidar_obstacle_points.empty() ||
        validateRawPointCloudSweptFootprint(
            world.latest_lidar_obstacle_points, previous_position, start_axis, position,
            stop_axis, inflated, world.launch_support_contact)
            .accepted();
    if (!occupancy_safe || !lidar_safe) {
      return false;
    }
    if (assessment(state, projection).accepted()) {
      return true;
    }
    previous_position = position;
    previous_projection = projection;
  }
  return false;
}

} // namespace execution_route_snapshot_3d_internal

TrackingErrorTubeHandoffAssessment3D assessCertifiedTrackingTubeHandoff3D(
    const ExecutionRouteSnapshot3D& current, const CertifiedRouteSuffix3D& target_route,
    const VersionedExecutionInput3D& current_execution_input) noexcept {
  if (!current_execution_input.valid() ||
      !current_execution_input.nominalStateAuthoritative() ||
      !current.finite_execution.has_value() ||
      current.finite_execution->horizon == nullptr ||
      current.finite_execution->horizon->states.empty() ||
      (current.finite_execution->kind != FiniteExecutionKind3D::kNominal &&
       current.finite_execution->kind != FiniteExecutionKind3D::kRetained) ||
      !current.finite_execution->validFor(&target_route) ||
      target_route.progress.execution_input == nullptr ||
      target_route.progress.execution_input !=
          current.finite_execution->execution_input ||
      executionInputProgressRelation(current_execution_input,
                                     *target_route.progress.execution_input) !=
          ExecutionInputProgressRelation3D::kStrictlyNewer ||
      !executionInputFreshAt(current_execution_input, *target_route.validation_policy,
                             current_execution_input.effectiveStampNs())) {
    return {};
  }
  const FiniteExecutionState3D& execution = *current.finite_execution;
  const mppi::State& progress_state = target_route.progress.execution_input->state();
  const RouteProjection3D progress_projection = projectOntoRoute3DWithinStationWindow(
      *target_route.geometry->route,
      Point3{progress_state.x, progress_state.y, progress_state.z},
      std::max(target_route.geometry->route->front().station_m,
               target_route.progress.station_m - kStationToleranceM),
      std::min(target_route.endStationM(),
               target_route.progress.station_m + kStationToleranceM));
  if (!progress_projection.valid) {
    return {};
  }
  const TrackingErrorTubeExecutionAssessment3D progress_tube =
      assessTrackingErrorTubeExecution3D(
          *target_route.geometry->route, *target_route.geometry->tracking_error_tube,
          TrackingErrorTubeExecutionObservation3D{
              .station_m = progress_projection.station_m,
              .cross_track_error_m = progress_projection.distance_m,
              .speed_mps =
                  std::hypot(std::hypot(static_cast<double>(progress_state.vx),
                                        static_cast<double>(progress_state.vy)),
                             static_cast<double>(progress_state.vz)),
          });
  if (progress_tube.accepted()) {
    return TrackingErrorTubeHandoffAssessment3D{
        .status = TrackingErrorTubeHandoffStatus3D::kNotRequired};
  }
  return assessTrackingErrorTubeHandoff3D(
      *target_route.geometry->route, *target_route.geometry->tracking_error_tube,
      execution.horizon->states, execution.begin_route_station_m,
      execution.valid_from_ns, execution.valid_until_ns, execution.control_interval_ns,
      TrackingErrorTubeHandoffObservation3D{
          .stamp_ns = current_execution_input.effectiveStampNs(),
          .state = current_execution_input.state(),
      });
}

} // namespace drone_city_nav
