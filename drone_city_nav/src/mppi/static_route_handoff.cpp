#include "drone_city_nav/mppi/static_route_handoff.hpp"

#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/mppi/mppi_route_projection.hpp"
#include "drone_city_nav/mppi/trajectory_reference_adapter_3d.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

[[nodiscard]] bool validGrid(const EsdfGrid& grid) noexcept {
  return grid.width > 0 && grid.height > 0 && grid.depth > 0 &&
         std::isfinite(grid.resolution_m) && grid.resolution_m > 0.0F &&
         std::isfinite(grid.origin_x_m) && std::isfinite(grid.origin_y_m) &&
         std::isfinite(grid.origin_z_m);
}

[[nodiscard]] std::size_t gridCellCount(const EsdfGrid& grid) noexcept {
  return static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height) *
         static_cast<std::size_t>(grid.depth);
}

} // namespace

StaticRouteHandoffResult validateStaticRouteHandoff(
    const State& current_state, const Control previous_applied_control,
    const std::span<const RouteSample3D> candidate_route,
    const float reference_speed_mps, const float maximum_cross_track_m,
    const float terminal_cross_track_tolerance_m, const BenchmarkConfig& config,
    const EsdfGrid& grid, const std::span<const float> esdf_m) {
  StaticRouteHandoffResult result;
  if (candidate_route.size() < 2U || config.steps < 2U ||
      !(config.dynamics.dt_s > 0.0F) || !(reference_speed_mps >= 0.0F) ||
      !(maximum_cross_track_m > 0.0F) || !std::isfinite(reference_speed_mps) ||
      !std::isfinite(maximum_cross_track_m) ||
      !(terminal_cross_track_tolerance_m > 0.0F) ||
      !std::isfinite(terminal_cross_track_tolerance_m) || !validGrid(grid) ||
      esdf_m.size() != gridCellCount(grid)) {
    result.status = StaticRouteHandoffStatus::kInvalidInput;
    return result;
  }

  const MppiRouteProjection3D projection =
      projectOntoMppiRoute3D(current_state, candidate_route, 0.0F);
  if (!projection.valid) {
    result.status = StaticRouteHandoffStatus::kInvalidProjection;
    return result;
  }
  result.cross_track_m = projection.distance_m;
  if (projection.distance_m > maximum_cross_track_m) {
    result.status = StaticRouteHandoffStatus::kExcessiveCrossTrack;
    return result;
  }

  const float current_horizontal_speed_mps =
      std::hypot(current_state.vx, current_state.vy);
  const float handoff_speed_mps =
      std::min(reference_speed_mps, current_horizontal_speed_mps);
  const RouteSample3D& endpoint = candidate_route.back();
  const State target{.x = endpoint.x_m, .y = endpoint.y_m, .z = endpoint.z_m};
  const std::vector<Control> controls = buildFiniteRouteDirectedSeed(
      current_state, target, candidate_route, projection.station_m, handoff_speed_mps,
      config.dynamics, config.steps, previous_applied_control,
      config.stopping_capability);
  std::vector<State> planned_states;
  planned_states.reserve(controls.size() + 1U);
  planned_states.push_back(current_state);
  for (const Control& control : controls) {
    planned_states.push_back(
        integrateReference(planned_states.back(), control, config.dynamics));
  }
  RouteConvergentFiniteHorizon finite_route = buildRouteConvergentFiniteHorizon(
      planned_states, controls, previous_applied_control, config.dynamics,
      candidate_route, projection.station_m, terminal_cross_track_tolerance_m,
      finiteHorizonArrivalSearchStepControls(config.dynamics.dt_s),
      makeFiniteHorizonConfig(config.stopping_capability));
  result.terminal_cross_track_m = finite_route.closest_terminal_cross_track_m;
  result.arrival_shaping_attempts = finite_route.arrival_shaping_attempts;
  result.nominal_prefix_control_count = finite_route.nominal_prefix_control_count;
  if (!finite_route.horizon.has_value()) {
    result.status = StaticRouteHandoffStatus::kNoRouteConvergentFiniteHorizon;
    return result;
  }
  const std::vector<Control>& finite_controls = finite_route.horizon.value().controls;
  const std::vector<Control> zero_noise(config.steps);
  const RolloutMetrics metrics = simulateReference(
      current_state, finite_controls, zero_noise, config.dynamics, config.risk,
      config.costs, grid, esdf_m, target.x, target.y, true, previous_applied_control,
      handoff_speed_mps, config.footprint, std::nullopt, nullptr, {}, std::nullopt,
      config.cooperative, std::nullopt, config.altitude_envelope, target.z);
  result.minimum_clearance_m = metrics.minimum_clearance_m;
  result.critical_exposure_m = metrics.critical_exposure_m;
  result.planning_exposure_m = metrics.planning_exposure_m;
  if (metrics.altitude_envelope_violation) {
    result.status = StaticRouteHandoffStatus::kAltitudeEnvelopeViolation;
    return result;
  }
  result.status = StaticRouteHandoffStatus::kAccepted;
  return result;
}

const char*
staticRouteHandoffStatusName(const StaticRouteHandoffStatus status) noexcept {
  return dynamicHandoffStatus3DName(status);
}

DynamicHandoffValidator3D makeMppiDynamicHandoffValidator3D(BenchmarkConfig config) {
  return [config = std::move(config)](const DynamicHandoffRequest3D& request) {
    if (request.candidate_trajectory == nullptr ||
        request.derived_distances_m == nullptr) {
      return DynamicHandoffResult3D{.status = DynamicHandoffStatus3D::kInvalidInput};
    }
    const std::shared_ptr<const std::vector<RouteSample3D>> reference =
        adaptTrajectoryReference3D(*request.candidate_trajectory);
    if (reference == nullptr) {
      return DynamicHandoffResult3D{.status = DynamicHandoffStatus3D::kInvalidInput};
    }
    return validateStaticRouteHandoff(
        request.current_state, request.previous_applied_control, *reference,
        request.reference_speed_mps, request.maximum_cross_track_m,
        request.terminal_cross_track_tolerance_m, config, request.grid,
        *request.derived_distances_m);
  };
}

} // namespace drone_city_nav::mppi
