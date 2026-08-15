#include "drone_city_nav/mppi/static_route_handoff.hpp"

#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/mppi/mppi_route_projection.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
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
    const BenchmarkConfig& config, const EsdfGrid& grid,
    const std::span<const float> esdf_m) {
  StaticRouteHandoffResult result;
  if (candidate_route.size() < 2U || config.steps < 2U ||
      !(config.dynamics.dt_s > 0.0F) || !(reference_speed_mps >= 0.0F) ||
      !(maximum_cross_track_m > 0.0F) || !std::isfinite(reference_speed_mps) ||
      !std::isfinite(maximum_cross_track_m) || !validGrid(grid) ||
      esdf_m.size() != gridCellCount(grid)) {
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
  const std::vector<Control> controls = buildRouteDirectedCruiseSeed(
      current_state, target, candidate_route, projection.station_m, handoff_speed_mps,
      config.dynamics, config.steps, previous_applied_control);
  const std::vector<Control> zero_noise(config.steps);
  const RolloutMetrics metrics = simulateReference(
      current_state, controls, zero_noise, config.dynamics, config.risk, config.costs,
      grid, esdf_m, target.x, target.y, true, previous_applied_control,
      handoff_speed_mps, config.footprint, std::nullopt, nullptr, {}, std::nullopt,
      config.cooperative, std::nullopt, config.altitude_envelope);
  result.minimum_clearance_m = metrics.minimum_clearance_m;
  result.critical_exposure_m = metrics.critical_exposure_m;
  result.planning_exposure_m = metrics.planning_exposure_m;
  if (metrics.altitude_envelope_violation) {
    result.status = StaticRouteHandoffStatus::kAltitudeEnvelopeViolation;
    return result;
  }
  if (metrics.collision) {
    result.status = StaticRouteHandoffStatus::kRawCollision;
    return result;
  }
  result.status = StaticRouteHandoffStatus::kAccepted;
  result.accepted = true;
  return result;
}

const char*
staticRouteHandoffStatusName(const StaticRouteHandoffStatus status) noexcept {
  switch (status) {
    case StaticRouteHandoffStatus::kAccepted:
      return "accepted";
    case StaticRouteHandoffStatus::kInvalidInput:
      return "invalid_input";
    case StaticRouteHandoffStatus::kInvalidProjection:
      return "invalid_projection";
    case StaticRouteHandoffStatus::kExcessiveCrossTrack:
      return "excessive_cross_track";
    case StaticRouteHandoffStatus::kAltitudeEnvelopeViolation:
      return "altitude_envelope_violation";
    case StaticRouteHandoffStatus::kRawCollision:
      return "raw_collision";
  }
  return "unknown";
}

} // namespace drone_city_nav::mppi
