#include "drone_city_nav/mppi/finite_execution_path.hpp"

#include "drone_city_nav/mppi/mppi_altitude_envelope.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <ranges>
#include <vector>

namespace drone_city_nav::mppi {
namespace {

constexpr double kNanosecondsPerSecond{1.0e9};
constexpr double kTerminalRestTolerance{1.0e-3};

[[nodiscard]] bool finite(const State& state) noexcept {
  return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.z) &&
         std::isfinite(state.vx) && std::isfinite(state.vy) &&
         std::isfinite(state.vz) && std::isfinite(state.yaw) &&
         std::isfinite(state.yaw_rate);
}

[[nodiscard]] bool finite(const Control& control) noexcept {
  return std::isfinite(control.ax) && std::isfinite(control.ay) &&
         std::isfinite(control.az) && std::isfinite(control.yaw_accel);
}

[[nodiscard]] Point3 position(const State& state) noexcept {
  return Point3{state.x, state.y, state.z};
}

[[nodiscard]] FootprintBodyAxis bodyAxis(const Control& control) noexcept {
  return bodyAxisFromWorldAcceleration(Vec3{control.ax, control.ay, control.az});
}

[[nodiscard]] bool validWorld(const FiniteExecutionPathWorld& world) noexcept {
  if (world.flight_envelope == nullptr || world.dynamics == nullptr ||
      world.altitude_envelope == nullptr || world.footprint == nullptr) {
    return false;
  }
  if (!world.terminal_boundary.has_value()) {
    return true;
  }
  const FiniteExecutionPathTerminalBoundary& boundary = *world.terminal_boundary;
  const double forward_norm = std::hypot(
      std::hypot(boundary.forward.x, boundary.forward.y), boundary.forward.z);
  return std::isfinite(boundary.endpoint.x) && std::isfinite(boundary.endpoint.y) &&
         std::isfinite(boundary.endpoint.z) && std::isfinite(boundary.forward.x) &&
         std::isfinite(boundary.forward.y) && std::isfinite(boundary.forward.z) &&
         std::isfinite(boundary.tolerance_m) && boundary.tolerance_m >= 0.0 &&
         forward_norm > 1.0e-6;
}

[[nodiscard]] bool withinTerminalBoundary(
    const State& state,
    const std::optional<FiniteExecutionPathTerminalBoundary>& boundary) noexcept {
  if (!boundary.has_value()) {
    return true;
  }
  const double forward_norm = std::hypot(
      std::hypot(boundary->forward.x, boundary->forward.y), boundary->forward.z);
  const double signed_distance_m =
      ((static_cast<double>(state.x) - boundary->endpoint.x) * boundary->forward.x +
       (static_cast<double>(state.y) - boundary->endpoint.y) * boundary->forward.y +
       (static_cast<double>(state.z) - boundary->endpoint.z) * boundary->forward.z) /
      forward_norm;
  return signed_distance_m <= boundary->tolerance_m;
}

[[nodiscard]] FiniteExecutionPathValidation
reject(const FiniteExecutionPathStatus status, const std::size_t first_remaining_index,
       const std::size_t segment_index, const Point3& failure_point,
       const double remaining_duration_s) noexcept {
  return FiniteExecutionPathValidation{
      .status = status,
      .first_remaining_point_index = first_remaining_index,
      .failure_segment_index = segment_index,
      .failure_point = failure_point,
      .remaining_duration_s = remaining_duration_s,
  };
}

[[nodiscard]] FiniteExecutionPathStatus
validatePhysicalSegment(const Point3& first, const FootprintBodyAxis& first_axis,
                        const Point3& second, const FootprintBodyAxis& second_axis,
                        const FiniteExecutionPathWorld& world,
                        Point3& failure_point) noexcept {
  SweptFootprintResult raw_validation;
  if (world.static_occupancy != nullptr) {
    raw_validation =
        validateRawSweptFootprint(*world.static_occupancy, first, first_axis, second,
                                  second_axis, *world.footprint);
  } else if (world.raw_occupancy != nullptr) {
    raw_validation = validateRawSweptFootprint(*world.raw_occupancy, first, second,
                                               *world.footprint);
  } else {
    failure_point = first;
    return FiniteExecutionPathStatus::kRawWorldUnavailable;
  }
  if (!raw_validation.accepted()) {
    failure_point = raw_validation.failure_point;
    return FiniteExecutionPathStatus::kRawCollision;
  }
  if (world.latest_lidar_obstacle_points.empty()) {
    return FiniteExecutionPathStatus::kValid;
  }
  const SweptFootprintResult lidar_validation = validateRawPointCloudSweptFootprint(
      world.latest_lidar_obstacle_points, first, first_axis, second, second_axis,
      *world.footprint);
  if (!lidar_validation.accepted()) {
    failure_point = lidar_validation.failure_point;
    return FiniteExecutionPathStatus::kLatestLidarRawCollision;
  }
  return FiniteExecutionPathStatus::kValid;
}

[[nodiscard]] FiniteExecutionPathValidation
validatePathContract(const std::span<const TimedExecutionPathPoint> points) noexcept {
  if (points.size() < 2U) {
    return {};
  }
  double previous_time_s{-std::numeric_limits<double>::infinity()};
  for (const TimedExecutionPathPoint& point : points) {
    if (!std::isfinite(point.time_from_start_s) || point.time_from_start_s < 0.0 ||
        point.time_from_start_s <= previous_time_s || !finite(point.state) ||
        !finite(point.control)) {
      return reject(FiniteExecutionPathStatus::kInvalidContract, 0U, 0U,
                    position(point.state), 0.0);
    }
    previous_time_s = point.time_from_start_s;
  }
  const TimedExecutionPathPoint& terminal = points.back();
  if (std::hypot(std::hypot(terminal.state.vx, terminal.state.vy), terminal.state.vz) >
          kTerminalRestTolerance ||
      std::abs(terminal.state.yaw_rate) > kTerminalRestTolerance ||
      std::hypot(std::hypot(terminal.control.ax, terminal.control.ay),
                 terminal.control.az) > kTerminalRestTolerance ||
      std::abs(terminal.control.yaw_accel) > kTerminalRestTolerance) {
    return reject(FiniteExecutionPathStatus::kInvalidContract, 0U, points.size() - 1U,
                  position(terminal.state), 0.0);
  }
  return FiniteExecutionPathValidation{
      .status = FiniteExecutionPathStatus::kValid,
  };
}

[[nodiscard]] FiniteExecutionPathStatus
validateAltitudeState(const State& state, const Control& applied_control,
                      const FiniteExecutionPathWorld& world) noexcept {
  if (!insideFlightEnvelope(state.z, *world.flight_envelope)) {
    return FiniteExecutionPathStatus::kFlightEnvelopeViolation;
  }
  if (!altitudeEnvelopeDynamicallyRecoverable(state, applied_control, *world.dynamics,
                                              *world.altitude_envelope)) {
    return FiniteExecutionPathStatus::kDynamicFlightEnvelopeViolation;
  }
  return FiniteExecutionPathStatus::kValid;
}

[[nodiscard]] std::vector<TimedExecutionPathPoint>
timedPathPoints(const FiniteHorizon& horizon, const float dt_s) {
  std::vector<TimedExecutionPathPoint> points;
  if (horizon.states.size() != horizon.controls.size() + 1U ||
      horizon.controls.empty() || !(dt_s > 0.0F)) {
    return points;
  }
  points.reserve(horizon.states.size());
  for (std::size_t index = 0U; index < horizon.states.size(); ++index) {
    points.push_back(TimedExecutionPathPoint{
        .time_from_start_s = static_cast<double>(index) * dt_s,
        .state = horizon.states[index],
        .control = horizon.controls[std::min(index, horizon.controls.size() - 1U)],
    });
  }
  return points;
}

} // namespace

FiniteExecutionPathValidation validateCompleteFiniteExecutionPath(
    const std::span<const TimedExecutionPathPoint> points,
    const Control& previous_applied_control,
    const FiniteExecutionPathWorld& world) noexcept {
  if (!validWorld(world) || !finite(previous_applied_control)) {
    return {};
  }
  const FiniteExecutionPathValidation contract = validatePathContract(points);
  if (!contract.accepted()) {
    return contract;
  }

  FiniteExecutionPathStatus altitude_status =
      validateAltitudeState(points.front().state, previous_applied_control, world);
  if (altitude_status != FiniteExecutionPathStatus::kValid) {
    return reject(altitude_status, 0U, 0U, position(points.front().state), 0.0);
  }
  if (!withinTerminalBoundary(points.front().state, world.terminal_boundary)) {
    return reject(FiniteExecutionPathStatus::kRouteEndpointExceeded, 0U, 0U,
                  position(points.front().state), 0.0);
  }

  Point3 failure_point{};
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const TimedExecutionPathPoint& first = points[index - 1U];
    const TimedExecutionPathPoint& second = points[index];
    altitude_status = validateAltitudeState(second.state, first.control, world);
    if (altitude_status != FiniteExecutionPathStatus::kValid) {
      return reject(altitude_status, 0U, index - 1U, position(second.state), 0.0);
    }
    if (!withinTerminalBoundary(second.state, world.terminal_boundary)) {
      return reject(FiniteExecutionPathStatus::kRouteEndpointExceeded, 0U, index - 1U,
                    position(second.state), 0.0);
    }
    const FiniteExecutionPathStatus segment_status = validatePhysicalSegment(
        position(first.state),
        bodyAxis(index == 1U ? previous_applied_control : points[index - 2U].control),
        position(second.state), bodyAxis(first.control), world, failure_point);
    if (segment_status != FiniteExecutionPathStatus::kValid) {
      return reject(segment_status, 0U, index - 1U, failure_point, 0.0);
    }
  }
  return FiniteExecutionPathValidation{
      .status = FiniteExecutionPathStatus::kValid,
  };
}

ValidatedFiniteExecutionPath
buildValidatedFiniteExecutionPath(const std::span<const State> planned_states,
                                  const std::span<const Control> planned_controls,
                                  const Control& previous_applied_control,
                                  const DynamicsConfig& dynamics,
                                  const std::size_t arrival_search_step_controls,
                                  const FiniteHorizonConfig& finite_horizon_config,
                                  const FiniteExecutionPathWorld& world) {
  ValidatedFiniteExecutionPath result;
  if (!validWorld(world) || !finite(previous_applied_control) ||
      planned_states.size() != planned_controls.size() + 1U ||
      planned_controls.empty() || arrival_search_step_controls == 0U) {
    return result;
  }

  std::size_t nominal_prefix_control_count = planned_controls.size();
  while (true) {
    ++result.arrival_shaping_attempts;
    std::optional<FiniteHorizon> candidate = buildFiniteHorizon(
        planned_states, planned_controls, nominal_prefix_control_count, dynamics,
        previous_applied_control, finite_horizon_config);
    if (candidate.has_value()) {
      result.validation = validateCompleteFiniteExecutionPath(
          timedPathPoints(*candidate, dynamics.dt_s), previous_applied_control, world);
      if (result.validation.accepted()) {
        result.horizon = std::move(candidate);
        return result;
      }
      result.path_validation_backoff = true;
      result.latest_lidar_path_validation_backoff |=
          result.validation.status ==
          FiniteExecutionPathStatus::kLatestLidarRawCollision;
    } else {
      result.validation =
          reject(FiniteExecutionPathStatus::kInvalidContract, 0U,
                 nominal_prefix_control_count, position(planned_states.front()), 0.0);
    }
    if (nominal_prefix_control_count == 0U) {
      return result;
    }
    nominal_prefix_control_count =
        nominal_prefix_control_count > arrival_search_step_controls
            ? nominal_prefix_control_count - arrival_search_step_controls
            : 0U;
  }
}

FiniteExecutionPathValidation validateFiniteExecutionTrajectoryContinuation(
    const std::span<const TimedExecutionPathPoint> points,
    const std::int64_t valid_from_ns, const std::int64_t valid_until_ns,
    const std::int64_t now_ns, const State& current_state,
    const Control& current_control, const FiniteExecutionPathWorld& world) noexcept {
  if (!validWorld(world) || points.size() < 2U || valid_from_ns <= 0 ||
      valid_until_ns <= valid_from_ns || !finite(current_state) ||
      !finite(current_control)) {
    return {};
  }
  const double remaining_duration_s =
      static_cast<double>(valid_until_ns - now_ns) / kNanosecondsPerSecond;
  if (now_ns < valid_from_ns || now_ns >= valid_until_ns) {
    return reject(FiniteExecutionPathStatus::kNotActive, 0U, 0U,
                  position(current_state), remaining_duration_s);
  }

  FiniteExecutionPathValidation contract = validatePathContract(points);
  if (!contract.accepted()) {
    contract.remaining_duration_s = remaining_duration_s;
    return contract;
  }
  const TimedExecutionPathPoint& terminal = points.back();

  const double elapsed_s =
      static_cast<double>(now_ns - valid_from_ns) / kNanosecondsPerSecond;
  const auto first_remaining =
      std::ranges::find_if(points, [elapsed_s](const TimedExecutionPathPoint& point) {
        return point.time_from_start_s >= elapsed_s;
      });
  if (first_remaining == points.end()) {
    return reject(FiniteExecutionPathStatus::kInvalidContract, points.size(),
                  points.size() - 1U, position(terminal.state), remaining_duration_s);
  }
  const std::size_t first_remaining_index =
      static_cast<std::size_t>(std::distance(points.begin(), first_remaining));
  FiniteExecutionPathStatus altitude_status =
      validateAltitudeState(current_state, current_control, world);
  if (altitude_status != FiniteExecutionPathStatus::kValid) {
    return reject(altitude_status, first_remaining_index, first_remaining_index,
                  position(current_state), remaining_duration_s);
  }
  if (!withinTerminalBoundary(current_state, world.terminal_boundary)) {
    return reject(FiniteExecutionPathStatus::kRouteEndpointExceeded,
                  first_remaining_index, first_remaining_index, position(current_state),
                  remaining_duration_s);
  }
  for (std::size_t index = first_remaining_index; index < points.size(); ++index) {
    const Control& applied_control =
        index == 0U ? current_control : points[index - 1U].control;
    altitude_status =
        validateAltitudeState(points[index].state, applied_control, world);
    if (altitude_status != FiniteExecutionPathStatus::kValid) {
      return reject(altitude_status, first_remaining_index, index,
                    position(points[index].state), remaining_duration_s);
    }
    if (!withinTerminalBoundary(points[index].state, world.terminal_boundary)) {
      return reject(FiniteExecutionPathStatus::kRouteEndpointExceeded,
                    first_remaining_index, index, position(points[index].state),
                    remaining_duration_s);
    }
  }

  Point3 failure_point{};
  FiniteExecutionPathStatus segment_status =
      validatePhysicalSegment(position(current_state), bodyAxis(current_control),
                              position(first_remaining->state),
                              bodyAxis(first_remaining->control), world, failure_point);
  if (segment_status != FiniteExecutionPathStatus::kValid) {
    return reject(segment_status, first_remaining_index, first_remaining_index,
                  failure_point, remaining_duration_s);
  }
  for (std::size_t index = first_remaining_index + 1U; index < points.size(); ++index) {
    const TimedExecutionPathPoint& first = points[index - 1U];
    const TimedExecutionPathPoint& second = points[index];
    segment_status = validatePhysicalSegment(
        position(first.state), bodyAxis(first.control), position(second.state),
        bodyAxis(second.control), world, failure_point);
    if (segment_status != FiniteExecutionPathStatus::kValid) {
      return reject(segment_status, first_remaining_index, index - 1U, failure_point,
                    remaining_duration_s);
    }
  }

  return FiniteExecutionPathValidation{
      .status = FiniteExecutionPathStatus::kValid,
      .first_remaining_point_index = first_remaining_index,
      .remaining_duration_s = remaining_duration_s,
  };
}

FiniteExecutionPathValidation validateFiniteExecutionPathContinuation(
    const std::span<const TimedExecutionPathPoint> points,
    const std::int64_t valid_from_ns, const std::int64_t valid_until_ns,
    const std::int64_t now_ns, const State& current_state,
    const Control& current_control, const FiniteExecutionPathWorld& world) noexcept {
  const FiniteExecutionPathValidation trajectory_validation =
      validateFiniteExecutionTrajectoryContinuation(
          points, valid_from_ns, valid_until_ns, now_ns, current_state, current_control,
          world);
  if (!trajectory_validation.accepted()) {
    return trajectory_validation;
  }

  const std::int64_t control_interval_ns =
      finitePathControlIntervalNanoseconds(world.dynamics->dt_s);
  if (control_interval_ns <= 0) {
    return reject(FiniteExecutionPathStatus::kInvalidContract,
                  trajectory_validation.first_remaining_point_index,
                  trajectory_validation.first_remaining_point_index,
                  position(current_state), trajectory_validation.remaining_duration_s);
  }
  const std::size_t source_control_index =
      std::min(static_cast<std::size_t>((now_ns - valid_from_ns) / control_interval_ns),
               points.size() - 2U);
  const std::size_t first_remaining_index =
      trajectory_validation.first_remaining_point_index;
  State simulated_state = current_state;
  Control previous_control = current_control;
  FiniteExecutionPathStatus altitude_status{FiniteExecutionPathStatus::kValid};
  FiniteExecutionPathStatus segment_status{FiniteExecutionPathStatus::kValid};
  Point3 failure_point{};
  for (std::size_t index = source_control_index; index + 1U < points.size(); ++index) {
    const Control& control = points[index].control;
    const State next_state =
        integrateReference(simulated_state, control, *world.dynamics);
    altitude_status = validateAltitudeState(next_state, control, world);
    if (altitude_status != FiniteExecutionPathStatus::kValid) {
      return reject(altitude_status, first_remaining_index, index, position(next_state),
                    trajectory_validation.remaining_duration_s);
    }
    if (!withinTerminalBoundary(next_state, world.terminal_boundary)) {
      return reject(FiniteExecutionPathStatus::kRouteEndpointExceeded,
                    first_remaining_index, index, position(next_state),
                    trajectory_validation.remaining_duration_s);
    }
    segment_status = validatePhysicalSegment(
        position(simulated_state), bodyAxis(previous_control), position(next_state),
        bodyAxis(control), world, failure_point);
    if (segment_status != FiniteExecutionPathStatus::kValid) {
      return reject(segment_status, first_remaining_index, index, failure_point,
                    trajectory_validation.remaining_duration_s);
    }
    simulated_state = next_state;
    previous_control = control;
  }
  return FiniteExecutionPathValidation{
      .status = FiniteExecutionPathStatus::kValid,
      .first_remaining_point_index = first_remaining_index,
      .remaining_duration_s = trajectory_validation.remaining_duration_s,
  };
}

RebuiltFiniteExecutionPathContinuation rebuildFiniteExecutionPathContinuation(
    const std::span<const TimedExecutionPathPoint> points,
    const std::int64_t valid_from_ns, const std::int64_t valid_until_ns,
    const std::int64_t now_ns, const State& current_state,
    const Control& current_control, const DynamicsConfig& dynamics,
    const std::size_t arrival_search_step_controls,
    const FiniteHorizonConfig& finite_horizon_config,
    const FiniteExecutionPathWorld& world) {
  RebuiltFiniteExecutionPathContinuation result;
  if (!validWorld(world) || !finite(current_state) || !finite(current_control) ||
      !(dynamics.dt_s > 0.0F) || arrival_search_step_controls == 0U ||
      valid_from_ns <= 0 || valid_until_ns <= valid_from_ns) {
    return result;
  }
  result.validation = validatePathContract(points);
  if (!result.validation.accepted()) {
    return result;
  }
  if (now_ns < valid_from_ns || now_ns >= valid_until_ns) {
    result.validation =
        reject(FiniteExecutionPathStatus::kNotActive, 0U, 0U, position(current_state),
               static_cast<double>(valid_until_ns - now_ns) / kNanosecondsPerSecond);
    return result;
  }

  const double remaining_s =
      static_cast<double>(valid_until_ns - now_ns) / kNanosecondsPerSecond;
  const std::int64_t control_interval_ns =
      finitePathControlIntervalNanoseconds(dynamics.dt_s);
  if (control_interval_ns <= 0) {
    return result;
  }
  result.source_control_index =
      std::min(static_cast<std::size_t>((now_ns - valid_from_ns) / control_interval_ns),
               points.size() - 2U);
  const std::size_t available_source_controls =
      points.size() - 1U - result.source_control_index;
  const std::size_t deadline_control_count =
      static_cast<std::size_t>((valid_until_ns - now_ns) / control_interval_ns);
  const std::size_t control_count =
      std::min(available_source_controls, deadline_control_count);
  if (control_count == 0U) {
    result.validation =
        reject(FiniteExecutionPathStatus::kNotActive, result.source_control_index,
               result.source_control_index, position(current_state), remaining_s);
    return result;
  }

  std::vector<Control> controls;
  controls.reserve(control_count);
  for (std::size_t offset = 0U; offset < control_count; ++offset) {
    controls.push_back(points[result.source_control_index + offset].control);
  }
  std::vector<State> states{current_state};
  states.reserve(controls.size() + 1U);
  for (const Control& control : controls) {
    states.push_back(integrateReference(states.back(), control, dynamics));
  }

  ValidatedFiniteExecutionPath rebuilt = buildValidatedFiniteExecutionPath(
      states, controls, current_control, dynamics, arrival_search_step_controls,
      finite_horizon_config, world);
  result.validation = rebuilt.validation;
  result.arrival_shaping_attempts = rebuilt.arrival_shaping_attempts;
  result.path_validation_backoff = rebuilt.path_validation_backoff;
  result.latest_lidar_path_validation_backoff =
      rebuilt.latest_lidar_path_validation_backoff;
  if (!rebuilt.accepted() || !rebuilt.horizon.has_value()) {
    return result;
  }
  FiniteHorizon rebuilt_horizon = std::move(*rebuilt.horizon);
  result.valid_until_ns =
      now_ns +
      static_cast<std::int64_t>(rebuilt_horizon.controls.size()) * control_interval_ns;
  result.valid_until_ns = std::min(result.valid_until_ns, valid_until_ns);
  result.horizon = std::move(rebuilt_horizon);
  return result;
}

const char*
finiteExecutionPathStatusName(const FiniteExecutionPathStatus status) noexcept {
  switch (status) {
    case FiniteExecutionPathStatus::kValid:
      return "valid";
    case FiniteExecutionPathStatus::kInvalidContract:
      return "invalid_contract";
    case FiniteExecutionPathStatus::kNotActive:
      return "not_active";
    case FiniteExecutionPathStatus::kRouteEndpointExceeded:
      return "route_endpoint_exceeded";
    case FiniteExecutionPathStatus::kFlightEnvelopeViolation:
      return "flight_envelope";
    case FiniteExecutionPathStatus::kDynamicFlightEnvelopeViolation:
      return "dynamic_flight_envelope";
    case FiniteExecutionPathStatus::kRawWorldUnavailable:
      return "raw_world_unavailable";
    case FiniteExecutionPathStatus::kRawCollision:
      return "raw_collision";
    case FiniteExecutionPathStatus::kLatestLidarRawCollision:
      return "latest_lidar_raw_collision";
  }
  return "unknown";
}

} // namespace drone_city_nav::mppi
