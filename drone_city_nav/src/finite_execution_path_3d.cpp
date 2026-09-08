#include "drone_city_nav/finite_execution_path_3d.hpp"

#include "drone_city_nav/control_route_projection_3d.hpp"
#include "drone_city_nav/finite_motion_horizon_3d.hpp"
#include "drone_city_nav/motion_altitude_envelope_3d.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <ranges>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kNanosecondsPerSecond{1.0e9};
// Terminal rest, in the one representation the builder shapes to, the
// certificate admits and the wire contract accepts.
constexpr double kTerminalRestVelocityTolerance{kTerminalRestVelocityToleranceMps};
constexpr double kTerminalRestControlTolerance{kTerminalRestControlToleranceMps2};

[[nodiscard]] bool finite(const MotionState3D& state) noexcept {
  return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.z) &&
         std::isfinite(state.vx) && std::isfinite(state.vy) &&
         std::isfinite(state.vz) && std::isfinite(state.yaw) &&
         std::isfinite(state.yaw_rate);
}

[[nodiscard]] bool finite(const MotionControl3D& control) noexcept {
  return std::isfinite(control.ax) && std::isfinite(control.ay) &&
         std::isfinite(control.az) && std::isfinite(control.yaw_accel);
}

[[nodiscard]] bool sameControl(const MotionControl3D& first,
                               const MotionControl3D& second) noexcept {
  return first.ax == second.ax && first.ay == second.ay && first.az == second.az &&
         first.yaw_accel == second.yaw_accel;
}

[[nodiscard]] Point3 position(const MotionState3D& state) noexcept {
  return Point3{state.x, state.y, state.z};
}

// The validation body stands upright: it is the physical body at every tilt
// the dynamics reach, enveloped once at configuration, so the commanded
// acceleration tilts nothing here and the planner's and the executor's
// verdicts about the same path agree.
constexpr FootprintBodyAxis kUprightBodyAxis{};

[[nodiscard]] bool
validRouteActivation(const FiniteExecutionPathTerminalBoundary3D& boundary) noexcept {
  if (boundary.activation_route.empty()) {
    return true;
  }
  if (boundary.activation_route.size() < 2U ||
      !std::isfinite(boundary.initial_route_station_m) ||
      !std::isfinite(boundary.activation_route_station_m)) {
    return false;
  }
  float previous_station_m{-std::numeric_limits<float>::infinity()};
  for (const ControlRouteSample3D& sample : boundary.activation_route) {
    if (!std::isfinite(sample.x_m) || !std::isfinite(sample.y_m) ||
        !std::isfinite(sample.z_m) || !std::isfinite(sample.station_m) ||
        sample.station_m + 1.0e-4F < previous_station_m) {
      return false;
    }
    previous_station_m = sample.station_m;
  }
  const float first_station_m = boundary.activation_route.front().station_m;
  const float last_station_m = boundary.activation_route.back().station_m;
  return last_station_m > first_station_m + 1.0e-4F &&
         boundary.initial_route_station_m + 1.0e-4F >= first_station_m &&
         boundary.initial_route_station_m <= last_station_m + 1.0e-4F &&
         boundary.activation_route_station_m + 1.0e-4F >= first_station_m &&
         boundary.activation_route_station_m <= last_station_m + 1.0e-4F;
}

[[nodiscard]] bool validWorld(const FiniteExecutionPathWorld3D& world) noexcept {
  if (world.flight_envelope == nullptr || world.dynamics == nullptr ||
      world.altitude_envelope == nullptr || world.footprint == nullptr) {
    return false;
  }
  if (!world.terminal_boundary.has_value()) {
    return true;
  }
  const FiniteExecutionPathTerminalBoundary3D& boundary = *world.terminal_boundary;
  const double forward_norm = std::hypot(
      std::hypot(boundary.forward.x, boundary.forward.y), boundary.forward.z);
  return std::isfinite(boundary.endpoint.x) && std::isfinite(boundary.endpoint.y) &&
         std::isfinite(boundary.endpoint.z) && std::isfinite(boundary.forward.x) &&
         std::isfinite(boundary.forward.y) && std::isfinite(boundary.forward.z) &&
         std::isfinite(boundary.tolerance_m) && boundary.tolerance_m >= 0.0 &&
         boundary.activation_distance_m > 0.0 && boundary.maximum_cross_track_m > 0.0 &&
         forward_norm > 1.0e-6 && validRouteActivation(boundary);
}

[[nodiscard]] bool withinTerminalBoundary(
    const MotionState3D& state,
    const std::optional<FiniteExecutionPathTerminalBoundary3D>& boundary,
    const float traveled_distance_m) noexcept {
  if (!boundary.has_value()) {
    return true;
  }
  if (!boundary->activation_route.empty()) {
    const ControlRouteProjection3D projection = projectOntoControlRoute3D(
        state, boundary->activation_route, boundary->initial_route_station_m);
    if (!projection.valid) {
      return true;
    }
    const float credited_station_m =
        boundary->initial_route_station_m +
        creditedControlRouteProgressM3D(projection.station_m,
                                        boundary->initial_route_station_m,
                                        traveled_distance_m);
    if (credited_station_m + 1.0e-4F < boundary->activation_route_station_m) {
      return true;
    }
  }
  const double forward_norm = std::hypot(
      std::hypot(boundary->forward.x, boundary->forward.y), boundary->forward.z);
  const Vec3 delta{
      static_cast<double>(state.x) - boundary->endpoint.x,
      static_cast<double>(state.y) - boundary->endpoint.y,
      static_cast<double>(state.z) - boundary->endpoint.z,
  };
  const double signed_distance_m =
      (delta.x * boundary->forward.x + delta.y * boundary->forward.y +
       delta.z * boundary->forward.z) /
      forward_norm;
  if (signed_distance_m <= boundary->tolerance_m) {
    return true;
  }
  if (!boundary->activation_route.empty()) {
    return false;
  }
  const double endpoint_distance_m = std::hypot(std::hypot(delta.x, delta.y), delta.z);
  const double cross_track_m =
      std::sqrt(std::max(0.0, endpoint_distance_m * endpoint_distance_m -
                                  signed_distance_m * signed_distance_m));
  // The terminal half-plane is meaningful only near its final route segment.
  // Applying it globally rejects valid curved approaches that happen to lie on
  // the forward side of the infinite terminal plane.
  return endpoint_distance_m > boundary->activation_distance_m ||
         cross_track_m > boundary->maximum_cross_track_m;
}

[[nodiscard]] FiniteExecutionPathValidation3D
reject(const FiniteExecutionPathStatus3D status,
       const std::size_t first_remaining_index, const std::size_t segment_index,
       const Point3& failure_point, const double remaining_duration_s) noexcept {
  return FiniteExecutionPathValidation3D{
      .status = status,
      .first_remaining_point_index = first_remaining_index,
      .failure_segment_index = segment_index,
      .failure_point = failure_point,
      .remaining_duration_s = remaining_duration_s,
  };
}

[[nodiscard]] FiniteExecutionPathStatus3D
validatePhysicalSegment(const Point3& first, const FootprintBodyAxis& first_axis,
                        const Point3& second, const FootprintBodyAxis& second_axis,
                        const FiniteExecutionPathWorld3D& world,
                        Point3& failure_point) noexcept {
  if (world.static_occupancy == nullptr && world.observed_occupancy == nullptr &&
      world.raw_occupancy == nullptr && world.latest_lidar_obstacle_points.empty()) {
    failure_point = first;
    return FiniteExecutionPathStatus3D::kRawWorldUnavailable;
  }
  const OccupiedCollisionOracle3D oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = world.observed_occupancy,
      .static_occupancy = world.static_occupancy,
      .planar_occupancy = world.raw_occupancy,
      .raw_point_cloud = world.latest_lidar_obstacle_points,
      .launch_support_contact = world.launch_support_contact,
      .proprioceptive_free_space_seed = world.proprioceptive_free_space_seed,
      .footprint = *world.footprint,
      .flight_envelope = *world.flight_envelope,
  }};
  const OccupiedCollisionResult3D validation =
      oracle.validateSegment(first, first_axis, second, second_axis);
  if (validation.clear()) {
    return FiniteExecutionPathStatus3D::kValid;
  }
  failure_point = validation.failure_point;
  if (validation.status == OccupiedCollisionStatus3D::kOutsideFlightEnvelope) {
    return FiniteExecutionPathStatus3D::kFlightEnvelopeViolation;
  }
  if (validation.status == OccupiedCollisionStatus3D::kInvalidInput) {
    return FiniteExecutionPathStatus3D::kRawWorldUnavailable;
  }
  if (validation.source == OccupiedCollisionSource3D::kRawPointCloud) {
    return FiniteExecutionPathStatus3D::kLatestLidarRawCollision;
  }
  return FiniteExecutionPathStatus3D::kRawCollision;
}

[[nodiscard]] FiniteExecutionPathValidation3D
validatePathContract(const std::span<const TimedExecutionPathPoint3D> points) noexcept {
  if (points.size() < 2U) {
    return {};
  }
  double previous_time_s{-std::numeric_limits<double>::infinity()};
  for (const TimedExecutionPathPoint3D& point : points) {
    if (!std::isfinite(point.time_from_start_s) || point.time_from_start_s < 0.0 ||
        point.time_from_start_s <= previous_time_s || !finite(point.state) ||
        !finite(point.control)) {
      return reject(FiniteExecutionPathStatus3D::kInvalidContract, 0U, 0U,
                    position(point.state), 0.0);
    }
    previous_time_s = point.time_from_start_s;
  }
  const TimedExecutionPathPoint3D& terminal = points.back();
  if (std::hypot(std::hypot(terminal.state.vx, terminal.state.vy), terminal.state.vz) >
          kTerminalRestVelocityTolerance ||
      std::abs(terminal.state.yaw_rate) > kTerminalRestVelocityTolerance ||
      std::abs(terminal.control.ax) > kTerminalRestControlTolerance ||
      std::abs(terminal.control.ay) > kTerminalRestControlTolerance ||
      std::abs(terminal.control.az) > kTerminalRestControlTolerance ||
      std::abs(terminal.control.yaw_accel) > kTerminalRestControlTolerance) {
    return reject(FiniteExecutionPathStatus3D::kInvalidContract, 0U, points.size() - 1U,
                  position(terminal.state), 0.0);
  }
  return FiniteExecutionPathValidation3D{
      .status = FiniteExecutionPathStatus3D::kValid,
  };
}

[[nodiscard]] FiniteExecutionPathStatus3D
validateAltitudeState(const MotionState3D& state,
                      const MotionControl3D& applied_control,
                      const FiniteExecutionPathWorld3D& world) noexcept {
  if (!insideFlightEnvelope(state.z, *world.flight_envelope)) {
    return FiniteExecutionPathStatus3D::kFlightEnvelopeViolation;
  }
  if (!motionAltitudeEnvelopeDynamicallyRecoverable3D(
          state, applied_control, *world.dynamics, *world.altitude_envelope)) {
    return FiniteExecutionPathStatus3D::kDynamicFlightEnvelopeViolation;
  }
  return FiniteExecutionPathStatus3D::kValid;
}

[[nodiscard]] std::vector<TimedExecutionPathPoint3D>
timedPathPoints(const FiniteMotionHorizon3D& horizon,
                const MotionControl3D& previous_applied_control, const float dt_s) {
  std::vector<TimedExecutionPathPoint3D> points;
  if (horizon.states.size() != horizon.controls.size() + 1U ||
      horizon.controls.empty() || !(dt_s > 0.0F)) {
    return points;
  }
  points.reserve(horizon.states.size());
  for (std::size_t index = 0U; index < horizon.states.size(); ++index) {
    points.push_back(TimedExecutionPathPoint3D{
        .time_from_start_s = static_cast<double>(index) * dt_s,
        .state = horizon.states[index],
        .control =
            index == 0U ? previous_applied_control : horizon.controls[index - 1U],
    });
  }
  return points;
}

} // namespace

FiniteExecutionPathValidation3D validateCompleteFiniteExecutionPath3D(
    const std::span<const TimedExecutionPathPoint3D> points,
    const MotionControl3D& previous_applied_control,
    const FiniteExecutionPathWorld3D& world,
    const std::size_t discharged_leading_point_count) noexcept {
  if (!validWorld(world) || !finite(previous_applied_control)) {
    return {};
  }
  const FiniteExecutionPathValidation3D contract = validatePathContract(points);
  if (!contract.accepted()) {
    return contract;
  }
  if (!sameControl(points.front().control, previous_applied_control)) {
    return reject(FiniteExecutionPathStatus3D::kInvalidContract, 0U, 0U,
                  position(points.front().state), 0.0);
  }

  FiniteExecutionPathStatus3D altitude_status =
      validateAltitudeState(points.front().state, previous_applied_control, world);
  if (altitude_status != FiniteExecutionPathStatus3D::kValid) {
    return reject(altitude_status, 0U, 0U, position(points.front().state), 0.0);
  }
  float terminal_boundary_travel_m{0.0F};
  if (!withinTerminalBoundary(points.front().state, world.terminal_boundary,
                              terminal_boundary_travel_m)) {
    return reject(FiniteExecutionPathStatus3D::kRouteEndpointExceeded, 0U, 0U,
                  position(points.front().state), 0.0);
  }

  Point3 failure_point{};
  FiniteExecutionPathValidation3D result{
      .status = FiniteExecutionPathStatus3D::kValid,
  };
  result.physically_validated_point_count = points.empty() ? 0U : 1U;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const TimedExecutionPathPoint3D& first = points[index - 1U];
    const TimedExecutionPathPoint3D& second = points[index];
    terminal_boundary_travel_m += std::hypot(
        std::hypot(second.state.x - first.state.x, second.state.y - first.state.y),
        second.state.z - first.state.z);
    altitude_status = validateAltitudeState(second.state, second.control, world);
    if (altitude_status != FiniteExecutionPathStatus3D::kValid) {
      return reject(altitude_status, 0U, index - 1U, position(second.state), 0.0);
    }
    if (!withinTerminalBoundary(second.state, world.terminal_boundary,
                                terminal_boundary_travel_m)) {
      return reject(FiniteExecutionPathStatus3D::kRouteEndpointExceeded, 0U, index - 1U,
                    position(second.state), 0.0);
    }
    // A leading point whose sweep the caller has discharged is not swept
    // again: either an earlier attempt proved it clear on an identical prefix,
    // or the vehicle has already flown past it.
    if (index < discharged_leading_point_count) {
      result.physically_validated_point_count = index + 1U;
      continue;
    }
    const FiniteExecutionPathStatus3D segment_status = validatePhysicalSegment(
        position(first.state), kUprightBodyAxis, position(second.state),
        kUprightBodyAxis, world, failure_point);
    if (segment_status != FiniteExecutionPathStatus3D::kValid) {
      FiniteExecutionPathValidation3D rejection =
          reject(segment_status, 0U, index - 1U, failure_point, 0.0);
      rejection.physically_validated_point_count = index;
      return rejection;
    }
    result.physically_validated_point_count = index + 1U;
  }
  return result;
}

[[nodiscard]] static ValidatedFiniteExecutionPath3D
buildValidatedFiniteExecutionPath3DFromPreservedPrefix(
    const std::span<const MotionState3D> planned_states,
    const std::span<const MotionControl3D> planned_controls,
    const MotionControl3D& previous_applied_control,
    const MotionDynamicsConfig3D& dynamics,
    const std::size_t arrival_search_step_controls,
    const FiniteMotionHorizonConfig3D& finite_horizon_config,
    const FiniteExecutionPathWorld3D& world,
    const std::size_t initial_preserved_prefix_control_count,
    const std::size_t maximum_nominal_prefix_control_count,
    FiniteExecutionPathCandidateValidator3D candidate_validator,
    const FiniteExecutionPathBudget3D& budget) {
  ValidatedFiniteExecutionPath3D result;
  const auto precondition_failure = [&]() -> const char* {
    if (!validWorld(world)) {
      return "world_invalid";
    }
    if (!finite(previous_applied_control)) {
      return "previous_control_not_finite";
    }
    if (planned_controls.empty()) {
      return "planned_controls_empty";
    }
    if (planned_states.size() != planned_controls.size() + 1U) {
      return "planned_states_controls_mismatch";
    }
    if (arrival_search_step_controls == 0U) {
      return "arrival_search_step_zero";
    }
    if (initial_preserved_prefix_control_count > planned_controls.size() ||
        maximum_nominal_prefix_control_count > initial_preserved_prefix_control_count) {
      return "prefix_counts_inconsistent";
    }
    return nullptr;
  }();
  if (precondition_failure != nullptr) {
    result.rejected_precondition = precondition_failure;
    return result;
  }

  // Every candidate this search builds shares its leading states and controls
  // with the longer one before it, so a point once proved clear of occupied
  // evidence stays clear for the rest of the search. The watermark is clamped
  // to the prefix each candidate actually shares.
  std::size_t physically_validated_point_count{0U};
  std::size_t preserved_prefix_control_count = initial_preserved_prefix_control_count;
  while (true) {
    if (result.arrival_shaping_attempts > 0U && budget.expired()) {
      // Out of budget with nothing accepted: the caller holds instead of
      // receiving a horizon several periods late.
      result.arrival_shaping_budget_exhausted = true;
      return result;
    }
    ++result.arrival_shaping_attempts;
    std::optional<FiniteMotionHorizon3D> candidate = buildFiniteMotionHorizon3D(
        planned_states, planned_controls, preserved_prefix_control_count, dynamics,
        previous_applied_control, finite_horizon_config);
    if (candidate.has_value()) {
      candidate->nominal_prefix_control_count = std::min(
          maximum_nominal_prefix_control_count, preserved_prefix_control_count);
      candidate->arrival_control_count =
          candidate->controls.size() - candidate->nominal_prefix_control_count;
      // The dynamics law the execution certificate admits by, applied to what
      // this builder emits. Checking it here is what keeps the builder from
      // handing on a horizon a later stage refuses, and names the law it broke
      // instead of leaving the caller with a bare rejection two stages away.
      const MotionDynamicsConsistency3D dynamics_consistency =
          finiteMotionHorizonDynamicsConsistency3D(*candidate, previous_applied_control,
                                                   dynamics);
      if (dynamics_consistency != MotionDynamicsConsistency3D::kConsistent) {
        result.validation = reject(FiniteExecutionPathStatus3D::kDynamicsInconsistent,
                                   0U, preserved_prefix_control_count,
                                   position(candidate->states.back()), 0.0);
        result.validation.dynamics_consistency = dynamics_consistency;
        if (!result.path_validation_backoff) {
          result.first_failed_validation_status = result.validation.status;
          result.first_failed_validation = result.validation;
        }
        result.path_validation_backoff = true;
        if (preserved_prefix_control_count == 0U) {
          return result;
        }
        preserved_prefix_control_count =
            preserved_prefix_control_count > arrival_search_step_controls
                ? preserved_prefix_control_count - arrival_search_step_controls
                : 0U;
        continue;
      }
      result.validation = validateCompleteFiniteExecutionPath3D(
          timedPathPoints(*candidate, previous_applied_control, dynamics.dt_s),
          previous_applied_control, world,
          std::min(physically_validated_point_count,
                   preserved_prefix_control_count + 1U));
      physically_validated_point_count =
          std::max(physically_validated_point_count,
                   result.validation.physically_validated_point_count);
      if (result.validation.accepted() &&
          (!candidate_validator || candidate_validator(*candidate))) {
        result.horizon = std::move(candidate);
        return result;
      }
      if (result.validation.accepted()) {
        result.validation = reject(FiniteExecutionPathStatus3D::kCandidateRejected, 0U,
                                   preserved_prefix_control_count,
                                   position(candidate->states.back()), 0.0);
      }
      if (!result.path_validation_backoff) {
        result.first_failed_validation_status = result.validation.status;
        result.first_failed_validation = result.validation;
      }
      result.path_validation_backoff = true;
      result.persistent_raw_path_validation_backoff |=
          result.validation.status == FiniteExecutionPathStatus3D::kRawCollision;
      result.latest_lidar_path_validation_backoff |=
          result.validation.status ==
          FiniteExecutionPathStatus3D::kLatestLidarRawCollision;
    } else {
      result.validation =
          reject(FiniteExecutionPathStatus3D::kInvalidContract, 0U,
                 preserved_prefix_control_count, position(planned_states.front()), 0.0);
    }
    if (preserved_prefix_control_count == 0U) {
      return result;
    }
    // The states of the nominal prefix are the same for every longer prefix,
    // so a failure inside the prefix persists until the prefix ends before
    // it: the search continues from the failing segment, not one step down.
    preserved_prefix_control_count =
        std::min(preserved_prefix_control_count > arrival_search_step_controls
                     ? preserved_prefix_control_count - arrival_search_step_controls
                     : 0U,
                 result.validation.failure_segment_index);
  }
}

ValidatedFiniteExecutionPath3D buildValidatedFiniteExecutionPath3D(
    const std::span<const MotionState3D> planned_states,
    const std::span<const MotionControl3D> planned_controls,
    const MotionControl3D& previous_applied_control,
    const MotionDynamicsConfig3D& dynamics,
    const std::size_t arrival_search_step_controls,
    const FiniteMotionHorizonConfig3D& finite_horizon_config,
    const FiniteExecutionPathWorld3D& world,
    FiniteExecutionPathCandidateValidator3D candidate_validator,
    const FiniteExecutionPathBudget3D& budget) {
  return buildValidatedFiniteExecutionPath3DFromPreservedPrefix(
      planned_states, planned_controls, previous_applied_control, dynamics,
      arrival_search_step_controls, finite_horizon_config, world,
      planned_controls.size(), planned_controls.size(), std::move(candidate_validator),
      budget);
}

FiniteExecutionPathValidation3D validateFiniteExecutionTrajectoryContinuation3D(
    const std::span<const TimedExecutionPathPoint3D> points,
    const std::int64_t valid_from_ns, const std::int64_t valid_until_ns,
    const std::int64_t now_ns, const MotionState3D& current_state,
    const MotionControl3D& current_control,
    const FiniteExecutionPathWorld3D& world) noexcept {
  if (!validWorld(world) || points.size() < 2U || valid_from_ns <= 0 ||
      valid_until_ns <= valid_from_ns || !finite(current_state) ||
      !finite(current_control)) {
    return {};
  }
  const double remaining_duration_s =
      static_cast<double>(valid_until_ns - now_ns) / kNanosecondsPerSecond;
  if (now_ns < valid_from_ns || now_ns >= valid_until_ns) {
    return reject(FiniteExecutionPathStatus3D::kNotActive, 0U, 0U,
                  position(current_state), remaining_duration_s);
  }

  FiniteExecutionPathValidation3D contract = validatePathContract(points);
  if (!contract.accepted()) {
    contract.remaining_duration_s = remaining_duration_s;
    return contract;
  }
  const TimedExecutionPathPoint3D& terminal = points.back();

  const double elapsed_s =
      static_cast<double>(now_ns - valid_from_ns) / kNanosecondsPerSecond;
  const auto first_remaining =
      std::ranges::find_if(points, [elapsed_s](const TimedExecutionPathPoint3D& point) {
        return point.time_from_start_s >= elapsed_s;
      });
  if (first_remaining == points.end()) {
    return reject(FiniteExecutionPathStatus3D::kInvalidContract, points.size(),
                  points.size() - 1U, position(terminal.state), remaining_duration_s);
  }
  const std::size_t first_remaining_index =
      static_cast<std::size_t>(std::distance(points.begin(), first_remaining));
  FiniteExecutionPathStatus3D altitude_status =
      validateAltitudeState(current_state, current_control, world);
  if (altitude_status != FiniteExecutionPathStatus3D::kValid) {
    return reject(altitude_status, first_remaining_index, first_remaining_index,
                  position(current_state), remaining_duration_s);
  }
  float terminal_boundary_travel_m{0.0F};
  if (!withinTerminalBoundary(current_state, world.terminal_boundary,
                              terminal_boundary_travel_m)) {
    return reject(FiniteExecutionPathStatus3D::kRouteEndpointExceeded,
                  first_remaining_index, first_remaining_index, position(current_state),
                  remaining_duration_s);
  }
  MotionState3D terminal_boundary_previous_state = current_state;
  for (std::size_t index = first_remaining_index; index < points.size(); ++index) {
    altitude_status =
        validateAltitudeState(points[index].state, points[index].control, world);
    if (altitude_status != FiniteExecutionPathStatus3D::kValid) {
      return reject(altitude_status, first_remaining_index, index,
                    position(points[index].state), remaining_duration_s);
    }
    terminal_boundary_travel_m += std::hypot(
        std::hypot(points[index].state.x - terminal_boundary_previous_state.x,
                   points[index].state.y - terminal_boundary_previous_state.y),
        points[index].state.z - terminal_boundary_previous_state.z);
    terminal_boundary_previous_state = points[index].state;
    if (!withinTerminalBoundary(points[index].state, world.terminal_boundary,
                                terminal_boundary_travel_m)) {
      return reject(FiniteExecutionPathStatus3D::kRouteEndpointExceeded,
                    first_remaining_index, index, position(points[index].state),
                    remaining_duration_s);
    }
  }

  Point3 failure_point{};
  FiniteExecutionPathStatus3D segment_status = validatePhysicalSegment(
      position(current_state), kUprightBodyAxis, position(first_remaining->state),
      kUprightBodyAxis, world, failure_point);
  if (segment_status != FiniteExecutionPathStatus3D::kValid) {
    return reject(segment_status, first_remaining_index, first_remaining_index,
                  failure_point, remaining_duration_s);
  }
  for (std::size_t index = first_remaining_index + 1U; index < points.size(); ++index) {
    const TimedExecutionPathPoint3D& first = points[index - 1U];
    const TimedExecutionPathPoint3D& second = points[index];
    segment_status = validatePhysicalSegment(position(first.state), kUprightBodyAxis,
                                             position(second.state), kUprightBodyAxis,
                                             world, failure_point);
    if (segment_status != FiniteExecutionPathStatus3D::kValid) {
      return reject(segment_status, first_remaining_index, index - 1U, failure_point,
                    remaining_duration_s);
    }
  }

  return FiniteExecutionPathValidation3D{
      .status = FiniteExecutionPathStatus3D::kValid,
      .first_remaining_point_index = first_remaining_index,
      .remaining_duration_s = remaining_duration_s,
  };
}

FiniteExecutionPathValidation3D validateFiniteExecutionPathContinuation3D(
    const std::span<const TimedExecutionPathPoint3D> points,
    const std::int64_t valid_from_ns, const std::int64_t valid_until_ns,
    const std::int64_t now_ns, const MotionState3D& current_state,
    const MotionControl3D& current_control,
    const FiniteExecutionPathWorld3D& world) noexcept {
  const FiniteExecutionPathValidation3D trajectory_validation =
      validateFiniteExecutionTrajectoryContinuation3D(
          points, valid_from_ns, valid_until_ns, now_ns, current_state, current_control,
          world);
  if (!trajectory_validation.accepted()) {
    return trajectory_validation;
  }

  const std::int64_t control_interval_ns =
      finitePathControlIntervalNanoseconds3D(world.dynamics->dt_s);
  if (control_interval_ns <= 0) {
    return reject(FiniteExecutionPathStatus3D::kInvalidContract,
                  trajectory_validation.first_remaining_point_index,
                  trajectory_validation.first_remaining_point_index,
                  position(current_state), trajectory_validation.remaining_duration_s);
  }
  const std::size_t source_control_index =
      std::min(static_cast<std::size_t>((now_ns - valid_from_ns) / control_interval_ns),
               points.size() - 2U);
  const std::size_t first_remaining_index =
      trajectory_validation.first_remaining_point_index;
  MotionState3D simulated_state = current_state;
  MotionControl3D previous_control = current_control;
  float terminal_boundary_travel_m{0.0F};
  FiniteExecutionPathStatus3D altitude_status{FiniteExecutionPathStatus3D::kValid};
  FiniteExecutionPathStatus3D segment_status{FiniteExecutionPathStatus3D::kValid};
  Point3 failure_point{};
  for (std::size_t index = source_control_index; index + 1U < points.size(); ++index) {
    const MotionControl3D& control = points[index + 1U].control;
    const MotionState3D next_state =
        integrateMotionState3D(simulated_state, control, *world.dynamics);
    altitude_status = validateAltitudeState(next_state, control, world);
    if (altitude_status != FiniteExecutionPathStatus3D::kValid) {
      return reject(altitude_status, first_remaining_index, index, position(next_state),
                    trajectory_validation.remaining_duration_s);
    }
    terminal_boundary_travel_m += std::hypot(
        std::hypot(next_state.x - simulated_state.x, next_state.y - simulated_state.y),
        next_state.z - simulated_state.z);
    if (!withinTerminalBoundary(next_state, world.terminal_boundary,
                                terminal_boundary_travel_m)) {
      return reject(FiniteExecutionPathStatus3D::kRouteEndpointExceeded,
                    first_remaining_index, index, position(next_state),
                    trajectory_validation.remaining_duration_s);
    }
    segment_status = validatePhysicalSegment(position(simulated_state),
                                             kUprightBodyAxis, position(next_state),
                                             kUprightBodyAxis, world, failure_point);
    if (segment_status != FiniteExecutionPathStatus3D::kValid) {
      return reject(segment_status, first_remaining_index, index, failure_point,
                    trajectory_validation.remaining_duration_s);
    }
    simulated_state = next_state;
    previous_control = control;
  }
  return FiniteExecutionPathValidation3D{
      .status = FiniteExecutionPathStatus3D::kValid,
      .first_remaining_point_index = first_remaining_index,
      .remaining_duration_s = trajectory_validation.remaining_duration_s,
  };
}

RebuiltFiniteExecutionPathContinuation3D rebuildFiniteExecutionPathContinuation3D(
    const std::span<const TimedExecutionPathPoint3D> points,
    const std::int64_t valid_from_ns, const std::int64_t valid_until_ns,
    const std::int64_t now_ns, const MotionState3D& current_state,
    const MotionControl3D& current_control,
    const std::size_t source_nominal_prefix_control_count,
    const std::size_t source_preserved_prefix_control_count,
    const MotionDynamicsConfig3D& dynamics,
    const std::size_t arrival_search_step_controls,
    const FiniteMotionHorizonConfig3D& finite_horizon_config,
    const FiniteExecutionPathWorld3D& world,
    FiniteExecutionPathCandidateValidator3D candidate_validator) {
  RebuiltFiniteExecutionPathContinuation3D result;
  if (!validWorld(world) || !finite(current_state) || !finite(current_control) ||
      !(dynamics.dt_s > 0.0F) || arrival_search_step_controls == 0U ||
      valid_from_ns <= 0 || valid_until_ns <= valid_from_ns || points.empty() ||
      source_nominal_prefix_control_count > points.size() - 1U ||
      source_preserved_prefix_control_count > points.size() - 1U ||
      source_nominal_prefix_control_count > source_preserved_prefix_control_count) {
    return result;
  }
  result.validation = validatePathContract(points);
  if (!result.validation.accepted()) {
    return result;
  }
  if (now_ns < valid_from_ns || now_ns >= valid_until_ns) {
    result.validation =
        reject(FiniteExecutionPathStatus3D::kNotActive, 0U, 0U, position(current_state),
               static_cast<double>(valid_until_ns - now_ns) / kNanosecondsPerSecond);
    return result;
  }

  const double remaining_s =
      static_cast<double>(valid_until_ns - now_ns) / kNanosecondsPerSecond;
  const std::int64_t control_interval_ns =
      finitePathControlIntervalNanoseconds3D(dynamics.dt_s);
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
        reject(FiniteExecutionPathStatus3D::kNotActive, result.source_control_index,
               result.source_control_index, position(current_state), remaining_s);
    return result;
  }

  std::vector<MotionControl3D> controls;
  controls.reserve(control_count);
  for (std::size_t offset = 0U; offset < control_count; ++offset) {
    controls.push_back(points[result.source_control_index + offset + 1U].control);
  }
  std::vector<MotionState3D> states{current_state};
  states.reserve(controls.size() + 1U);
  for (const MotionControl3D& control : controls) {
    states.push_back(integrateMotionState3D(states.back(), control, dynamics));
  }

  const std::size_t remaining_nominal_prefix_control_count =
      result.source_control_index < source_nominal_prefix_control_count
          ? std::min(source_nominal_prefix_control_count - result.source_control_index,
                     controls.size())
          : 0U;
  const std::size_t remaining_preserved_prefix_control_count =
      result.source_control_index < source_preserved_prefix_control_count
          ? std::min(source_preserved_prefix_control_count -
                         result.source_control_index,
                     controls.size())
          : 0U;
  ValidatedFiniteExecutionPath3D rebuilt =
      buildValidatedFiniteExecutionPath3DFromPreservedPrefix(
          states, controls, current_control, dynamics, arrival_search_step_controls,
          finite_horizon_config, world, remaining_preserved_prefix_control_count,
          remaining_nominal_prefix_control_count, std::move(candidate_validator),
          FiniteExecutionPathBudget3D{});
  result.validation = rebuilt.validation;
  result.arrival_shaping_attempts = rebuilt.arrival_shaping_attempts;
  result.path_validation_backoff = rebuilt.path_validation_backoff;
  result.persistent_raw_path_validation_backoff =
      rebuilt.persistent_raw_path_validation_backoff;
  result.latest_lidar_path_validation_backoff =
      rebuilt.latest_lidar_path_validation_backoff;
  if (!rebuilt.accepted() || !rebuilt.horizon.has_value()) {
    return result;
  }
  FiniteMotionHorizon3D rebuilt_horizon = std::move(*rebuilt.horizon);
  result.valid_until_ns =
      now_ns +
      static_cast<std::int64_t>(rebuilt_horizon.controls.size()) * control_interval_ns;
  result.valid_until_ns = std::min(result.valid_until_ns, valid_until_ns);
  result.horizon = std::move(rebuilt_horizon);
  return result;
}

const char*
finiteExecutionPathStatus3DName(const FiniteExecutionPathStatus3D status) noexcept {
  switch (status) {
    case FiniteExecutionPathStatus3D::kValid:
      return "valid";
    case FiniteExecutionPathStatus3D::kInvalidContract:
      return "invalid_contract";
    case FiniteExecutionPathStatus3D::kDynamicsInconsistent:
      return "dynamics_inconsistent";
    case FiniteExecutionPathStatus3D::kCandidateRejected:
      return "candidate_rejected";
    case FiniteExecutionPathStatus3D::kNotActive:
      return "not_active";
    case FiniteExecutionPathStatus3D::kRouteEndpointExceeded:
      return "route_endpoint_exceeded";
    case FiniteExecutionPathStatus3D::kFlightEnvelopeViolation:
      return "flight_envelope";
    case FiniteExecutionPathStatus3D::kDynamicFlightEnvelopeViolation:
      return "dynamic_flight_envelope";
    case FiniteExecutionPathStatus3D::kRawWorldUnavailable:
      return "raw_world_unavailable";
    case FiniteExecutionPathStatus3D::kRawCollision:
      return "raw_collision";
    case FiniteExecutionPathStatus3D::kLatestLidarRawCollision:
      return "latest_lidar_raw_collision";
  }
  return "unknown";
}

} // namespace drone_city_nav
