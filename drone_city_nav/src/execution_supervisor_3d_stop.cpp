#include "drone_city_nav/execution_stop_3d.hpp"
#include "drone_city_nav/execution_supervisor_3d.hpp"
#include "drone_city_nav/finite_execution_path_3d.hpp"
#include "drone_city_nav/finite_motion_horizon_3d.hpp"
#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {
namespace {

using namespace execution_route_snapshot_3d_internal;

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  return value.has_value() ? std::addressof(value.value()) : nullptr;
}

[[nodiscard]] double speedMps(const MotionState3D& state) noexcept {
  return std::hypot(
      std::hypot(static_cast<double>(state.vx), static_cast<double>(state.vy)),
      static_cast<double>(state.vz));
}

// The world a stop is validated against: the flight envelope, the vehicle's
// dynamics and the swept body against the raw evidence the caller owns. No
// route, no corridor, no terminal boundary. The contact seed is taken at the
// pose the stop starts from, so evidence the body already overlaps cannot
// forbid the vehicle from braking out of it.
[[nodiscard]] std::optional<FiniteExecutionPathWorld3D>
stopValidationWorld(const ExecutionStopRequest3D& request,
                    const SweptFootprintConfig& footprint,
                    std::optional<ProprioceptiveFreeSpaceSeed3D>& live_seed) {
  const bool raw_mode = request.observed_raw_world != nullptr;
  const bool static_mode = request.static_world != nullptr;
  if (raw_mode == static_mode || request.validation_policy == nullptr ||
      !request.validation_policy->valid() ||
      (raw_mode && !request.observed_raw_world->valid()) ||
      (static_mode && !request.static_world->valid()) ||
      request.latest_lidar_evidence == nullptr ||
      !request.latest_lidar_evidence->valid()) {
    return std::nullopt;
  }
  live_seed = proprioceptiveContactSeed3D(
      Point3{request.exact_initial_state.x, request.exact_initial_state.y,
             request.exact_initial_state.z},
      request.exact_previous_control, footprint,
      raw_mode ? std::addressof(request.observed_raw_world->occupancy()) : nullptr);
  return FiniteExecutionPathWorld3D{
      .flight_envelope = &request.validation_policy->flightEnvelope(),
      .dynamics = &request.validation_policy->dynamics(),
      .altitude_envelope = &request.validation_policy->altitudeEnvelope(),
      .footprint = &footprint,
      .static_occupancy = static_mode ? &request.static_world->occupancy() : nullptr,
      .observed_occupancy =
          raw_mode ? &request.observed_raw_world->occupancy() : nullptr,
      .launch_support_contact =
          raw_mode ? optionalAddress(request.observed_raw_world->launchSupportContact())
                   : nullptr,
      .proprioceptive_free_space_seed = optionalAddress(live_seed),
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = request.latest_lidar_evidence->indexedHitPoints(),
      .terminal_boundary = std::nullopt,
  };
}

[[nodiscard]] std::vector<TimedExecutionPathPoint3D>
stopPathPoints(const StopExecution3D& stop) {
  return stop.horizon != nullptr && stop.execution_input != nullptr
             ? timedExecutionPathPoints(*stop.horizon,
                                        stop.execution_input->previousControl(),
                                        stop.control_interval_ns)
             : std::vector<TimedExecutionPathPoint3D>{};
}

// A resident stop keeps the vehicle only while the part of it that has not
// been flown yet is still executable on the newest evidence. Once it no
// longer is, a fresh stop is derived from where the vehicle now stands.
[[nodiscard]] bool
residentStopStillExecutable(const StopExecution3D& stop,
                            const ExecutionStopRequest3D& request,
                            const FiniteExecutionPathWorld3D& world) {
  const std::vector<TimedExecutionPathPoint3D> points = stopPathPoints(stop);
  if (points.empty() || request.now_ns >= stop.valid_until_ns) {
    return false;
  }
  return validateFiniteExecutionPathContinuation3D(
             points, stop.valid_from_ns, stop.valid_until_ns, request.now_ns,
             request.exact_initial_state, request.exact_previous_control, world)
      .accepted();
}

// Whether a path verdict means the swept body met occupied evidence, as
// opposed to a broken contract, envelope or dynamics law that no smaller body
// would satisfy either.
[[nodiscard]] constexpr bool
stopCollisionVerdict(const FiniteExecutionPathStatus3D status) noexcept {
  return status == FiniteExecutionPathStatus3D::kRawCollision ||
         status == FiniteExecutionPathStatus3D::kLatestLidarRawCollision;
}

// The longest stop the supervisor will ask for. At the absolute speed limit
// and the guaranteed vertical deceleration a stop takes about six seconds,
// so this bounds the arrival search well above every state the vehicle can
// reach while leaving the trajectory finite.
constexpr std::size_t kMaximumStopControlCount{200U};

// A stop is a physics artifact: its length follows from the state the
// vehicle is in and the deceleration it is guaranteed, never from the
// number of controls the controller happened to produce. A route whose
// horizon was truncated to a handful of controls is exactly the situation
// that asks for a stop, and braking inside that many controls is
// impossible at cruise speed.
[[nodiscard]] std::size_t
stopControlCountEstimate(const MotionState3D& state, const MotionControl3D& previous,
                         const MotionDynamicsConfig3D& dynamics,
                         const FiniteMotionHorizonConfig3D& config) noexcept {
  const double dt_s = static_cast<double>(dynamics.dt_s);
  const double jerk_mps3 = static_cast<double>(dynamics.maximum_control_jerk_mps3);
  const double horizontal_deceleration_mps2 =
      std::min(static_cast<double>(dynamics.maximum_horizontal_acceleration_mps2),
               config.stopping_capability.guaranteed_horizontal_deceleration_mps2);
  const double vertical_deceleration_mps2 =
      std::min(static_cast<double>(dynamics.maximum_vertical_acceleration_mps2),
               config.stopping_capability.guaranteed_vertical_deceleration_mps2);
  if (!(dt_s > 0.0) || !(jerk_mps3 > 0.0) || !(horizontal_deceleration_mps2 > 0.0) ||
      !(vertical_deceleration_mps2 > 0.0)) {
    return kMaximumStopControlCount;
  }
  // The arrival ramps from the applied control to the deceleration and back
  // to zero; the applied control bounds how much further the first ramp is.
  const double release_s = std::max({std::abs(static_cast<double>(previous.ax)),
                                     std::abs(static_cast<double>(previous.ay)),
                                     std::abs(static_cast<double>(previous.az))}) /
                           jerk_mps3;
  const double translation_s =
      std::max(
          std::hypot(static_cast<double>(state.vx), static_cast<double>(state.vy)) /
              horizontal_deceleration_mps2,
          std::abs(static_cast<double>(state.vz)) / vertical_deceleration_mps2) +
      2.0 * std::max(horizontal_deceleration_mps2, vertical_deceleration_mps2) /
          jerk_mps3;
  const double yaw_s =
      dynamics.maximum_yaw_acceleration_radps2 > 0.0F
          ? std::abs(static_cast<double>(state.yaw_rate)) /
                static_cast<double>(dynamics.maximum_yaw_acceleration_radps2)
          : 0.0;
  // The arrival profile is shaped, not a bang profile: it spends longer than
  // the constant-deceleration bound, and the terminal rest state needs a
  // control step of its own.
  constexpr double kShapedProfileMargin{1.5};
  constexpr double kTerminalControls{8.0};
  const double controls =
      std::ceil(kShapedProfileMargin * (release_s + std::max(translation_s, yaw_s)) /
                dt_s) +
      kTerminalControls;
  return !std::isfinite(controls) ||
                 controls >= static_cast<double>(kMaximumStopControlCount)
             ? kMaximumStopControlCount
             : static_cast<std::size_t>(controls);
}

[[nodiscard]] std::uint64_t
nextStopTrajectoryRevision(const ExecutionPlan3D& plan) noexcept {
  std::uint64_t resident{0U};
  if (const FiniteExecutionState3D* const finite = plan.finiteExecution()) {
    resident = finite->trajectory_revision;
  }
  if (const FiniteExecutionState3D* const braking = plan.brakingFallback()) {
    resident = std::max(resident, braking->trajectory_revision);
  }
  if (const DirectTrackingFiniteExecution3D* const direct =
          plan.directTrackingExecution()) {
    resident = std::max(resident, direct->trajectory_revision);
  }
  if (const StopExecution3D* const stop = plan.stopExecution()) {
    resident = std::max(resident, stop->trajectory_revision);
  }
  return resident == std::numeric_limits<std::uint64_t>::max() ? 0U : resident + 1U;
}

} // namespace

const char* executionStopStatus3DName(const ExecutionStopStatus3D status) noexcept {
  switch (status) {
    case ExecutionStopStatus3D::kPrepared:
      return "prepared";
    case ExecutionStopStatus3D::kMissingAuthority:
      return "missing_authority";
    case ExecutionStopStatus3D::kSourceNotCurrent:
      return "source_not_current";
    case ExecutionStopStatus3D::kExecutionInputInvalid:
      return "execution_input_invalid";
    case ExecutionStopStatus3D::kAtRest:
      return "at_rest";
    case ExecutionStopStatus3D::kResidentStopCurrent:
      return "resident_stop_current";
    case ExecutionStopStatus3D::kValidationWorldUnavailable:
      return "validation_world_unavailable";
    case ExecutionStopStatus3D::kLidarEvidenceNotCurrent:
      return "lidar_evidence_not_current";
    case ExecutionStopStatus3D::kHorizonUnavailable:
      return "horizon_unavailable";
    case ExecutionStopStatus3D::kRevisionExhausted:
      return "revision_exhausted";
    case ExecutionStopStatus3D::kTransitionRejected:
      return "transition_rejected";
  }
  return "unknown";
}

bool ExecutionStopPreparation3D::prepared() const noexcept {
  const std::shared_ptr<const ExecutionPlan3D> expected = expectedPlan();
  const std::shared_ptr<const ExecutionPlan3D> plan = preparedPlan();
  return status == ExecutionStopStatus3D::kPrepared && transition != nullptr &&
         transition_status == ExecutionRouteTransitionStatus3D::kApplied &&
         transition->applied() && expected != nullptr &&
         transition->predecessor == expected.get() && expected_authority != nullptr &&
         expected_authority->valid() && plan != nullptr && plan->publishable() &&
         plan->stopExecution() != nullptr;
}

std::shared_ptr<const ExecutionPlan3D>
ExecutionStopPreparation3D::expectedPlan() const noexcept {
  return expected_authority != nullptr ? expected_authority->plan() : nullptr;
}

std::shared_ptr<const ExecutionPlan3D>
ExecutionStopPreparation3D::preparedPlan() const noexcept {
  return transition != nullptr ? transition->next : expectedPlan();
}

const StopExecution3D* ExecutionStopPreparation3D::stopExecution() const noexcept {
  const std::shared_ptr<const ExecutionPlan3D> plan = preparedPlan();
  return plan != nullptr ? plan->stopExecution() : nullptr;
}

ExecutionStopPreparation3D
ExecutionSupervisor3D::prepareStop(ExecutionStopRequest3D request) const {
  const ExecutionStopRequest3D owned_request{std::move(request)};
  ExecutionStopPreparation3D result;
  const std::shared_ptr<const CommittedExecutionAuthority3D> expected_authority =
      manager_.authority();
  const std::shared_ptr<const ExecutionPlan3D> expected =
      expected_authority != nullptr ? expected_authority->plan() : nullptr;
  if (expected_authority == nullptr || !expected_authority->valid() ||
      expected == nullptr) {
    return result;
  }
  result.expected_authority = expected_authority;
  if (owned_request.cycle_source_plan != expected) {
    result.status = ExecutionStopStatus3D::kSourceNotCurrent;
    return result;
  }
  if (owned_request.execution_input == nullptr ||
      !owned_request.execution_input->valid() ||
      !owned_request.execution_input->nominalStateAuthoritative() ||
      owned_request.now_ns <= 0 || owned_request.minimum_control_count == 0U) {
    result.status = ExecutionStopStatus3D::kExecutionInputInvalid;
    return result;
  }
  result.initial_speed_mps = speedMps(owned_request.exact_initial_state);
  // A vehicle that no longer moves has nothing to stop: the stationary hold
  // owns that state, and deriving a zero-motion stop every tick would only
  // churn the lease.
  if (result.initial_speed_mps <= kStationaryExecutionHoldSpeedToleranceMps) {
    result.status = ExecutionStopStatus3D::kAtRest;
    return result;
  }
  // A resident stop is judged with the body it was certified with; a fresh
  // stop is certified below against its own world.
  const StopExecution3D* const resident = expected->stopExecution();
  std::optional<ProprioceptiveFreeSpaceSeed3D> live_seed;
  const std::optional<FiniteExecutionPathWorld3D> world = stopValidationWorld(
      owned_request,
      resident != nullptr ? resident->validation_footprint
                          : owned_request.validation_policy->sweptFootprint(),
      live_seed);
  if (!world.has_value()) {
    result.status = ExecutionStopStatus3D::kValidationWorldUnavailable;
    return result;
  }
  if (resident != nullptr &&
      residentStopStillExecutable(*resident, owned_request, *world)) {
    result.status = ExecutionStopStatus3D::kResidentStopCurrent;
    return result;
  }
  const std::uint64_t trajectory_revision = nextStopTrajectoryRevision(*expected);
  if (trajectory_revision == 0U) {
    result.status = ExecutionStopStatus3D::kRevisionExhausted;
    return result;
  }
  const MotionDynamicsConfig3D& dynamics = owned_request.validation_policy->dynamics();
  const std::size_t requested_control_count =
      std::max(owned_request.minimum_control_count,
               stopControlCountEstimate(owned_request.exact_initial_state,
                                        owned_request.exact_previous_control, dynamics,
                                        owned_request.finite_horizon_config));
  std::optional<FiniteMotionHorizon3D> horizon = buildFiniteBrakingHorizon3D(
      owned_request.exact_initial_state, requested_control_count, dynamics,
      owned_request.exact_previous_control, owned_request.finite_horizon_config);
  if (!horizon.has_value() && requested_control_count < kMaximumStopControlCount) {
    // The estimate bounds a shaped profile from below. A state it underrates
    // still deserves the longest stop the supervisor will ask for before the
    // vehicle is left without one.
    horizon = buildFiniteBrakingHorizon3D(
        owned_request.exact_initial_state, kMaximumStopControlCount, dynamics,
        owned_request.exact_previous_control, owned_request.finite_horizon_config);
  }
  if (!horizon.has_value() || horizon->states.empty()) {
    result.status = ExecutionStopStatus3D::kHorizonUnavailable;
    return result;
  }
  const MotionState3D& rest_state = horizon->states.back();
  result.stop_distance_m = distance3D(Point3{owned_request.exact_initial_state.x,
                                             owned_request.exact_initial_state.y,
                                             owned_request.exact_initial_state.z},
                                      Point3{rest_state.x, rest_state.y, rest_state.z});
  // A vehicle that would come to rest within the tolerance a stationary hold
  // pins its position with is already where it will rest: there is nothing
  // for a stop to change, and publishing one only churns the lease that the
  // hold, or the goal capture, is about to take.
  if (result.stop_distance_m <= kStationaryExecutionHoldPositionToleranceM) {
    result.status = ExecutionStopStatus3D::kAtRest;
    return result;
  }
  StopExecutionCertification3D certification{
      .trajectory_revision = trajectory_revision,
      .horizon = *horizon,
      .observed_raw_world = owned_request.observed_raw_world,
      .static_world = owned_request.static_world,
      .validation_policy = owned_request.validation_policy,
      .execution_input = owned_request.execution_input,
      .latest_lidar_evidence = owned_request.latest_lidar_evidence,
      .valid_from_ns = owned_request.now_ns,
      .physical_body_only = false,
  };
  StopCertificationResult3D certification_report;
  const ExecutionRouteTransitionResult3D transition = [&] {
    ExecutionRouteTransitionResult3D envelope = enterStopExecution3D(
        *expected, expected->version, certification, &certification_report);
    // The envelope keeps clearance the vehicle may already have lost:
    // evidence confirmed beside the path it was following, or a wall it is
    // braking towards. A stop is the last motion the vehicle can be given, so
    // when the envelope cannot sweep clear, the same trajectory is certified
    // against the physical body alone. Braking with the body clear is
    // strictly safer than the horizon it replaces; leaving that horizon in
    // flight is what ended one flight against a wall.
    if (envelope.applied() ||
        certification_report.status !=
            StopCertificationStatus3D::kPathValidationRejected ||
        !stopCollisionVerdict(certification_report.path_validation_status)) {
      return envelope;
    }
    certification.physical_body_only = true;
    return enterStopExecution3D(*expected, expected->version, certification,
                                &certification_report);
  }();
  result.transition_status = transition.status;
  result.transition_detail = transition.detail;
  result.certification.status = certification_report.status;
  result.certification.dynamics_consistency = certification_report.dynamics_consistency;
  result.certification.path_validation_status =
      certification_report.path_validation_status;
  result.certification.physical_body_only = certification_report.physical_body_only;
  if (!transition.applied() || transition.next == nullptr ||
      transition.next->stopExecution() == nullptr) {
    result.status = ExecutionStopStatus3D::kTransitionRejected;
    return result;
  }
  result.transition =
      std::make_shared<const ExecutionRouteTransitionResult3D>(transition);
  result.status = ExecutionStopStatus3D::kPrepared;
  return result;
}

} // namespace drone_city_nav
