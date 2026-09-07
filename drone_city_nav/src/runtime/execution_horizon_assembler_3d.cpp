#include "execution_horizon_assembler_3d.hpp"

#include <algorithm>
#include <limits>
#include <ranges>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] HorizonCandidate3D
failedCandidate(const HorizonCandidateStatus3D status,
                const ProductionMppiExecutionReason reason =
                    ProductionMppiExecutionReason::kNoExecutableHorizon) noexcept {
  HorizonCandidate3D candidate;
  candidate.status = status;
  candidate.failure_reason = reason;
  return candidate;
}

[[nodiscard]] HorizonCandidate3D
explicitHoldCandidate(const Point3& position,
                      const ProductionMppiExecutionReason reason) noexcept {
  HorizonCandidate3D candidate;
  candidate.status = HorizonCandidateStatus3D::kExplicitHold;
  candidate.explicit_hold_position = position;
  candidate.explicit_hold_reason = reason;
  return candidate;
}

void captureValidationTelemetry(HorizonCandidate3D& candidate,
                                const mppi::ValidatedFiniteExecutionPath& validation,
                                const bool nominal_candidate_degraded) noexcept {
  candidate.arrival_shaping_attempts = validation.arrival_shaping_attempts;
  candidate.validation_failure_segment = validation.validation.failure_segment_index;
  candidate.validation_first_remaining_point =
      validation.validation.first_remaining_point_index;
  candidate.validation_status = validation.validation.status;
  candidate.validation_dynamics_consistency =
      validation.validation.dynamics_consistency;
  candidate.first_failed_validation_status = validation.first_failed_validation_status;
  candidate.first_failed_validation_segment =
      validation.first_failed_validation.failure_segment_index;
  candidate.first_failed_validation_point =
      validation.first_failed_validation.failure_point;
  candidate.finite_path_rejected_precondition = validation.rejected_precondition;
  candidate.nominal_candidate_degraded = nominal_candidate_degraded;
  candidate.path_validation_backoff = validation.path_validation_backoff;
  candidate.persistent_raw_path_validation_backoff =
      validation.persistent_raw_path_validation_backoff;
  candidate.latest_lidar_path_validation_backoff =
      validation.latest_lidar_path_validation_backoff;
}

} // namespace

std::string_view
horizonCandidateStatus3DName(const HorizonCandidateStatus3D status) noexcept {
  switch (status) {
    case HorizonCandidateStatus3D::kPlanned:
      return "planned";
    case HorizonCandidateStatus3D::kExplicitHold:
      return "explicit_hold";
    case HorizonCandidateStatus3D::kNoExecutableRoute:
      return "no_executable_route";
    case HorizonCandidateStatus3D::kMissingExactEvidence:
      return "missing_exact_evidence";
    case HorizonCandidateStatus3D::kInvalidControllerOutput:
      return "invalid_controller_output";
    case HorizonCandidateStatus3D::kStaleExecutionSnapshot:
      return "stale_execution_snapshot";
    case HorizonCandidateStatus3D::kFinitePathRejected:
      return "finite_path_rejected";
    case HorizonCandidateStatus3D::kCertificationRejected:
      return "certification_rejected";
    case HorizonCandidateStatus3D::kTransitionRejected:
      return "transition_rejected";
  }
  return "unknown";
}

ExecutionHorizonAssembler3D::ExecutionHorizonAssembler3D(
    ExecutionHorizonAssemblerConfig3D config)
    : config_{std::move(config)} {
}

HorizonCandidate3D ExecutionHorizonAssembler3D::assemble(
    const ProductionMppiExecutionCycle& cycle,
    const std::shared_ptr<const ExecutionPlan3D>& resident_plan) const {
  if (!cycle.validForAssembly()) {
    return failedCandidate(HorizonCandidateStatus3D::kMissingExactEvidence);
  }
  const mppi::MppiTickInput& input = cycle.controller.inputRef();
  const mppi::MppiTickResult& result = cycle.controller.resultRef();
  const EvidenceSnapshot3D& evidence = cycle.evidence;
  const RouteDecision3D& route = cycle.route;
  const ProductionRouteExecutionSelection3D& route_execution = route.execution;

  switch (route.planning_state) {
    case ProductionMppiPlanningState::kMissionGoalPositionHold:
      // The goal is captured once the vehicle rests inside the capture radius;
      // the hold pins that rest position instead of the goal coordinate, so no
      // controller has to creep the vehicle onto the exact goal first.
      return explicitHoldCandidate(Point3{evidence.exact_initial_state.x,
                                          evidence.exact_initial_state.y,
                                          evidence.exact_initial_state.z},
                                   ProductionMppiExecutionReason::kGoalCapture);
    case ProductionMppiPlanningState::kMissionCommandPositionHold:
      return explicitHoldCandidate(
          Point3{input.target.x, input.target.y, input.target.z},
          ProductionMppiExecutionReason::kGoalCapture);
    case ProductionMppiPlanningState::kNoExecutableRouteHold:
      return failedCandidate(HorizonCandidateStatus3D::kNoExecutableRoute,
                             ProductionMppiExecutionReason::kNoExecutableRoute);
    case ProductionMppiPlanningState::kCooperativePassageYieldHold:
      return explicitHoldCandidate(
          Point3{input.target.x, input.target.y, input.target.z},
          ProductionMppiExecutionReason::kCooperativePassageYield);
    case ProductionMppiPlanningState::kPlanned:
      break;
  }

  const bool latest_lidar_evidence_usable =
      evidence.latest_lidar_evidence != nullptr &&
      (evidence.latest_lidar_obstacle_fresh ||
       (evidence.selected_policy != nullptr &&
        !evidence.selected_policy->latestLidarFreshnessRequired()));
  if (!latest_lidar_evidence_usable || evidence.execution_dynamics == nullptr ||
      evidence.execution_flight_envelope == nullptr ||
      evidence.execution_altitude_envelope == nullptr ||
      evidence.execution_footprint == nullptr || !evidence.exact_snapshot_world) {
    return failedCandidate(HorizonCandidateStatus3D::kMissingExactEvidence);
  }

  const std::span<const mppi::State> states{result.horizon};
  const std::span<const mppi::Control> controls{result.controls};
  if (states.size() < 2U || controls.empty()) {
    return failedCandidate(HorizonCandidateStatus3D::kInvalidControllerOutput);
  }
  const bool nominal_altitude_violation =
      result.altitude_envelope_violation ||
      std::ranges::any_of(states, [this](const mppi::State& state) {
        return !insideFlightEnvelope(state.z, config_.flight_envelope);
      });
  const bool nominal_candidate_degraded =
      nominal_altitude_violation || !result.post_update_classification.executable;

  const std::shared_ptr<const ExecutionPlan3D> expected_snapshot =
      route_execution.source_snapshot;
  const std::shared_ptr<const ExecutionPlan3D> execution_certification_snapshot =
      route_execution.certification_snapshot != nullptr
          ? route_execution.certification_snapshot
          : expected_snapshot;
  if (execution_certification_snapshot == nullptr ||
      resident_plan != expected_snapshot) {
    return failedCandidate(HorizonCandidateStatus3D::kStaleExecutionSnapshot);
  }

  const CertifiedRouteSuffix3D* route_certification_target{nullptr};
  std::optional<FiniteExecutionPlanCertificationResult3D> route_certification;
  mppi::FiniteExecutionPathCandidateValidator route_candidate_validator;
  if (!route.direct_tracking_requested) {
    route_certification_target = route_execution.pending_activation
                                     ? route_execution.route.get()
                                     : execution_certification_snapshot->route();
    std::uint64_t previous_trajectory_revision{0U};
    if (const FiniteExecutionState3D* const finite =
            execution_certification_snapshot->finiteExecution()) {
      previous_trajectory_revision = finite->trajectory_revision;
    } else if (const DirectTrackingFiniteExecution3D* const direct =
                   execution_certification_snapshot->directTrackingExecution()) {
      previous_trajectory_revision = direct->trajectory_revision;
    } else if (const StopExecution3D* const stop =
                   execution_certification_snapshot->stopExecution()) {
      // A route certified from a stopping vehicle takes the vehicle back from
      // the stop, so its trajectory succeeds the stop's.
      previous_trajectory_revision = stop->trajectory_revision;
    }
    if (route_certification_target == nullptr ||
        previous_trajectory_revision == std::numeric_limits<std::uint64_t>::max()) {
      return failedCandidate(HorizonCandidateStatus3D::kCertificationRejected);
    }
    const std::uint64_t trajectory_revision = previous_trajectory_revision + 1U;
    route_candidate_validator =
        [&, trajectory_revision](const mppi::FiniteHorizon& candidate) {
          if (candidate.states.empty() || candidate.controls.empty()) {
            route_certification.reset();
            return false;
          }
          const std::optional<mppi::FiniteHorizon> braking_tail =
              mppi::buildFiniteBrakingHorizon(
                  candidate.states.front(), candidate.controls.size(),
                  *evidence.execution_dynamics, evidence.exact_previous_control,
                  config_.finite_horizon);
          if (!braking_tail.has_value()) {
            route_certification.reset();
            return false;
          }
          route_certification.emplace(certifyFiniteExecutionPlan3DDetailed(
              *execution_certification_snapshot, *route_certification_target,
              FiniteExecutionPlanCertification3D{
                  .command_horizon =
                      FiniteExecutionCertification3D{
                          .trajectory_revision = trajectory_revision,
                          .horizon = candidate,
                          .execution_input = evidence.execution_input,
                          .latest_lidar_evidence = evidence.latest_lidar_evidence,
                          .valid_from_ns = cycle.controller.now_ns,
                          .kind = FiniteExecutionKind3D::kNominal,
                      },
                  .braking_tail = *braking_tail,
              }));
          return route_certification->certified();
        };
  }

  mppi::ValidatedFiniteExecutionPath validated_path =
      mppi::buildValidatedFiniteExecutionPath(
          states, controls, evidence.exact_previous_control,
          *evidence.execution_dynamics, cycle.controller.arrival_search_step_controls,
          config_.finite_horizon, evidence.execution_path_world,
          std::move(route_candidate_validator));
  HorizonCandidate3D candidate;
  captureValidationTelemetry(candidate, validated_path, nominal_candidate_degraded);
  if (route_certification.has_value()) {
    candidate.certification_status = route_certification->command_horizon.status;
    candidate.braking_tail_certification_status =
        route_certification->braking_tail.status;
    candidate.route_adherence_status =
        route_certification->command_horizon.route_adherence_status;
    candidate.route_adherence_failure_distance_m =
        route_certification->command_horizon.route_adherence_failure_distance_m;
  }
  if (!route.direct_tracking_requested && !route_execution.pending_activation &&
      route.selected_snapshot_route != nullptr &&
      validated_path.physicalObstacleValidationBackoff()) {
    const bool persistent_raw = validated_path.persistent_raw_path_validation_backoff;
    candidate.physical_rejection = HorizonCandidatePhysicalRejection3D{
        .observed_raw_world = persistent_raw
                                  ? route.selected_snapshot_route->observed_raw_world
                                  : nullptr,
        .route_generation = route.selected_snapshot_route->identity.generation,
        .source = persistent_raw ? HorizonCandidateObstacleSource3D::kPersistentRaw
                                 : HorizonCandidateObstacleSource3D::kLatestLidar,
    };
  }
  if (!validated_path.accepted()) {
    candidate.status = HorizonCandidateStatus3D::kFinitePathRejected;
    return candidate;
  }
  mppi::FiniteHorizon executable_path =
      std::move(validated_path.horizon).value_or(mppi::FiniteHorizon{});

  const std::shared_ptr<const ExecutionPlan3D>& expected = expected_snapshot;
  if (route.direct_tracking_requested) {
    if (!route_execution.direct_tracking_identity.has_value() ||
        !route_execution.direct_tracking_identity->valid() ||
        config_.direct_tracking_validation_policy == nullptr ||
        (evidence.direct_observed_world == nullptr) ==
            (evidence.direct_static_world == nullptr)) {
      candidate.status = HorizonCandidateStatus3D::kCertificationRejected;
      return candidate;
    }
    const DirectTrackingFiniteExecution3D* const expected_direct =
        expected->directTrackingExecution();
    const FiniteExecutionState3D* const expected_finite = expected->finiteExecution();
    std::uint64_t previous_trajectory_revision{0U};
    if (expected_direct != nullptr) {
      previous_trajectory_revision = expected_direct->trajectory_revision;
    } else if (expected_finite != nullptr) {
      previous_trajectory_revision = expected_finite->trajectory_revision;
    } else if (const StopExecution3D* const expected_stop = expected->stopExecution()) {
      previous_trajectory_revision = expected_stop->trajectory_revision;
    }
    if (previous_trajectory_revision == std::numeric_limits<std::uint64_t>::max()) {
      candidate.status = HorizonCandidateStatus3D::kCertificationRejected;
      return candidate;
    }
    const std::optional<DirectTrackingFiniteExecution3D> certified_execution =
        certifyDirectTrackingExecution3D(
            *expected,
            DirectTrackingExecutionCertification3D{
                .identity = *route_execution.direct_tracking_identity,
                .trajectory_revision = previous_trajectory_revision + 1U,
                .target = Point3{input.target.x, input.target.y, input.target.z},
                .horizon = std::move(executable_path),
                .observed_raw_world = evidence.direct_observed_world,
                .static_world = evidence.direct_static_world,
                .validation_policy = config_.direct_tracking_validation_policy,
                .execution_input = evidence.execution_input,
                .latest_lidar_evidence = evidence.latest_lidar_evidence,
                .valid_from_ns = cycle.controller.now_ns,
                .kind = FiniteExecutionKind3D::kNominal,
            });
    if (!certified_execution.has_value()) {
      candidate.status = HorizonCandidateStatus3D::kCertificationRejected;
      return candidate;
    }
    const ExecutionRouteTransitionResult3D transition =
        expected->phase() == ExecutionRoutePhase3D::kDirectTracking
            ? replaceDirectTrackingExecution3D(*expected, expected->version,
                                               *certified_execution)
            : transferToDirectTracking3D(*expected, expected->version,
                                         *certified_execution);
    if (!transition.applied() || transition.next == nullptr ||
        transition.next->directTrackingExecution() == nullptr) {
      candidate.status = HorizonCandidateStatus3D::kTransitionRejected;
      candidate.transition_status = transition.status;
      candidate.transition_detail = transition.detail;
      return candidate;
    }
    candidate.committed_snapshot = transition.next;
    candidate.transition.emplace(transition);
  } else {
    if (route_certification_target == nullptr || !route_certification.has_value() ||
        !route_certification->certified() || !route_certification->plan.has_value()) {
      candidate.status = HorizonCandidateStatus3D::kCertificationRejected;
      return candidate;
    }
    const FiniteExecutionPlan3D& execution = route_certification->plan.value();
    const std::shared_ptr<const ExecutionPlan3D>& transition_base =
        execution_certification_snapshot;
    if (route_execution.pending_activation && transition_base->route() != nullptr &&
        (route_execution.pending_route == nullptr ||
         (route_execution.pending_route->base_kind ==
              PendingExecutionBaseKind3D::kRoute &&
          !route_execution.pending_route->route_splice.has_value()))) {
      candidate.status = HorizonCandidateStatus3D::kTransitionRejected;
      return candidate;
    }
    const ExecutionRouteTransitionResult3D prepared_transition = [&] {
      if (transition_base->phase() == ExecutionRoutePhase3D::kDirectTracking) {
        return transferDirectTrackingToCertifiedRoute3D(
            *transition_base, transition_base->version, *route_certification_target,
            execution);
      }
      if (transition_base->route() == nullptr) {
        return activateCertifiedRoute3D(*transition_base, transition_base->version,
                                        *route_certification_target, execution);
      }
      const ExecutionRouteTransitionGuard3D guard{
          .expected_snapshot_version = transition_base->version,
          .expected_route_generation = transition_base->route()->identity.generation,
          .expected_geometry_revision =
              transition_base->route()->geometry->compiled_trajectory_revision,
      };
      if (!route_execution.pending_activation) {
        return replaceFiniteExecutionPlan3D(*transition_base, guard, execution);
      }
      if (route_execution.pending_route->base_kind ==
          PendingExecutionBaseKind3D::kRouteHandoff) {
        return replaceCertifiedRouteAtHandoff3D(*transition_base, guard,
                                                *route_certification_target, execution);
      }
      return replaceCertifiedRoute3D(*transition_base, guard,
                                     *route_certification_target, execution,
                                     *route_execution.pending_route->route_splice);
    }();
    const ExecutionRouteTransitionResult3D transition =
        route_execution.progress_preparation != nullptr
            ? composeExecutionPlanTransition3D(
                  *expected, *route_execution.progress_preparation, prepared_transition)
            : prepared_transition;
    if (!transition.applied() || transition.next == nullptr ||
        transition.next->route() == nullptr ||
        transition.next->finiteExecution() == nullptr ||
        transition.next->brakingFallback() == nullptr) {
      candidate.status = HorizonCandidateStatus3D::kTransitionRejected;
      candidate.transition_status = transition.status;
      candidate.transition_detail = transition.detail;
      return candidate;
    }
    candidate.committed_snapshot = transition.next;
    candidate.transition.emplace(transition);
  }

  candidate.status = HorizonCandidateStatus3D::kPlanned;
  candidate.failure_reason = ProductionMppiExecutionReason::kNone;
  return candidate;
}

} // namespace drone_city_nav
