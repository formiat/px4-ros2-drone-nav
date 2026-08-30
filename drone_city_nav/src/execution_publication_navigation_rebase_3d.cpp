#include "execution_publication_navigation_rebase_3d.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

struct FiniteExecutionCandidateView3D {
  const mppi::FiniteHorizon* horizon{nullptr};
  const VersionedExecutionInput3D* execution_input{nullptr};
  const VersionedExecutionValidationPolicy3D* policy{nullptr};
  const VersionedStaticWorld3D* static_world{nullptr};
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  std::int64_t control_interval_ns{0};
  std::size_t nominal_prefix_control_count{0U};
};

[[nodiscard]] std::optional<FiniteExecutionCandidateView3D>
candidateView(const ExecutionRouteSnapshot3D& snapshot) noexcept {
  const auto make_view =
      [](const auto& execution) -> std::optional<FiniteExecutionCandidateView3D> {
    if (execution.horizon == nullptr || execution.execution_input == nullptr ||
        execution.validation_policy == nullptr ||
        (execution.observed_raw_world == nullptr) ==
            (execution.static_world == nullptr)) {
      return std::nullopt;
    }
    return FiniteExecutionCandidateView3D{
        .horizon = execution.horizon.get(),
        .execution_input = execution.execution_input.get(),
        .policy = execution.validation_policy.get(),
        .static_world = execution.static_world.get(),
        .valid_from_ns = execution.valid_from_ns,
        .valid_until_ns = execution.valid_until_ns,
        .control_interval_ns = execution.control_interval_ns,
        .nominal_prefix_control_count = execution.horizon->nominal_prefix_control_count,
    };
  };
  if (snapshot.finite_execution.has_value()) {
    return make_view(*snapshot.finite_execution);
  }
  if (snapshot.direct_tracking_execution.has_value()) {
    return make_view(*snapshot.direct_tracking_execution);
  }
  return std::nullopt;
}

[[nodiscard]] std::vector<mppi::TimedExecutionPathPoint>
timedPathPoints(const FiniteExecutionCandidateView3D& view) {
  std::vector<mppi::TimedExecutionPathPoint> points;
  if (view.horizon == nullptr || view.execution_input == nullptr ||
      view.control_interval_ns <= 0 || view.horizon->controls.empty() ||
      view.horizon->states.size() != view.horizon->controls.size() + 1U) {
    return points;
  }
  const double step_s = static_cast<double>(view.control_interval_ns) * 1.0e-9;
  if (!std::isfinite(step_s) || step_s <= 0.0) {
    return points;
  }
  points.reserve(view.horizon->states.size());
  for (std::size_t index = 0U; index < view.horizon->states.size(); ++index) {
    points.push_back(mppi::TimedExecutionPathPoint{
        .time_from_start_s = static_cast<double>(index) * step_s,
        .state = view.horizon->states[index],
        .control = index == 0U ? view.execution_input->previousControl()
                               : view.horizon->controls[index - 1U],
    });
  }
  return points;
}

[[nodiscard]] std::optional<std::int64_t>
publicationValidUntilNs(const FiniteExecutionCandidateView3D& view,
                        const std::int64_t publication_now_ns) noexcept {
  if (view.horizon == nullptr || view.control_interval_ns <= 0 ||
      publication_now_ns <= 0 ||
      view.horizon->controls.size() >
          static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max() /
                                   view.control_interval_ns)) {
    return std::nullopt;
  }
  const std::int64_t duration_ns =
      static_cast<std::int64_t>(view.horizon->controls.size()) *
      view.control_interval_ns;
  if (duration_ns <= 0 ||
      publication_now_ns > std::numeric_limits<std::int64_t>::max() - duration_ns) {
    return std::nullopt;
  }
  return publication_now_ns + duration_ns;
}

[[nodiscard]] ExecutionRouteTransitionResult3D
rebaseRouteExecution(const ExecutionPublicationNavigationRebaseRequest3D& request,
                     const mppi::FiniteHorizon& horizon,
                     FiniteExecutionCertificationResult3D& diagnostic,
                     ExecutionRouteTransitionStatus3D& transition_diagnostic) {
  const ExecutionRouteSnapshot3D& resident = *request.expected_snapshot;
  const ExecutionRouteSnapshot3D& certification_base =
      request.certification_snapshot != nullptr ? *request.certification_snapshot
                                                : resident;
  const ExecutionRouteSnapshot3D& candidate = *request.candidate_snapshot;
  if (!candidate.finite_execution.has_value() ||
      !candidate.braking_fallback.has_value() || !candidate.route.has_value() ||
      request.finite_horizon_config == nullptr) {
    return {};
  }
  const FiniteExecutionState3D& candidate_execution =
      candidate.finite_execution.value();
  if (horizon.states.empty() || horizon.controls.empty()) {
    return {};
  }
  const bool retaining_route = certification_base.route.has_value() &&
                               certification_base.route->route_instance_id ==
                                   candidate.route.value().route_instance_id;
  const CertifiedRouteSuffix3D& target_route = candidate.route.value();
  const ExecutionRouteTransitionGuard3D guard{
      .expected_snapshot_version = certification_base.version,
      .expected_route_generation = certification_base.route.has_value()
                                       ? certification_base.route->identity.generation
                                       : 0U,
      .expected_geometry_revision =
          certification_base.route.has_value()
              ? certification_base.route->geometry->executable_geometry_revision
              : 0U,
  };
  if (candidate_execution.kind == FiniteExecutionKind3D::kEmergencyBrakeTail) {
    if (!retaining_route || candidate.phase != ExecutionRoutePhase3D::kBraking ||
        request.lifecycle_event == nullptr || request.progress_preparation != nullptr ||
        request.expected_pending != nullptr || !certification_base.route.has_value() ||
        request.lifecycle_event->generation != target_route.identity.generation) {
      return {};
    }
    FiniteExecutionCertification3D braking_certification{
        .trajectory_revision = candidate_execution.trajectory_revision,
        .horizon = horizon,
        .execution_input = request.current_execution_input,
        .latest_lidar_evidence = request.current_lidar_evidence,
        .valid_from_ns = request.publication_now_ns,
        .kind = FiniteExecutionKind3D::kEmergencyBrakeTail,
    };
    RouteLifecycleEvent3D current_event = *request.lifecycle_event;
    FiniteExecutionCertificationResult3D certified;
    if (current_event.kind == RouteLifecycleEventKind3D::kRawInvalidated) {
      if (request.current_observed_raw_world == nullptr ||
          !request.current_observed_raw_world->valid()) {
        return {};
      }
      current_event.raw_producer_instance_id =
          request.current_observed_raw_world->version().producer_instance_id;
      current_event.raw_revision =
          request.current_observed_raw_world->version().revision;
      certified = certifyRawInvalidatedFiniteExecution3DDetailed(
          certification_base,
          RawInvalidatedFiniteExecutionCertification3D{
              .invalidation = current_event,
              .invalidating_observed_raw_world = request.current_observed_raw_world,
              .finite_execution = std::move(braking_certification),
          });
    } else {
      certified = certifyLifecycleBrakingFiniteExecution3DDetailed(
          certification_base, LifecycleBrakingFiniteExecutionCertification3D{
                                  .lifecycle_event = current_event,
                                  .finite_execution = std::move(braking_certification),
                              });
    }
    diagnostic = certified;
    if (!certified.certified() || !certified.execution.has_value()) {
      return {};
    }
    const ExecutionRouteTransitionResult3D transition = retireCertifiedRoute3D(
        certification_base, guard, current_event, certified.execution);
    transition_diagnostic = transition.status;
    return transition;
  }
  const std::optional<mppi::FiniteHorizon> braking_tail =
      mppi::buildFiniteBrakingHorizon(
          horizon.states.front(), horizon.controls.size(),
          candidate_execution.validation_policy->dynamics(),
          request.current_execution_input->previousControl(),
          *request.finite_horizon_config);
  if (!braking_tail.has_value()) {
    return {};
  }
  const FiniteExecutionPlanCertificationResult3D certified =
      certifyFiniteExecutionPlan3DDetailed(
          certification_base, target_route,
          FiniteExecutionPlanCertification3D{
              .command_horizon =
                  FiniteExecutionCertification3D{
                      .trajectory_revision = candidate_execution.trajectory_revision,
                      .horizon = horizon,
                      .execution_input = request.current_execution_input,
                      .latest_lidar_evidence = request.current_lidar_evidence,
                      .valid_from_ns = request.publication_now_ns,
                      .kind = candidate_execution.kind,
                  },
              .braking_tail = *braking_tail,
          });
  diagnostic.status = certified.command_horizon.status;
  diagnostic.route_adherence_status = certified.command_horizon.route_adherence_status;
  diagnostic.route_adherence_failure_state_index =
      certified.command_horizon.route_adherence_failure_state_index;
  diagnostic.route_adherence_failure_distance_m =
      certified.command_horizon.route_adherence_failure_distance_m;
  if (!certified.certified() || !certified.plan.has_value()) {
    return {};
  }
  const auto make_transition = [&]() -> ExecutionRouteTransitionResult3D {
    if (certification_base.phase == ExecutionRoutePhase3D::kDirectTracking) {
      return transferDirectTrackingToCertifiedRoute3D(
          certification_base, certification_base.version, target_route,
          certified.plan.value());
    }
    if (!certification_base.route.has_value()) {
      return activateCertifiedRoute3D(certification_base, certification_base.version,
                                      target_route, *certified.plan);
    }
    if (retaining_route) {
      return replaceFiniteExecutionPlan3D(certification_base, guard, *certified.plan);
    }
    if (request.expected_pending != nullptr &&
        request.expected_pending->base_kind ==
            PendingExecutionBaseKind3D::kRouteHandoff) {
      return replaceCertifiedRouteAtHandoff3D(certification_base, guard, target_route,
                                              *certified.plan);
    }
    if (request.expected_pending != nullptr &&
        request.expected_pending->route_splice.has_value()) {
      return replaceCertifiedRoute3D(certification_base, guard, target_route,
                                     *certified.plan,
                                     *request.expected_pending->route_splice);
    }
    return {};
  };
  const ExecutionRouteTransitionResult3D prepared_transition = make_transition();
  const ExecutionRouteTransitionResult3D transition =
      request.progress_preparation != nullptr
          ? composeExecutionPlanTransition3D(resident, *request.progress_preparation,
                                             prepared_transition)
          : prepared_transition;
  transition_diagnostic = transition.status;
  return transition;
}

[[nodiscard]] ExecutionRouteTransitionResult3D
rebaseDirectExecution(const ExecutionPublicationNavigationRebaseRequest3D& request,
                      const mppi::FiniteHorizon& horizon) {
  const ExecutionRouteSnapshot3D& expected = *request.expected_snapshot;
  if (!request.candidate_snapshot->direct_tracking_execution.has_value()) {
    return {};
  }
  const DirectTrackingFiniteExecution3D& candidate =
      request.candidate_snapshot->direct_tracking_execution.value();
  const std::optional<DirectTrackingFiniteExecution3D> certified =
      certifyDirectTrackingExecution3D(
          expected, DirectTrackingExecutionCertification3D{
                        .identity = candidate.identity,
                        .trajectory_revision = candidate.trajectory_revision,
                        .target = candidate.target,
                        .horizon = horizon,
                        .observed_raw_world = candidate.observed_raw_world,
                        .static_world = candidate.static_world,
                        .validation_policy = candidate.validation_policy,
                        .execution_input = request.current_execution_input,
                        .latest_lidar_evidence = request.current_lidar_evidence,
                        .valid_from_ns = request.publication_now_ns,
                        .kind = candidate.kind,
                    });
  if (!certified.has_value()) {
    return {};
  }
  return expected.phase == ExecutionRoutePhase3D::kDirectTracking
             ? replaceDirectTrackingExecution3D(expected, expected.version, *certified)
             : transferToDirectTracking3D(expected, expected.version, *certified);
}

} // namespace

bool ExecutionPublicationNavigationRebaseResult3D::rebased() const noexcept {
  return status == ExecutionPublicationNavigationRebaseStatus3D::kRebased &&
         transition.has_value() && transition->applied();
}

ExecutionPublicationNavigationRebaseResult3D
rebaseExecutionPublicationForCurrentNavigation3D(
    const ExecutionPublicationNavigationRebaseRequest3D& request) {
  ExecutionPublicationNavigationRebaseResult3D result;
  const auto reject =
      [&result](const ExecutionPublicationNavigationRebaseStatus3D status) {
        result.status = status;
        return result;
      };
  const ExecutionRouteSnapshot3D* const certification_base =
      request.certification_snapshot != nullptr ? request.certification_snapshot
                                                : request.expected_snapshot;
  const bool progress_preparation_valid =
      request.progress_preparation != nullptr && request.expected_snapshot != nullptr &&
      certification_base != nullptr && request.progress_preparation->applied() &&
      request.progress_preparation->predecessor == request.expected_snapshot &&
      request.progress_preparation->next.get() == certification_base;
  const bool certification_base_valid =
      request.progress_preparation != nullptr
          ? progress_preparation_valid
          : certification_base == request.expected_snapshot;
  if (request.expected_snapshot == nullptr || !certification_base_valid ||
      request.candidate_snapshot == nullptr ||
      request.current_execution_input == nullptr ||
      !request.current_execution_input->valid() ||
      !request.current_execution_input->nominalStateAuthoritative() ||
      request.current_lidar_evidence == nullptr || request.publication_now_ns <= 0 ||
      request.arrival_search_step_controls == 0U ||
      request.finite_horizon_config == nullptr) {
    return reject(ExecutionPublicationNavigationRebaseStatus3D::kInvalidRequest);
  }
  const std::optional<FiniteExecutionCandidateView3D> candidate_view =
      candidateView(*request.candidate_snapshot);
  if (!candidate_view.has_value() || candidate_view->policy == nullptr ||
      !candidate_view->policy->valid() ||
      (candidate_view->policy->latestLidarFreshnessRequired() &&
       !assessLatestLidarEvidenceFreshness3D(
            *request.current_lidar_evidence, request.publication_now_ns,
            candidate_view->policy->latestLidarMaximumAgeMs())
            .fresh)) {
    return reject(ExecutionPublicationNavigationRebaseStatus3D::kEvidenceUnavailable);
  }
  const bool retained_published_route_continuation =
      request.expected_snapshot->finite_execution.has_value() &&
      request.candidate_snapshot->finite_execution.has_value() &&
      request.expected_snapshot->route.has_value() &&
      request.candidate_snapshot->route.has_value() &&
      request.candidate_snapshot->finite_execution->kind ==
          FiniteExecutionKind3D::kRetained &&
      request.expected_snapshot->route->route_instance_id ==
          request.candidate_snapshot->route->route_instance_id;
  const bool emergency_braking_candidate =
      request.candidate_snapshot->finite_execution.has_value() &&
      request.candidate_snapshot->finite_execution->kind ==
          FiniteExecutionKind3D::kEmergencyBrakeTail;
  const std::optional<FiniteExecutionCandidateView3D> published_view =
      retained_published_route_continuation ? candidateView(*request.expected_snapshot)
                                            : std::nullopt;
  const FiniteExecutionCandidateView3D* const path_source =
      published_view.has_value() ? std::addressof(*published_view)
                                 : std::addressof(*candidate_view);
  const std::vector<mppi::TimedExecutionPathPoint> points =
      timedPathPoints(*path_source);
  std::optional<std::int64_t> reset_valid_until_ns;
  if (retained_published_route_continuation) {
    reset_valid_until_ns = path_source->valid_until_ns;
  } else if (emergency_braking_candidate) {
    reset_valid_until_ns = candidate_view->valid_until_ns;
  } else {
    reset_valid_until_ns =
        publicationValidUntilNs(*path_source, request.publication_now_ns);
  }
  if (points.empty() || !reset_valid_until_ns.has_value()) {
    return reject(ExecutionPublicationNavigationRebaseStatus3D::kPathUnavailable);
  }
  const std::int64_t source_valid_from_ns = retained_published_route_continuation
                                                ? path_source->valid_from_ns
                                                : request.publication_now_ns;
  const bool static_world = candidate_view->static_world != nullptr;
  if ((!static_world && (request.current_observed_raw_world == nullptr ||
                         !request.current_observed_raw_world->valid())) ||
      (static_world && !candidate_view->static_world->valid())) {
    return reject(ExecutionPublicationNavigationRebaseStatus3D::kEvidenceUnavailable);
  }
  const std::optional<LaunchSupportContact3D>& launch_support =
      !static_world ? request.current_observed_raw_world->launchSupportContact()
                    : std::optional<LaunchSupportContact3D>{};
  const mppi::FiniteExecutionPathWorld current_world{
      .flight_envelope = &candidate_view->policy->flightEnvelope(),
      .dynamics = &candidate_view->policy->dynamics(),
      .altitude_envelope = &candidate_view->policy->altitudeEnvelope(),
      .footprint = &candidate_view->policy->sweptFootprint(),
      .static_occupancy =
          static_world ? &candidate_view->static_world->occupancy() : nullptr,
      .observed_occupancy =
          !static_world ? &request.current_observed_raw_world->occupancy() : nullptr,
      .launch_support_contact =
          launch_support ? std::addressof(*launch_support) : nullptr,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = request.current_lidar_evidence->hitPointsMapM(),
      .terminal_boundary =
          emergency_braking_candidate ? std::nullopt : request.terminal_boundary,
  };
  std::optional<ExecutionRouteTransitionResult3D> transition;
  FiniteExecutionCertificationResult3D route_certification_diagnostic;
  ExecutionRouteTransitionStatus3D route_transition_diagnostic{
      ExecutionRouteTransitionStatus3D::kInvalidCandidate};
  mppi::FiniteExecutionPathCandidateValidator route_candidate_validator;
  if (request.candidate_snapshot->finite_execution.has_value() &&
      request.candidate_snapshot->route.has_value()) {
    route_candidate_validator =
        [&request, &transition, &route_certification_diagnostic,
         &route_transition_diagnostic](const mppi::FiniteHorizon& candidate) {
          ExecutionRouteTransitionResult3D candidate_transition =
              rebaseRouteExecution(request, candidate, route_certification_diagnostic,
                                   route_transition_diagnostic);
          if (!candidate_transition.applied() || candidate_transition.next == nullptr) {
            return false;
          }
          transition.emplace(std::move(candidate_transition));
          return true;
        };
  }
  const mppi::RebuiltFiniteExecutionPathContinuation rebuilt =
      mppi::rebuildFiniteExecutionPathContinuation(
          // Only a retained candidate is a continuation of controls that were
          // actually published and may have executed during commit. A new
          // candidate starts at control zero: motion under a different owner
          // does not prove that any of its controls ran. Reintegrate the whole
          // candidate from current navigation evidence and certify that result.
          // Emergency candidates keep their original stopping deadline through
          // reset_valid_until_ns even though their unpublished controls start
          // here.
          points, source_valid_from_ns, *reset_valid_until_ns,
          request.publication_now_ns, request.current_execution_input->state(),
          request.current_execution_input->previousControl(),
          path_source->nominal_prefix_control_count,
          // Preserve the complete remaining source sequence first, including its
          // already certified arrival tail. The validator may back off only the
          // minimum suffix that current evidence requires rebuilding.
          path_source->horizon->controls.size(), candidate_view->policy->dynamics(),
          request.arrival_search_step_controls, *request.finite_horizon_config,
          current_world, std::move(route_candidate_validator));
  result.path_validation_status = rebuilt.validation.status;
  result.source_control_index = rebuilt.source_control_index;
  result.route_certification_status = route_certification_diagnostic.status;
  result.route_adherence_status = route_certification_diagnostic.route_adherence_status;
  result.route_adherence_failure_state_index =
      route_certification_diagnostic.route_adherence_failure_state_index;
  result.route_adherence_failure_distance_m =
      route_certification_diagnostic.route_adherence_failure_distance_m;
  result.transition_status = route_transition_diagnostic;
  if (!rebuilt.accepted() || !rebuilt.horizon.has_value()) {
    return reject(ExecutionPublicationNavigationRebaseStatus3D::kPathRejected);
  }
  if (!transition.has_value() &&
      request.candidate_snapshot->direct_tracking_execution.has_value()) {
    transition.emplace(rebaseDirectExecution(request, rebuilt.horizon.value()));
  }
  if (transition.has_value()) {
    result.transition_status = transition->status;
  }
  if (!transition.has_value() || !transition->applied() ||
      transition->next == nullptr) {
    return reject(ExecutionPublicationNavigationRebaseStatus3D::kCertificationRejected);
  }
  result.status = ExecutionPublicationNavigationRebaseStatus3D::kRebased;
  result.transition.emplace(std::move(*transition));
  return result;
}

const char* executionPublicationNavigationRebaseStatus3DName(
    const ExecutionPublicationNavigationRebaseStatus3D status) noexcept {
  switch (status) {
    case ExecutionPublicationNavigationRebaseStatus3D::kRebased:
      return "late_rebase_rebased";
    case ExecutionPublicationNavigationRebaseStatus3D::kInvalidRequest:
      return "late_rebase_invalid_request";
    case ExecutionPublicationNavigationRebaseStatus3D::kEvidenceUnavailable:
      return "late_rebase_evidence_unavailable";
    case ExecutionPublicationNavigationRebaseStatus3D::kPathUnavailable:
      return "late_rebase_path_unavailable";
    case ExecutionPublicationNavigationRebaseStatus3D::kPathRejected:
      return "late_rebase_path_rejected";
    case ExecutionPublicationNavigationRebaseStatus3D::kCertificationRejected:
      return "late_rebase_certification_rejected";
  }
  return "late_rebase_unknown";
}

} // namespace drone_city_nav
