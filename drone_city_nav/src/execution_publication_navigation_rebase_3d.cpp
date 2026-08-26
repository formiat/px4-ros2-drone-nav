#include "execution_publication_navigation_rebase_3d.hpp"

#include <cmath>
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

[[nodiscard]] ExecutionRouteTransitionResult3D
rebaseRouteExecution(const ExecutionPublicationNavigationRebaseRequest3D& request,
                     const mppi::FiniteHorizon& horizon,
                     FiniteExecutionCertificationResult3D& diagnostic) {
  const ExecutionRouteSnapshot3D& expected = *request.expected_snapshot;
  const ExecutionRouteSnapshot3D& candidate = *request.candidate_snapshot;
  if (!candidate.finite_execution.has_value() || !candidate.route.has_value()) {
    return {};
  }
  const FiniteExecutionState3D& candidate_execution =
      candidate.finite_execution.value();
  const bool retaining_route =
      expected.route.has_value() && expected.route->identity.generation ==
                                        candidate.route.value().identity.generation;
  const CertifiedRouteSuffix3D target_route =
      retaining_route ? expected.route.value() : candidate.route.value();
  const FiniteExecutionCertificationResult3D certified =
      certifyFiniteExecution3DDetailed(
          expected, target_route,
          FiniteExecutionCertification3D{
              .trajectory_revision = candidate_execution.trajectory_revision,
              .horizon = horizon,
              .execution_input = request.current_execution_input,
              .latest_lidar_evidence = request.current_lidar_evidence,
              .valid_from_ns = request.publication_now_ns,
              .kind = candidate_execution.kind,
          });
  diagnostic.status = certified.status;
  diagnostic.route_adherence_status = certified.route_adherence_status;
  diagnostic.route_adherence_failure_state_index =
      certified.route_adherence_failure_state_index;
  diagnostic.route_adherence_failure_distance_m =
      certified.route_adherence_failure_distance_m;
  if (!certified.certified() || !certified.execution.has_value()) {
    return {};
  }
  if (expected.phase == ExecutionRoutePhase3D::kDirectTracking) {
    return transferDirectTrackingToCertifiedRoute3D(
        expected, expected.version, target_route, certified.execution.value());
  }
  if (!expected.route.has_value()) {
    return activateCertifiedRoute3D(expected, expected.version, target_route,
                                    *certified.execution);
  }
  const ExecutionRouteTransitionGuard3D guard{
      .expected_snapshot_version = expected.version,
      .expected_route_generation = expected.route->identity.generation,
      .expected_geometry_revision =
          expected.route->geometry->executable_geometry_revision,
  };
  if (retaining_route) {
    return replaceFiniteExecution3D(expected, guard, certified.execution);
  }
  if (request.expected_pending == nullptr ||
      !request.expected_pending->route_splice.has_value()) {
    return {};
  }
  return replaceCertifiedRoute3D(expected, guard, target_route, certified.execution,
                                 *request.expected_pending->route_splice);
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
  if (request.expected_snapshot == nullptr || request.candidate_snapshot == nullptr ||
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
      !assessLatestLidarEvidenceFreshness3D(
           *request.current_lidar_evidence, request.publication_now_ns,
           candidate_view->policy->latestLidarMaximumAgeMs())
           .fresh) {
    return reject(ExecutionPublicationNavigationRebaseStatus3D::kEvidenceUnavailable);
  }
  const std::vector<mppi::TimedExecutionPathPoint> points =
      timedPathPoints(*candidate_view);
  if (points.empty()) {
    return reject(ExecutionPublicationNavigationRebaseStatus3D::kPathUnavailable);
  }
  const bool static_world = candidate_view->static_world != nullptr;
  if ((!static_world && (request.current_observed_raw_world == nullptr ||
                         !request.current_observed_raw_world->valid())) ||
      (static_world && !candidate_view->static_world->valid())) {
    return reject(ExecutionPublicationNavigationRebaseStatus3D::kEvidenceUnavailable);
  }
  const std::optional<ProprioceptiveFreeSpaceSeed3D>& seed =
      !static_world ? request.current_observed_raw_world->proprioceptiveFreeSpaceSeed()
                    : std::optional<ProprioceptiveFreeSpaceSeed3D>{};
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
      .require_known_free_space = static_world,
      .proprioceptive_free_space_seed = seed ? std::addressof(*seed) : nullptr,
      .launch_support_contact =
          launch_support ? std::addressof(*launch_support) : nullptr,
      .raw_occupancy = nullptr,
      .latest_lidar_obstacle_points = request.current_lidar_evidence->hitPointsMapM(),
      .terminal_boundary = request.terminal_boundary,
  };
  std::optional<ExecutionRouteTransitionResult3D> transition;
  FiniteExecutionCertificationResult3D route_certification_diagnostic;
  mppi::FiniteExecutionPathCandidateValidator route_candidate_validator;
  if (request.candidate_snapshot->finite_execution.has_value() &&
      request.candidate_snapshot->route.has_value()) {
    route_candidate_validator =
        [&request, &transition,
         &route_certification_diagnostic](const mppi::FiniteHorizon& candidate) {
          ExecutionRouteTransitionResult3D candidate_transition =
              rebaseRouteExecution(request, candidate, route_certification_diagnostic);
          if (!candidate_transition.applied() || candidate_transition.next == nullptr) {
            return false;
          }
          transition.emplace(std::move(candidate_transition));
          return true;
        };
  }
  const mppi::RebuiltFiniteExecutionPathContinuation rebuilt =
      mppi::rebuildFiniteExecutionPathContinuation(
          points, candidate_view->valid_from_ns, candidate_view->valid_until_ns,
          request.publication_now_ns, request.current_execution_input->state(),
          request.current_execution_input->previousControl(),
          candidate_view->nominal_prefix_control_count,
          candidate_view->policy->dynamics(), request.arrival_search_step_controls,
          *request.finite_horizon_config, current_world,
          std::move(route_candidate_validator));
  result.path_validation_status = rebuilt.validation.status;
  result.route_certification_status = route_certification_diagnostic.status;
  result.route_adherence_status = route_certification_diagnostic.route_adherence_status;
  result.route_adherence_failure_state_index =
      route_certification_diagnostic.route_adherence_failure_state_index;
  result.route_adherence_failure_distance_m =
      route_certification_diagnostic.route_adherence_failure_distance_m;
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
