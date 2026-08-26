#include "drone_city_nav/route_strategy_arbitrator_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kMissionIdentityToleranceM{1.0e-6};
constexpr double kBudgetToleranceM{1.0e-6};

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool sameMission(const Point3& first, const Point3& second) noexcept {
  return finitePoint(first) && finitePoint(second) &&
         distance3D(first, second) <= kMissionIdentityToleranceM;
}

[[nodiscard]] bool strategicKind(const RouteStrategyKind3D kind) noexcept {
  return kind == RouteStrategyKind3D::kTopologyMission ||
         kind == RouteStrategyKind3D::kObservationFrontier ||
         kind == RouteStrategyKind3D::kTopologicalBacktrack;
}

[[nodiscard]] bool
releaseAction(const RouteStrategyArbitrationAction3D action) noexcept {
  return action == RouteStrategyArbitrationAction3D::kLeaseReleasedMissionChanged ||
         action == RouteStrategyArbitrationAction3D::kLeaseReleasedIntentCompleted ||
         action == RouteStrategyArbitrationAction3D::kLeaseReleasedMissionTarget ||
         action == RouteStrategyArbitrationAction3D::kLeaseReleasedBudgetExhausted ||
         action == RouteStrategyArbitrationAction3D::kLeaseReleasedCandidateInvalid ||
         action == RouteStrategyArbitrationAction3D::kLeaseReleasedDirectAdvantage;
}

[[nodiscard]] bool reasonMatchesKind(const RouteStrategyKind3D kind,
                                     const RouteStrategyLeaseReason3D reason) noexcept {
  switch (kind) {
    case RouteStrategyKind3D::kTopologyMission:
      return reason == RouteStrategyLeaseReason3D::kMissionTopologyContinuation;
    case RouteStrategyKind3D::kObservationFrontier:
      return reason == RouteStrategyLeaseReason3D::kObservationFrontier;
    case RouteStrategyKind3D::kTopologicalBacktrack:
      return reason == RouteStrategyLeaseReason3D::kBacktrackConfirmedTerminal ||
             reason == RouteStrategyLeaseReason3D::kBacktrackNoReachableFrontier ||
             reason ==
                 RouteStrategyLeaseReason3D::kBacktrackAllReachableBranchesExplored;
    case RouteStrategyKind3D::kNone:
    case RouteStrategyKind3D::kDirect:
    case RouteStrategyKind3D::kLaunchDeparture:
      return reason == RouteStrategyLeaseReason3D::kNone;
  }
  return false;
}

[[nodiscard]] bool leaseable(const RouteProposal3D& proposal) noexcept {
  const RouteStrategyKind3D kind = routeStrategyKind3D(proposal.intent);
  return routeProposalEligible3D(proposal) && strategicKind(kind) &&
         proposal.intent.strategic_plan_id != 0U &&
         proposal.intent.return_lineage.validForMission(
             proposal.intent.mission_target) &&
         proposal.intent.return_lineage.strategic_plan_id ==
             proposal.intent.strategic_plan_id &&
         proposal.intent.return_lineage.excursion_target_identity ==
             proposal.intent.target_identity &&
         proposal.intent.return_lineage.planned_on_revision ==
             proposal.intent.source_graph_revision &&
         reasonMatchesKind(kind, proposal.intent.lease_reason);
}

[[nodiscard]] bool leaseMatchesIntent(const RouteStrategyLease3D& lease,
                                      const RouteIntent3D& intent) noexcept {
  return lease.valid() && routeStrategyKind3D(intent) == lease.kind &&
         intent.strategic_plan_id == lease.strategic_plan_id &&
         intent.target_identity == lease.target_identity &&
         intent.return_lineage.validForMission(intent.mission_target) &&
         intent.return_lineage.id == lease.return_lineage.id &&
         sameMission(intent.mission_target, lease.mission_target);
}

[[nodiscard]] bool leaseMatchesProposal(const RouteStrategyLease3D& lease,
                                        const RouteProposal3D& proposal) noexcept {
  return leaseable(proposal) && leaseMatchesIntent(lease, proposal.intent);
}

[[nodiscard]] bool retiredMatchesProposal(const RetiredRouteStrategyLineage3D& retired,
                                          const RouteProposal3D& proposal) noexcept {
  const RouteStrategyKind3D kind = routeStrategyKind3D(proposal.intent);
  return retired.valid() && leaseable(proposal) && retired.kind == kind &&
         sameMission(retired.mission_target, proposal.intent.mission_target) &&
         (retired.return_lineage_id == proposal.intent.return_lineage.id ||
          retired.target_identity == proposal.intent.target_identity);
}

template<typename Predicate>
[[nodiscard]] std::optional<std::size_t>
bestCandidate(const std::span<const RouteProposal3D> proposals,
              const RouteProposalSelection3DConfig& config,
              const Predicate& predicate) noexcept {
  std::optional<std::size_t> selected;
  for (std::size_t index = 0U; index < proposals.size(); ++index) {
    if (!routeProposalEligible3D(proposals[index]) || !predicate(proposals[index])) {
      continue;
    }
    if (!selected.has_value() ||
        betterRouteProposal3D(proposals[index], proposals[*selected], config)) {
      selected = index;
    }
  }
  return selected;
}

[[nodiscard]] double progressRatio(const RouteProposal3D& proposal) noexcept {
  return proposal.evidence.route_length_m > 0.0 &&
                 std::isfinite(proposal.evidence.route_length_m) &&
                 std::isfinite(proposal.evidence.mission_progress_m)
             ? proposal.evidence.mission_progress_m / proposal.evidence.route_length_m
             : -std::numeric_limits<double>::infinity();
}

[[nodiscard]] bool
directHasReleaseAdvantage(const RouteProposal3D& direct,
                          const RouteProposal3D& strategic,
                          const RouteProposalSelection3DConfig& proposal_config,
                          const RouteStrategyArbitration3DConfig& config) noexcept {
  return isProductiveDirectTransit3D(direct, proposal_config) &&
         direct.evidence.mission_progress_m >=
             strategic.evidence.mission_progress_m +
                 config.direct_release_minimum_progress_advantage_m &&
         progressRatio(direct) >=
             progressRatio(strategic) +
                 config.direct_release_minimum_progress_ratio_advantage;
}

[[nodiscard]] double
budgetForKind(const RouteStrategyKind3D kind,
              const RouteStrategyArbitration3DConfig& config) noexcept {
  switch (kind) {
    case RouteStrategyKind3D::kTopologyMission:
      return config.topology_mission_lease_budget_m;
    case RouteStrategyKind3D::kObservationFrontier:
      return config.observation_frontier_lease_budget_m;
    case RouteStrategyKind3D::kTopologicalBacktrack:
      return config.topological_backtrack_lease_budget_m;
    case RouteStrategyKind3D::kNone:
    case RouteStrategyKind3D::kDirect:
    case RouteStrategyKind3D::kLaunchDeparture:
      return 0.0;
  }
  return 0.0;
}

[[nodiscard]] RetiredRouteStrategyLineage3D
retireLease(const RouteStrategyLease3D& lease,
            const RouteStrategyArbitrationObservation3D& observation) noexcept {
  return RetiredRouteStrategyLineage3D{
      .kind = lease.kind,
      .return_lineage_id = lease.return_lineage.id,
      .target_identity = lease.target_identity,
      .mission_target = lease.mission_target,
      .retired_position = observation.position,
      .mission_distance_at_retirement_m =
          distance3D(observation.position, lease.mission_target),
  };
}

void releaseLease(RouteStrategyArbitrationState3D& state,
                  const RouteStrategyArbitrationObservation3D& observation,
                  const bool require_return) noexcept {
  if (!state.lease.has_value()) {
    return;
  }
  if (require_return && finitePoint(observation.position)) {
    state.retired_lineage = retireLease(*state.lease, observation);
  }
  state.lease.reset();
}

[[nodiscard]] RouteStrategyArbitrationState3D
observeState(RouteStrategyArbitrationState3D state,
             const RouteStrategyArbitrationObservation3D& observation,
             const RouteStrategyArbitration3DConfig& config,
             RouteStrategyArbitrationAction3D& action) noexcept {
  if (!finitePoint(observation.position) || !finitePoint(observation.mission_target)) {
    return state;
  }
  if (state.lease.has_value()) {
    RouteStrategyLease3D& lease = *state.lease;
    if (!sameMission(lease.mission_target, observation.mission_target)) {
      state.lease.reset();
      state.retired_lineage.reset();
      action = RouteStrategyArbitrationAction3D::kLeaseReleasedMissionChanged;
    } else {
      const double travelled_delta_m =
          distance3D(lease.last_observed_position, observation.position);
      if (std::isfinite(travelled_delta_m)) {
        lease.travelled_m += travelled_delta_m;
      }
      lease.last_observed_position = observation.position;
      lease.last_observed_revision =
          std::max(lease.last_observed_revision, observation.world_revision);
    }
  }
  if (state.retired_lineage.has_value()) {
    const RetiredRouteStrategyLineage3D& retired = *state.retired_lineage;
    if (!sameMission(retired.mission_target, observation.mission_target)) {
      state.retired_lineage.reset();
    } else {
      const double mission_distance_m =
          distance3D(observation.position, retired.mission_target);
      if (std::isfinite(mission_distance_m) &&
          mission_distance_m + config.minimum_return_mission_progress_m <=
              retired.mission_distance_at_retirement_m) {
        state.retired_lineage.reset();
      }
    }
  }
  return state;
}

[[nodiscard]] RouteStrategyLease3D
makeLease(const RouteProposal3D& proposal,
          const RouteStrategyArbitrationObservation3D& observation,
          const RouteStrategyArbitration3DConfig& config,
          const std::uint64_t lease_id) noexcept {
  return RouteStrategyLease3D{
      .lease_id = lease_id,
      .kind = routeStrategyKind3D(proposal.intent),
      .reason = proposal.intent.lease_reason,
      .intent_id = proposal.intent.id,
      .strategic_plan_id = proposal.intent.strategic_plan_id,
      .target_identity = proposal.intent.target_identity,
      .return_lineage = proposal.intent.return_lineage,
      .mission_target = proposal.intent.mission_target,
      .acquisition_position = observation.position,
      .last_observed_position = observation.position,
      .acquired_revision = observation.world_revision,
      .last_observed_revision = observation.world_revision,
      .distance_budget_m = budgetForKind(routeStrategyKind3D(proposal.intent), config),
  };
}

void selectIndex(RouteProposalSelection3D& selection,
                 const std::optional<std::size_t> index,
                 const RouteProposalSelectionReason3D reason) noexcept {
  selection.selected_index = index;
  selection.reason =
      index.has_value() ? reason : RouteProposalSelectionReason3D::kNoEligibleCandidate;
}

} // namespace

bool routeStrategyArbitration3DConfigIsValid(
    const RouteStrategyArbitration3DConfig& config) noexcept {
  return std::isfinite(config.topology_mission_lease_budget_m) &&
         config.topology_mission_lease_budget_m > 0.0 &&
         std::isfinite(config.observation_frontier_lease_budget_m) &&
         config.observation_frontier_lease_budget_m > 0.0 &&
         std::isfinite(config.topological_backtrack_lease_budget_m) &&
         config.topological_backtrack_lease_budget_m > 0.0 &&
         std::isfinite(config.minimum_lease_commitment_m) &&
         config.minimum_lease_commitment_m >= 0.0 &&
         std::isfinite(config.minimum_return_mission_progress_m) &&
         config.minimum_return_mission_progress_m > 0.0 &&
         std::isfinite(config.direct_release_minimum_progress_advantage_m) &&
         config.direct_release_minimum_progress_advantage_m >= 0.0 &&
         std::isfinite(config.direct_release_minimum_progress_ratio_advantage) &&
         config.direct_release_minimum_progress_ratio_advantage >= 0.0 &&
         config.direct_release_minimum_progress_ratio_advantage <= 1.0 &&
         config.direct_release_confirmation_count > 0U;
}

RouteStrategyKind3D routeStrategyKind3D(const RouteIntent3D& intent) noexcept {
  if (intent.source == RouteIntentSource3D::kLaunchDeparture ||
      intent.purpose == RouteIntentPurpose3D::kLaunchDeparture) {
    return RouteStrategyKind3D::kLaunchDeparture;
  }
  if (intent.source == RouteIntentSource3D::kDirect) {
    return RouteStrategyKind3D::kDirect;
  }
  if (intent.source != RouteIntentSource3D::kTopology) {
    return RouteStrategyKind3D::kNone;
  }
  switch (intent.purpose) {
    case RouteIntentPurpose3D::kMissionTransit:
      return RouteStrategyKind3D::kTopologyMission;
    case RouteIntentPurpose3D::kObservationFrontier:
      return RouteStrategyKind3D::kObservationFrontier;
    case RouteIntentPurpose3D::kTopologicalBacktrack:
      return RouteStrategyKind3D::kTopologicalBacktrack;
    case RouteIntentPurpose3D::kLaunchDeparture:
      return RouteStrategyKind3D::kLaunchDeparture;
  }
  return RouteStrategyKind3D::kNone;
}

bool RouteStrategyLease3D::valid() const noexcept {
  return lease_id != 0U && strategicKind(kind) && reasonMatchesKind(kind, reason) &&
         intent_id != 0U && strategic_plan_id != 0U && target_identity != 0U &&
         return_lineage.validForMission(mission_target) &&
         return_lineage.strategic_plan_id == strategic_plan_id &&
         return_lineage.excursion_target_identity == target_identity &&
         finitePoint(mission_target) && finitePoint(acquisition_position) &&
         finitePoint(last_observed_position) && acquired_revision != 0U &&
         last_observed_revision >= acquired_revision &&
         std::isfinite(distance_budget_m) && distance_budget_m > 0.0 &&
         std::isfinite(travelled_m) && travelled_m >= 0.0;
}

double RouteStrategyLease3D::remainingBudgetM() const noexcept {
  return valid() ? std::max(0.0, distance_budget_m - travelled_m) : 0.0;
}

bool RetiredRouteStrategyLineage3D::valid() const noexcept {
  return strategicKind(kind) && return_lineage_id != 0U && target_identity != 0U &&
         finitePoint(mission_target) && finitePoint(retired_position) &&
         std::isfinite(mission_distance_at_retirement_m) &&
         mission_distance_at_retirement_m >= 0.0;
}

RouteStrategyArbitrator3D::RouteStrategyArbitrator3D(
    const RouteStrategyArbitration3DConfig& config)
    : config_{config} {
  if (!routeStrategyArbitration3DConfigIsValid(config_)) {
    throw std::invalid_argument{"invalid route strategy arbitration configuration"};
  }
}

RouteStrategyArbitrationDecision3D RouteStrategyArbitrator3D::evaluate(
    const std::span<const RouteProposal3D> proposals,
    const RouteProposalSelection3DConfig& proposal_config,
    const RouteStrategyArbitrationObservation3D& observation) noexcept {
  RouteStrategyArbitrationDecision3D decision;
  if (pending_decision_sequence_.has_value()) {
    decision.action = RouteStrategyArbitrationAction3D::kPendingOutcome;
    decision.state_without_commit = state_;
    decision.state_after_commit = state_;
    return decision;
  }
  if (last_decision_sequence_ == std::numeric_limits<std::uint64_t>::max() ||
      !routeProposalSelection3DConfigIsValid(proposal_config) ||
      !finitePoint(observation.position) || !finitePoint(observation.mission_target) ||
      observation.world_revision == 0U) {
    decision.state_without_commit = state_;
    decision.state_after_commit = state_;
    return decision;
  }

  decision.sequence = ++last_decision_sequence_;
  pending_decision_sequence_ = decision.sequence;
  decision.selection = selectRouteProposal3D(proposals, proposal_config);
  decision.action = decision.selection.selected_index.has_value()
                        ? RouteStrategyArbitrationAction3D::kStatelessSelection
                        : RouteStrategyArbitrationAction3D::kNoSelection;
  if (!config_.leases_enabled) {
    decision.state_without_commit = state_;
    decision.state_after_commit = state_;
    return decision;
  }
  RouteStrategyArbitrationState3D observed =
      observeState(state_, observation, config_, decision.action);
  decision.state_without_commit = observed;
  decision.state_after_commit = observed;

  if (observed.lease.has_value() && observation.active_intent_completed &&
      observation.active_intent.has_value() &&
      leaseMatchesIntent(*observed.lease, *observation.active_intent)) {
    const bool mission_complete =
        observation.active_intent->intent_reaches_mission_target;
    releaseLease(observed, observation, !mission_complete);
    decision.action = RouteStrategyArbitrationAction3D::kLeaseReleasedIntentCompleted;
  }
  if (observed.lease.has_value() &&
      observed.lease->remainingBudgetM() <= kBudgetToleranceM) {
    releaseLease(observed, observation, true);
    decision.action = RouteStrategyArbitrationAction3D::kLeaseReleasedBudgetExhausted;
  }

  if (observed.lease.has_value()) {
    const std::optional<std::size_t> leased_index =
        bestCandidate(proposals, proposal_config, [&](const RouteProposal3D& proposal) {
          return leaseMatchesProposal(*observed.lease, proposal);
        });
    if (!leased_index.has_value()) {
      // Candidate materialization may intentionally retain the immutable
      // resident route instead of publishing an equivalent replacement. The
      // absence of a new eligible proposal therefore does not invalidate a
      // lease while that exact strategic intent is still executing.
      const bool resident_strategy_route_active =
          observation.active_intent.has_value() &&
          observation.active_intent_executable &&
          !observation.active_intent_completed &&
          leaseMatchesIntent(*observed.lease, *observation.active_intent);
      if (resident_strategy_route_active) {
        decision.selection.selected_index.reset();
        decision.selection.reason =
            RouteProposalSelectionReason3D::kActiveStrategyLease;
        decision.state_without_commit = observed;
        decision.state_after_commit = observed;
        decision.action = RouteStrategyArbitrationAction3D::kLeaseResidentRouteRetained;
        return decision;
      }
      releaseLease(observed, observation, true);
      decision.action =
          RouteStrategyArbitrationAction3D::kLeaseReleasedCandidateInvalid;
    } else {
      const std::optional<std::size_t> mission_index = bestCandidate(
          proposals, proposal_config, [](const RouteProposal3D& proposal) {
            return proposal.evidence.reaches_mission_target;
          });
      if (mission_index.has_value() && *mission_index != *leased_index) {
        selectIndex(decision.selection, mission_index,
                    RouteProposalSelectionReason3D::kMissionTarget);
        decision.state_without_commit = observed;
        decision.state_after_commit = observed;
        releaseLease(decision.state_after_commit, observation, false);
        decision.state_after_commit.retired_lineage.reset();
        decision.action = RouteStrategyArbitrationAction3D::kLeaseReleasedMissionTarget;
        return decision;
      }

      const std::optional<std::size_t> direct_index = bestCandidate(
          proposals, proposal_config, [&](const RouteProposal3D& proposal) {
            return routeStrategyKind3D(proposal.intent) ==
                       RouteStrategyKind3D::kDirect &&
                   isProductiveDirectTransit3D(proposal, proposal_config);
          });
      RouteStrategyLease3D& lease = *observed.lease;
      // A short direct segment only proves local progress. It may shorten an
      // exploration or return excursion, but it cannot supersede the only
      // candidate that proves strategic continuation toward the mission.
      // A direct candidate that actually reaches the mission was handled by
      // the mission-target preemption above.
      const bool direct_advantage =
          lease.kind != RouteStrategyKind3D::kTopologyMission &&
          direct_index.has_value() &&
          directHasReleaseAdvantage(proposals[*direct_index], proposals[*leased_index],
                                    proposal_config, config_);
      lease.direct_advantage_confirmations =
          direct_advantage ? std::min(lease.direct_advantage_confirmations + 1U,
                                      config_.direct_release_confirmation_count)
                           : 0U;
      const bool release_confirmed =
          direct_advantage &&
          lease.direct_advantage_confirmations >=
              config_.direct_release_confirmation_count &&
          lease.travelled_m + kBudgetToleranceM >= config_.minimum_lease_commitment_m;
      if (release_confirmed) {
        selectIndex(decision.selection, direct_index,
                    RouteProposalSelectionReason3D::kProductiveDirectTransit);
        decision.state_without_commit = observed;
        decision.state_after_commit = observed;
        releaseLease(decision.state_after_commit, observation, true);
        decision.action =
            RouteStrategyArbitrationAction3D::kLeaseReleasedDirectAdvantage;
        return decision;
      }

      const bool hysteresis_override =
          decision.selection.selected_index != leased_index;
      selectIndex(decision.selection, leased_index,
                  hysteresis_override
                      ? RouteProposalSelectionReason3D::kStrategyLeaseHysteresis
                      : RouteProposalSelectionReason3D::kActiveStrategyLease);
      decision.state_without_commit = observed;
      decision.state_after_commit = observed;
      decision.action = hysteresis_override
                            ? RouteStrategyArbitrationAction3D::kLeaseHysteresisRetained
                            : RouteStrategyArbitrationAction3D::kLeaseRetained;
      return decision;
    }
  }

  if (decision.selection.selected_index.has_value() &&
      observed.retired_lineage.has_value() &&
      retiredMatchesProposal(*observed.retired_lineage,
                             proposals[*decision.selection.selected_index])) {
    const std::optional<std::size_t> alternative =
        bestCandidate(proposals, proposal_config, [&](const RouteProposal3D& proposal) {
          return !retiredMatchesProposal(*observed.retired_lineage, proposal);
        });
    if (alternative.has_value()) {
      selectIndex(decision.selection, alternative,
                  RouteProposalSelectionReason3D::kStrategyReturnRequired);
    }
    if (!releaseAction(decision.action)) {
      decision.action = RouteStrategyArbitrationAction3D::kRetiredLineageSuppressed;
    }
  }

  if (decision.selection.selected_index.has_value()) {
    const RouteProposal3D& selected = proposals[*decision.selection.selected_index];
    const RouteStrategyKind3D selected_kind = routeStrategyKind3D(selected.intent);
    if (strategicKind(selected_kind)) {
      if (!leaseable(selected) || observed.last_allocated_lease_id ==
                                      std::numeric_limits<std::uint64_t>::max()) {
        const std::optional<std::size_t> fallback = bestCandidate(
            proposals, proposal_config, [](const RouteProposal3D& proposal) {
              return !strategicKind(routeStrategyKind3D(proposal.intent));
            });
        selectIndex(decision.selection, fallback,
                    RouteProposalSelectionReason3D::kInvalidStrategyLineageFallback);
        decision.action = RouteStrategyArbitrationAction3D::kInvalidStrategicLineage;
      } else if (!observed.retired_lineage.has_value() ||
                 !retiredMatchesProposal(*observed.retired_lineage, selected)) {
        RouteStrategyArbitrationState3D acquired = observed;
        ++acquired.last_allocated_lease_id;
        acquired.lease =
            makeLease(selected, observation, config_, acquired.last_allocated_lease_id);
        if (acquired.lease->valid()) {
          decision.state_after_commit = acquired;
          decision.action = RouteStrategyArbitrationAction3D::kLeaseAcquired;
        }
      }
    }
  }

  decision.state_without_commit = observed;
  if (decision.action != RouteStrategyArbitrationAction3D::kLeaseAcquired) {
    decision.state_after_commit = observed;
  }
  return decision;
}

bool RouteStrategyArbitrator3D::recordOutcome(
    const RouteStrategyArbitrationDecision3D& decision,
    const bool selection_committed) noexcept {
  if (!pending_decision_sequence_.has_value() || decision.sequence == 0U ||
      *pending_decision_sequence_ != decision.sequence) {
    return false;
  }
  state_ =
      selection_committed ? decision.state_after_commit : decision.state_without_commit;
  pending_decision_sequence_.reset();
  return true;
}

void RouteStrategyArbitrator3D::reset() noexcept {
  state_.lease.reset();
  state_.retired_lineage.reset();
  pending_decision_sequence_.reset();
}

const RouteStrategyArbitrationState3D&
RouteStrategyArbitrator3D::state() const noexcept {
  return state_;
}

const RouteStrategyArbitration3DConfig&
RouteStrategyArbitrator3D::config() const noexcept {
  return config_;
}

std::string_view routeStrategyKind3DName(const RouteStrategyKind3D kind) noexcept {
  switch (kind) {
    case RouteStrategyKind3D::kNone:
      return "none";
    case RouteStrategyKind3D::kDirect:
      return "direct";
    case RouteStrategyKind3D::kTopologyMission:
      return "topology_mission";
    case RouteStrategyKind3D::kObservationFrontier:
      return "observation_frontier";
    case RouteStrategyKind3D::kTopologicalBacktrack:
      return "topological_backtrack";
    case RouteStrategyKind3D::kLaunchDeparture:
      return "launch_departure";
  }
  return "unknown";
}

std::string_view routeStrategyArbitrationAction3DName(
    const RouteStrategyArbitrationAction3D action) noexcept {
  switch (action) {
    case RouteStrategyArbitrationAction3D::kNoSelection:
      return "no_selection";
    case RouteStrategyArbitrationAction3D::kStatelessSelection:
      return "stateless_selection";
    case RouteStrategyArbitrationAction3D::kLeaseAcquired:
      return "lease_acquired";
    case RouteStrategyArbitrationAction3D::kLeaseRetained:
      return "lease_retained";
    case RouteStrategyArbitrationAction3D::kLeaseResidentRouteRetained:
      return "lease_resident_route_retained";
    case RouteStrategyArbitrationAction3D::kLeaseHysteresisRetained:
      return "lease_hysteresis_retained";
    case RouteStrategyArbitrationAction3D::kLeaseReleasedMissionChanged:
      return "lease_released_mission_changed";
    case RouteStrategyArbitrationAction3D::kLeaseReleasedIntentCompleted:
      return "lease_released_intent_completed";
    case RouteStrategyArbitrationAction3D::kLeaseReleasedMissionTarget:
      return "lease_released_mission_target";
    case RouteStrategyArbitrationAction3D::kLeaseReleasedBudgetExhausted:
      return "lease_released_budget_exhausted";
    case RouteStrategyArbitrationAction3D::kLeaseReleasedCandidateInvalid:
      return "lease_released_candidate_invalid";
    case RouteStrategyArbitrationAction3D::kLeaseReleasedDirectAdvantage:
      return "lease_released_direct_advantage";
    case RouteStrategyArbitrationAction3D::kRetiredLineageSuppressed:
      return "retired_lineage_suppressed";
    case RouteStrategyArbitrationAction3D::kInvalidStrategicLineage:
      return "invalid_strategic_lineage";
    case RouteStrategyArbitrationAction3D::kPendingOutcome:
      return "pending_outcome";
  }
  return "unknown";
}

} // namespace drone_city_nav
