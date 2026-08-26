#include "drone_city_nav/route_strategy_arbitrator_3d.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr Point3 kMission{100.0, 0.0, 5.0};

[[nodiscard]] RouteStrategyLeaseReason3D
reasonFor(const RouteIntentPurpose3D purpose) noexcept {
  switch (purpose) {
    case RouteIntentPurpose3D::kMissionTransit:
      return RouteStrategyLeaseReason3D::kMissionTopologyContinuation;
    case RouteIntentPurpose3D::kObservationFrontier:
      return RouteStrategyLeaseReason3D::kObservationFrontier;
    case RouteIntentPurpose3D::kTopologicalBacktrack:
      return RouteStrategyLeaseReason3D::kBacktrackConfirmedTerminal;
    case RouteIntentPurpose3D::kLaunchDeparture:
      return RouteStrategyLeaseReason3D::kNone;
  }
  return RouteStrategyLeaseReason3D::kNone;
}

[[nodiscard]] RouteProposal3D directProposal(const double mission_progress_m,
                                             const double route_length_m = 20.0,
                                             const bool reaches_mission = false,
                                             const std::uint64_t fingerprint = 10U) {
  return RouteProposal3D{
      .intent = {.id = fingerprint + 100U,
                 .planned_on_revision = 10U,
                 .mission_target = kMission,
                 .intent_target = {20.0, 0.0, 5.0},
                 .segment_target = {20.0, 0.0, 5.0},
                 .source = RouteIntentSource3D::kDirect,
                 .purpose = RouteIntentPurpose3D::kMissionTransit,
                 .valid = true},
      .evidence = {.status = SegmentEvidenceStatus3D::kValid,
                   .route_length_m = route_length_m,
                   .endpoint_displacement_m = route_length_m,
                   .mission_progress_m = mission_progress_m,
                   .objective_cost = 10.0,
                   .physical_executable = true,
                   .reaches_segment_target = true,
                   .reaches_intent_target = true,
                   .reaches_mission_target = reaches_mission},
      .route_fingerprint = fingerprint,
      .activation_eligible = true,
  };
}

[[nodiscard]] RouteProposal3D
strategicProposal(const RouteIntentPurpose3D purpose, const double mission_progress_m,
                  const std::uint64_t plan_id = 7U,
                  const std::uint64_t target_identity = 9U,
                  const std::uint64_t fingerprint = 20U) {
  const RouteStrategyReturnLineage3D lineage = makeRouteStrategyReturnLineage3D(
      plan_id, 101U, 10U, 3U, target_identity, kMission);
  return RouteProposal3D{
      .intent = {.id = fingerprint + 100U,
                 .strategic_plan_id = plan_id,
                 .planned_on_revision = 10U,
                 .source_graph_revision = 10U,
                 .target_identity = target_identity,
                 .return_lineage = lineage,
                 .mission_target = kMission,
                 .intent_target = {5.0, 10.0, 5.0},
                 .segment_target = {5.0, 5.0, 5.0},
                 .source = RouteIntentSource3D::kTopology,
                 .purpose = purpose,
                 .lease_reason = reasonFor(purpose),
                 .graph_step_count = 3U,
                 .strategic_continuation_available = true,
                 .strategic_mission_continuation =
                     purpose == RouteIntentPurpose3D::kMissionTransit,
                 .segment_reaches_intent_target = true,
                 .valid = true},
      .evidence = {.status = SegmentEvidenceStatus3D::kValid,
                   .route_length_m = 20.0,
                   .endpoint_displacement_m = 10.0,
                   .mission_progress_m = mission_progress_m,
                   .objective_cost = 5.0,
                   .physical_executable = true,
                   .reaches_segment_target = true,
                   .reaches_intent_target = true},
      .route_fingerprint = fingerprint,
      .activation_eligible = true,
  };
}

[[nodiscard]] RouteStrategyArbitrationObservation3D
observation(const Point3 position, const std::uint64_t revision = 10U) {
  return RouteStrategyArbitrationObservation3D{
      .position = position,
      .mission_target = kMission,
      .world_revision = revision,
      .active_intent = std::nullopt,
  };
}

constexpr RouteProposalSelection3DConfig kProposalConfig{.heuristic_precedence_enabled =
                                                             true};

constexpr RouteStrategyArbitration3DConfig kLeaseConfig{.leases_enabled = true};

[[nodiscard]] const RouteStrategyLease3D*
activeLease(const RouteStrategyArbitrator3D& arbitrator) noexcept {
  const std::optional<RouteStrategyLease3D>& lease = arbitrator.state().lease;
  return lease.has_value() ? std::addressof(lease.value()) : nullptr;
}

TEST(RouteStrategyArbitrator3DTest,
     StrategicMissionAcquisitionIsTransactionalAndOwnsReturnLineage) {
  RouteStrategyArbitrator3D arbitrator{kLeaseConfig};
  const std::vector<RouteProposal3D> proposals{
      directProposal(0.5),
      strategicProposal(RouteIntentPurpose3D::kMissionTransit, -2.0),
  };

  const RouteStrategyArbitrationDecision3D rejected =
      arbitrator.evaluate(proposals, kProposalConfig, observation({0.0, 0.0, 5.0}));
  ASSERT_EQ(rejected.selection.selected_index, std::optional<std::size_t>{1U});
  EXPECT_EQ(rejected.action, RouteStrategyArbitrationAction3D::kLeaseAcquired);
  EXPECT_TRUE(arbitrator.recordOutcome(rejected, false));
  EXPECT_FALSE(arbitrator.state().lease.has_value());
  EXPECT_EQ(arbitrator.state().last_allocated_lease_id, 0U);

  const RouteStrategyArbitrationDecision3D accepted = arbitrator.evaluate(
      proposals, kProposalConfig, observation({0.0, 0.0, 5.0}, 11U));
  EXPECT_TRUE(arbitrator.recordOutcome(accepted, true));
  const RouteStrategyLease3D* const lease = activeLease(arbitrator);
  ASSERT_NE(lease, nullptr);
  EXPECT_EQ(lease->kind, RouteStrategyKind3D::kTopologyMission);
  EXPECT_EQ(lease->reason, RouteStrategyLeaseReason3D::kMissionTopologyContinuation);
  EXPECT_EQ(lease->return_lineage.id, proposals[1].intent.return_lineage.id);
  EXPECT_EQ(lease->return_lineage.return_anchor_identity, 3U);
}

TEST(RouteStrategyArbitrator3DTest,
     DirectReleaseFromExplorationRequiresTravelAndRepeatedMeasuredAdvantage) {
  RouteStrategyArbitration3DConfig config;
  config.leases_enabled = true;
  config.minimum_lease_commitment_m = 5.0;
  config.direct_release_confirmation_count = 2U;
  config.direct_release_minimum_progress_advantage_m = 2.0;
  config.direct_release_minimum_progress_ratio_advantage = 0.0;
  RouteStrategyArbitrator3D arbitrator{config};
  const std::vector<RouteProposal3D> acquisition_proposals{
      directProposal(0.5),
      strategicProposal(RouteIntentPurpose3D::kObservationFrontier, -2.0),
  };
  const std::vector<RouteProposal3D> release_proposals{
      directProposal(20.0),
      acquisition_proposals[1],
  };
  RouteStrategyArbitrationDecision3D decision = arbitrator.evaluate(
      acquisition_proposals, kProposalConfig, observation({0.0, 0.0, 5.0}));
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));

  decision = arbitrator.evaluate(release_proposals, kProposalConfig,
                                 observation({6.0, 0.0, 5.0}, 11U));
  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{1U});
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));
  const RouteStrategyLease3D* const retained_lease = activeLease(arbitrator);
  ASSERT_NE(retained_lease, nullptr);
  EXPECT_EQ(retained_lease->direct_advantage_confirmations, 1U);

  decision = arbitrator.evaluate(release_proposals, kProposalConfig,
                                 observation({7.0, 0.0, 5.0}, 12U));
  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{0U});
  EXPECT_EQ(decision.action,
            RouteStrategyArbitrationAction3D::kLeaseReleasedDirectAdvantage);
  ASSERT_TRUE(arbitrator.recordOutcome(decision, false));
  EXPECT_TRUE(arbitrator.state().lease.has_value());

  decision = arbitrator.evaluate(release_proposals, kProposalConfig,
                                 observation({8.0, 0.0, 5.0}, 13U));
  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{0U});
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));
  EXPECT_FALSE(arbitrator.state().lease.has_value());
  EXPECT_TRUE(arbitrator.state().retired_lineage.has_value());
}

TEST(RouteStrategyArbitrator3DTest,
     LocalDirectProgressCannotPreemptStrategicMissionContinuation) {
  RouteStrategyArbitration3DConfig config;
  config.leases_enabled = true;
  config.minimum_lease_commitment_m = 5.0;
  config.direct_release_confirmation_count = 1U;
  config.direct_release_minimum_progress_advantage_m = 0.0;
  config.direct_release_minimum_progress_ratio_advantage = 0.0;
  RouteStrategyArbitrator3D arbitrator{config};
  const std::vector<RouteProposal3D> proposals{
      directProposal(0.5),
      strategicProposal(RouteIntentPurpose3D::kMissionTransit, -2.0),
  };
  RouteStrategyArbitrationDecision3D decision =
      arbitrator.evaluate(proposals, kProposalConfig, observation({0.0, 0.0, 5.0}));
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));

  decision = arbitrator.evaluate(proposals, kProposalConfig,
                                 observation({6.0, 0.0, 5.0}, 11U));

  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{1U});
  EXPECT_EQ(decision.action, RouteStrategyArbitrationAction3D::kLeaseRetained);
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));
  const RouteStrategyLease3D* const lease = activeLease(arbitrator);
  ASSERT_NE(lease, nullptr);
  EXPECT_EQ(lease->kind, RouteStrategyKind3D::kTopologyMission);
  EXPECT_EQ(lease->direct_advantage_confirmations, 0U);
}

TEST(RouteStrategyArbitrator3DTest,
     ResidentStrategicRouteRetainsLeaseWithoutEquivalentReplacement) {
  RouteStrategyArbitrator3D arbitrator{kLeaseConfig};
  const RouteProposal3D strategic =
      strategicProposal(RouteIntentPurpose3D::kObservationFrontier, -2.0);
  RouteStrategyArbitrationDecision3D decision =
      arbitrator.evaluate(std::vector<RouteProposal3D>{directProposal(0.5), strategic},
                          kProposalConfig, observation({0.0, 0.0, 5.0}));
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));

  RouteStrategyArbitrationObservation3D executing = observation({1.0, 0.0, 5.0}, 11U);
  executing.active_intent = strategic.intent;
  executing.active_intent_executable = true;
  RouteProposal3D rejected_replacement = strategic;
  rejected_replacement.activation_eligible = false;
  decision = arbitrator.evaluate(
      std::vector<RouteProposal3D>{directProposal(0.5), rejected_replacement},
      kProposalConfig, executing);

  EXPECT_FALSE(decision.selection.selected_index.has_value());
  EXPECT_EQ(decision.selection.reason,
            RouteProposalSelectionReason3D::kActiveStrategyLease);
  EXPECT_EQ(decision.action,
            RouteStrategyArbitrationAction3D::kLeaseResidentRouteRetained);
  ASSERT_TRUE(arbitrator.recordOutcome(decision, false));
  const RouteStrategyLease3D* const retained = activeLease(arbitrator);
  ASSERT_NE(retained, nullptr);
  EXPECT_EQ(retained->return_lineage.id, strategic.intent.return_lineage.id);
  EXPECT_FALSE(arbitrator.state().retired_lineage.has_value());
}

TEST(RouteStrategyArbitrator3DTest,
     MissingStrategicCandidateReleasesLeaseWithoutExecutableResidentRoute) {
  RouteStrategyArbitrator3D arbitrator{kLeaseConfig};
  const RouteProposal3D strategic =
      strategicProposal(RouteIntentPurpose3D::kObservationFrontier, -2.0);
  RouteStrategyArbitrationDecision3D decision =
      arbitrator.evaluate(std::vector<RouteProposal3D>{directProposal(0.5), strategic},
                          kProposalConfig, observation({0.0, 0.0, 5.0}));
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));

  RouteProposal3D rejected_replacement = strategic;
  rejected_replacement.activation_eligible = false;
  RouteStrategyArbitrationObservation3D invalidated = observation({1.0, 0.0, 5.0}, 11U);
  invalidated.active_intent = strategic.intent;
  decision = arbitrator.evaluate(
      std::vector<RouteProposal3D>{directProposal(0.5), rejected_replacement},
      kProposalConfig, invalidated);

  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{0U});
  EXPECT_EQ(decision.action,
            RouteStrategyArbitrationAction3D::kLeaseReleasedCandidateInvalid);
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));
  EXPECT_FALSE(arbitrator.state().lease.has_value());
  EXPECT_TRUE(arbitrator.state().retired_lineage.has_value());
}

TEST(RouteStrategyArbitrator3DTest,
     MissionTargetCandidateCanPreemptLeaseWithoutHysteresis) {
  RouteStrategyArbitrator3D arbitrator{kLeaseConfig};
  const RouteProposal3D strategic =
      strategicProposal(RouteIntentPurpose3D::kObservationFrontier, -1.0);
  RouteStrategyArbitrationDecision3D decision =
      arbitrator.evaluate(std::vector<RouteProposal3D>{directProposal(0.5), strategic},
                          kProposalConfig, observation({0.0, 0.0, 5.0}));
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));
  ASSERT_TRUE(arbitrator.state().lease.has_value());

  decision = arbitrator.evaluate(
      std::vector<RouteProposal3D>{directProposal(100.0, 100.0, true), strategic},
      kProposalConfig, observation({1.0, 0.0, 5.0}, 11U));
  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{0U});
  EXPECT_EQ(decision.action,
            RouteStrategyArbitrationAction3D::kLeaseReleasedMissionTarget);
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));
  EXPECT_FALSE(arbitrator.state().lease.has_value());
  EXPECT_FALSE(arbitrator.state().retired_lineage.has_value());
}

TEST(RouteStrategyArbitrator3DTest,
     ExhaustedFrontierBudgetRequiresMissionReturnBeforeReacquisition) {
  RouteStrategyArbitration3DConfig config;
  config.leases_enabled = true;
  config.observation_frontier_lease_budget_m = 5.0;
  config.minimum_return_mission_progress_m = 8.0;
  RouteStrategyArbitrator3D arbitrator{config};
  const std::vector<RouteProposal3D> proposals{
      directProposal(0.5),
      strategicProposal(RouteIntentPurpose3D::kObservationFrontier, -1.0),
  };
  RouteStrategyArbitrationDecision3D decision =
      arbitrator.evaluate(proposals, kProposalConfig, observation({0.0, 0.0, 5.0}));
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));

  decision = arbitrator.evaluate(proposals, kProposalConfig,
                                 observation({6.0, 0.0, 5.0}, 11U));
  EXPECT_EQ(decision.action,
            RouteStrategyArbitrationAction3D::kLeaseReleasedBudgetExhausted);
  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{0U});
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));
  EXPECT_TRUE(arbitrator.state().retired_lineage.has_value());

  decision = arbitrator.evaluate(proposals, kProposalConfig,
                                 observation({7.0, 0.0, 5.0}, 12U));
  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{0U});
  EXPECT_EQ(decision.selection.reason,
            RouteProposalSelectionReason3D::kStrategyReturnRequired);
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));

  decision = arbitrator.evaluate(proposals, kProposalConfig,
                                 observation({20.0, 0.0, 5.0}, 13U));
  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{1U});
  EXPECT_EQ(decision.action, RouteStrategyArbitrationAction3D::kLeaseAcquired);
  EXPECT_TRUE(arbitrator.recordOutcome(decision, true));
  EXPECT_TRUE(arbitrator.state().lease.has_value());
}

TEST(RouteStrategyArbitrator3DTest,
     BacktrackingLeaseCarriesExplicitReasonBudgetAndReturnAnchor) {
  RouteStrategyArbitration3DConfig config;
  config.leases_enabled = true;
  config.topological_backtrack_lease_budget_m = 42.0;
  RouteStrategyArbitrator3D arbitrator{config};
  const std::vector<RouteProposal3D> proposals{
      directProposal(-1.0),
      strategicProposal(RouteIntentPurpose3D::kTopologicalBacktrack, -10.0),
  };

  const RouteStrategyArbitrationDecision3D decision =
      arbitrator.evaluate(proposals, kProposalConfig, observation({0.0, 0.0, 5.0}));
  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{1U});
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));
  const RouteStrategyLease3D* const lease = activeLease(arbitrator);
  ASSERT_NE(lease, nullptr);
  EXPECT_EQ(lease->kind, RouteStrategyKind3D::kTopologicalBacktrack);
  EXPECT_EQ(lease->reason, RouteStrategyLeaseReason3D::kBacktrackConfirmedTerminal);
  EXPECT_DOUBLE_EQ(lease->distance_budget_m, 42.0);
  EXPECT_EQ(lease->return_lineage.return_anchor_identity, 3U);
}

TEST(RouteStrategyArbitrator3DTest,
     StrategicCandidateWithoutReturnLineageCannotOwnArbitration) {
  RouteStrategyArbitrator3D arbitrator{kLeaseConfig};
  RouteProposal3D invalid =
      strategicProposal(RouteIntentPurpose3D::kMissionTransit, -3.0);
  invalid.intent.return_lineage = {};
  const std::vector<RouteProposal3D> proposals{directProposal(0.5), invalid};

  const RouteStrategyArbitrationDecision3D decision =
      arbitrator.evaluate(proposals, kProposalConfig, observation({0.0, 0.0, 5.0}));

  EXPECT_EQ(decision.action,
            RouteStrategyArbitrationAction3D::kInvalidStrategicLineage);
  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{0U});
  EXPECT_EQ(decision.selection.reason,
            RouteProposalSelectionReason3D::kInvalidStrategyLineageFallback);
  EXPECT_TRUE(arbitrator.recordOutcome(decision, true));
  EXPECT_FALSE(arbitrator.state().lease.has_value());
}

TEST(RouteStrategyArbitrator3DTest,
     CompletedFrontierRetiresItsLineageUntilMissionProgressResumes) {
  RouteStrategyArbitrator3D arbitrator{kLeaseConfig};
  const RouteProposal3D frontier =
      strategicProposal(RouteIntentPurpose3D::kObservationFrontier, -1.0);
  const std::vector<RouteProposal3D> proposals{directProposal(0.5), frontier};
  RouteStrategyArbitrationDecision3D decision =
      arbitrator.evaluate(proposals, kProposalConfig, observation({0.0, 0.0, 5.0}));
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));

  RouteStrategyArbitrationObservation3D completed = observation({2.0, 0.0, 5.0}, 11U);
  completed.active_intent = frontier.intent;
  completed.active_intent_completed = true;
  decision = arbitrator.evaluate(proposals, kProposalConfig, completed);

  EXPECT_EQ(decision.action,
            RouteStrategyArbitrationAction3D::kLeaseReleasedIntentCompleted);
  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{0U});
  EXPECT_TRUE(arbitrator.recordOutcome(decision, true));
  EXPECT_FALSE(arbitrator.state().lease.has_value());
  EXPECT_TRUE(arbitrator.state().retired_lineage.has_value());
}

TEST(RouteStrategyArbitrator3DTest, PendingDecisionMustBeResolvedBeforeNextEvaluation) {
  RouteStrategyArbitrator3D arbitrator;
  const std::vector<RouteProposal3D> proposals{directProposal(5.0)};
  const RouteStrategyArbitrationDecision3D first =
      arbitrator.evaluate(proposals, kProposalConfig, observation({0.0, 0.0, 5.0}));

  const RouteStrategyArbitrationDecision3D blocked = arbitrator.evaluate(
      proposals, kProposalConfig, observation({1.0, 0.0, 5.0}, 11U));
  EXPECT_EQ(blocked.sequence, 0U);
  EXPECT_EQ(blocked.action, RouteStrategyArbitrationAction3D::kPendingOutcome);
  EXPECT_FALSE(arbitrator.recordOutcome(blocked, true));
  EXPECT_TRUE(arbitrator.recordOutcome(first, true));
}

TEST(RouteStrategyArbitrator3DTest, DefaultPolicySelectsWithoutLeaseOrHysteresis) {
  RouteStrategyArbitrator3D arbitrator;
  const std::vector<RouteProposal3D> proposals{
      directProposal(1.0, 5.0),
      strategicProposal(RouteIntentPurpose3D::kMissionTransit, 10.0),
  };

  const RouteStrategyArbitrationDecision3D decision = arbitrator.evaluate(
      proposals, RouteProposalSelection3DConfig{}, observation({0.0, 0.0, 5.0}));

  EXPECT_EQ(decision.selection.selected_index, std::optional<std::size_t>{1U});
  EXPECT_EQ(decision.action, RouteStrategyArbitrationAction3D::kStatelessSelection);
  ASSERT_TRUE(arbitrator.recordOutcome(decision, true));
  EXPECT_FALSE(arbitrator.state().lease.has_value());
  EXPECT_FALSE(arbitrator.state().retired_lineage.has_value());
}

TEST(RouteStrategyArbitrator3DTest, ReturnLineageIsStableAndGenerationBound) {
  const RouteStrategyReturnLineage3D first =
      makeRouteStrategyReturnLineage3D(7U, 101U, 10U, 3U, 9U, kMission);
  const RouteStrategyReturnLineage3D repeated =
      makeRouteStrategyReturnLineage3D(7U, 101U, 10U, 3U, 9U, kMission);
  const RouteStrategyReturnLineage3D newer =
      makeRouteStrategyReturnLineage3D(7U, 101U, 11U, 3U, 9U, kMission);

  EXPECT_TRUE(first.valid());
  EXPECT_TRUE(first.validForMission(kMission));
  EXPECT_EQ(first.id, repeated.id);
  EXPECT_NE(first.id, newer.id);
  RouteStrategyReturnLineage3D tampered = first;
  ++tampered.planned_on_revision;
  EXPECT_FALSE(tampered.validForMission(kMission));
  EXPECT_FALSE(makeRouteStrategyReturnLineage3D(7U, 0U, 10U, 3U, 9U, kMission).valid());
}

} // namespace
} // namespace drone_city_nav
