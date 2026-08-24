#pragma once

#include "drone_city_nav/route_planning_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace drone_city_nav {

enum class RouteStrategyKind3D : std::uint8_t {
  kNone,
  kDirect,
  kTopologyMission,
  kObservationFrontier,
  kTopologicalBacktrack,
  kLaunchDeparture,
};

struct RouteStrategyArbitration3DConfig {
  double topology_mission_lease_budget_m{160.0};
  double observation_frontier_lease_budget_m{80.0};
  double topological_backtrack_lease_budget_m{120.0};
  double minimum_lease_commitment_m{5.0};
  double minimum_return_mission_progress_m{8.0};
  double direct_release_minimum_progress_advantage_m{5.0};
  double direct_release_minimum_progress_ratio_advantage{0.10};
  std::size_t direct_release_confirmation_count{2U};
};

[[nodiscard]] bool routeStrategyArbitration3DConfigIsValid(
    const RouteStrategyArbitration3DConfig& config) noexcept;

struct RouteStrategyArbitrationObservation3D {
  Point3 position{};
  Point3 mission_target{};
  std::uint64_t world_revision{0U};
  std::optional<RouteIntent3D> active_intent;
  bool active_intent_completed{false};
};

struct RouteStrategyLease3D {
  std::uint64_t lease_id{0U};
  RouteStrategyKind3D kind{RouteStrategyKind3D::kNone};
  RouteStrategyLeaseReason3D reason{RouteStrategyLeaseReason3D::kNone};
  std::uint64_t intent_id{0U};
  std::uint64_t strategic_plan_id{0U};
  std::uint64_t target_identity{0U};
  RouteStrategyReturnLineage3D return_lineage{};
  Point3 mission_target{};
  Point3 acquisition_position{};
  Point3 last_observed_position{};
  std::uint64_t acquired_revision{0U};
  std::uint64_t last_observed_revision{0U};
  double distance_budget_m{0.0};
  double travelled_m{0.0};
  std::size_t direct_advantage_confirmations{0U};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] double remainingBudgetM() const noexcept;
};

struct RetiredRouteStrategyLineage3D {
  RouteStrategyKind3D kind{RouteStrategyKind3D::kNone};
  std::uint64_t return_lineage_id{0U};
  std::uint64_t target_identity{0U};
  Point3 mission_target{};
  Point3 retired_position{};
  double mission_distance_at_retirement_m{0.0};

  [[nodiscard]] bool valid() const noexcept;
};

struct RouteStrategyArbitrationState3D {
  std::optional<RouteStrategyLease3D> lease;
  std::optional<RetiredRouteStrategyLineage3D> retired_lineage;
  std::uint64_t last_allocated_lease_id{0U};
};

enum class RouteStrategyArbitrationAction3D : std::uint8_t {
  kNoSelection,
  kStatelessSelection,
  kLeaseAcquired,
  kLeaseRetained,
  kLeaseHysteresisRetained,
  kLeaseReleasedMissionChanged,
  kLeaseReleasedIntentCompleted,
  kLeaseReleasedMissionTarget,
  kLeaseReleasedBudgetExhausted,
  kLeaseReleasedCandidateInvalid,
  kLeaseReleasedDirectAdvantage,
  kRetiredLineageSuppressed,
  kInvalidStrategicLineage,
  kPendingOutcome,
};

struct RouteStrategyArbitrationDecision3D {
  std::uint64_t sequence{0U};
  RouteProposalSelection3D selection{};
  RouteStrategyArbitrationAction3D action{
      RouteStrategyArbitrationAction3D::kNoSelection};
  RouteStrategyArbitrationState3D state_without_commit{};
  RouteStrategyArbitrationState3D state_after_commit{};
};

class RouteStrategyArbitrator3D final {
public:
  explicit RouteStrategyArbitrator3D(
      const RouteStrategyArbitration3DConfig& config = {});

  [[nodiscard]] RouteStrategyArbitrationDecision3D
  evaluate(std::span<const RouteProposal3D> proposals,
           const RouteProposalSelection3DConfig& proposal_config,
           const RouteStrategyArbitrationObservation3D& observation) noexcept;
  [[nodiscard]] bool recordOutcome(const RouteStrategyArbitrationDecision3D& decision,
                                   bool selection_committed) noexcept;

  void reset() noexcept;
  [[nodiscard]] const RouteStrategyArbitrationState3D& state() const noexcept;
  [[nodiscard]] const RouteStrategyArbitration3DConfig& config() const noexcept;

private:
  RouteStrategyArbitration3DConfig config_{};
  RouteStrategyArbitrationState3D state_{};
  std::uint64_t last_decision_sequence_{0U};
  std::optional<std::uint64_t> pending_decision_sequence_;
};

[[nodiscard]] RouteStrategyKind3D
routeStrategyKind3D(const RouteIntent3D& intent) noexcept;
[[nodiscard]] std::string_view
routeStrategyKind3DName(RouteStrategyKind3D kind) noexcept;
[[nodiscard]] std::string_view
routeStrategyArbitrationAction3DName(RouteStrategyArbitrationAction3D action) noexcept;

} // namespace drone_city_nav
