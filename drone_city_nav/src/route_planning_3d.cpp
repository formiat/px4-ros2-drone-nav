#include "drone_city_nav/route_planning_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <tuple>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  hash ^= value;
  hash *= kFnvPrime;
}

void hashPoint(std::uint64_t& hash, const Point3& point) noexcept {
  hashValue(hash, std::bit_cast<std::uint64_t>(point.x));
  hashValue(hash, std::bit_cast<std::uint64_t>(point.y));
  hashValue(hash, std::bit_cast<std::uint64_t>(point.z));
}

void mergeFootprintEvidence(SegmentEvidence3D& target,
                            const SweptFootprintResult& source) noexcept {
  target.raw_collision = target.raw_collision || source.evidence.raw_collision;
  target.outside_grid_exposure =
      target.outside_grid_exposure || source.evidence.outside_grid_exposure;
  target.unknown_exposure = target.unknown_exposure || source.evidence.unknown_exposure;
  target.invalid_esdf_exposure =
      target.invalid_esdf_exposure || source.evidence.invalid_esdf_exposure;
  if (source.evidence.known_clearance_observed) {
    target.known_clearance_observed = true;
    target.minimum_known_clearance_m = std::min(
        target.minimum_known_clearance_m, source.evidence.minimum_known_clearance_m);
  }
}

[[nodiscard]] SegmentEvidenceStatus3D
rejectedStatus(const SweptFootprintResult& result, const bool require_known_free_space,
               const bool reject_invalid_esdf) noexcept {
  if (result.evidence.raw_collision ||
      result.status == SweptFootprintStatus::kRawCollision) {
    return SegmentEvidenceStatus3D::kRawCollision;
  }
  if (reject_invalid_esdf && (result.evidence.invalid_esdf_exposure ||
                              result.status == SweptFootprintStatus::kInvalidEsdf)) {
    return SegmentEvidenceStatus3D::kInvalidEsdf;
  }
  if (result.status == SweptFootprintStatus::kOutsideGrid) {
    return SegmentEvidenceStatus3D::kOutsideGrid;
  }
  if (require_known_free_space &&
      result.status == SweptFootprintStatus::kUnknownSpace) {
    return SegmentEvidenceStatus3D::kUnknownRejected;
  }
  return SegmentEvidenceStatus3D::kValid;
}

[[nodiscard]] double finiteCost(const double value) noexcept {
  return std::isfinite(value) ? value : std::numeric_limits<double>::infinity();
}

[[nodiscard]] double maximumValueRank(const double value) noexcept {
  return std::isfinite(value) ? -value : std::numeric_limits<double>::infinity();
}

[[nodiscard]] double netCoordinateProgress(const SegmentEvidence3D& evidence) noexcept {
  if (!std::isfinite(evidence.endpoint_displacement_m) ||
      !std::isfinite(evidence.mission_progress_m)) {
    return -std::numeric_limits<double>::infinity();
  }
  return evidence.endpoint_displacement_m + evidence.mission_progress_m;
}

[[nodiscard]] int purposeRank(const RouteIntentPurpose3D purpose) noexcept {
  switch (purpose) {
    case RouteIntentPurpose3D::kLaunchDeparture:
    case RouteIntentPurpose3D::kMissionTransit:
      return 0;
    case RouteIntentPurpose3D::kObservationFrontier:
      return 1;
    case RouteIntentPurpose3D::kTopologicalBacktrack:
      return 2;
  }
  return 3;
}

[[nodiscard]] auto proposalRank(const RouteProposal3D& proposal,
                                const RouteProposalSelection3DConfig& config) noexcept {
  const SegmentEvidence3D& evidence = proposal.evidence;
  const bool heuristic_precedence = config.heuristic_precedence_enabled;
  // Coordinate displacement plus signed radial mission progress rewards
  // forward motion twice, lateral detours once, and pure reversal not at all.
  // Only the endpoints participate, so route length cannot reward zigzags or
  // loops. Objective cost remains a late tie-breaker among equally productive
  // endpoint choices.
  const double permissive_progress = netCoordinateProgress(evidence);
  return std::tuple{
      evidence.reaches_mission_target ? 0 : 1,
      heuristic_precedence && evidence.reaches_intent_target ? 0 : 1,
      heuristic_precedence && isProductiveDirectTransit3D(proposal, config) ? 0 : 1,
      heuristic_precedence && isStrategicMissionContinuation3D(proposal) ? 0 : 1,
      heuristic_precedence && proposal.intent.strategic_continuation_available ? 0 : 1,
      heuristic_precedence ? purposeRank(proposal.intent.purpose) : 0,
      heuristic_precedence && evidence.reaches_segment_target ? 0 : 1,
      heuristic_precedence ? maximumValueRank(evidence.mission_progress_m)
                           : maximumValueRank(permissive_progress),
      heuristic_precedence ? maximumValueRank(evidence.endpoint_displacement_m)
                           : maximumValueRank(evidence.mission_progress_m),
      heuristic_precedence ? 0.0 : maximumValueRank(evidence.endpoint_displacement_m),
      finiteCost(evidence.objective_cost),
      finiteCost(evidence.route_length_m),
      proposal.route_fingerprint,
      proposal.intent.id,
  };
}

} // namespace

std::uint64_t makeRouteIntentId3D(const RouteIntentSource3D source,
                                  const RouteIntentPurpose3D purpose,
                                  const Point3& mission_target,
                                  const Point3& intent_target,
                                  const std::uint64_t target_identity,
                                  const bool observation_stop_required) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, static_cast<std::uint64_t>(source));
  hashValue(hash, static_cast<std::uint64_t>(purpose));
  hashValue(hash, target_identity);
  if (purpose == RouteIntentPurpose3D::kObservationFrontier) {
    hashValue(hash, observation_stop_required ? 1U : 0U);
  }
  hashPoint(hash, mission_target);
  hashPoint(hash, intent_target);
  return hash == 0U ? 1U : hash;
}

bool RouteStrategyReturnLineage3D::valid() const noexcept {
  return id != 0U && strategic_plan_id != 0U && topology_lineage_id != 0U &&
         planned_on_revision != 0U && return_anchor_identity != 0U &&
         excursion_target_identity != 0U;
}

bool RouteStrategyReturnLineage3D::validForMission(
    const Point3& mission_target) const noexcept {
  if (!valid()) {
    return false;
  }
  return makeRouteStrategyReturnLineage3D(strategic_plan_id, topology_lineage_id,
                                          planned_on_revision, return_anchor_identity,
                                          excursion_target_identity, mission_target)
             .id == id;
}

RouteStrategyReturnLineage3D makeRouteStrategyReturnLineage3D(
    const std::uint64_t strategic_plan_id, const std::uint64_t topology_lineage_id,
    const std::uint64_t planned_on_revision, const std::uint64_t return_anchor_identity,
    const std::uint64_t excursion_target_identity,
    const Point3& mission_target) noexcept {
  if (strategic_plan_id == 0U || topology_lineage_id == 0U ||
      planned_on_revision == 0U || return_anchor_identity == 0U ||
      excursion_target_identity == 0U || !std::isfinite(mission_target.x) ||
      !std::isfinite(mission_target.y) || !std::isfinite(mission_target.z)) {
    return {};
  }
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, strategic_plan_id);
  hashValue(hash, topology_lineage_id);
  hashValue(hash, planned_on_revision);
  hashValue(hash, return_anchor_identity);
  hashValue(hash, excursion_target_identity);
  hashPoint(hash, mission_target);
  return RouteStrategyReturnLineage3D{
      .id = hash == 0U ? 1U : hash,
      .strategic_plan_id = strategic_plan_id,
      .topology_lineage_id = topology_lineage_id,
      .planned_on_revision = planned_on_revision,
      .return_anchor_identity = return_anchor_identity,
      .excursion_target_identity = excursion_target_identity,
  };
}

SegmentEvidence3D evaluateSegmentEvidence3D(
    const RouteIntent3D& intent, const std::span<const RouteSample3D> route,
    const Point3& search_start, const bool planner_executable,
    const bool reaches_segment_target, const bool reaches_mission_target,
    const double objective_cost, const SegmentEvidenceWorld3D& world) noexcept {
  SegmentEvidence3D result{
      .planned_on_revision = intent.planned_on_revision,
      .validated_through_revision = world.validated_through_revision,
      .status = SegmentEvidenceStatus3D::kInvalidWorld,
      .objective_cost = objective_cost,
      .planner_executable = planner_executable,
      .reaches_segment_target = reaches_segment_target,
      .reaches_intent_target =
          reaches_segment_target && intent.segment_reaches_intent_target,
      .reaches_mission_target = reaches_mission_target,
  };
  if (!planner_executable) {
    result.status = SegmentEvidenceStatus3D::kPlannerRejected;
    return result;
  }
  if (route.size() < 2U) {
    result.status = SegmentEvidenceStatus3D::kEmptyRoute;
    return result;
  }
  result.materialized = true;
  result.endpoint_displacement_m = distance3D(search_start, route.back().position);
  result.mission_progress_m = distance3D(search_start, intent.mission_target) -
                              distance3D(route.back().position, intent.mission_target);
  for (std::size_t index = 1U; index < route.size(); ++index) {
    result.route_length_m +=
        distance3D(route[index - 1U].position, route[index].position);
  }
  if (world.grid == nullptr || world.esdf_m.empty()) {
    return result;
  }
  if (!std::ranges::all_of(route, [&](const RouteSample3D& sample) {
        return insideFlightEnvelope(sample.position, world.flight_envelope);
      })) {
    result.status = SegmentEvidenceStatus3D::kOutsideFlightEnvelope;
    return result;
  }

  for (std::size_t index = 1U; index < route.size(); ++index) {
    const Point3& first = route[index - 1U].position;
    const Point3& second = route[index].position;
    const SweptFootprintResult esdf_validation = validateSweptFootprint(
        *world.grid, world.esdf_m, first, second, world.footprint);
    mergeFootprintEvidence(result, esdf_validation);
    // Invalid derived clearance may be ignored only when the authoritative raw
    // occupancy is available for the same segment. This keeps raw collision a
    // mandatory constraint while making ESDF completeness an opt-in policy.
    const bool reject_invalid_esdf =
        world.reject_invalid_esdf || world.latest_observed_occupancy == nullptr;
    SegmentEvidenceStatus3D status = rejectedStatus(
        esdf_validation, world.require_known_free_space, reject_invalid_esdf);
    if (status != SegmentEvidenceStatus3D::kValid) {
      result.status = status;
      result.failure_segment_index = index - 1U;
      result.failure_point = esdf_validation.failure_point;
      return result;
    }
    if (world.latest_observed_occupancy == nullptr) {
      continue;
    }
    const ObservedSpaceValidationPolicy observed_policy =
        world.require_known_free_space
            ? ObservedSpaceValidationPolicy::kRequireKnownFree
            : ObservedSpaceValidationPolicy::kAllowUnknown;
    const SweptFootprintResult raw_validation = validateObservedSweptFootprint(
        *world.latest_observed_occupancy, first, FootprintBodyAxis{}, second,
        FootprintBodyAxis{}, world.footprint, observed_policy,
        world.proprioceptive_free_space_seed, world.launch_support_contact);
    mergeFootprintEvidence(result, raw_validation);
    status = rejectedStatus(raw_validation, world.require_known_free_space,
                            world.reject_invalid_esdf);
    if (status != SegmentEvidenceStatus3D::kValid) {
      result.status = status;
      result.failure_segment_index = index - 1U;
      result.failure_point = raw_validation.failure_point;
      return result;
    }
  }
  result.status = SegmentEvidenceStatus3D::kValid;
  result.physical_executable = true;
  return result;
}

bool routeProposalSelection3DConfigIsValid(
    const RouteProposalSelection3DConfig& config) noexcept {
  return std::isfinite(config.productive_direct_minimum_mission_progress_m) &&
         config.productive_direct_minimum_mission_progress_m >= 0.0 &&
         std::isfinite(config.productive_direct_minimum_progress_ratio) &&
         config.productive_direct_minimum_progress_ratio >= 0.0 &&
         config.productive_direct_minimum_progress_ratio <= 1.0;
}

bool routeProposalEligible3D(const RouteProposal3D& proposal) noexcept {
  return proposal.intent.valid && proposal.activation_eligible &&
         proposal.evidence.physical_executable;
}

bool isProductiveDirectTransit3D(
    const RouteProposal3D& proposal,
    const RouteProposalSelection3DConfig& config) noexcept {
  if (!routeProposalSelection3DConfigIsValid(config) ||
      !routeProposalEligible3D(proposal) ||
      proposal.intent.source != RouteIntentSource3D::kDirect ||
      proposal.intent.purpose != RouteIntentPurpose3D::kMissionTransit ||
      !(proposal.evidence.route_length_m > 0.0) ||
      !std::isfinite(proposal.evidence.route_length_m) ||
      !std::isfinite(proposal.evidence.mission_progress_m)) {
    return false;
  }
  const double progress_ratio =
      proposal.evidence.mission_progress_m / proposal.evidence.route_length_m;
  return proposal.evidence.mission_progress_m >=
             config.productive_direct_minimum_mission_progress_m &&
         progress_ratio >= config.productive_direct_minimum_progress_ratio;
}

bool isStrategicMissionContinuation3D(const RouteProposal3D& proposal) noexcept {
  return proposal.intent.valid && proposal.intent.strategic_plan_id != 0U &&
         proposal.intent.source == RouteIntentSource3D::kTopology &&
         proposal.intent.purpose == RouteIntentPurpose3D::kMissionTransit &&
         proposal.intent.strategic_continuation_available &&
         proposal.intent.strategic_mission_continuation;
}

bool betterRouteProposal3D(const RouteProposal3D& candidate,
                           const RouteProposal3D& current,
                           const RouteProposalSelection3DConfig& config) noexcept {
  if (routeProposalEligible3D(candidate) != routeProposalEligible3D(current)) {
    return routeProposalEligible3D(candidate);
  }
  return proposalRank(candidate, config) < proposalRank(current, config);
}

RouteProposalSelection3D
selectRouteProposal3D(const std::span<const RouteProposal3D> proposals,
                      const RouteProposalSelection3DConfig& config) noexcept {
  RouteProposalSelection3D result{
      .selected_index = std::nullopt,
      .reason = RouteProposalSelectionReason3D::kNoEligibleCandidate,
      .considered_candidates = proposals.size(),
      .eligible_candidates = 0U,
  };
  for (std::size_t index = 0U; index < proposals.size(); ++index) {
    if (!routeProposalEligible3D(proposals[index])) {
      continue;
    }
    ++result.eligible_candidates;
    if (!result.selected_index.has_value() ||
        betterRouteProposal3D(proposals[index], proposals[*result.selected_index],
                              config)) {
      result.selected_index = index;
    }
  }
  if (!result.selected_index.has_value()) {
    return result;
  }
  if (result.eligible_candidates == 1U) {
    result.reason = RouteProposalSelectionReason3D::kOnlyEligibleCandidate;
    return result;
  }
  const RouteProposal3D& selected = proposals[*result.selected_index];
  if (selected.evidence.reaches_mission_target) {
    result.reason = RouteProposalSelectionReason3D::kMissionTarget;
  } else if (!config.heuristic_precedence_enabled) {
    result.reason = RouteProposalSelectionReason3D::kNetCoordinateProgress;
  } else if (selected.evidence.reaches_intent_target) {
    result.reason = RouteProposalSelectionReason3D::kIntentTarget;
  } else if (isStrategicMissionContinuation3D(selected)) {
    result.reason = RouteProposalSelectionReason3D::kStrategicMissionContinuation;
  } else if (isProductiveDirectTransit3D(selected, config)) {
    result.reason = RouteProposalSelectionReason3D::kProductiveDirectTransit;
  } else if (selected.intent.strategic_continuation_available) {
    result.reason = RouteProposalSelectionReason3D::kStrategicContinuation;
  } else {
    result.reason = RouteProposalSelectionReason3D::kRouteQuality;
  }
  return result;
}

const char* routeIntentSource3DName(const RouteIntentSource3D source) noexcept {
  switch (source) {
    case RouteIntentSource3D::kDirect:
      return "direct";
    case RouteIntentSource3D::kTopology:
      return "topology";
    case RouteIntentSource3D::kLaunchDeparture:
      return "launch_departure";
  }
  return "unknown";
}

const char* routeIntentPurpose3DName(const RouteIntentPurpose3D purpose) noexcept {
  switch (purpose) {
    case RouteIntentPurpose3D::kMissionTransit:
      return "mission_transit";
    case RouteIntentPurpose3D::kLaunchDeparture:
      return "launch_departure";
    case RouteIntentPurpose3D::kObservationFrontier:
      return "observation_frontier";
    case RouteIntentPurpose3D::kTopologicalBacktrack:
      return "topological_backtrack";
  }
  return "unknown";
}

const char*
routeStrategyLeaseReason3DName(const RouteStrategyLeaseReason3D reason) noexcept {
  switch (reason) {
    case RouteStrategyLeaseReason3D::kNone:
      return "none";
    case RouteStrategyLeaseReason3D::kMissionTopologyContinuation:
      return "mission_topology_continuation";
    case RouteStrategyLeaseReason3D::kObservationFrontier:
      return "observation_frontier";
    case RouteStrategyLeaseReason3D::kBacktrackConfirmedTerminal:
      return "backtrack_confirmed_terminal";
    case RouteStrategyLeaseReason3D::kBacktrackNoReachableFrontier:
      return "backtrack_no_reachable_frontier";
    case RouteStrategyLeaseReason3D::kBacktrackAllReachableBranchesExplored:
      return "backtrack_all_reachable_branches_explored";
  }
  return "unknown";
}

const char* segmentEvidenceStatus3DName(const SegmentEvidenceStatus3D status) noexcept {
  switch (status) {
    case SegmentEvidenceStatus3D::kValid:
      return "valid";
    case SegmentEvidenceStatus3D::kEmptyRoute:
      return "empty_route";
    case SegmentEvidenceStatus3D::kPlannerRejected:
      return "planner_rejected";
    case SegmentEvidenceStatus3D::kInvalidWorld:
      return "invalid_world";
    case SegmentEvidenceStatus3D::kOutsideFlightEnvelope:
      return "outside_flight_envelope";
    case SegmentEvidenceStatus3D::kOutsideGrid:
      return "outside_grid";
    case SegmentEvidenceStatus3D::kUnknownRejected:
      return "unknown_rejected";
    case SegmentEvidenceStatus3D::kInvalidEsdf:
      return "invalid_esdf";
    case SegmentEvidenceStatus3D::kRawCollision:
      return "raw_collision";
  }
  return "unknown";
}

const char* routeProposalSelectionReason3DName(
    const RouteProposalSelectionReason3D reason) noexcept {
  switch (reason) {
    case RouteProposalSelectionReason3D::kNoEligibleCandidate:
      return "no_eligible_candidate";
    case RouteProposalSelectionReason3D::kOnlyEligibleCandidate:
      return "only_eligible_candidate";
    case RouteProposalSelectionReason3D::kMissionTarget:
      return "mission_target";
    case RouteProposalSelectionReason3D::kMissionProgress:
      return "mission_progress";
    case RouteProposalSelectionReason3D::kNetCoordinateProgress:
      return "net_coordinate_progress";
    case RouteProposalSelectionReason3D::kIntentTarget:
      return "intent_target";
    case RouteProposalSelectionReason3D::kStrategicMissionContinuation:
      return "strategic_mission_continuation";
    case RouteProposalSelectionReason3D::kProductiveDirectTransit:
      return "productive_direct_transit";
    case RouteProposalSelectionReason3D::kStrategicContinuation:
      return "strategic_continuation";
    case RouteProposalSelectionReason3D::kRouteQuality:
      return "route_quality";
    case RouteProposalSelectionReason3D::kActiveStrategyLease:
      return "active_strategy_lease";
    case RouteProposalSelectionReason3D::kStrategyLeaseHysteresis:
      return "strategy_lease_hysteresis";
    case RouteProposalSelectionReason3D::kStrategyReturnRequired:
      return "strategy_return_required";
    case RouteProposalSelectionReason3D::kInvalidStrategyLineageFallback:
      return "invalid_strategy_lineage_fallback";
  }
  return "unknown";
}

} // namespace drone_city_nav
