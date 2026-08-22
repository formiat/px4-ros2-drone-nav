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
rejectedStatus(const SweptFootprintResult& result,
               const bool require_known_free_space) noexcept {
  if (result.evidence.raw_collision ||
      result.status == SweptFootprintStatus::kRawCollision) {
    return SegmentEvidenceStatus3D::kRawCollision;
  }
  if (result.evidence.invalid_esdf_exposure ||
      result.status == SweptFootprintStatus::kInvalidEsdf) {
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

[[nodiscard]] bool eligible(const RouteProposal3D& proposal) noexcept {
  return proposal.intent.valid && proposal.activation_eligible &&
         proposal.evidence.physical_executable;
}

[[nodiscard]] double finiteCost(const double value) noexcept {
  return std::isfinite(value) ? value : std::numeric_limits<double>::infinity();
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

[[nodiscard]] auto proposalRank(const RouteProposal3D& proposal) noexcept {
  const SegmentEvidence3D& evidence = proposal.evidence;
  return std::tuple{
      evidence.reaches_mission_target ? 0 : 1,
      evidence.reaches_intent_target ? 0 : 1,
      purposeRank(proposal.intent.purpose),
      proposal.intent.strategic_continuation_available ? 0 : 1,
      evidence.reaches_segment_target ? 0 : 1,
      finiteCost(evidence.objective_cost),
      -evidence.endpoint_displacement_m,
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
                                  const std::uint64_t target_identity) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, static_cast<std::uint64_t>(source));
  hashValue(hash, static_cast<std::uint64_t>(purpose));
  hashValue(hash, target_identity);
  hashPoint(hash, mission_target);
  hashPoint(hash, intent_target);
  return hash == 0U ? 1U : hash;
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
    SegmentEvidenceStatus3D status =
        rejectedStatus(esdf_validation, world.require_known_free_space);
    if (status != SegmentEvidenceStatus3D::kValid) {
      result.status = status;
      result.failure_segment_index = index - 1U;
      result.failure_point = esdf_validation.failure_point;
      return result;
    }
    if (world.latest_observed_occupancy == nullptr) {
      continue;
    }
    const SweptFootprintResult raw_validation = validateRawSweptFootprint(
        *world.latest_observed_occupancy, first, FootprintBodyAxis{}, second,
        FootprintBodyAxis{}, world.footprint, world.proprioceptive_free_space_seed,
        world.launch_support_contact);
    mergeFootprintEvidence(result, raw_validation);
    status = rejectedStatus(raw_validation, world.require_known_free_space);
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

bool betterRouteProposal3D(const RouteProposal3D& candidate,
                           const RouteProposal3D& current) noexcept {
  if (eligible(candidate) != eligible(current)) {
    return eligible(candidate);
  }
  return proposalRank(candidate) < proposalRank(current);
}

RouteProposalSelection3D
selectRouteProposal3D(const std::span<const RouteProposal3D> proposals) noexcept {
  RouteProposalSelection3D result{
      .selected_index = std::nullopt,
      .reason = RouteProposalSelectionReason3D::kNoEligibleCandidate,
      .considered_candidates = proposals.size(),
      .eligible_candidates = 0U,
  };
  for (std::size_t index = 0U; index < proposals.size(); ++index) {
    if (!eligible(proposals[index])) {
      continue;
    }
    ++result.eligible_candidates;
    if (!result.selected_index.has_value() ||
        betterRouteProposal3D(proposals[index], proposals[*result.selected_index])) {
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
  } else if (selected.evidence.reaches_intent_target) {
    result.reason = RouteProposalSelectionReason3D::kIntentTarget;
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
    case RouteProposalSelectionReason3D::kIntentTarget:
      return "intent_target";
    case RouteProposalSelectionReason3D::kStrategicContinuation:
      return "strategic_continuation";
    case RouteProposalSelectionReason3D::kRouteQuality:
      return "route_quality";
  }
  return "unknown";
}

} // namespace drone_city_nav
