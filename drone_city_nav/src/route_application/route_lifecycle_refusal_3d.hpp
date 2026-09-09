#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "production_planner_search_transaction_3d.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"

namespace drone_city_nav {

// How an activation refusal is read: where along the candidate it lies, and
// whether the raw world or the search decided it. Both answers steer what the
// lifecycle does next -- whether the incumbent survives, and how soon the
// failed-search latch lets another search run.
namespace route_lifecycle_refusal_3d {

// Where along the candidate the refusal lies, when the refusal names a sample
// at all. A candidate the activation refused within what the vehicle is
// committed to -- the departure leg it is flown at hover, and the certified
// overlap a successor has to splice onto -- is a route the vehicle cannot
// enter, and the search is better off starting again from the vehicle. Beyond
// that the refusal is a block ahead, the same thing a blocked release is: the
// route is enterable, the search repairs the blocked part against the world it
// re-validates every update, and dropping the incumbent for it restarted the
// feasibility search from nothing. One recorded flight lost a route to a
// refusal twelve metres ahead of it and stood for nine tenths of a second
// while the search explored its way back out to twenty-five.
[[nodiscard]] inline bool
refusedBeyondTheCommittedRoute3D(const ProductionRouteActivationResult3D& activation,
                                 const double required_certified_overlap_m) {
  const MaterializedRoute3D& candidate = activation.materialized;
  const RouteAdmissionReport3D& report = activation.admission;
  if (candidate.route == nullptr || !std::isfinite(required_certified_overlap_m)) {
    return false;
  }
  const std::size_t refused_index =
      report.activation_status == StaticRouteActivationStatus::kInvalidExecutionGeometry
          ? report.trajectory_validation.sample_index
      : report.activation_status ==
              StaticRouteActivationStatus::kCandidateValidationRejected
          ? report.candidate_validation.failure_segment_index
          : 0U;
  if (refused_index == 0U || refused_index >= candidate.route->size()) {
    return false;
  }
  return (*candidate.route)[refused_index].station_m >
         candidate.departure_end_station_m +
             std::max(0.0, required_certified_overlap_m);
}

// The raw world, not the search, refused this candidate: its own validation,
// its certification or its execution geometry was rejected against the
// evidence. A newer world is a different answer to exactly that refusal, so
// the failed-search latch lifts on one instead of holding until the retry
// interval.
[[nodiscard]] inline bool
worldRefusedCandidate3D(const StaticRouteActivationStatus status) noexcept {
  return status == StaticRouteActivationStatus::kCandidateValidationRejected ||
         status == StaticRouteActivationStatus::kCandidateNotExecutable ||
         status == StaticRouteActivationStatus::kInvalidExecutionGeometry ||
         status == StaticRouteActivationStatus::kRouteCertificationRejected ||
         status == StaticRouteActivationStatus::kCertifiedSpliceRejected;
}

// The search ran on a world the activation snapshot has since contradicted with
// exact raw evidence. The candidate and the session that produced it are both
// stale, so neither a retry nor a continuation can recover them.
[[nodiscard]] inline bool
searchInvalidatedByActivationWorld(const PlannerSearchTransaction3D& transaction,
                                   const RouteAdmissionReport3D& admission) noexcept {
  return admission.activation_status ==
             StaticRouteActivationStatus::kCandidateValidationRejected &&
         admission.candidate_validation.status ==
             StaticRouteCandidateStatus::kRawCollision &&
         transaction.planner_world != nullptr &&
         transaction.planner_world->revision != 0U &&
         admission.snapshot_raw_revision != 0U &&
         transaction.planner_world->revision != admission.snapshot_raw_revision &&
         transaction.planner_world->occupied_fingerprint != 0U &&
         admission.tracking_profile_activation_occupied_fingerprint != 0U &&
         transaction.planner_world->occupied_fingerprint !=
             admission.tracking_profile_activation_occupied_fingerprint;
}

} // namespace route_lifecycle_refusal_3d
} // namespace drone_city_nav
