#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

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

// Whether a compile refusal is the raw world's verdict on the candidate's
// geometry. Only one of them is: a segment whose swept hull the evidence
// refuses. Every other reason is the pipeline's own -- a route that never
// arrived, a tracking world that was not current, a fingerprint or a span set
// that did not match -- and says nothing about where the vehicle may fly.
// Read as a verdict, they dropped the search's incumbent: one recorded flight
// lost its route to twenty-four compiles that had no route to compile, and
// each one cost a feasibility search started over from nothing.
[[nodiscard]] inline bool compileRefusalIsAWorldVerdict3D(
    const CompiledTrajectoryFailureReason3D reason) noexcept {
  return reason == CompiledTrajectoryFailureReason3D::kInvalidTrackingErrorTubeSegment;
}

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
  const bool compiled_geometry_refused =
      report.activation_status ==
      StaticRouteActivationStatus::kInvalidExecutionGeometry;
  if (compiled_geometry_refused &&
      !compileRefusalIsAWorldVerdict3D(report.trajectory_validation.reason)) {
    // The compile failed on the pipeline's own terms, so it says nothing about
    // the incumbent at all: it is kept wherever the refusal lies.
    return true;
  }
  const std::size_t refused_index =
      compiled_geometry_refused ? report.trajectory_validation.sample_index
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

// The candidate was planned on a world the activation snapshot has since
// contradicted with exact raw evidence: the candidate and the session that
// produced it are both stale, and neither a retry nor a continuation can
// recover them. The world compared is the one the planner reports having
// planned on, not the world the transaction was opened on: a persistent
// session keeps ingesting revisions while it runs, and read against the
// transaction's world half the recorded refusals on the very revision the
// activation validated on were taken for a newer world, each retiring a
// session with nothing stale about it. A refusal on the same world is the
// raw validators' verdict on the candidate, which the rejection sequence
// answers without starting the search over. A planner that reports no world
// falls back to the transaction's.
[[nodiscard]] inline bool
searchInvalidatedByActivationWorld(const PlannerSearchTransaction3D& transaction,
                                   const PlannerTelemetry3D& planned_on,
                                   const RouteAdmissionReport3D& admission) noexcept {
  if (admission.activation_status !=
          StaticRouteActivationStatus::kCandidateValidationRejected ||
      admission.candidate_validation.status !=
          StaticRouteCandidateStatus::kRawCollision ||
      admission.snapshot_raw_revision == 0U ||
      admission.tracking_profile_activation_occupied_fingerprint == 0U) {
    return false;
  }
  std::uint64_t planned_revision = planned_on.planned_on_revision;
  std::uint64_t planned_fingerprint = planned_on.occupied_fingerprint;
  if (planned_revision == 0U || planned_fingerprint == 0U) {
    if (transaction.planner_world == nullptr) {
      return false;
    }
    planned_revision = transaction.planner_world->revision;
    planned_fingerprint = transaction.planner_world->occupied_fingerprint;
  }
  return planned_revision != 0U && planned_fingerprint != 0U &&
         planned_revision != admission.snapshot_raw_revision &&
         planned_fingerprint !=
             admission.tracking_profile_activation_occupied_fingerprint;
}

} // namespace route_lifecycle_refusal_3d
} // namespace drone_city_nav
