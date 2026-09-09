#include "drone_city_nav/execution_route_store_3d.hpp"

#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool exclusiveExecutionHold(const ExecutionPlan3D& plan) noexcept {
  return plan.phase() == ExecutionRoutePhase3D::kStopped &&
         plan.stationaryHold() != nullptr && plan.route() == nullptr &&
         plan.finiteExecution() == nullptr && plan.directTrackingExecution() == nullptr;
}

[[nodiscard]] bool
sameDirectTrackingIdentity(const DirectTrackingOwnerIdentity3D& first,
                           const DirectTrackingOwnerIdentity3D& second) noexcept {
  return first.mission_epoch == second.mission_epoch &&
         first.assignment_generation == second.assignment_generation &&
         first.target_detection_id == second.target_detection_id &&
         first.target_track_id == second.target_track_id &&
         first.objective_sample_sequence == second.objective_sample_sequence &&
         first.line_of_sight_generation == second.line_of_sight_generation;
}

} // namespace

bool sameExecutionRouteBase3D(
    const std::shared_ptr<const ExecutionPlan3D>& first,
    const std::shared_ptr<const ExecutionPlan3D>& second) noexcept {
  if (first == nullptr || second == nullptr || !first->valid() || !second->valid()) {
    return false;
  }
  if (first->execution_owner_epoch != second->execution_owner_epoch ||
      (first->route() != nullptr) != (second->route() != nullptr) ||
      (first->directTrackingExecution() != nullptr) !=
          (second->directTrackingExecution() != nullptr) ||
      (first->stationaryHold() != nullptr) != (second->stationaryHold() != nullptr) ||
      first->routeGenerationHighWater() != second->routeGenerationHighWater()) {
    return false;
  }

  const StationaryExecutionHold3D* const first_hold = first->stationaryHold();
  const StationaryExecutionHold3D* const second_hold = second->stationaryHold();
  if (exclusiveExecutionHold(*first)) {
    return exclusiveExecutionHold(*second) && first_hold != nullptr &&
           second_hold != nullptr && first_hold->hold_id == second_hold->hold_id;
  }

  const DirectTrackingFiniteExecution3D* const first_direct =
      first->directTrackingExecution();
  const DirectTrackingFiniteExecution3D* const second_direct =
      second->directTrackingExecution();
  if (first_direct != nullptr) {
    return second_direct != nullptr &&
           sameDirectTrackingIdentity(first_direct->identity, second_direct->identity);
  }

  const CertifiedRouteSuffix3D* const first_route = first->route();
  const CertifiedRouteSuffix3D* const second_route = second->route();
  if (first_route == nullptr) {
    return first->phase() == second->phase();
  }
  return second_route != nullptr &&
         first_route->identity.generation == second_route->identity.generation &&
         first_route->geometry != nullptr && second_route->geometry != nullptr &&
         first_route->geometry->compiled_trajectory_revision ==
             second_route->geometry->compiled_trajectory_revision &&
         first_route->continuity_id == second_route->continuity_id;
}

std::shared_ptr<const ExecutionPlan3D>
RouteExecutionManagerSnapshot3D::plan() const noexcept {
  return authority != nullptr ? authority->plan() : nullptr;
}

bool RouteExecutionManagerSnapshot3D::valid() const noexcept {
  const std::shared_ptr<const ExecutionPlan3D> execution = plan();
  return authority != nullptr && authority->valid() && execution != nullptr &&
         (pending == nullptr ||
          (pending->valid() && pendingCertifiedRouteEligible3D(*pending, *execution)));
}

RouteExecutionManager3D::RouteExecutionManager3D() {
  std::shared_ptr<const ExecutionPlan3D> initial_plan =
      makeInitialExecutionRouteSnapshot3D();
  auto initial_authority = std::make_shared<const CommittedExecutionAuthority3D>(
      CommittedExecutionAuthority3D::CaptureToken{}, 1U, std::move(initial_plan),
      ExecutionOwnerIdentity3D{}, nullptr, AppliedControlEvidence3D{});
  if (!initial_authority->valid()) {
    throw std::logic_error{"invalid initial execution authority"};
  }
  authority_.store(std::move(initial_authority), std::memory_order_release);
}

RouteExecutionManagerSnapshot3D RouteExecutionManager3D::snapshot() const {
  const std::scoped_lock lock{mutex_};
  return RouteExecutionManagerSnapshot3D{
      .authority = authority_.load(std::memory_order_acquire),
      .pending = pending_,
  };
}

std::shared_ptr<const CommittedExecutionAuthority3D>
RouteExecutionManager3D::authority() const noexcept {
  return authority_.load(std::memory_order_acquire);
}

std::shared_ptr<const ExecutionPlan3D> RouteExecutionManager3D::plan() const {
  const std::shared_ptr<const CommittedExecutionAuthority3D> current = authority();
  return current != nullptr ? current->plan() : nullptr;
}

std::shared_ptr<const PendingCertifiedRoute3D>
RouteExecutionManager3D::pending() const {
  const std::scoped_lock lock{mutex_};
  return pending_;
}

ExecutionRoutePublicationStatus3D RouteExecutionManager3D::publishAuthorityLocked(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    std::shared_ptr<const ExecutionPlan3D> next_plan,
    const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input,
    const AppliedControlEvidence3D& control) {
  const std::shared_ptr<const CommittedExecutionAuthority3D> current =
      authority_.load(std::memory_order_acquire);
  if (expected_authority == nullptr || current != expected_authority) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  if (!expected_authority->valid() || next_plan == nullptr || !next_plan->valid()) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  if (expected_authority->revision() == std::numeric_limits<std::uint64_t>::max()) {
    return ExecutionRoutePublicationStatus3D::kAuthorityRevisionExhausted;
  }

  auto next = std::make_shared<const CommittedExecutionAuthority3D>(
      CommittedExecutionAuthority3D::CaptureToken{},
      expected_authority->revision() + 1U, std::move(next_plan), owner,
      std::move(input), control);
  if (!next->valid()) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  authority_.store(std::move(next), std::memory_order_release);
  return ExecutionRoutePublicationStatus3D::kPublished;
}

namespace {

// The applied control a new lease inherits from the authority it replaces:
// the offboard keeps applying the previous horizon until it receives the new
// one, so feedback that still witnesses the new owner (its immediate
// predecessor) stays the authority's applied control instead of leaving the
// lease unwitnessed until the next feedback lands.
[[nodiscard]] AppliedControlEvidence3D
carriedAppliedControl(const CommittedExecutionAuthority3D& previous,
                      const ExecutionOwnerIdentity3D& owner) noexcept {
  const AppliedControlEvidence3D& control = previous.control();
  if (control.valid && owner.valid && control.validFor(owner)) {
    return control;
  }
  return AppliedControlEvidence3D{};
}

} // namespace

ExecutionRoutePublicationStatus3D RouteExecutionManager3D::publishTransitionLocked(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const ExecutionRouteTransitionResult3D& transition,
    const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input,
    const DetachedTransitionIntent3D intent) {
  const std::shared_ptr<const CommittedExecutionAuthority3D> current =
      authority_.load(std::memory_order_acquire);
  if (expected_authority == nullptr || current != expected_authority) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  const std::shared_ptr<const ExecutionPlan3D>& expected_plan =
      expected_authority->plan();
  // Only the transition that exists to take the horizon away may install a
  // plan nothing can be published from. Requiring it of that one too made the
  // fail-closed revocation's suspension uncommittable -- the suspension is
  // what keeps the certified route for the successor to resume from -- so the
  // revocation was refused here every time and returned having sent nothing
  // while the offboard flew on.
  const bool installs_a_publishable_plan =
      intent == DetachedTransitionIntent3D::kRelinquishTheHorizon ||
      transition.next == nullptr || transition.next->publishable();
  if (expected_plan == nullptr || !transition.applied() || transition.next == nullptr ||
      !installs_a_publishable_plan ||
      expected_plan->version == std::numeric_limits<std::uint64_t>::max() ||
      transition.next->version <= expected_plan->version) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  if (transition.predecessor != expected_plan.get()) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  return publishAuthorityLocked(expected_authority, transition.next, owner,
                                std::move(input),
                                carriedAppliedControl(*expected_authority, owner));
}

ExecutionRoutePublicationStatus3D RouteExecutionManager3D::publishDetachedTransition(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const ExecutionRouteTransitionResult3D& transition,
    const DetachedTransitionIntent3D intent) {
  const std::scoped_lock lock{mutex_};
  return publishTransitionLocked(expected_authority, transition,
                                 ExecutionOwnerIdentity3D{}, nullptr, intent);
}

ExecutionRoutePublicationStatus3D RouteExecutionManager3D::publishLeasedTransition(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const ExecutionRouteTransitionResult3D& transition,
    const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input) {
  const std::scoped_lock lock{mutex_};
  return publishTransitionLocked(expected_authority, transition, owner,
                                 std::move(input),
                                 DetachedTransitionIntent3D::kKeepThePlanPublishable);
}

ExecutionRoutePublicationStatus3D
RouteExecutionManager3D::publishLeaseForUnchangedPlanIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const std::shared_ptr<const ExecutionPlan3D>& expected_plan,
    const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input) {
  const std::scoped_lock lock{mutex_};
  const std::shared_ptr<const CommittedExecutionAuthority3D> current =
      authority_.load(std::memory_order_acquire);
  if (expected_authority == nullptr || current != expected_authority ||
      expected_authority->plan() != expected_plan) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  if (expected_plan == nullptr || !expected_plan->publishable()) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  return publishAuthorityLocked(expected_authority, expected_plan, owner,
                                std::move(input),
                                carriedAppliedControl(*expected_authority, owner));
}

bool RouteExecutionManager3D::publishAppliedControlIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const AppliedControlEvidence3D& control) {
  const std::scoped_lock lock{mutex_};
  if (expected_authority == nullptr || !expected_authority->owner().valid ||
      expected_authority->input() == nullptr || !control.valid ||
      !control.validFor(expected_authority->owner())) {
    return false;
  }
  return publishAuthorityLocked(expected_authority, expected_authority->plan(),
                                expected_authority->owner(),
                                expected_authority->input(), control) ==
         ExecutionRoutePublicationStatus3D::kPublished;
}

bool RouteExecutionManager3D::clearAppliedControlIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority) {
  const std::scoped_lock lock{mutex_};
  if (expected_authority == nullptr || !expected_authority->control().valid) {
    return false;
  }
  return publishAuthorityLocked(expected_authority, expected_authority->plan(),
                                expected_authority->owner(),
                                expected_authority->input(),
                                AppliedControlEvidence3D{}) ==
         ExecutionRoutePublicationStatus3D::kPublished;
}

bool RouteExecutionManager3D::clearLeaseIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority) {
  const std::scoped_lock lock{mutex_};
  if (expected_authority == nullptr || !expected_authority->owner().valid) {
    return false;
  }
  return publishAuthorityLocked(expected_authority, expected_authority->plan(),
                                ExecutionOwnerIdentity3D{}, nullptr,
                                AppliedControlEvidence3D{}) ==
         ExecutionRoutePublicationStatus3D::kPublished;
}

PendingRoutePublicationResult3D RouteExecutionManager3D::publishPendingForCurrentBase(
    const std::shared_ptr<const ExecutionPlan3D>& expected_execution_base,
    PendingCertifiedRoute3D candidate) {
  if (candidate.publication_sequence != 0U) {
    return {
        .status = PendingRoutePublicationStatus3D::kInvalidCandidate,
        .pending = nullptr,
    };
  }

  const std::scoped_lock lock{mutex_};
  const std::shared_ptr<const CommittedExecutionAuthority3D> current_authority =
      authority_.load(std::memory_order_acquire);
  const std::shared_ptr<const ExecutionPlan3D> current_execution =
      current_authority != nullptr ? current_authority->plan() : nullptr;
  if (!sameExecutionRouteBase3D(expected_execution_base, current_execution)) {
    return {
        .status = PendingRoutePublicationStatus3D::kStaleExecutionBase,
        .pending = nullptr,
    };
  }
  if (pending_ != nullptr) {
    return {
        .status = PendingRoutePublicationStatus3D::kPendingOccupied,
        .pending = nullptr,
    };
  }
  if (last_accepted_pending_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return {
        .status = PendingRoutePublicationStatus3D::kSequenceExhausted,
        .pending = nullptr,
    };
  }

  candidate.publication_sequence = last_accepted_pending_sequence_ + 1U;
  auto sealed = std::make_shared<const PendingCertifiedRoute3D>(std::move(candidate));
  if (!sealed->valid() || current_execution == nullptr ||
      !pendingCertifiedRouteEligible3D(*sealed, *current_execution)) {
    return {
        .status = PendingRoutePublicationStatus3D::kInvalidCandidate,
        .pending = nullptr,
    };
  }
  pending_ = sealed;
  last_accepted_pending_sequence_ = sealed->publication_sequence;
  return {
      .status = PendingRoutePublicationStatus3D::kPublished,
      .pending = std::move(sealed),
  };
}

PendingRoutePublicationResult3D RouteExecutionManager3D::replacePendingForCurrentBase(
    const std::shared_ptr<const ExecutionPlan3D>& expected_execution_base,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    PendingCertifiedRoute3D candidate) {
  if (expected_pending == nullptr || candidate.publication_sequence != 0U) {
    return {
        .status = PendingRoutePublicationStatus3D::kInvalidCandidate,
        .pending = nullptr,
    };
  }

  const std::scoped_lock lock{mutex_};
  const std::shared_ptr<const CommittedExecutionAuthority3D> current_authority =
      authority_.load(std::memory_order_acquire);
  const std::shared_ptr<const ExecutionPlan3D> current_execution =
      current_authority != nullptr ? current_authority->plan() : nullptr;
  if (!sameExecutionRouteBase3D(expected_execution_base, current_execution)) {
    return {
        .status = PendingRoutePublicationStatus3D::kStaleExecutionBase,
        .pending = nullptr,
    };
  }
  if (pending_ != expected_pending) {
    return {
        .status = PendingRoutePublicationStatus3D::kPendingChanged,
        .pending = nullptr,
    };
  }
  if (last_accepted_pending_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    return {
        .status = PendingRoutePublicationStatus3D::kSequenceExhausted,
        .pending = nullptr,
    };
  }

  candidate.publication_sequence = last_accepted_pending_sequence_ + 1U;
  auto sealed = std::make_shared<const PendingCertifiedRoute3D>(std::move(candidate));
  if (!sealed->valid() || current_execution == nullptr ||
      !pendingCertifiedRouteEligible3D(*expected_pending, *current_execution) ||
      !pendingCertifiedRouteEligible3D(*sealed, *current_execution)) {
    return {
        .status = PendingRoutePublicationStatus3D::kInvalidCandidate,
        .pending = nullptr,
    };
  }
  pending_ = sealed;
  last_accepted_pending_sequence_ = sealed->publication_sequence;
  return {
      .status = PendingRoutePublicationStatus3D::kReplaced,
      .pending = std::move(sealed),
  };
}

bool RouteExecutionManager3D::acknowledgePendingIfSame(
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending) {
  if (expected_pending == nullptr) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  if (pending_ != expected_pending) {
    return false;
  }
  pending_.reset();
  return true;
}

ExecutionRoutePublicationStatus3D
RouteExecutionManager3D::commitPendingLeasedTransition(
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const ExecutionRouteTransitionResult3D& transition,
    const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input) {
  if (expected_pending == nullptr || expected_authority == nullptr ||
      transition.next == nullptr) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  const std::scoped_lock lock{mutex_};
  const std::shared_ptr<const CommittedExecutionAuthority3D> current =
      authority_.load(std::memory_order_acquire);
  const std::shared_ptr<const ExecutionPlan3D>& expected_plan =
      expected_authority->plan();
  const CertifiedRouteSuffix3D* const committed_route = transition.next->route();
  const bool committed_route_is_pending_revision =
      committed_route != nullptr &&
      (committed_route->route_instance_id ==
           expected_pending->route.route_instance_id ||
       committed_route->parent_route_instance_id ==
           std::optional<RouteInstanceId3D>{expected_pending->route.route_instance_id});
  if (current != expected_authority || pending_ != expected_pending ||
      expected_plan == nullptr) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  if (!pendingCertifiedRouteEligible3D(*expected_pending, *expected_plan) ||
      !committed_route_is_pending_revision) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  const ExecutionRoutePublicationStatus3D publication =
      publishTransitionLocked(expected_authority, transition, owner, std::move(input),
                              DetachedTransitionIntent3D::kKeepThePlanPublishable);
  if (publication != ExecutionRoutePublicationStatus3D::kPublished) {
    return publication;
  }
  pending_.reset();
  return ExecutionRoutePublicationStatus3D::kPublished;
}

} // namespace drone_city_nav
