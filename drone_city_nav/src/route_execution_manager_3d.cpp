#include "drone_city_nav/route_execution_manager_3d.hpp"

#include <limits>
#include <memory>
#include <optional>
#include <utility>

namespace drone_city_nav {

bool RouteExecutionManagerSnapshot3D::valid() const noexcept {
  return plan != nullptr && plan->valid() && (pending == nullptr || pending->valid());
}

RouteExecutionManager3D::RouteExecutionManager3D()
    : plan_{makeInitialExecutionRouteSnapshot3D()} {
}

RouteExecutionManagerSnapshot3D RouteExecutionManager3D::snapshot() const {
  const std::scoped_lock lock{mutex_};
  return RouteExecutionManagerSnapshot3D{.plan = plan_, .pending = pending_};
}

std::shared_ptr<const ExecutionPlan3D> RouteExecutionManager3D::plan() const {
  const std::scoped_lock lock{mutex_};
  return plan_;
}

std::shared_ptr<const PendingCertifiedRoute3D>
RouteExecutionManager3D::pending() const {
  const std::scoped_lock lock{mutex_};
  return pending_;
}

ExecutionRoutePublicationStatus3D RouteExecutionManager3D::publishPlanLocked(
    const std::shared_ptr<const ExecutionPlan3D>& expected_plan,
    const ExecutionRouteTransitionResult3D& transition) {
  if (plan_ == nullptr || expected_plan == nullptr || plan_ != expected_plan) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  if (!transition.applied() || transition.next == nullptr ||
      !transition.next->publishable() ||
      expected_plan->version == std::numeric_limits<std::uint64_t>::max() ||
      transition.next->version <= expected_plan->version) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  if (transition.predecessor != expected_plan.get()) {
    return ExecutionRoutePublicationStatus3D::kStaleSnapshotVersion;
  }
  plan_ = transition.next;
  return ExecutionRoutePublicationStatus3D::kPublished;
}

ExecutionRoutePublicationStatus3D RouteExecutionManager3D::publishPlan(
    const std::shared_ptr<const ExecutionPlan3D>& expected_plan,
    const ExecutionRouteTransitionResult3D& transition) {
  const std::scoped_lock lock{mutex_};
  return publishPlanLocked(expected_plan, transition);
}

bool RouteExecutionManager3D::publishPending(
    std::shared_ptr<const PendingCertifiedRoute3D> candidate) {
  if (candidate == nullptr || !candidate->valid()) {
    return false;
  }
  // Seal the publication boundary even when the caller retains a mutable alias
  // to the original allocation.
  const auto sealed = std::make_shared<const PendingCertifiedRoute3D>(*candidate);
  if (!sealed->valid()) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  if (pending_ != nullptr ||
      sealed->publication_sequence <= last_accepted_pending_sequence_) {
    return false;
  }
  pending_ = sealed;
  last_accepted_pending_sequence_ = sealed->publication_sequence;
  return true;
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

bool RouteExecutionManager3D::commitPendingTransitionIfSame(
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    const std::shared_ptr<const ExecutionPlan3D>& expected_plan,
    const ExecutionRouteTransitionResult3D& transition) {
  if (expected_pending == nullptr || expected_plan == nullptr ||
      transition.next == nullptr) {
    return false;
  }
  const std::scoped_lock lock{mutex_};
  const CertifiedRouteSuffix3D* const committed_route = transition.next->route();
  const bool committed_route_is_pending_revision =
      committed_route != nullptr &&
      (committed_route->route_instance_id ==
           expected_pending->route.route_instance_id ||
       committed_route->parent_route_instance_id ==
           std::optional<RouteInstanceId3D>{expected_pending->route.route_instance_id});
  if (pending_ != expected_pending || plan_ != expected_plan ||
      !pendingCertifiedRouteEligible3D(*expected_pending, *expected_plan) ||
      !committed_route_is_pending_revision) {
    return false;
  }
  if (publishPlanLocked(expected_plan, transition) !=
      ExecutionRoutePublicationStatus3D::kPublished) {
    return false;
  }
  pending_.reset();
  return true;
}

} // namespace drone_city_nav
