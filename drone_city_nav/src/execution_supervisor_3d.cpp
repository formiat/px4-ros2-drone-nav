#include "drone_city_nav/execution_supervisor_3d.hpp"

#include <utility>

namespace drone_city_nav {

RouteExecutionManagerSnapshot3D ExecutionSupervisor3D::snapshot() const {
  return manager_.snapshot();
}

std::shared_ptr<const CommittedExecutionAuthority3D>
ExecutionSupervisor3D::authority() const noexcept {
  return manager_.authority();
}

std::shared_ptr<const ExecutionPlan3D> ExecutionSupervisor3D::plan() const {
  return manager_.plan();
}

std::shared_ptr<const PendingCertifiedRoute3D> ExecutionSupervisor3D::pending() const {
  return manager_.pending();
}

PendingRoutePublicationResult3D ExecutionSupervisor3D::publishPendingForCurrentBase(
    const std::shared_ptr<const ExecutionPlan3D>& expected_execution_base,
    PendingCertifiedRoute3D candidate) {
  return manager_.publishPendingForCurrentBase(expected_execution_base,
                                               std::move(candidate));
}

bool ExecutionSupervisor3D::acknowledgePendingIfSame(
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending) {
  return manager_.acknowledgePendingIfSame(expected_pending);
}

ExecutionRoutePublicationStatus3D
ExecutionSupervisor3D::commitLease(ExecutionLeaseCommit3D commit) {
  if (commit.expected_authority == nullptr || commit.expected_plan == nullptr ||
      !commit.expected_authority->valid() ||
      commit.expected_authority->plan() != commit.expected_plan ||
      !commit.owner.valid || commit.input == nullptr) {
    return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
  }
  switch (commit.kind) {
    case ExecutionLeaseCommitKind3D::kTransition:
      if (!commit.transition.has_value() || commit.expected_pending != nullptr) {
        return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
      }
      return manager_.publishLeasedTransition(commit.expected_authority,
                                              *commit.transition, commit.owner,
                                              std::move(commit.input));
    case ExecutionLeaseCommitKind3D::kUnchangedPlan:
      if (commit.transition.has_value() || commit.expected_pending != nullptr) {
        return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
      }
      return manager_.publishLeaseForUnchangedPlanIfSame(
          commit.expected_authority, commit.expected_plan, commit.owner,
          std::move(commit.input));
    case ExecutionLeaseCommitKind3D::kPendingTransition:
      if (!commit.transition.has_value() || commit.expected_pending == nullptr) {
        return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
      }
      return manager_.commitPendingLeasedTransition(
          commit.expected_pending, commit.expected_authority, *commit.transition,
          commit.owner, std::move(commit.input));
  }
  return ExecutionRoutePublicationStatus3D::kInvalidCandidate;
}

ExecutionRoutePublicationStatus3D ExecutionSupervisor3D::commitDetachedTransition(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const ExecutionRouteTransitionResult3D& transition) {
  return manager_.publishDetachedTransition(expected_authority, transition);
}

bool ExecutionSupervisor3D::publishAppliedControlIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const AppliedControlEvidence3D& control) {
  return manager_.publishAppliedControlIfSame(expected_authority, control);
}

bool ExecutionSupervisor3D::clearAppliedControlIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority) {
  return manager_.clearAppliedControlIfSame(expected_authority);
}

bool ExecutionSupervisor3D::clearLeaseIfSame(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority) {
  return manager_.clearLeaseIfSame(expected_authority);
}

} // namespace drone_city_nav
