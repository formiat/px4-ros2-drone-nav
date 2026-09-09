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

PendingRoutePublicationResult3D ExecutionSupervisor3D::replacePendingForCurrentBase(
    const std::shared_ptr<const ExecutionPlan3D>& expected_execution_base,
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending,
    PendingCertifiedRoute3D candidate) {
  return manager_.replacePendingForCurrentBase(expected_execution_base,
                                               expected_pending, std::move(candidate));
}

bool ExecutionSupervisor3D::acknowledgePendingIfSame(
    const std::shared_ptr<const PendingCertifiedRoute3D>& expected_pending) {
  return manager_.acknowledgePendingIfSame(expected_pending);
}

ExecutionRoutePublicationStatus3D ExecutionSupervisor3D::commitDetachedTransition(
    const std::shared_ptr<const CommittedExecutionAuthority3D>& expected_authority,
    const ExecutionRouteTransitionResult3D& transition,
    const DetachedTransitionIntent3D intent) {
  return manager_.publishDetachedTransition(expected_authority, transition, intent);
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
