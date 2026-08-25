#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::recordPendingRouteStrategyOutcome(
    const std::shared_ptr<const PendingCertifiedRoute3D>& pending,
    const bool selection_committed) noexcept {
  if (pending == nullptr) {
    return;
  }
  if (pending->strategy_decision.has_value()) {
    const std::scoped_lock lock{route_strategy_arbitrator_mutex_};
    static_cast<void>(route_strategy_arbitrator_3d_.recordOutcome(
        *pending->strategy_decision, selection_committed));
  }
  if (!selection_committed || !topological_navigation_3d_) {
    return;
  }
  switch (pending->topology_effect.kind) {
    case PendingTopologyEffectKind3D::kNone:
      break;
    case PendingTopologyEffectKind3D::kCommitAcceptedPlan:
      if (pending->topology_effect.plan.has_value()) {
        static_cast<void>(topological_navigation_3d_->commitAcceptedPlan(
            *pending->topology_effect.plan));
      }
      break;
    case PendingTopologyEffectKind3D::kSupersedeAcceptedPlan:
      if (topological_navigation_3d_->supersedeAcceptedPlan()) {
        RCLCPP_INFO(get_logger(), "INCREMENTAL_TOPOLOGY3D_PLAN_SUPERSEDED "
                                  "replacement=non_topology_route");
      }
      break;
  }
}

} // namespace drone_city_nav
