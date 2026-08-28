#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::recordPendingRouteStrategyOutcome(
    const std::shared_ptr<const PendingCertifiedRoute3D>& pending,
    const bool selection_committed) noexcept {
  const std::scoped_lock lock{pending_route_transaction_mutex_};
  recordPendingRouteStrategyOutcomeLocked(pending, selection_committed);
}

void ProductionMppiNode::recordPendingRouteStrategyOutcomeLocked(
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
}

} // namespace drone_city_nav
