#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::recordPendingRouteStrategyOutcome(
    const std::shared_ptr<const PendingCertifiedRoute3D>& pending,
    const bool selection_committed) noexcept {
  if (pending == nullptr || !pending->strategy_decision.has_value()) {
    return;
  }
  static_cast<void>(route_strategy_arbitrator_3d_.recordOutcome(
      *pending->strategy_decision, selection_committed));
}

} // namespace drone_city_nav
