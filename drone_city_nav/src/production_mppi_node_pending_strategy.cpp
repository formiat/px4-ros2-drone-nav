#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::commitPendingRouteStrategyDecision() noexcept {
  if (!pending_route_strategy_decision_.has_value()) {
    return;
  }
  static_cast<void>(route_strategy_arbitrator_3d_.recordOutcome(
      *pending_route_strategy_decision_, true));
  pending_route_strategy_decision_.reset();
}

void ProductionMppiNode::rollbackPendingRouteStrategyDecision() noexcept {
  const std::scoped_lock strategy_lock{execution_evidence_commit_mutex_};
  if (!pending_route_strategy_decision_.has_value()) {
    return;
  }
  static_cast<void>(route_strategy_arbitrator_3d_.recordOutcome(
      *pending_route_strategy_decision_, false));
  pending_route_strategy_decision_.reset();
}

} // namespace drone_city_nav
