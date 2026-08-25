#include <cstdint>
#include <stdexcept>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::configureRouteStrategyArbitration() {
  route_proposal_selection_3d_config_.productive_direct_minimum_mission_progress_m =
      declare_parameter<double>(
          "route_proposal_productive_direct_minimum_mission_progress_m", 2.0);
  route_proposal_selection_3d_config_.productive_direct_minimum_progress_ratio =
      declare_parameter<double>(
          "route_proposal_productive_direct_minimum_progress_ratio", 0.15);

  RouteStrategyArbitration3DConfig config;
  config.topology_mission_lease_budget_m = declare_parameter<double>(
      "route_strategy_topology_mission_lease_budget_m", 160.0);
  config.observation_frontier_lease_budget_m = declare_parameter<double>(
      "route_strategy_observation_frontier_lease_budget_m", 80.0);
  config.topological_backtrack_lease_budget_m = declare_parameter<double>(
      "route_strategy_topological_backtrack_lease_budget_m", 120.0);
  config.minimum_lease_commitment_m =
      declare_parameter<double>("route_strategy_minimum_lease_commitment_m", 5.0);
  config.minimum_return_mission_progress_m = declare_parameter<double>(
      "route_strategy_minimum_return_mission_progress_m", 8.0);
  config.direct_release_minimum_progress_advantage_m = declare_parameter<double>(
      "route_strategy_direct_release_minimum_progress_advantage_m", 5.0);
  config.direct_release_minimum_progress_ratio_advantage = declare_parameter<double>(
      "route_strategy_direct_release_minimum_progress_ratio_advantage", 0.10);
  const std::int64_t direct_release_confirmation_count =
      declare_parameter<std::int64_t>(
          "route_strategy_direct_release_confirmation_count", 2);
  if (direct_release_confirmation_count <= 0) {
    throw std::invalid_argument{
        "invalid route strategy direct release confirmation count"};
  }
  config.direct_release_confirmation_count =
      static_cast<std::size_t>(direct_release_confirmation_count);
  if (!routeProposalSelection3DConfigIsValid(route_proposal_selection_3d_config_) ||
      !routeStrategyArbitration3DConfigIsValid(config)) {
    throw std::invalid_argument{"invalid route strategy arbitration configuration"};
  }
  const std::scoped_lock lock{pending_route_transaction_mutex_,
                              route_strategy_arbitrator_mutex_};
  route_strategy_arbitrator_3d_ = RouteStrategyArbitrator3D{config};
}

} // namespace drone_city_nav
