#include <chrono>
#include <cinttypes>
#include <cmath>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "production_mppi_node.hpp"
#include "production_mppi_route_helpers.hpp"

namespace drone_city_nav {

void ProductionMppiNode::processStaticGuideSearch(
    const ProductionMppiPreparedEsdf& world,
    const ProductionMppiNavigation& navigation) {
  Vec3 preferred_direction{static_cast<double>(navigation.state.vx),
                           static_cast<double>(navigation.state.vy),
                           static_cast<double>(navigation.state.vz)};
  if (std::hypot(preferred_direction.x, preferred_direction.y) < 0.5) {
    preferred_direction =
        Vec3{mission_goal_.x - navigation.state.x, mission_goal_.y - navigation.state.y,
             mission_goal_.z - navigation.state.z};
  }
  const auto search_started = std::chrono::steady_clock::now();
  const std::span<const ConstrainedFreeSpaceEdge> channel_edges =
      world.channel_edges
          ? std::span<const ConstrainedFreeSpaceEdge>{*world.channel_edges}
          : std::span<const ConstrainedFreeSpaceEdge>{};
  const RiskAwareLattice3DResult lattice = planRiskAwareLattice3D(
      world.grid, *world.distances_m,
      Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      preferred_direction, mission_goal_, channel_edges, lattice_3d_config_);
  const double search_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - search_started)
                               .count();
  ProductionMppiPreparedEsdf prepared = world;
  prepared.global_guide_search_ms = search_ms;
  prepared.lattice_search_performed = true;
  prepared.lattice_executable =
      lattice.status == Lattice3DStatus::kReachedPlanningGoal ||
      lattice.status == Lattice3DStatus::kViableFrontier;
  prepared.global_guide_expansions = lattice.expansions;
  prepared.lattice_terminal_successor_count = lattice.terminal_successor_count;
  prepared.lattice_continuation_reachable_states =
      lattice.continuation_reachable_states;
  prepared.lattice_reachable_depth_m = lattice.continuation_reachable_depth_m;
  prepared.global_guide_cost = lattice.objective_cost;
  prepared.global_guide_reaches_mission_goal = lattice.reached_mission_goal;
  prepared.topology_candidates = lattice.topology_candidates;
  prepared.topology_objective_cost = lattice.objective_cost;
  prepared.topology_route_length_m = lattice.route_length_m;
  prepared.topology_travel_time_s = lattice.estimated_travel_time_s;
  prepared.topology_vertical_alignment_time_s = lattice.vertical_alignment_time_s;
  prepared.topology_planning_exposure_m = lattice.planning_exposure_m;
  prepared.topology_critical_exposure_m = lattice.critical_exposure_m;

  StaticRouteCandidateValidation validation{.status =
                                                StaticRouteCandidateStatus::kEmpty};
  if (prepared.lattice_executable) {
    auto mutable_route = std::make_shared<std::vector<RouteSample3D>>(lattice.route);
    if (assignRouteRiskTiers(*mutable_route, world.grid, *world.distances_m,
                             mppi_config_.risk.critical_distance_m,
                             mppi_config_.risk.preferred_distance_m)) {
      validation = validateStaticRouteCandidate(
          world.route_3d ? std::span<const RouteSample3D>{*world.route_3d}
                         : std::span<const RouteSample3D>{},
          *mutable_route, world.grid, *world.distances_m, mission_goal_,
          static_route_extension_config_.minimum_endpoint_improvement_m,
          lattice.reached_mission_goal,
          world.static_route_extension_request || world.static_route_replan_request);
    } else {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidEsdf};
    }
    const std::shared_ptr<const std::vector<RouteSample3D>> route = mutable_route;
    const std::uint64_t candidate_generation = static_route_generation_ + 1U;
    auto spans = std::make_shared<const std::vector<ConstrainedRouteSpan>>(
        makeConstrainedRouteSpans(*route, lattice.selected_channels,
                                  candidate_generation, route_envelope_config_));
    if (validation.accepted && !validateConstrainedRouteSpans(
                                   *route, *spans, world.grid, *world.distances_m)) {
      validation = StaticRouteCandidateValidation{
          .status = StaticRouteCandidateStatus::kInvalidChannelSpan};
    }
    std::vector<std::string> selected_channel_ids;
    selected_channel_ids.reserve(lattice.selected_channels.size());
    for (const SelectedChannelTraversal& traversal : lattice.selected_channels) {
      selected_channel_ids.push_back(traversal.channel_id);
    }
    prepared.route_3d = route;
    prepared.route_2d_projection = projectRouteTo2D(*route);
    prepared.constrained_spans = spans;
    prepared.selected_channel_ids = std::make_shared<const std::vector<std::string>>(
        std::move(selected_channel_ids));
    prepared.mppi_route =
        makeMppiRoute3D(*route, *spans, speed_policy_config_.cruise_speed_mps,
                        constrained_route_speed_limit_mps_);
    prepared.global_guide_projection = projectOntoGlobalGuide(
        *prepared.route_2d_projection, Point2{navigation.state.x, navigation.state.y});
  }

  bool activated = false;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    const std::uint64_t required_base_generation =
        world.static_route_replan_request
            ? world.static_route_replan_base_generation
            : world.static_route_extension_base_generation;
    const bool generation_matches =
        (!world.static_route_extension_request && !world.static_route_replan_request) ||
        (prepared_esdf_ &&
         prepared_esdf_->global_guide_generation == required_base_generation);
    if (validation.accepted && prepared_esdf_ &&
        prepared_esdf_->revision == prepared.revision && generation_matches) {
      prepared.global_guide_generation = ++static_route_generation_;
      prepared.global_guide_release_reason = GlobalGuideReleaseReason::kNone;
      prepared.static_route_extension_request = false;
      prepared.static_route_extension_base_generation = 0U;
      prepared.static_route_replan_request = false;
      prepared.static_route_replan_base_generation = 0U;
      prepared.static_route_replan_reason = GlobalGuideReleaseReason::kNone;
      prepared_esdf_ = prepared;
      activated = true;
    }
  }
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_GUIDE3D revision=%" PRIu64
      " activated=%s extension=%s replan=%s base_generation=%" PRIu64
      " replan_reason=%s"
      " validation=%.*s endpoint_improvement_m=%.2f status=%s "
      "points=%zu samples=%zu spans=%zu expansions=%zu "
      "risk_stage=%u objective=%.3f route_length_m=%.2f travel_time_s=%.2f "
      "vertical_alignment_time_s=%.2f planning_exposure_m=%.2f "
      "critical_exposure_m=%.2f selected_channels=%zu search_ms=%.2f",
      prepared.revision, activated ? "true" : "false",
      world.static_route_extension_request ? "true" : "false",
      world.static_route_replan_request ? "true" : "false",
      world.static_route_replan_request ? world.static_route_replan_base_generation
                                        : world.static_route_extension_base_generation,
      globalGuideReleaseReasonName(world.static_route_replan_reason),
      static_cast<int>(staticRouteCandidateStatusName(validation.status).size()),
      staticRouteCandidateStatusName(validation.status).data(),
      validation.endpoint_improvement_m, lattice3DStatusName(lattice.status),
      lattice.points.size(), lattice.route.size(),
      prepared.constrained_spans ? prepared.constrained_spans->size() : 0U,
      lattice.expansions, static_cast<unsigned>(lattice.risk_stage),
      lattice.objective_cost, lattice.route_length_m, lattice.estimated_travel_time_s,
      lattice.vertical_alignment_time_s, lattice.planning_exposure_m,
      lattice.critical_exposure_m, lattice.selected_channels.size(), search_ms);
  for (const Lattice3DTopologyCandidate& candidate : lattice.topology_candidates) {
    RCLCPP_INFO(get_logger(),
                "PRODUCTION_MPPI_TOPOLOGY_CANDIDATE revision=%" PRIu64
                " topology=%s risk_stage=%u status=%s objective=%.3f "
                "route_length_m=%.2f travel_time_s=%.2f "
                "vertical_alignment_time_s=%.2f planning_exposure_m=%.2f "
                "critical_exposure_m=%.2f turn_cost=%.3f selected=%s reason=%s",
                prepared.revision, candidate.topology.c_str(),
                static_cast<unsigned>(candidate.risk_stage),
                lattice3DStatusName(candidate.status), candidate.objective_cost,
                candidate.route_length_m, candidate.estimated_travel_time_s,
                candidate.vertical_alignment_time_s, candidate.planning_exposure_m,
                candidate.critical_exposure_m, candidate.turn_cost,
                candidate.selected ? "true" : "false",
                candidate.decision_reason.c_str());
  }
  if (world.static_route_extension_request) {
    finishStaticRouteExtension(world.static_route_extension_base_generation);
  }
  if (world.static_route_replan_request) {
    finishStaticRouteReplan(world.static_route_replan_base_generation);
  }
}

} // namespace drone_city_nav
