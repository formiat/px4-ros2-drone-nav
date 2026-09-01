#include "production_mppi_node.hpp"

#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/producer_instance_id.hpp"
#include "drone_city_nav/static_esdf_cache.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cinttypes>
#include <filesystem>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>
#include <utility>

#include "execution_horizon_assembler_3d.hpp"
#include "mppi_controller_3d.hpp"
#include "navigation_diagnostics_sink.hpp"
#include "planning_cycle_coordinator_3d.hpp"
#include "production_mppi_config_ros.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"
#include "raw_world_ingress_ros_3d.hpp"
#include "static_world_builder_3d.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kExecutionHorizonProducerDomain{0x45584543484f5249ULL};
constexpr std::uint64_t kNavigationHealthProducerDomain{0x4e41564845414c54ULL};

[[nodiscard]] std::filesystem::path
resolvePackagePath(const std::filesystem::path& package_share,
                   std::filesystem::path path) {
  if (path.is_relative()) {
    path = package_share / path;
  }
  return path;
}

[[nodiscard]] StaticWorldResources3D
loadStaticWorldResources(const ProductionMppiConfig& config,
                         const rclcpp::Logger& logger) {
  StaticWorldResources3D resources;
  if (!config.world.use_static_map) {
    return resources;
  }

  const std::filesystem::path package_share{
      ament_index_cpp::get_package_share_directory("drone_city_nav")};
  const std::filesystem::path occupancy_path =
      resolvePackagePath(package_share, config.world.static_occupancy_3d_path);
  resources.occupancy =
      std::make_shared<const OccupancyGrid3D>(OccupancyGrid3D::load(occupancy_path));
  const std::shared_ptr<const OccupancyGrid3D>& occupancy = resources.occupancy;
  if (occupancy->contentFingerprint() == 0U) {
    throw std::runtime_error{"invalid static occupancy content fingerprint"};
  }

  std::filesystem::path cache_path = config.world.static_esdf_3d_cache_path;
  if (cache_path.empty()) {
    cache_path = occupancy_path;
    cache_path.replace_extension(".esdf3d");
  } else {
    cache_path = resolvePackagePath(package_share, std::move(cache_path));
  }
  const double required_maximum_distance_m =
      static_cast<double>(config.control.mppi.risk.preferred_distance_m) + 20.0;
  try {
    StaticEsdfCache cache = StaticEsdfCache::load(cache_path);
    if (cache.compatibleWith(*occupancy, required_maximum_distance_m)) {
      RCLCPP_INFO(logger,
                  "STATIC_ESDF_CACHE_READY path=%s fingerprint=%" PRIu64
                  " maximum_distance_m=%.2f chunks=%zu bytes=%zu "
                  "shared_resource_reused=%s",
                  cache_path.c_str(), cache.occupancyFingerprint(),
                  cache.maximumDistanceM(), cache.storedChunkCount(),
                  cache.compressedBytes(),
                  cache.sharedResourceReused() ? "true" : "false");
      resources.esdf_cache = std::move(cache);
    } else {
      RCLCPP_WARN(
          logger,
          "STATIC_ESDF_CACHE_FALLBACK path=%s reason=incompatible_world_or_"
          "distance cache_fingerprint=%" PRIu64 " occupancy_fingerprint=%" PRIu64
          " cache_maximum_distance_m=%.2f requested_maximum_distance_m=%.2f",
          cache_path.c_str(), cache.occupancyFingerprint(), occupancy->fingerprint(),
          cache.maximumDistanceM(), required_maximum_distance_m);
    }
  } catch (const std::exception& error) {
    RCLCPP_WARN(logger,
                "STATIC_ESDF_CACHE_FALLBACK path=%s reason=load_failed error=%s",
                cache_path.c_str(), error.what());
  }

  std::filesystem::path topology_path = config.world.static_free_space_topology_3d_path;
  if (topology_path.empty()) {
    topology_path = occupancy_path;
    topology_path.replace_extension(".topology3d");
  } else {
    topology_path = resolvePackagePath(package_share, std::move(topology_path));
  }
  try {
    FreeSpaceTopology3D topology = FreeSpaceTopology3D::load(topology_path);
    if (topology.compatibleWith(*occupancy)) {
      resources.topology =
          std::make_shared<const FreeSpaceTopology3D>(std::move(topology));
    } else {
      RCLCPP_WARN(logger,
                  "FREE_SPACE_TOPOLOGY_FALLBACK path=%s reason=incompatible_world "
                  "topology_fingerprint=%" PRIu64 " occupancy_fingerprint=%" PRIu64,
                  topology_path.c_str(), topology.occupancyFingerprint(),
                  occupancy->fingerprint());
    }
  } catch (const std::exception& error) {
    RCLCPP_WARN(logger,
                "FREE_SPACE_TOPOLOGY_FALLBACK path=%s reason=load_failed error=%s",
                topology_path.c_str(), error.what());
  }

  const std::shared_ptr<const FreeSpaceTopology3D>& topology = resources.topology;
  RCLCPP_INFO(logger,
              "STATIC_WORLD_3D path=%s fingerprint=%" PRIu64
              " occupied_voxels=%zu passage_regions=%zu portals=%zu "
              "passage_segments=%zu portal_edges=%zu topology_path=%s "
              "topology_ready=%s dimensions=%dx%dx%d",
              occupancy_path.c_str(), occupancy->fingerprint(),
              occupancy->occupiedVoxelCount(),
              topology ? topology->regions().size() : 0U,
              topology ? topology->portals().size() : 0U,
              topology ? topology->segments().size() : 0U,
              topology ? topology->traversalEdges().size() : 0U, topology_path.c_str(),
              topology ? "true" : "false", occupancy->bounds().width_cells,
              occupancy->bounds().height_cells, occupancy->bounds().depth_cells);
  return resources;
}

void logConfiguration(const ProductionMppiConfig& config,
                      const rclcpp::Logger& logger) {
  RCLCPP_INFO(
      logger,
      "Production MPPI ready: rollouts=%zu open_static_rollouts=%zu "
      "direct_tracking_rollouts=%zu adaptive_clearance_m=%.1f "
      "steps=%zu rate=%.1fHz deadline=%.1fms static_map=%s route3d=%s "
      "horizon=%.1fs static_esdf_lookahead=%.1fm cruise=%.1fmps "
      "horizontal_speed_cap=%.1fmps translational_speed_cap=%.1fmps "
      "acceleration_cap=%.1fmps2 jerk_cap=%.1fmps3 speed_tracking_weight=%.2f "
      "constrained_route_speed_limit=%.1fmps head_progress=%.2fs "
      "far_cost_sampling=(%.2fs,%u) liveness=%s "
      "persistent_route=true route_replan_remaining=%.1fm planner_workers=%zu "
      "planner_tick_phase_ms=%.1f no_static_world=observed_occupancy_3d "
      "no_static_esdf=(%.1fHz/h%.1f/v%.1f/hm%.1f/vm%.1fm/incremental_ratio=%.2f/"
      "audit_builds=%zu)",
      config.control.mppi.rollouts, config.control.rollout_budget.open_static_rollouts,
      config.control.rollout_budget.direct_tracking_rollouts,
      config.control.rollout_budget.minimum_reduced_clearance_m,
      config.control.mppi.steps, config.planning.tick_rate_hz,
      config.planning.deadline_ms, config.world.use_static_map ? "true" : "false",
      "true",
      static_cast<double>(config.control.mppi.steps) *
          config.control.mppi.dynamics.dt_s,
      config.planning.static_esdf_route_lookahead_m,
      config.control.speed_policy.cruise_speed_mps,
      config.control.mppi.dynamics.maximum_horizontal_speed_mps,
      config.control.mppi.dynamics.maximum_translational_speed_mps,
      config.control.mppi.dynamics.maximum_horizontal_acceleration_mps2,
      config.control.mppi.dynamics.maximum_control_jerk_mps3,
      config.control.mppi.costs.speed_tracking_weight,
      config.world.use_static_map ? config.planning.constrained_route_speed_limit_mps
                                  : 0.0F,
      config.control.mppi.costs.head_progress_horizon_s,
      config.control.mppi.horizon_sampling.full_rate_duration_s,
      config.control.mppi.horizon_sampling.far_cost_stride,
      config.planning.liveness.enabled ? "true" : "false",
      config.planning.route_tracking_policy.minimum_remaining_m,
      config.planning.planner_worker_count,
      config.planning.planning_tick_phase_offset_s * 1000.0,
      config.world.no_static_3d_esdf_update_rate_hz,
      config.world.no_static_3d_esdf_window.horizontal_half_extent_m,
      config.world.no_static_3d_esdf_window.vertical_half_extent_m,
      config.world.no_static_3d_esdf_window.horizontal_recenter_margin_m,
      config.world.no_static_3d_esdf_window.vertical_recenter_margin_m,
      config.world.no_static_3d_esdf_incremental_maximum_rebuild_ratio,
      config.world.no_static_3d_esdf_full_audit_interval_builds);

  if (config.planning.noncooperative_avoidance_enabled) {
    RCLCPP_INFO(logger,
                "NONCOOPERATIVE_AVOIDANCE_CONFIG enabled=true vehicle_id='%s' "
                "tracks_topic='%s' prediction_horizon_s=%.2f strong_separation_m=%.2f "
                "anticipation_separation_m=%.2f release_separation_m=%.2f "
                "maximum_track_age_s=%.2f strong_cost_weight=%.1f",
                config.planning.vehicle_id.c_str(),
                config.planning.topics.noncooperative_tracks.c_str(),
                config.planning.noncooperative_avoidance.prediction_horizon_s,
                config.planning.noncooperative_avoidance.strong_separation_m,
                config.planning.noncooperative_avoidance.anticipation_separation_m,
                config.planning.noncooperative_avoidance.release_separation_m,
                config.planning.noncooperative_avoidance.maximum_track_age_s,
                config.planning.noncooperative_avoidance.strong_cost_weight);
  }
}

} // namespace

ProductionMppiNode::ProductionMppiNode(const rclcpp::NodeOptions& options)
    : Node{"production_mppi_node", options},
      config_{declareProductionMppiConfig(*this)},
      mission_goal_{config_.planning.mission_waypoints.front()},
      navigation_health_supervisor_{std::make_unique<NavigationHealthSupervisor>(
          config_.execution.navigation_health)},
      mission_waypoint_sequence_{std::make_unique<MissionWaypointSequence>(
          config_.planning.mission_waypoints,
          config_.planning.mission_waypoint_sequence)},
      mission_waypoint_capture_gate_{std::make_unique<MissionWaypointCaptureGate>(
          config_.execution.mission_waypoint_capture_gate)},
      planning_worker_pool_{
          std::make_unique<BoundedWorkerPool>(config_.planning.planner_worker_count)},
      mppi_controller_{std::make_unique<MppiController3D>(config_.control.mppi)},
      navigation_angular_derivative_estimator_{
          config_.control.navigation_angular_derivative},
      objective_replan_anchor_{mission_goal_},
      execution_horizon_producer_instance_id_{
          createProducerInstanceId(kExecutionHorizonProducerDomain)},
      navigation_health_producer_instance_id_{
          createProducerInstanceId(kNavigationHealthProducerDomain)} {
  navigation_objective_state_.store(
      std::make_shared<const ProductionNavigationObjectiveState>(
          ProductionNavigationObjectiveState{
              .objective = std::make_shared<const ProductionNavigationObjective>(
                  ProductionNavigationObjective{
                      .goal = mission_goal_,
                      .tracking = std::nullopt,
                      .mission_epoch =
                          config_.planning.configured_mission_objective_enabled ? 1U
                                                                                : 0U,
                      .sample_sequence = 0U,
                  }),
              .minimum_tracking_route_mission_epoch = 0U,
              .minimum_tracking_route_sample_sequence = 0U,
          }),
      std::memory_order_release);

  initializeRuntimeInterfaces(loadStaticWorldResources(config_, get_logger()));
  logConfiguration(config_, get_logger());
}

} // namespace drone_city_nav

RCLCPP_COMPONENTS_REGISTER_NODE(drone_city_nav::ProductionMppiNode)
