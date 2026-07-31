#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/navigation_state_prediction.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/semantic_portal_occupancy.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <span>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] double distance3(const mppi::State& first, const mppi::State& second) {
  return std::hypot(std::hypot(static_cast<double>(first.x - second.x),
                               static_cast<double>(first.y - second.y)),
                    static_cast<double>(first.z - second.z));
}

[[nodiscard]] mppi::State interpolateState(const mppi::State& first,
                                           const mppi::State& second,
                                           const double ratio) {
  const float clamped = static_cast<float>(std::clamp(ratio, 0.0, 1.0));
  return mppi::State{
      .x = std::lerp(first.x, second.x, clamped),
      .y = std::lerp(first.y, second.y, clamped),
      .z = std::lerp(first.z, second.z, clamped),
      .vx = std::lerp(first.vx, second.vx, clamped),
      .vy = std::lerp(first.vy, second.vy, clamped),
      .vz = std::lerp(first.vz, second.vz, clamped),
      .yaw = std::lerp(first.yaw, second.yaw, clamped),
      .yaw_rate = std::lerp(first.yaw_rate, second.yaw_rate, clamped),
  };
}

[[nodiscard]] mppi::State sampleState(const std::span<const mppi::State> states,
                                      const double offset_steps) {
  if (states.empty()) {
    return {};
  }
  const double source =
      std::clamp(offset_steps, 0.0, static_cast<double>(states.size() - 1U));
  const std::size_t lower = static_cast<std::size_t>(std::floor(source));
  const std::size_t upper = std::min(lower + 1U, states.size() - 1U);
  return interpolateState(states[lower], states[upper],
                          source - static_cast<double>(lower));
}

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RoutePoint>>
makeMppiRoute(const SemanticPortalRoute& route) {
  auto points = std::make_shared<std::vector<mppi::RoutePoint>>();
  points->reserve(route.polyline ? route.polyline->size() : 0U);
  if (!route.polyline || route.polyline->size() != route.point_stations_m.size()) {
    return points;
  }
  for (std::size_t index = 0U; index < route.polyline->size(); ++index) {
    points->push_back(mppi::RoutePoint{
        .x_m = static_cast<float>((*route.polyline)[index].x),
        .y_m = static_cast<float>((*route.polyline)[index].y),
        .station_m = static_cast<float>(route.point_stations_m[index]),
    });
  }
  return points;
}

} // namespace

void ProductionMppiNode::esdfWorker(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    msg::RawObstacleSnapshot::ConstSharedPtr snapshot;
    {
      std::unique_lock lock{raw_queue_mutex_};
      raw_queue_condition_.wait(lock, stop_token,
                                [this]() { return pending_raw_snapshot_ != nullptr; });
      if (stop_token.stop_requested()) {
        return;
      }
      snapshot = std::exchange(pending_raw_snapshot_, nullptr);
    }
    if (!snapshot) {
      continue;
    }
    const std::int64_t source_stamp_ns = get_clock()->now().nanoseconds();
    RawOccupancyGridFromRosResult conversion =
        rawOccupancyGridFromRos(snapshot->grid, RawOccupancyGridFromRosConfig{100, 0});
    if (!conversion.grid.has_value()) {
      RCLCPP_WARN(get_logger(),
                  "PRODUCTION_MPPI_ESDF rejected revision=%" PRIu64
                  " reason=invalid_raw_grid",
                  snapshot->obstacle_snapshot_revision);
      continue;
    }
    const SemanticPortalOccupancyResult semantic_occupancy =
        known_passages_.has_value()
            ? overlaySemanticPortalSideSolids(*conversion.grid, *known_passages_)
            : SemanticPortalOccupancyResult{};
    const auto build_started = std::chrono::steady_clock::now();
    const DistanceField2D field = DistanceField2D::build(
        *conversion.grid,
        static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0,
        DistanceFieldSource::kOccupied);
    const double build_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - build_started)
                                .count();
    const auto conversion_started = std::chrono::steady_clock::now();
    std::vector<float> distances;
    distances.reserve(field.distancesM().size());
    for (const double distance_m : field.distancesM()) {
      distances.push_back(static_cast<float>(distance_m));
    }
    const double conversion_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  conversion_started)
            .count();
    const GridBounds& bounds = field.bounds();
    const mppi::EsdfGrid grid{bounds.width_cells, bounds.height_cells,
                              static_cast<float>(bounds.resolution_m),
                              static_cast<float>(bounds.origin_x),
                              static_cast<float>(bounds.origin_y)};
    const mppi::EsdfUploadResult upload = engine_->updateEsdf(
        mppi::EsdfSnapshot{grid, distances, snapshot->obstacle_snapshot_revision});
    if (!upload.accepted) {
      continue;
    }

    ProductionMppiPreparedEsdf prepared;
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      if (prepared_esdf_.has_value()) {
        prepared = *prepared_esdf_;
      }
    }
    prepared.producer_instance_id = snapshot->producer_instance_id;
    prepared.revision = snapshot->obstacle_snapshot_revision;
    prepared.source_stamp_ns = source_stamp_ns;
    prepared.ready_stamp_ns = get_clock()->now().nanoseconds();
    prepared.build_ms = build_ms;
    prepared.conversion_ms = conversion_ms;
    prepared.upload_ms = upload.upload_ms;
    prepared.grid = grid;
    prepared.distances_m =
        std::make_shared<const std::vector<float>>(std::move(distances));
    prepared.semantic_side_volumes = semantic_occupancy.side_volumes_considered;
    prepared.semantic_side_cells = semantic_occupancy.cells_marked_occupied;
    prepared.lattice_search_performed = false;
    prepared.lattice_continuation_attempt = 0U;

    {
      const std::scoped_lock lock{esdf_state_mutex_};
      prepared_esdf_ = prepared;
    }
    auto guide_world = std::make_shared<const ProductionMppiPreparedEsdf>(prepared);
    {
      const std::scoped_lock lock{guide_queue_mutex_};
      if (pending_guide_world_) {
        dropped_guide_worlds_.fetch_add(1U, std::memory_order_relaxed);
      }
      pending_guide_world_ = std::move(guide_world);
    }
    guide_queue_condition_.notify_all();

    RCLCPP_INFO(
        get_logger(),
        "PRODUCTION_MPPI_ESDF revision=%" PRIu64
        " build_ms=%.2f conversion_ms=%.2f upload_ms=%.2f "
        "raw_to_ready_ms=%.2f dropped_raw=%" PRIu64 " dropped_guide_worlds=%" PRIu64,
        prepared.revision, prepared.build_ms, prepared.conversion_ms,
        prepared.upload_ms,
        static_cast<double>(prepared.ready_stamp_ns - prepared.source_stamp_ns) / 1.0e6,
        dropped_raw_snapshots_.load(std::memory_order_relaxed),
        dropped_guide_worlds_.load(std::memory_order_relaxed));
  }
}

void ProductionMppiNode::guideWorker(const std::stop_token stop_token) {
  std::size_t active_guide_expansions = 0U;
  double active_guide_cost = 0.0;
  std::shared_ptr<const SemanticPortalRoute> semantic_route;
  std::shared_ptr<const std::vector<mppi::RoutePoint>> mppi_route;
  std::shared_ptr<const std::vector<Point2>> semantic_route_source;
  SemanticPortalRouteBuildResult semantic_route_build;
  std::shared_ptr<const ProductionMppiPreparedEsdf> world;
  RiskAwareLatticeSearchSession search_session;
  std::optional<ProductionMppiNavigation> search_navigation;
  std::size_t continuation_attempt = 0U;
  while (!stop_token.stop_requested()) {
    {
      std::unique_lock lock{guide_queue_mutex_};
      if (!world) {
        guide_queue_condition_.wait(
            lock, stop_token, [this]() { return pending_guide_world_ != nullptr; });
      }
      if (stop_token.stop_requested()) {
        return;
      }
      if (pending_guide_world_) {
        world = std::exchange(pending_guide_world_, nullptr);
        search_session.reset();
        search_navigation.reset();
        continuation_attempt = 0U;
      }
    }
    if (!world || !world->distances_m) {
      world.reset();
      continue;
    }
    const mppi::EsdfGrid& grid = world->grid;
    const std::shared_ptr<const std::vector<float>>& host_distances =
        world->distances_m;
    RiskAwareLatticeConfig search_config = lattice_config_;
    const std::size_t continuation_scale =
        std::size_t{1U} << std::min<std::size_t>(continuation_attempt, 3U);
    search_config.maximum_expansions =
        lattice_config_.maximum_expansions * continuation_scale;
    search_config.maximum_search_time_ms = lattice_config_.maximum_search_time_ms *
                                           static_cast<double>(continuation_scale);
    if (!search_navigation.has_value()) {
      const std::scoped_lock lock{input_mutex_};
      search_navigation = navigation_;
    }
    const ProductionMppiNavigation navigation = *search_navigation;
    ActiveGlobalGuideUpdate guide_update;
    GlobalGuideHeading guide_heading;
    RiskAwareLatticeResult lattice_observation;
    bool lattice_search_performed = false;
    GlobalGuideAcceptanceResult guide_acceptance;
    std::shared_ptr<const std::vector<Point2>> guide;
    if (navigation.valid && active_guide_lifecycle_) {
      const Point2 position{navigation.state.x, navigation.state.y};
      const std::int64_t blacklist_now_ns = get_clock()->now().nanoseconds();
      std::erase_if(frontier_blacklist_,
                    [blacklist_now_ns](const LatticeFrontierBlacklistEntry& entry) {
                      return entry.expires_at_ns <= blacklist_now_ns;
                    });
      const std::shared_ptr<const std::vector<Point2>> previous_active_guide =
          active_guide_lifecycle_->guide();
      guide_update = active_guide_lifecycle_->update(
          grid, *host_distances, position,
          guide_release_generation_.load(std::memory_order_acquire),
          guide_release_reason_.load(std::memory_order_relaxed));
      if (frontier_blacklist_enabled_ && !guide_update.active &&
          previous_active_guide && previous_active_guide->size() >= 2U &&
          (guide_update.release_reason == GlobalGuideReleaseReason::kStalled ||
           guide_update.release_reason ==
               GlobalGuideReleaseReason::kPersistentSafetyRejection)) {
        const GlobalGuideProjection failure_projection =
            projectOntoGlobalGuide(*previous_active_guide, position);
        const Point2 failure_point =
            failure_projection.valid ? failure_projection.point : position;
        const Point2 failure_tangent =
            failure_projection.valid ? failure_projection.tangent
                                     : Point2{navigation.state.vx, navigation.state.vy};
        frontier_blacklist_.push_back(LatticeFrontierBlacklistEntry{
            .failure_point = failure_point,
            .approach_heading_rad = std::atan2(failure_tangent.y, failure_tangent.x),
            .expires_at_ns = blacklist_now_ns + static_cast<std::int64_t>(
                                                    frontier_blacklist_ttl_s_ * 1.0e9),
        });
        constexpr std::size_t kMaximumFrontierBlacklistEntries{8U};
        if (frontier_blacklist_.size() > kMaximumFrontierBlacklistEntries) {
          frontier_blacklist_.erase(frontier_blacklist_.begin());
        }
      }
      if (guide_update.active) {
        guide = active_guide_lifecycle_->guide();
        guide_heading.source = GlobalGuideHeadingSource::kActiveGuide;
        if (guide_update.requires_replan) {
          guide_heading = active_guide_lifecycle_->selectPlanningHeading(
              navigation.state, Point2{mission_goal_.x, mission_goal_.y});
          lattice_observation = planRiskAwareMotionPrimitiveGuide(
              grid, *host_distances, position, guide_heading.heading_rad,
              Point2{mission_goal_.x, mission_goal_.y}, search_config,
              semantic_portal_primitives_, frontier_blacklist_, &search_session);
          lattice_search_performed = true;
          const auto candidate =
              std::make_shared<const std::vector<Point2>>(lattice_observation.guide);
          const bool reaches_mission_goal =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal &&
              lattice_observation.reached_mission_goal &&
              lattice_observation.exact_terminal_connector && !candidate->empty() &&
              distance(candidate->back(), Point2{mission_goal_.x, mission_goal_.y}) <=
                  1.0e-6;
          const bool executable =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal ||
              lattice_observation.status == LatticePlanStatus::kViableFrontier;
          if (executable) {
            ActiveGlobalGuideLifecycle validator{active_guide_config_};
            if (validator
                    .accept(candidate, reaches_mission_goal, grid, *host_distances,
                            position)
                    .accepted) {
              if (guide_update.release_reason == GlobalGuideReleaseReason::kExhausted) {
                guide_acceptance = active_guide_lifecycle_->accept(
                    candidate, reaches_mission_goal, grid, *host_distances, position);
                if (guide_acceptance.accepted) {
                  guide = active_guide_lifecycle_->guide();
                  guide_update = active_guide_lifecycle_->status();
                  active_guide_expansions = lattice_observation.expansions;
                  active_guide_cost = lattice_observation.cost;
                }
              } else {
                pending_global_guide_ = candidate;
                pending_global_guide_reaches_mission_goal_ = reaches_mission_goal;
              }
            }
          }
        }
      } else {
        if (pending_global_guide_) {
          guide_acceptance = active_guide_lifecycle_->accept(
              pending_global_guide_, pending_global_guide_reaches_mission_goal_, grid,
              *host_distances, position);
          pending_global_guide_.reset();
          pending_global_guide_reaches_mission_goal_ = false;
        }
        if (guide_acceptance.accepted) {
          guide = active_guide_lifecycle_->guide();
          guide_update = active_guide_lifecycle_->status();
        } else {
          guide_heading = active_guide_lifecycle_->selectPlanningHeading(
              navigation.state, Point2{mission_goal_.x, mission_goal_.y});
          lattice_observation = planRiskAwareMotionPrimitiveGuide(
              grid, *host_distances, position, guide_heading.heading_rad,
              Point2{mission_goal_.x, mission_goal_.y}, search_config,
              semantic_portal_primitives_, frontier_blacklist_, &search_session);
          lattice_search_performed = true;
          const auto candidate = std::make_shared<const std::vector<Point2>>(
              std::move(lattice_observation.guide));
          const bool reaches_mission_goal =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal &&
              lattice_observation.reached_mission_goal &&
              lattice_observation.exact_terminal_connector && !candidate->empty() &&
              distance(candidate->back(), Point2{mission_goal_.x, mission_goal_.y}) <=
                  1.0e-6;
          const bool executable =
              lattice_observation.status == LatticePlanStatus::kReachedPlanningGoal ||
              lattice_observation.status == LatticePlanStatus::kViableFrontier;
          if (executable) {
            guide_acceptance = active_guide_lifecycle_->accept(
                candidate, reaches_mission_goal, grid, *host_distances, position);
          }
          if (guide_acceptance.accepted) {
            guide = active_guide_lifecycle_->guide();
            guide_update = active_guide_lifecycle_->status();
            active_guide_expansions = lattice_observation.expansions;
            active_guide_cost = lattice_observation.cost;
          } else {
            active_guide_expansions = 0U;
            active_guide_cost = 0.0;
          }
        }
      }
    } else if (active_guide_lifecycle_) {
      guide = active_guide_lifecycle_->guide();
      guide_update = active_guide_lifecycle_->status();
      guide_update.retained = guide != nullptr;
    }
    const ActiveGlobalGuideUpdate active_status =
        active_guide_lifecycle_ ? active_guide_lifecycle_->status()
                                : ActiveGlobalGuideUpdate{};
    if (!guide) {
      semantic_route.reset();
      mppi_route.reset();
      semantic_route_source.reset();
      semantic_route_build = {};
    } else if (guide.get() != semantic_route_source.get() || !semantic_route ||
               semantic_route->generation != active_status.generation) {
      semantic_route_build = buildSemanticPortalRoute(
          guide, active_status.generation, known_passages_.value_or(KnownPassageMap{}),
          activePassageSpeedLimitMps(passage_speed_policy_), semantic_route_config_);
      semantic_route = semantic_route_build.route;
      mppi_route = semantic_route ? makeMppiRoute(*semantic_route) : nullptr;
      semantic_route_source = guide;
    }
    ProductionMppiPreparedEsdf prepared = *world;
    prepared.semantic_route = semantic_route;
    prepared.mppi_route = mppi_route;
    prepared.portal_events = semantic_route_build.portal_events_created;
    prepared.rejected_portal_route_misses = semantic_route_build.rejected_route_miss;
    prepared.rejected_portal_overlaps = semantic_route_build.rejected_overlap;
    prepared.global_guide_expansions = active_guide_expansions;
    prepared.global_guide_cost = active_guide_cost;
    prepared.global_guide_generation = active_status.generation;
    prepared.global_guide_reused = guide_update.retained;
    prepared.global_guide_reaches_mission_goal = active_status.reaches_mission_goal;
    prepared.global_guide_release_reason = guide_update.release_reason;
    prepared.global_guide_heading_source = guide_heading.source;
    prepared.global_guide_risk = active_status.current_risk;
    prepared.global_guide_acceptance_reason = guide_acceptance.reason;
    prepared.global_guide_projection = active_status.projection;
    prepared.lattice_search_performed = lattice_search_performed;
    prepared.lattice_executable = lattice_observation.valid;
    prepared.lattice_status = lattice_observation.status;
    prepared.lattice_termination = lattice_observation.termination;
    prepared.lattice_planning_goal_reached = lattice_observation.planning_goal_reached;
    prepared.lattice_achieved_progress_m = lattice_observation.achieved_progress_m;
    prepared.lattice_guide_length_m = lattice_observation.guide_length_m;
    prepared.lattice_remaining_goal_distance_m =
        lattice_observation.remaining_goal_distance_m;
    prepared.lattice_terminal_successor_count =
        lattice_observation.terminal_successor_count;
    prepared.lattice_risk_stage = lattice_observation.risk_stage;
    prepared.lattice_stale_queue_pops = lattice_observation.stale_queue_pops;
    prepared.lattice_open_peak = lattice_observation.open_peak;
    prepared.lattice_records_peak = lattice_observation.records_peak;
    prepared.lattice_two_step_reachable_states =
        lattice_observation.two_step_reachable_states;
    prepared.lattice_reachable_depth_m = lattice_observation.reachable_depth_m;
    prepared.lattice_frontier_candidates_considered =
        lattice_observation.frontier_candidates_considered;
    prepared.lattice_successor_diagnostics = lattice_observation.successor_diagnostics;
    prepared.lattice_continuation_attempt = continuation_attempt;
    prepared.lattice_search_session_resumed =
        lattice_observation.search_session_resumed;
    bool activated = false;
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      if (prepared_esdf_.has_value() && prepared_esdf_->revision == prepared.revision) {
        prepared_esdf_ = prepared;
        activated = true;
      }
    }
    RCLCPP_INFO(
        get_logger(),
        "PRODUCTION_MPPI_GUIDE revision=%" PRIu64
        " activated=%s continuation_attempt=%zu search_session_resumed=%s "
        "dropped_guide_worlds=%" PRIu64
        " guide_valid=%s guide_points=%zu guide_expansions=%zu guide_cost=%.2f "
        "portal_events=%zu portal_route_misses=%zu portal_overlaps=%zu "
        "semantic_side_volumes=%zu semantic_side_cells=%zu "
        "guide_generation=%" PRIu64
        " guide_reused=%s guide_reaches_mission_goal=%s guide_release=%s "
        "guide_heading_source=%s guide_risk=%s guide_acceptance=%s "
        "guide_station_m=%.2f "
        "guide_remaining_m=%.2f guide_cross_track_m=%.2f "
        "lattice_search_performed=%s lattice_executable=%s lattice_status=%s "
        "lattice_termination=%s lattice_planning_goal_reached=%s "
        "lattice_achieved_progress_m=%.2f lattice_guide_length_m=%.2f "
        "lattice_remaining_goal_distance_m=%.2f "
        "lattice_terminal_successors=%zu successor_generated=%zu "
        "successor_accepted=%zu successor_reject_roi=%zu "
        "successor_reject_grid=%zu successor_reject_invalid=%zu "
        "successor_reject_collision=%zu successor_reject_portal=%zu "
        "successor_reject_risk=%zu successor_reject_blacklist=%zu "
        "successor_reject_cost=%zu",
        prepared.revision, activated ? "true" : "false", continuation_attempt,
        prepared.lattice_search_session_resumed ? "true" : "false",
        dropped_guide_worlds_.load(std::memory_order_relaxed),
        guide && guide->size() >= 2U ? "true" : "false", guide ? guide->size() : 0U,
        prepared.global_guide_expansions, prepared.global_guide_cost,
        prepared.portal_events, prepared.rejected_portal_route_misses,
        prepared.rejected_portal_overlaps, prepared.semantic_side_volumes,
        prepared.semantic_side_cells, prepared.global_guide_generation,
        prepared.global_guide_reused ? "true" : "false",
        prepared.global_guide_reaches_mission_goal ? "true" : "false",
        globalGuideReleaseReasonName(prepared.global_guide_release_reason),
        globalGuideHeadingSourceName(prepared.global_guide_heading_source),
        globalGuideRiskTierName(prepared.global_guide_risk),
        globalGuideAcceptanceReasonName(prepared.global_guide_acceptance_reason),
        prepared.global_guide_projection.station_m,
        prepared.global_guide_projection.remaining_m,
        prepared.global_guide_projection.cross_track_m,
        prepared.lattice_search_performed ? "true" : "false",
        prepared.lattice_executable ? "true" : "false",
        latticePlanStatusName(prepared.lattice_status),
        latticeSearchTerminationName(prepared.lattice_termination),
        prepared.lattice_planning_goal_reached ? "true" : "false",
        prepared.lattice_achieved_progress_m, prepared.lattice_guide_length_m,
        prepared.lattice_remaining_goal_distance_m,
        prepared.lattice_terminal_successor_count,
        prepared.lattice_successor_diagnostics.generated,
        prepared.lattice_successor_diagnostics.accepted,
        prepared.lattice_successor_diagnostics.rejected_outside_roi,
        prepared.lattice_successor_diagnostics.rejected_outside_grid,
        prepared.lattice_successor_diagnostics.rejected_invalid_clearance,
        prepared.lattice_successor_diagnostics.rejected_raw_collision,
        prepared.lattice_successor_diagnostics.rejected_portal_footprint,
        prepared.lattice_successor_diagnostics.rejected_risk_stage,
        prepared.lattice_successor_diagnostics.rejected_blacklisted_failure,
        prepared.lattice_successor_diagnostics.rejected_no_cost_improvement);

    const bool search_incomplete =
        lattice_search_performed &&
        lattice_observation.status == LatticePlanStatus::kSearchIncomplete;
    bool newer_world_pending = false;
    {
      const std::scoped_lock lock{guide_queue_mutex_};
      newer_world_pending = pending_guide_world_ != nullptr;
    }
    if (search_incomplete && !newer_world_pending &&
        continuation_attempt + 1U < lattice_maximum_continuation_attempts_) {
      ++continuation_attempt;
      continue;
    }
    world.reset();
    search_navigation.reset();
    continuation_attempt = 0U;
  }
}

mppi::State ProductionMppiNode::selectTarget(const ProductionMppiNavigation& navigation,
                                             const ProductionMppiPreparedEsdf& esdf,
                                             const double lookahead_m,
                                             std::string& target_source,
                                             double& target_station_m) const {
  mppi::State target{static_cast<float>(mission_goal_.x),
                     static_cast<float>(mission_goal_.y),
                     static_cast<float>(mission_goal_.z)};
  target_source = "mission_goal_direct";
  target_station_m = 0.0;
  if (!esdf.semantic_route || !esdf.semantic_route->polyline ||
      esdf.semantic_route->polyline->empty()) {
    return target;
  }
  const GlobalGuideProjection projection = projectOntoGlobalGuide(
      *esdf.semantic_route->polyline, Point2{navigation.state.x, navigation.state.y},
      esdf.global_guide_projection.station_m);
  if (!projection.valid) {
    return target;
  }
  target_station_m = std::min(esdf.semantic_route->total_length_m,
                              projection.station_m + std::max(0.0, lookahead_m));
  const Point2 selected =
      sampleGlobalGuide(*esdf.semantic_route->polyline, target_station_m);
  target.x = static_cast<float>(selected.x);
  target.y = static_cast<float>(selected.y);
  target_source = "motion_primitive_guide";
  return target;
}

void ProductionMppiNode::planningTick() {
  if (!engine_) {
    return;
  }
  const auto snapshot_started = std::chrono::steady_clock::now();
  ProductionMppiNavigation navigation;
  ProductionMppiPredictionError prediction;
  ProductionMppiAppliedControl applied_control;
  std::uint64_t memory_sequence{0U};
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    prediction = latest_prediction_error_;
    applied_control = applied_control_;
    memory_sequence = memory_sequence_;
  }
  std::optional<ProductionMppiPreparedEsdf> esdf;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    esdf = prepared_esdf_;
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const double pose_age_ms =
      static_cast<double>(now_ns - navigation.receive_stamp_ns) / 1.0e6;
  const double esdf_age_ms =
      esdf.has_value() ? static_cast<double>(now_ns - esdf->ready_stamp_ns) / 1.0e6
                       : std::numeric_limits<double>::infinity();
  const double control_feedback_age_ms =
      applied_control.valid
          ? static_cast<double>(now_ns - applied_control.receive_stamp_ns) / 1.0e6
          : std::numeric_limits<double>::infinity();
  if (!navigation.valid || pose_age_ms < 0.0) {
    return;
  }
  bool pose_predicted = false;
  if (pose_age_ms > maximum_pose_age_ms_) {
    const NavigationStatePredictionResult predicted =
        predictNavigationState(navigation.state, pose_age_ms / 1000.0,
                               maximum_pose_prediction_age_ms_ / 1000.0);
    if (!predicted.valid) {
      return;
    }
    navigation.state = predicted.state;
    pose_predicted = predicted.predicted;
  }
  if (!esdf.has_value() || esdf_age_ms < 0.0 ||
      esdf_age_ms > maximum_esdf_age_ms_ + stale_esdf_execution_window_ms_) {
    const ProductionMppiPreparedEsdf stale_esdf =
        esdf.value_or(ProductionMppiPreparedEsdf{});
    mppi::MppiTickInput input{
        .initial_state = navigation.state,
        .target = navigation.state,
        .passage = std::nullopt,
        .pose_revision = navigation.revision,
        .obstacle_revision = stale_esdf.revision,
        .planning_stamp_ns = now_ns,
        .previous_applied_control = std::nullopt,
        .nominal_reseed_generation = 0U,
        .reference_speed_mps = 0.0F,
        .route = std::nullopt,
    };
    const MppiHorizonSafetyResult fallback =
        buildMppiBrakingFallback(input.initial_state, safety_config_);
    mppi::MppiTickResult result;
    result.horizon = fallback.fallback_horizon;
    result.controls = fallback.fallback_controls;
    result.selected_tier = mppi::RiskTier::kPreferred;
    result.raw_collision = false;
    result.known_solid_collision = false;
    result.esdf_revision = stale_esdf.revision;
    result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  snapshot_started)
            .count();
    const PassageCoordinatorResult passage_result;
    const ProductionMppiPlanningState planning_state =
        ProductionMppiPlanningState::kUnavailableWorldBrakingHold;
    ++tick_sequence_;
    recordTickStatistics(result, passage_result, planning_state, false);
    ProductionMppiExecutionPublication execution = publishExecutionHorizon(
        input, result, stale_esdf, passage_result, planning_state, now_ns);
    std::optional<ProductionMppiRvizSnapshot> rviz;
    if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
      rviz = ProductionMppiRvizSnapshot{
          .candidate_horizon = result.horizon,
          .previous_horizon = previous_result_.has_value() ? previous_result_->horizon
                                                           : std::vector<mppi::State>{},
          .execution_horizon = execution.horizon,
          .semantic_route = stale_esdf.semantic_route,
      };
      last_rviz_stamp_ns_ = now_ns;
    }
    ProductionMppiPreparedEsdf diagnostic_esdf = stale_esdf;
    diagnostic_esdf.distances_m.reset();
    diagnostic_esdf.semantic_route.reset();
    diagnostic_esdf.mppi_route.reset();
    enqueueDiagnostics(ProductionMppiDiagnosticsSnapshot{
        .input = input,
        .result = result,
        .esdf = std::move(diagnostic_esdf),
        .stability = {},
        .prediction = prediction,
        .liveness = {},
        .speed_policy = {},
        .passage_coordinator = {},
        .guide_progress = {},
        .goal_capture = {},
        .execution = std::move(execution),
        .planning_state = planning_state,
        .rviz = std::move(rviz),
        .target_source = "unavailable_world_braking",
        .tick_sequence = tick_sequence_,
        .memory_sequence = memory_sequence,
        .pose_age_ms = pose_age_ms,
        .esdf_age_ms = esdf_age_ms,
        .control_feedback_age_ms = control_feedback_age_ms,
        .snapshot_ms = result.timings.host_total_ms,
        .stability_ms = 0.0,
        .liveness_reseed_requested = false,
        .pose_predicted = pose_predicted,
        .maximum_eligible_risk_tier = maximum_eligible_risk_tier_,
    });
    previous_result_ = std::move(result);
    return;
  }
  if (!engine_->ready()) {
    return;
  }
  const std::span<const Point2> guide =
      esdf->semantic_route && esdf->semantic_route->polyline
          ? std::span<const Point2>{*esdf->semantic_route->polyline}
          : std::span<const Point2>{};
  const MissionGoalCaptureResult goal_capture =
      mission_goal_capture_latch_
          ? mission_goal_capture_latch_->update(MissionGoalCaptureObservation{
                .mission_goal = mission_goal_,
                .state = navigation.state,
                .terminal_route_available = esdf->global_guide_reaches_mission_goal,
            })
          : MissionGoalCaptureResult{};
  MppiSpeedPolicyResult speed_policy = evaluateMppiSpeedPolicy(
      speed_policy_config_, MppiSpeedPolicyInput{
                                .state = navigation.state,
                                .mission_goal = mission_goal_,
                                .guide = guide,
                                .passage_speed_limit_mps = std::nullopt,
                            });
  std::string target_source;
  double target_station_m = 0.0;
  mppi::State target = selectTarget(navigation, *esdf, speed_policy.target_lookahead_m,
                                    target_source, target_station_m);
  PassageCoordinatorResult passage_result;
  const GlobalGuideProjection route_projection =
      guide.empty()
          ? GlobalGuideProjection{}
          : projectOntoGlobalGuide(guide,
                                   Point2{navigation.state.x, navigation.state.y},
                                   esdf->global_guide_projection.station_m);
  if (passage_coordinator_) {
    passage_result = passage_coordinator_->update(PassageCoordinatorInput{
        .state = navigation.state,
        .route = esdf->semantic_route,
        .route_station_m = route_projection.valid ? route_projection.station_m : 0.0,
        .normal_flight_z_m = mission_goal_.z,
        .approach_speed_mps = speed_policy.reference_speed_mps,
    });
  }
  std::optional<mppi::PassageConstraint> passage = passage_result.constraint;
  if (passage.has_value()) {
    speed_policy = evaluateMppiSpeedPolicy(
        speed_policy_config_,
        MppiSpeedPolicyInput{
            .state = navigation.state,
            .mission_goal = mission_goal_,
            .guide = guide,
            .passage_speed_limit_mps =
                passage_result.speed_limit_active
                    ? std::optional<double>{passage->speed_limit_mps}
                    : std::nullopt,
        });
    target = selectTarget(navigation, *esdf, speed_policy.target_lookahead_m,
                          target_source, target_station_m);
  }
  if (esdf->semantic_route) {
    if (passage_result.active && passage_result.route_event_index <
                                     esdf->semantic_route->passage_events.size()) {
      target.z = static_cast<float>(routePassageZReference(
          esdf->semantic_route->passage_events[passage_result.route_event_index],
          target_station_m, mission_goal_.z,
          passage_result.effective_approach_station_m,
          passage_result.alignment_completion_station_m));
    } else {
      target.z = static_cast<float>(semanticRouteZReference(
          *esdf->semantic_route, target_station_m, mission_goal_.z));
    }
  }
  ProductionMppiPlanningState planning_state = ProductionMppiPlanningState::kPlanned;
  if (goal_capture.latched) {
    planning_state = ProductionMppiPlanningState::kMissionGoalPositionHold;
    target = mppi::State{
        .x = static_cast<float>(mission_goal_.x),
        .y = static_cast<float>(mission_goal_.y),
        .z = static_cast<float>(mission_goal_.z),
        .yaw = navigation.state.yaw,
    };
    if (passage_coordinator_) {
      passage_coordinator_->reset();
    }
    passage_result = {};
    passage.reset();
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "mission_goal_position_hold";
  } else if (target_source == "mission_goal_direct") {
    planning_state = ProductionMppiPlanningState::kNoGuideBrakingHold;
    target = navigation.state;
    if (passage_coordinator_) {
      passage_coordinator_->reset();
    }
    passage_result = {};
    passage.reset();
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "no_guide_braking_hold";
  } else if (passage.has_value()) {
    if (passage_result.hold_xy) {
      target.x = static_cast<float>(passage_result.hold_position.x);
      target.y = static_cast<float>(passage_result.hold_position.y);
      target.z = static_cast<float>(passage_result.preferred_z_m);
      speed_policy.reference_speed_mps = 0.0;
      speed_policy.target_lookahead_m = 0.0;
      target_source = "passage_vertical_alignment";
    } else {
      target_source = "semantic_portal_route";
    }
  }
  const bool control_feedback_fresh =
      applied_control.valid && control_feedback_age_ms >= 0.0 &&
      control_feedback_age_ms <= maximum_control_feedback_age_ms_;
  MppiLivenessResult liveness;
  if (liveness_supervisor_) {
    liveness = liveness_supervisor_->evaluate(MppiLivenessObservation{
        .stamp_ns = now_ns,
        .actual_state = navigation.state,
        .controller_active = control_feedback_fresh && !passage_result.hold_xy &&
                             planning_state == ProductionMppiPlanningState::kPlanned,
        .emergency_braking =
            control_feedback_fresh && applied_control.emergency_braking,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .predicted_terminal_progress_m =
            previous_result_.has_value() ? previous_result_->terminal_progress_m : 0.0,
    });
  }
  GlobalGuideProgressUpdate guide_progress;
  if (guide_progress_tracker_) {
    const GlobalGuideProjection projection =
        guide.empty()
            ? GlobalGuideProjection{}
            : projectOntoGlobalGuide(guide,
                                     Point2{navigation.state.x, navigation.state.y},
                                     esdf->global_guide_projection.station_m);
    guide_progress = guide_progress_tracker_->evaluate(GlobalGuideProgressObservation{
        .stamp_ns = now_ns,
        .guide_generation =
            projection.valid && planning_state == ProductionMppiPlanningState::kPlanned
                ? esdf->global_guide_generation
                : 0U,
        .station_m = projection.station_m,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .controller_active = control_feedback_fresh && !passage_result.hold_xy,
        .emergency_braking =
            control_feedback_fresh && applied_control.emergency_braking,
    });
    if (guide_progress.stalled) {
      requestGuideRelease(guide_progress.persistent_safety_rejection
                              ? GlobalGuideReleaseReason::kPersistentSafetyRejection
                              : GlobalGuideReleaseReason::kStalled);
    }
  }
  const MppiNominalReseedUpdate nominal_reseed =
      nominal_reseed_tracker_.update(MppiNominalReseedObservation{
          .guide_generation = esdf->global_guide_generation,
          .local_liveness_generation = liveness.reseed_generation,
          .guide_liveness_generation = guide_progress.local_reseed_generation,
          .safety_rejection_generation = guide_progress.persistent_safety_rejection
                                             ? guide_progress.stall_generation
                                             : 0U,
      });
  if (risk_escalation_) {
    const bool stable_progress = previous_result_.has_value() &&
                                 previous_result_->eligible_risk_contract.available &&
                                 previous_result_->head_progress_m >=
                                     liveness_config_.minimum_actual_displacement_m &&
                                 std::hypot(navigation.state.vx, navigation.state.vy) >=
                                     liveness_config_.minimum_actual_displacement_m;
    maximum_eligible_risk_tier_ =
        risk_escalation_
            ->update(MppiRiskEscalationObservation{
                .reseed_generation = liveness.reseed_generation,
                .no_eligible_recovery_generation =
                    nominal_reseed.no_eligible_recovery_generation,
                .stable_progress = stable_progress,
            })
            .maximum_eligible_tier;
  }
  mppi::MppiTickInput input{
      .initial_state = navigation.state,
      .target = target,
      .passage = passage,
      .pose_revision = navigation.revision,
      .obstacle_revision = esdf->revision,
      .planning_stamp_ns = now_ns,
      .previous_applied_control =
          control_feedback_fresh ? std::optional<mppi::Control>{applied_control.control}
                                 : std::nullopt,
      .nominal_reseed_generation = nominal_reseed.generation,
      .reference_speed_mps = speed_policy.enabled
                                 ? static_cast<float>(speed_policy.reference_speed_mps)
                                 : -1.0F,
      .maximum_eligible_risk_tier = maximum_eligible_risk_tier_,
      .route =
          esdf->mppi_route && route_projection.valid
              ? std::optional<mppi::RouteReference>{mppi::RouteReference{
                    .points = esdf->mppi_route,
                    .generation = esdf->global_guide_generation,
                    .initial_station_m = static_cast<float>(route_projection.station_m),
                }}
              : std::nullopt,
  };
  const double snapshot_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - snapshot_started)
                                 .count();
  mppi::MppiTickResult result;
  MppiEligibleRolloutUpdate no_eligible_recovery{
      .no_eligible_recovery_generation = nominal_reseed.no_eligible_recovery_generation,
      .phase = nominal_reseed.no_eligible_phase,
  };
  if (planning_state == ProductionMppiPlanningState::kNoGuideBrakingHold) {
    const MppiHorizonSafetyResult fallback =
        buildMppiBrakingFallback(input.initial_state, safety_config_);
    result.horizon = fallback.fallback_horizon;
    result.controls = fallback.fallback_controls;
    result.selected_tier = mppi::RiskTier::kPreferred;
    result.raw_collision = false;
    result.esdf_revision = esdf->revision;
    result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  snapshot_started)
            .count();
  } else if (planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold) {
    result.horizon = {target, target};
    result.controls = {mppi::Control{}};
    result.selected_tier = mppi::RiskTier::kPreferred;
    result.raw_collision = false;
    result.known_solid_collision = false;
    result.esdf_revision = esdf->revision;
    result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  snapshot_started)
            .count();
  } else {
    try {
      result = engine_->plan(input);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "PRODUCTION_MPPI_TICK failed: %s", error.what());
      return;
    }
    no_eligible_recovery = nominal_reseed_tracker_.observeEligibleRolloutResult(
        result.eligible_risk_contract.available, result.nominal_reseeded);
    if (no_eligible_recovery.guide_replan_requested) {
      requestGuideRelease(GlobalGuideReleaseReason::kNoEligibleRollouts);
    }
  }
  ++tick_sequence_;
  recordTickStatistics(result, passage_result, planning_state,
                       liveness.reseed_requested ||
                           guide_progress.local_reseed_requested);
  ProductionMppiExecutionPublication execution = publishExecutionHorizon(
      input, result, *esdf, passage_result, planning_state, now_ns);

  const auto stability_started = std::chrono::steady_clock::now();
  const ProductionMppiStability stability = compareWithPrevious(result);
  const double stability_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - stability_started)
                                  .count();
  std::optional<ProductionMppiRvizSnapshot> rviz;
  if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
    rviz = ProductionMppiRvizSnapshot{
        .candidate_horizon = result.horizon,
        .previous_horizon = previous_result_.has_value() ? previous_result_->horizon
                                                         : std::vector<mppi::State>{},
        .execution_horizon = execution.horizon,
        .semantic_route = esdf->semantic_route,
    };
    last_rviz_stamp_ns_ = now_ns;
  }

  mppi::MppiTickResult diagnostic_result;
  diagnostic_result.eligible_risk_contract = result.eligible_risk_contract;
  diagnostic_result.post_update_classification = result.post_update_classification;
  diagnostic_result.post_update_repair = result.post_update_repair;
  diagnostic_result.post_update_backtrack_ratio = result.post_update_backtrack_ratio;
  diagnostic_result.selected_tier = result.selected_tier;
  diagnostic_result.raw_collision = result.raw_collision;
  diagnostic_result.known_solid_collision = result.known_solid_collision;
  diagnostic_result.critical_exposure_m = result.critical_exposure_m;
  diagnostic_result.planning_exposure_m = result.planning_exposure_m;
  diagnostic_result.minimum_esdf_distance_m = result.minimum_esdf_distance_m;
  diagnostic_result.head_progress_m = result.head_progress_m;
  diagnostic_result.terminal_progress_m = result.terminal_progress_m;
  diagnostic_result.maximum_acceleration_mps2 = result.maximum_acceleration_mps2;
  diagnostic_result.maximum_jerk_mps3 = result.maximum_jerk_mps3;
  diagnostic_result.first_control_delta = result.first_control_delta;
  diagnostic_result.warm_start_shift_s = result.warm_start_shift_s;
  diagnostic_result.nominal_reseeded = result.nominal_reseeded;
  diagnostic_result.esdf_revision = result.esdf_revision;
  diagnostic_result.timings = result.timings;
  ProductionMppiPreparedEsdf diagnostic_esdf = *esdf;
  diagnostic_esdf.distances_m.reset();
  if (!rviz.has_value()) {
    diagnostic_esdf.semantic_route.reset();
    diagnostic_esdf.mppi_route.reset();
  }
  enqueueDiagnostics(ProductionMppiDiagnosticsSnapshot{
      .input = input,
      .result = std::move(diagnostic_result),
      .esdf = std::move(diagnostic_esdf),
      .stability = stability,
      .prediction = prediction,
      .liveness = liveness,
      .speed_policy = speed_policy,
      .passage_coordinator = passage_result,
      .guide_progress = guide_progress,
      .no_eligible_recovery = no_eligible_recovery,
      .goal_capture = goal_capture,
      .execution = std::move(execution),
      .planning_state = planning_state,
      .rviz = std::move(rviz),
      .target_source = target_source,
      .tick_sequence = tick_sequence_,
      .memory_sequence = memory_sequence,
      .pose_age_ms = pose_age_ms,
      .esdf_age_ms = esdf_age_ms,
      .control_feedback_age_ms = control_feedback_age_ms,
      .snapshot_ms = snapshot_ms,
      .stability_ms = stability_ms,
      .liveness_reseed_requested = liveness.reseed_requested,
      .pose_predicted = pose_predicted,
      .maximum_eligible_risk_tier = maximum_eligible_risk_tier_,
  });
  {
    const std::scoped_lock lock{input_mutex_};
    if (result.horizon.size() > 1U) {
      previous_predicted_next_state_ = result.horizon[1U];
      previous_prediction_stamp_ns_ = now_ns;
    }
  }
  previous_result_ = result;
}

void ProductionMppiNode::requestGuideRelease(
    const GlobalGuideReleaseReason reason) noexcept {
  guide_release_reason_.store(reason, std::memory_order_relaxed);
  guide_release_generation_.fetch_add(1U, std::memory_order_release);
}

ProductionMppiStability
ProductionMppiNode::compareWithPrevious(const mppi::MppiTickResult& result) const {
  ProductionMppiStability stability;
  if (!previous_result_.has_value() || result.controls.empty() ||
      previous_result_->controls.size() < 2U || result.horizon.empty() ||
      previous_result_->horizon.size() < 2U) {
    return stability;
  }
  const mppi::Control& current = result.controls.front();
  const double offset_steps =
      result.warm_start_shift_s / static_cast<double>(mppi_config_.dynamics.dt_s);
  const std::vector<mppi::Control> shifted_controls =
      mppi::shiftControlSequence(previous_result_->controls, mppi_config_.dynamics.dt_s,
                                 result.warm_start_shift_s);
  const mppi::Control& previous = shifted_controls.front();
  stability.first_control_delta =
      std::hypot(std::hypot(current.ax - previous.ax, current.ay - previous.ay),
                 current.az - previous.az);
  const std::size_t count = result.horizon.size();
  double squared_sum = 0.0;
  for (std::size_t index = 0U; index < count; ++index) {
    const mppi::State previous_state = sampleState(
        previous_result_->horizon, offset_steps + static_cast<double>(index));
    const double difference = distance3(result.horizon[index], previous_state);
    squared_sum += difference * difference;
    stability.position_max_m = std::max(stability.position_max_m, difference);
  }
  stability.position_rms_m = std::sqrt(squared_sum / static_cast<double>(count));
  stability.terminal_shift_m =
      distance3(result.horizon.back(), previous_result_->horizon.back());
  stability.valid = true;
  return stability;
}

} // namespace drone_city_nav
