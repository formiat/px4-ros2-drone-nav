#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
#include "drone_city_nav/navigation_state_prediction.hpp"
#include "drone_city_nav/ros_conversions.hpp"

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

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute3D(std::span<const RouteSample3D> route,
                std::span<const ConstrainedRouteSpan> spans,
                double unconstrained_speed_mps, double constrained_speed_mps);

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute2D(const std::span<const Point2> route, const double z_m,
                const double reference_speed_mps) {
  std::vector<Point3> points;
  points.reserve(route.size());
  for (const Point2 point : route) {
    points.push_back(Point3{point.x, point.y, z_m});
  }
  return makeMppiRoute3D(sampleRoute3D(points, 0.5, reference_speed_mps), {},
                         reference_speed_mps, reference_speed_mps);
}

[[nodiscard]] std::shared_ptr<const std::vector<mppi::RouteSample3D>>
makeMppiRoute3D(const std::span<const RouteSample3D> route,
                const std::span<const ConstrainedRouteSpan> spans,
                const double unconstrained_speed_mps,
                const double constrained_speed_mps) {
  auto points = std::make_shared<std::vector<mppi::RouteSample3D>>();
  points->reserve(route.size());
  for (const RouteSample3D& sample : route) {
    const bool constrained =
        std::ranges::any_of(spans, [&sample](const ConstrainedRouteSpan& span) {
          return sample.station_m >= span.begin_station_m &&
                 sample.station_m <= span.end_station_m;
        });
    points->push_back(mppi::RouteSample3D{
        .x_m = static_cast<float>(sample.position.x),
        .y_m = static_cast<float>(sample.position.y),
        .z_m = static_cast<float>(sample.position.z),
        .tangent_x = static_cast<float>(sample.tangent.x),
        .tangent_y = static_cast<float>(sample.tangent.y),
        .tangent_z = static_cast<float>(sample.tangent.z),
        .station_m = static_cast<float>(sample.station_m),
        .reference_speed_mps = static_cast<float>(
            constrained ? constrained_speed_mps : unconstrained_speed_mps),
    });
  }
  return points;
}

[[nodiscard]] std::shared_ptr<const std::vector<Point2>>
projectRouteTo2D(const std::span<const RouteSample3D> route) {
  auto points = std::make_shared<std::vector<Point2>>();
  points->reserve(route.size());
  for (const RouteSample3D& sample : route) {
    points->push_back(Point2{sample.position.x, sample.position.y});
  }
  return points;
}

} // namespace

void ProductionMppiNode::guideWorker(const std::stop_token stop_token) {
  std::size_t active_guide_expansions = 0U;
  double active_guide_cost = 0.0;
  std::shared_ptr<const std::vector<mppi::RouteSample3D>> mppi_route;
  std::shared_ptr<const std::vector<Point2>> route_source;
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
    if (use_static_map_ && world->grid.depth > 1) {
      ProductionMppiNavigation navigation;
      {
        const std::scoped_lock lock{input_mutex_};
        navigation = navigation_;
      }
      if (!navigation.valid) {
        world.reset();
        continue;
      }
      Vec3 preferred_direction{static_cast<double>(navigation.state.vx),
                               static_cast<double>(navigation.state.vy),
                               static_cast<double>(navigation.state.vz)};
      if (std::hypot(preferred_direction.x, preferred_direction.y) < 0.5) {
        preferred_direction = Vec3{mission_goal_.x - navigation.state.x,
                                   mission_goal_.y - navigation.state.y,
                                   mission_goal_.z - navigation.state.z};
      }
      const RiskAwareLattice3DResult lattice = planRiskAwareLattice3D(
          world->grid, *world->distances_m,
          Point3{navigation.state.x, navigation.state.y, navigation.state.z},
          preferred_direction, mission_goal_, lattice_3d_config_);
      ProductionMppiPreparedEsdf prepared = *world;
      prepared.lattice_search_performed = true;
      prepared.lattice_executable =
          lattice.status == Lattice3DStatus::kReachedPlanningGoal ||
          lattice.status == Lattice3DStatus::kViableFrontier;
      prepared.global_guide_expansions = lattice.expansions;
      prepared.global_guide_generation = ++static_route_generation_;
      prepared.global_guide_reaches_mission_goal = lattice.reached_mission_goal;
      if (prepared.lattice_executable) {
        auto route = std::make_shared<const std::vector<RouteSample3D>>(lattice.route);
        auto spans = std::make_shared<const std::vector<ConstrainedRouteSpan>>(
            analyzeConstrainedRouteSpans(*route, world->grid, *world->distances_m,
                                         prepared.global_guide_generation,
                                         route_envelope_config_));
        prepared.route_3d = route;
        prepared.route_2d_projection = projectRouteTo2D(*route);
        prepared.constrained_spans = spans;
        prepared.mppi_route =
            makeMppiRoute3D(*route, *spans, speed_policy_config_.cruise_speed_mps,
                            constrained_route_speed_limit_mps_);
        prepared.global_guide_projection =
            projectOntoGlobalGuide(*prepared.route_2d_projection,
                                   Point2{navigation.state.x, navigation.state.y});
      }
      bool activated = false;
      {
        const std::scoped_lock lock{esdf_state_mutex_};
        if (prepared_esdf_ && prepared_esdf_->revision == prepared.revision) {
          prepared_esdf_ = prepared;
          activated = true;
        }
      }
      RCLCPP_INFO(get_logger(),
                  "PRODUCTION_MPPI_GUIDE3D revision=%" PRIu64
                  " activated=%s status=%s points=%zu samples=%zu spans=%zu "
                  "expansions=%zu risk_stage=%u",
                  prepared.revision, activated ? "true" : "false",
                  lattice3DStatusName(lattice.status), lattice.points.size(),
                  lattice.route.size(),
                  prepared.constrained_spans ? prepared.constrained_spans->size() : 0U,
                  lattice.expansions, static_cast<unsigned>(lattice.risk_stage));
      world.reset();
      search_navigation.reset();
      continuation_attempt = 0U;
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
              frontier_blacklist_, &search_session);
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
              frontier_blacklist_, &search_session);
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
      mppi_route.reset();
      route_source.reset();
    } else if (guide.get() != route_source.get()) {
      mppi_route = makeMppiRoute2D(*guide, mission_goal_.z,
                                   speed_policy_config_.cruise_speed_mps);
      route_source = guide;
    }
    ProductionMppiPreparedEsdf prepared = *world;
    prepared.mppi_route = mppi_route;
    prepared.route_2d_projection = guide;
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
        "successor_reject_collision=%zu "
        "successor_reject_risk=%zu successor_reject_blacklist=%zu "
        "successor_reject_cost=%zu",
        prepared.revision, activated ? "true" : "false", continuation_attempt,
        prepared.lattice_search_session_resumed ? "true" : "false",
        dropped_guide_worlds_.load(std::memory_order_relaxed),
        guide && guide->size() >= 2U ? "true" : "false", guide ? guide->size() : 0U,
        prepared.global_guide_expansions, prepared.global_guide_cost,
        prepared.global_guide_generation,
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

mppi::State ProductionMppiNode::selectTarget(const ProductionMppiPreparedEsdf& esdf,
                                             const double current_station_m,
                                             const double lookahead_m,
                                             std::string& target_source,
                                             double& target_station_m) const {
  mppi::State target{static_cast<float>(mission_goal_.x),
                     static_cast<float>(mission_goal_.y),
                     static_cast<float>(mission_goal_.z)};
  target_source = "mission_goal_direct";
  target_station_m = 0.0;
  if (esdf.mppi_route && !esdf.mppi_route->empty()) {
    const float desired_station =
        static_cast<float>(current_station_m + std::max(0.0, lookahead_m));
    const auto selected = std::ranges::lower_bound(*esdf.mppi_route, desired_station,
                                                   {}, &mppi::RouteSample3D::station_m);
    const mppi::RouteSample3D& sample =
        selected == esdf.mppi_route->end() ? esdf.mppi_route->back() : *selected;
    target.x = sample.x_m;
    target.y = sample.y_m;
    target.z = sample.z_m;
    target_station_m = sample.station_m;
    target_source = "global_route_3d";
    return target;
  }
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
    const ProductionMppiPlanningState planning_state =
        ProductionMppiPlanningState::kUnavailableWorldBrakingHold;
    ++tick_sequence_;
    recordTickStatistics(result, planning_state, false);
    ProductionMppiExecutionPublication execution =
        publishExecutionHorizon(input, result, stale_esdf, planning_state, now_ns);
    std::optional<ProductionMppiRvizSnapshot> rviz;
    if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
      rviz = ProductionMppiRvizSnapshot{
          .candidate_horizon = result.horizon,
          .previous_horizon = previous_result_.has_value() ? previous_result_->horizon
                                                           : std::vector<mppi::State>{},
          .execution_horizon = execution.horizon,
          .route = stale_esdf.mppi_route,
      };
      last_rviz_stamp_ns_ = now_ns;
    }
    ProductionMppiPreparedEsdf diagnostic_esdf = stale_esdf;
    diagnostic_esdf.distances_m.reset();
    diagnostic_esdf.mppi_route.reset();
    enqueueDiagnostics(ProductionMppiDiagnosticsSnapshot{
        .input = input,
        .result = result,
        .esdf = std::move(diagnostic_esdf),
        .stability = {},
        .prediction = prediction,
        .liveness = {},
        .speed_policy = {},
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
      esdf->route_2d_projection ? std::span<const Point2>{*esdf->route_2d_projection}
                                : std::span<const Point2>{};
  if (tracked_route_generation_ != esdf->global_guide_generation) {
    tracked_route_generation_ = esdf->global_guide_generation;
    tracked_route_station_m_ = 0.0;
  }
  const GlobalGuideProjection measured_route_projection =
      guide.empty()
          ? GlobalGuideProjection{}
          : projectOntoGlobalGuide(guide,
                                   Point2{navigation.state.x, navigation.state.y},
                                   tracked_route_station_m_);
  if (measured_route_projection.valid) {
    tracked_route_station_m_ =
        std::max(tracked_route_station_m_, measured_route_projection.station_m);
  }
  GlobalGuideProjection route_projection = measured_route_projection;
  if (route_projection.valid) {
    route_projection.station_m = tracked_route_station_m_;
    route_projection.remaining_m =
        std::max(0.0, route_projection.station_m + route_projection.remaining_m -
                          tracked_route_station_m_);
  }
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
                                .route_constraint_speed_limit_mps = std::nullopt,
                            });
  std::string target_source;
  double target_station_m = 0.0;
  mppi::State target =
      selectTarget(*esdf, tracked_route_station_m_, speed_policy.target_lookahead_m,
                   target_source, target_station_m);
  ProductionMppiPlanningState planning_state = ProductionMppiPlanningState::kPlanned;
  if (goal_capture.latched) {
    planning_state = ProductionMppiPlanningState::kMissionGoalPositionHold;
    target = mppi::State{
        .x = static_cast<float>(mission_goal_.x),
        .y = static_cast<float>(mission_goal_.y),
        .z = static_cast<float>(mission_goal_.z),
        .yaw = navigation.state.yaw,
    };
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "mission_goal_position_hold";
  } else if (target_source == "mission_goal_direct") {
    planning_state = ProductionMppiPlanningState::kNoGuideBrakingHold;
    target = navigation.state;
    speed_policy.reference_speed_mps = 0.0;
    speed_policy.target_lookahead_m = 0.0;
    target_source = "no_guide_braking_hold";
  }
  const bool control_feedback_fresh =
      applied_control.valid && control_feedback_age_ms >= 0.0 &&
      control_feedback_age_ms <= maximum_control_feedback_age_ms_;
  MppiLivenessResult liveness;
  if (liveness_supervisor_) {
    liveness = liveness_supervisor_->evaluate(MppiLivenessObservation{
        .stamp_ns = now_ns,
        .actual_state = navigation.state,
        .controller_active = control_feedback_fresh &&
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
    const GlobalGuideProjection& projection = route_projection;
    guide_progress = guide_progress_tracker_->evaluate(GlobalGuideProgressObservation{
        .stamp_ns = now_ns,
        .guide_generation =
            projection.valid && planning_state == ProductionMppiPlanningState::kPlanned
                ? esdf->global_guide_generation
                : 0U,
        .station_m = projection.station_m,
        .predicted_head_progress_m =
            previous_result_.has_value() ? previous_result_->head_progress_m : 0.0,
        .controller_active = control_feedback_fresh,
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
  recordTickStatistics(result, planning_state,
                       liveness.reseed_requested ||
                           guide_progress.local_reseed_requested);
  ProductionMppiExecutionPublication execution =
      publishExecutionHorizon(input, result, *esdf, planning_state, now_ns);

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
        .route = esdf->mppi_route,
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
      .route_station_m = route_projection.station_m,
      .route_remaining_m = route_projection.remaining_m,
      .snapshot_ms = snapshot_ms,
      .stability_ms = stability_ms,
      .route_projection_valid = route_projection.valid,
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
