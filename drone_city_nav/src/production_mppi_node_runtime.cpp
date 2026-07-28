#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/mppi/mppi_control_sequence.hpp"
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

} // namespace

void ProductionMppiNode::esdfWorker(const std::stop_token stop_token) {
  std::size_t active_guide_expansions = 0U;
  double active_guide_cost = 0.0;
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
    const RawOccupancyGridFromRosResult conversion =
        rawOccupancyGridFromRos(snapshot->grid, RawOccupancyGridFromRosConfig{100, 0});
    if (!conversion.grid.has_value()) {
      RCLCPP_WARN(get_logger(),
                  "PRODUCTION_MPPI_ESDF rejected revision=%" PRIu64
                  " reason=invalid_raw_grid",
                  snapshot->obstacle_snapshot_revision);
      continue;
    }
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
    const auto host_distances =
        std::make_shared<const std::vector<float>>(std::move(distances));
    ProductionMppiNavigation navigation;
    {
      const std::scoped_lock lock{input_mutex_};
      navigation = navigation_;
    }
    ActiveGlobalGuideUpdate guide_update;
    GlobalGuideHeading guide_heading;
    RiskAwareLatticeResult lattice_observation;
    bool lattice_search_performed = false;
    std::shared_ptr<const std::vector<Point2>> guide;
    if (navigation.valid && active_guide_lifecycle_) {
      const Point2 position{navigation.state.x, navigation.state.y};
      guide_update = active_guide_lifecycle_->update(
          grid, *host_distances, position,
          guide_stall_generation_.load(std::memory_order_relaxed));
      if (guide_update.active) {
        guide = active_guide_lifecycle_->guide();
        guide_heading.source = GlobalGuideHeadingSource::kActiveGuide;
      } else {
        guide_heading = active_guide_lifecycle_->selectPlanningHeading(
            navigation.state, navigation.state.yaw);
        lattice_observation = planRiskAwareMotionPrimitiveGuide(
            grid, *host_distances, position, guide_heading.heading_rad,
            Point2{mission_goal_.x, mission_goal_.y}, lattice_config_);
        lattice_search_performed = true;
        const auto candidate = std::make_shared<const std::vector<Point2>>(
            std::move(lattice_observation.guide));
        const bool reaches_mission_goal =
            lattice_observation.reached_mission_goal && !candidate->empty() &&
            distance(candidate->back(), Point2{mission_goal_.x, mission_goal_.y}) <=
                lattice_config_.goal_tolerance_m;
        if (lattice_observation.valid &&
            active_guide_lifecycle_->accept(candidate, reaches_mission_goal, grid,
                                            *host_distances, position)) {
          guide = active_guide_lifecycle_->guide();
          active_guide_expansions = lattice_observation.expansions;
          active_guide_cost = lattice_observation.cost;
        } else {
          active_guide_expansions = 0U;
          active_guide_cost = 0.0;
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
    const ProductionMppiPreparedEsdf prepared{
        .producer_instance_id = snapshot->producer_instance_id,
        .revision = snapshot->obstacle_snapshot_revision,
        .source_stamp_ns = source_stamp_ns,
        .ready_stamp_ns = get_clock()->now().nanoseconds(),
        .build_ms = build_ms,
        .conversion_ms = conversion_ms,
        .upload_ms = upload.upload_ms,
        .grid = grid,
        .distances_m = host_distances,
        .global_guide = guide,
        .global_guide_expansions = active_guide_expansions,
        .global_guide_cost = active_guide_cost,
        .global_guide_generation = active_status.generation,
        .global_guide_reused = guide_update.retained,
        .global_guide_mission_goal_hold = active_status.mission_goal_hold,
        .global_guide_release_reason = guide_update.release_reason,
        .global_guide_heading_source = guide_heading.source,
        .global_guide_risk = active_status.current_risk,
        .global_guide_projection = active_status.projection,
        .lattice_search_performed = lattice_search_performed,
        .lattice_legacy_valid = lattice_observation.valid,
        .lattice_status = lattice_observation.status,
        .lattice_termination = lattice_observation.termination,
        .lattice_planning_goal_reached = lattice_observation.planning_goal_reached,
        .lattice_achieved_progress_m = lattice_observation.achieved_progress_m,
        .lattice_guide_length_m = lattice_observation.guide_length_m,
        .lattice_remaining_goal_distance_m =
            lattice_observation.remaining_goal_distance_m,
        .lattice_terminal_successor_count =
            lattice_observation.terminal_successor_count,
    };
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      prepared_esdf_ = prepared;
    }
    RCLCPP_INFO(
        get_logger(),
        "PRODUCTION_MPPI_ESDF revision=%" PRIu64
        " build_ms=%.2f conversion_ms=%.2f upload_ms=%.2f "
        "raw_to_ready_ms=%.2f dropped_updates=%" PRIu64
        " guide_valid=%s guide_points=%zu guide_expansions=%zu guide_cost=%.2f "
        "guide_generation=%" PRIu64
        " guide_reused=%s guide_mission_goal_hold=%s guide_release=%s "
        "guide_heading_source=%s guide_risk=%s guide_station_m=%.2f "
        "guide_remaining_m=%.2f guide_cross_track_m=%.2f "
        "lattice_search_performed=%s lattice_legacy_valid=%s lattice_status=%s "
        "lattice_termination=%s lattice_planning_goal_reached=%s "
        "lattice_achieved_progress_m=%.2f lattice_guide_length_m=%.2f "
        "lattice_remaining_goal_distance_m=%.2f "
        "lattice_terminal_successors=%zu",
        prepared.revision, prepared.build_ms, prepared.conversion_ms,
        prepared.upload_ms,
        static_cast<double>(prepared.ready_stamp_ns - prepared.source_stamp_ns) / 1.0e6,
        dropped_raw_snapshots_, guide && guide->size() >= 2U ? "true" : "false",
        guide ? guide->size() : 0U, prepared.global_guide_expansions,
        prepared.global_guide_cost, prepared.global_guide_generation,
        prepared.global_guide_reused ? "true" : "false",
        prepared.global_guide_mission_goal_hold ? "true" : "false",
        globalGuideReleaseReasonName(prepared.global_guide_release_reason),
        globalGuideHeadingSourceName(prepared.global_guide_heading_source),
        globalGuideRiskTierName(prepared.global_guide_risk),
        prepared.global_guide_projection.station_m,
        prepared.global_guide_projection.remaining_m,
        prepared.global_guide_projection.cross_track_m,
        prepared.lattice_search_performed ? "true" : "false",
        prepared.lattice_legacy_valid ? "true" : "false",
        latticePlanStatusName(prepared.lattice_status),
        latticeSearchTerminationName(prepared.lattice_termination),
        prepared.lattice_planning_goal_reached ? "true" : "false",
        prepared.lattice_achieved_progress_m, prepared.lattice_guide_length_m,
        prepared.lattice_remaining_goal_distance_m,
        prepared.lattice_terminal_successor_count);
  }
}

mppi::State ProductionMppiNode::selectTarget(const ProductionMppiNavigation& navigation,
                                             const ProductionMppiPreparedEsdf& esdf,
                                             const double lookahead_m,
                                             std::string& target_source) const {
  mppi::State target{static_cast<float>(mission_goal_.x),
                     static_cast<float>(mission_goal_.y),
                     static_cast<float>(mission_goal_.z)};
  target_source = "mission_goal_direct";
  if (!esdf.global_guide || esdf.global_guide->empty()) {
    return target;
  }
  const GlobalGuideProjection projection = projectOntoGlobalGuide(
      *esdf.global_guide, Point2{navigation.state.x, navigation.state.y},
      esdf.global_guide_projection.station_m);
  if (!projection.valid) {
    return target;
  }
  const Point2 selected = sampleGlobalGuide(
      *esdf.global_guide, projection.station_m + std::max(0.0, lookahead_m));
  target.x = static_cast<float>(selected.x);
  target.y = static_cast<float>(selected.y);
  target_source = "motion_primitive_guide";
  return target;
}

std::optional<mppi::PassageConstraint>
ProductionMppiNode::selectPassageConstraint(const mppi::State& state,
                                            const std::span<const Point2> guide) const {
  if (!known_passages_.has_value()) {
    return std::nullopt;
  }
  const PassageOpening* selected = nullptr;
  double selected_distance = passage_route_selection_config_.activation_distance_m;
  for (const PassageStructure& structure : known_passages_->structures) {
    for (const PassageOpening& opening : structure.openings) {
      const double opening_distance =
          std::hypot(opening.center.x - state.x, opening.center.y - state.y);
      if (opening_distance < selected_distance &&
          guideCrossesPassageAhead(state, guide, opening,
                                   passage_route_selection_config_)) {
        selected = &opening;
        selected_distance = opening_distance;
      }
    }
  }
  if (selected == nullptr) {
    return std::nullopt;
  }
  const double local_x = state.x - selected->center.x;
  const double local_y = state.y - selected->center.y;
  const double longitudinal =
      local_x * selected->normal_xy.x + local_y * selected->normal_xy.y;
  const bool inside = std::abs(longitudinal) <= 0.5 * selected->depth_m &&
                      state.z >= selected->min_z_m && state.z <= selected->max_z_m;
  return mppi::PassageConstraint{
      static_cast<float>(selected->center.x),
      static_cast<float>(selected->center.y),
      static_cast<float>(selected->normal_xy.x),
      static_cast<float>(selected->normal_xy.y),
      static_cast<float>(0.5 * selected->depth_m),
      static_cast<float>(selected->min_z_m + 1.0),
      static_cast<float>(selected->max_z_m - 1.0),
      inside ? state.z
             : static_cast<float>(0.5 * (selected->min_z_m + selected->max_z_m)),
      static_cast<float>(selected->approach_distance_m),
      static_cast<float>(selected->exit_distance_m),
      activePassageSpeedLimitMps(passage_speed_policy_),
      inside ? mppi::PassagePhase::kPartialFromInside : mppi::PassagePhase::kApproach};
}

void ProductionMppiNode::planningTick() {
  if (!engine_ || !engine_->ready()) {
    return;
  }
  const auto snapshot_started = std::chrono::steady_clock::now();
  ProductionMppiNavigation navigation;
  ProductionMppiPredictionError prediction;
  ProductionMppiAppliedControl applied_control;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    prediction = latest_prediction_error_;
    applied_control = applied_control_;
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
  if (!navigation.valid || !esdf.has_value() || pose_age_ms < 0.0 ||
      pose_age_ms > maximum_pose_age_ms_ || esdf_age_ms < 0.0 ||
      esdf_age_ms > maximum_esdf_age_ms_) {
    return;
  }
  const std::span<const Point2> guide =
      esdf->global_guide ? std::span<const Point2>{*esdf->global_guide}
                         : std::span<const Point2>{};
  MppiSpeedPolicyResult speed_policy;
  speed_policy.target_lookahead_m = no_static_guide_lookahead_m_;
  if (passage_speed_policy_.use_static_map) {
    speed_policy = evaluateStaticMppiSpeedPolicy(
        static_speed_policy_config_, MppiSpeedPolicyInput{
                                         .state = navigation.state,
                                         .mission_goal = mission_goal_,
                                         .guide = guide,
                                         .passage_speed_limit_mps = std::nullopt,
                                     });
  }
  std::string target_source;
  mppi::State target =
      selectTarget(navigation, *esdf, speed_policy.target_lookahead_m, target_source);
  std::optional<mppi::PassageConstraint> passage =
      selectPassageConstraint(navigation.state, guide);
  if (passage_speed_policy_.use_static_map && passage.has_value()) {
    speed_policy = evaluateStaticMppiSpeedPolicy(
        static_speed_policy_config_,
        MppiSpeedPolicyInput{
            .state = navigation.state,
            .mission_goal = mission_goal_,
            .guide = guide,
            .passage_speed_limit_mps = passage->speed_limit_mps,
        });
    target =
        selectTarget(navigation, *esdf, speed_policy.target_lookahead_m, target_source);
  }
  if (passage.has_value()) {
    target.z = passage->preferred_z_m;
    target_source = "passage_primitive";
  }
  const bool control_feedback_fresh =
      applied_control.valid && control_feedback_age_ms >= 0.0 &&
      control_feedback_age_ms <= maximum_control_feedback_age_ms_;
  MppiLivenessResult liveness;
  if (liveness_supervisor_) {
    liveness = liveness_supervisor_->evaluate(MppiLivenessObservation{
        .stamp_ns = now_ns,
        .actual_state = navigation.state,
        .controller_active = control_feedback_fresh,
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
        .guide_generation = projection.valid && !esdf->global_guide_mission_goal_hold
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
      guide_stall_generation_.store(guide_progress.stall_generation,
                                    std::memory_order_relaxed);
      RCLCPP_WARN(
          get_logger(),
          "GLOBAL_GUIDE_STALL guide_generation=%" PRIu64 " stall_generation=%" PRIu64
          " observation_age_s=%.3f along_guide_progress_m=%.3f "
          "predicted_head_progress_m=%.3f",
          esdf->global_guide_generation, guide_progress.stall_generation,
          guide_progress.observation_age_s, guide_progress.progress_m,
          previous_result_.has_value() ? previous_result_->head_progress_m : 0.0F);
    }
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
      .nominal_reseed_generation = liveness.reseed_generation,
      .reference_speed_mps = speed_policy.enabled
                                 ? static_cast<float>(speed_policy.reference_speed_mps)
                                 : -1.0F,
  };
  const double snapshot_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - snapshot_started)
                                 .count();
  mppi::MppiTickResult result;
  try {
    result = engine_->plan(input);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "PRODUCTION_MPPI_TICK failed: %s", error.what());
    return;
  }
  if (liveness.reseed_requested) {
    ++liveness_reseeds_;
    RCLCPP_WARN(get_logger(),
                "MPPI_LIVENESS_RESEED generation=%" PRIu64
                " observation_age_s=%.3f actual_displacement_m=%.3f speed_mps=%.3f "
                "predicted_head_progress_m=%.3f predicted_terminal_progress_m=%.3f "
                "feedback_sequence=%" PRIu64,
                liveness.reseed_generation, liveness.observation_age_s,
                liveness.actual_displacement_m, liveness.actual_speed_mps,
                liveness.predicted_head_progress_m,
                liveness.predicted_terminal_progress_m,
                applied_control.horizon_sequence);
  }
  const auto stability_started = std::chrono::steady_clock::now();
  const ProductionMppiStability stability = compareWithPrevious(result);
  const double stability_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - stability_started)
                                  .count();
  const auto rviz_started = std::chrono::steady_clock::now();
  if (now_ns - last_rviz_stamp_ns_ >= rviz_period_ns_) {
    publishRviz(input, result, *esdf);
    last_rviz_stamp_ns_ = now_ns;
  }
  const double rviz_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - rviz_started)
                             .count();
  publishDiagnostics(input, result, *esdf, stability, prediction, liveness,
                     speed_policy, target_source, pose_age_ms, esdf_age_ms,
                     control_feedback_age_ms, snapshot_ms, stability_ms, rviz_ms);
  publishExecutionHorizon(input, result, *esdf, now_ns);
  {
    const std::scoped_lock lock{input_mutex_};
    if (result.horizon.size() > 1U) {
      previous_predicted_next_state_ = result.horizon[1U];
      previous_prediction_stamp_ns_ = now_ns;
    }
  }
  previous_result_ = result;
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
