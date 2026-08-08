#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <ranges>
#include <span>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] builtin_interfaces::msg::Time
timeFromNanoseconds(const std::int64_t nanoseconds) {
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  time.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return time;
}

void appendStationaryHoldPoint(msg::MppiTrajectoryHorizon& horizon,
                               const Point3& hold_position,
                               const float time_from_start_s, const float yaw_rad) {
  msg::MppiHorizonPoint point;
  point.time_from_start_s = time_from_start_s;
  point.position.x = hold_position.x;
  point.position.y = hold_position.y;
  point.position.z = hold_position.z;
  point.yaw_rad = yaw_rad;
  horizon.points.push_back(point);
}

} // namespace

ProductionMppiExecutionPublication ProductionMppiNode::publishExecutionHorizon(
    const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
    const ProductionMppiPreparedEsdf& esdf,
    const ProductionMppiPlanningState planning_state, const std::int64_t now_ns) {
  ProductionMppiExecutionPublication publication;
  if (!execution_horizon_pub_) {
    return publication;
  }
  const std::shared_ptr<const ProductionNavigationObjective> objective =
      navigationObjective();
  const Point3 mission_goal = objective ? objective->goal : mission_goal_;
  const std::shared_ptr<const ProductionMppiRawWorld2D> latest_raw_world =
      latest_raw_world_.load(std::memory_order_acquire);
  const OccupancyGrid2D* latest_raw_occupancy =
      !use_static_map_ && latest_raw_world && latest_raw_world->occupancy
          ? latest_raw_world->occupancy.get()
          : nullptr;
  const std::shared_ptr<const LatestLidarSafetySnapshot> latest_lidar_safety_scan =
      latest_lidar_safety_scan_.load(std::memory_order_acquire);
  constexpr std::int64_t kMaximumFutureScanSkewNs{100'000'000};
  const std::int64_t latest_lidar_age_ns =
      latest_lidar_safety_scan ? now_ns - latest_lidar_safety_scan->acquisition_stamp_ns
                               : std::numeric_limits<std::int64_t>::max();
  const bool latest_lidar_safety_fresh =
      latest_lidar_safety_enabled_ && latest_lidar_safety_scan &&
      latest_lidar_safety_scan->acquisition_stamp_ns > 0 &&
      latest_lidar_age_ns >= -kMaximumFutureScanSkewNs &&
      static_cast<double>(latest_lidar_age_ns) * 1.0e-6 <=
          latest_lidar_safety_maximum_age_ms_;
  const std::span<const Point3> latest_lidar_hit_points =
      latest_lidar_safety_fresh
          ? std::span<const Point3>{latest_lidar_safety_scan->hit_points_map_m}
          : std::span<const Point3>{};

  const auto make_horizon = [&](const std::int64_t valid_until_ns,
                                const ProductionMppiExecutionMode mode,
                                const ProductionMppiExecutionReason reason) {
    msg::MppiTrajectoryHorizon horizon;
    horizon.header.stamp = now();
    horizon.header.frame_id = frame_id_;
    horizon.sequence = tick_sequence_;
    horizon.valid_from = timeFromNanoseconds(now_ns);
    horizon.valid_until = timeFromNanoseconds(valid_until_ns);
    horizon.pose_revision = input.pose_revision;
    horizon.obstacle_revision =
        latest_raw_world ? latest_raw_world->revision : input.obstacle_revision;
    horizon.risk_tier = static_cast<std::uint8_t>(result.selected_tier);
    horizon.execution_mode = static_cast<std::uint8_t>(mode);
    horizon.execution_reason = static_cast<std::uint8_t>(reason);
    horizon.route_constrained =
        esdf.constrained_spans != nullptr && !esdf.constrained_spans->empty();
    const bool goal_hold =
        planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold;
    horizon.stationary_position_hold = goal_hold;
    horizon.stationary_hold_position.x = mission_goal.x;
    horizon.stationary_hold_position.y = mission_goal.y;
    horizon.stationary_hold_position.z = mission_goal.z;
    return horizon;
  };

  const bool goal_hold =
      planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold;
  if (goal_hold) {
    if (!insideFlightEnvelope(mission_goal, flight_envelope_config_)) {
      RCLCPP_ERROR(get_logger(),
                   "EXECUTION_HORIZON rejected reason=goal_outside_flight_envelope "
                   "target_z=%.3f",
                   mission_goal.z);
      return publication;
    }
    safety_intervention_tracker_.reset();
    brake_hold_lifecycle_.reset();
    const Point3 hold_position = mission_goal;
    const auto hold_duration_ns = static_cast<std::int64_t>(
        std::max(0.2, 2.0 * static_cast<double>(mppi_config_.dynamics.dt_s)) * 1.0e9);
    const ProductionMppiExecutionReason reason =
        ProductionMppiExecutionReason::kGoalCapture;
    msg::MppiTrajectoryHorizon horizon = make_horizon(
        now_ns + hold_duration_ns, ProductionMppiExecutionMode::kPositionHold, reason);
    horizon.points.reserve(2U);
    appendStationaryHoldPoint(horizon, hold_position, 0.0F, input.initial_state.yaw);
    appendStationaryHoldPoint(horizon, hold_position, mppi_config_.dynamics.dt_s,
                              input.initial_state.yaw);
    execution_horizon_pub_->publish(horizon);
    publication.horizon = {
        mppi::State{.x = static_cast<float>(hold_position.x),
                    .y = static_cast<float>(hold_position.y),
                    .z = static_cast<float>(hold_position.z),
                    .yaw = input.initial_state.yaw},
        mppi::State{.x = static_cast<float>(hold_position.x),
                    .y = static_cast<float>(hold_position.y),
                    .z = static_cast<float>(hold_position.z),
                    .yaw = input.initial_state.yaw},
    };
    publication.mode = ProductionMppiExecutionMode::kPositionHold;
    publication.reason = reason;
    publication.published = true;
    return publication;
  }

  if (result.horizon.size() < 2U) {
    return publication;
  }
  const bool forced_braking_hold =
      planning_state == ProductionMppiPlanningState::kNoGuideBrakingHold ||
      planning_state == ProductionMppiPlanningState::kUnavailableWorldBrakingHold;
  MppiHorizonSafetyResult safety;
  MppiSafetyInterventionUpdate intervention;
  if (!forced_braking_hold) {
    if (!esdf.distances_m) {
      return publication;
    }
    safety = evaluateMppiHorizonSafety(
        input.initial_state, result.horizon, *esdf.distances_m, esdf.grid,
        safety_config_, false, {},
        use_static_map_ && static_occupancy_3d_ ? &*static_occupancy_3d_ : nullptr,
        latest_raw_occupancy, latest_lidar_hit_points);
    intervention = safety_intervention_tracker_.update(now_ns, safety);
    if (safety.global_raw_fallback_samples > 0U) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "STATIC_SAFETY_GLOBAL_RAW_FALLBACK samples=%zu collision=%s",
                           safety.global_raw_fallback_samples,
                           safety.global_raw_collision ? "true" : "false");
    }
    if (!use_static_map_ && safety.global_raw_validation_samples > 0U) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "NO_STATIC_SAFETY_LATEST_RAW samples=%zu collision=%s raw_revision=%" PRIu64,
          safety.global_raw_validation_samples,
          safety.global_raw_collision ? "true" : "false",
          latest_raw_world ? latest_raw_world->revision : 0U);
    }
    if (latest_lidar_safety_fresh) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LATEST_LIDAR_SAFETY sequence=%" PRIu64
          " age_ms=%.1f hit_points=%zu validation_samples=%zu point_checks=%zu "
          "stopping_samples=%zu stopping_point_checks=%zu stopping_collision=%s "
          "stopping_ttc_s=%.3f collision=%s",
          latest_lidar_safety_scan->sequence,
          static_cast<double>(latest_lidar_age_ns) * 1.0e-6,
          latest_lidar_hit_points.size(), safety.latest_lidar_validation_samples,
          safety.latest_lidar_point_checks,
          safety.latest_lidar_stopping_validation_samples,
          safety.latest_lidar_stopping_point_checks,
          safety.latest_lidar_stopping_path_collision ? "true" : "false",
          safety.latest_lidar_stopping_time_to_collision_s,
          safety.latest_lidar_collision ? "true" : "false");
      constexpr std::int64_t kStoppingCollisionLogPeriodNs{1'000'000'000};
      if (safety.latest_lidar_stopping_path_collision &&
          (!latest_lidar_stopping_collision_active_ ||
           now_ns - latest_lidar_stopping_collision_log_ns_ >=
               kStoppingCollisionLogPeriodNs)) {
        RCLCPP_WARN(
            get_logger(),
            "LATEST_LIDAR_STOPPING_SAFETY collision=true sequence=%" PRIu64
            " age_ms=%.1f speed_mps=%.3f stopping_distance_m=%.3f "
            "time_to_collision_s=%.3f hit_points=%zu validation_samples=%zu "
            "point_checks=%zu action=brake",
            latest_lidar_safety_scan->sequence,
            static_cast<double>(latest_lidar_age_ns) * 1.0e-6,
            std::hypot(std::hypot(input.initial_state.vx, input.initial_state.vy),
                       input.initial_state.vz),
            safety.stopping_distance_m,
            safety.latest_lidar_stopping_time_to_collision_s,
            latest_lidar_hit_points.size(),
            safety.latest_lidar_stopping_validation_samples,
            safety.latest_lidar_stopping_point_checks);
        latest_lidar_stopping_collision_log_ns_ = now_ns;
      }
      latest_lidar_stopping_collision_active_ =
          safety.latest_lidar_stopping_path_collision;
    } else if (latest_lidar_safety_enabled_ && latest_lidar_safety_scan) {
      latest_lidar_stopping_collision_active_ = false;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "LATEST_LIDAR_SAFETY usable=false reason=stale age_ms=%.1f "
                           "maximum_age_ms=%.1f sequence=%" PRIu64,
                           static_cast<double>(latest_lidar_age_ns) * 1.0e-6,
                           latest_lidar_safety_maximum_age_ms_,
                           latest_lidar_safety_scan->sequence);
    } else {
      latest_lidar_stopping_collision_active_ = false;
    }
    if (safety.flight_envelope_violation) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EXECUTION_HORIZON flight_envelope_violation=true action=safety_fallback "
          "minimum_z=%.3f maximum_z_exclusive=%.3f",
          flight_envelope_config_.minimum_target_z_m,
          flight_envelope_config_.maximum_target_z_m);
    }
  } else {
    safety_intervention_tracker_.reset();
  }
  const bool braking =
      forced_braking_hold ||
      (!forced_braking_hold &&
       intervention.decision != MppiHorizonSafetyDecision::kExecute &&
       intervention.decision != MppiHorizonSafetyDecision::kExecuteUntilDeadline);
  const MppiBrakeHoldUpdate brake_hold = brake_hold_lifecycle_.update(
      braking, input.initial_state, safety_config_.position_hold_capture_speed_mps,
      flight_envelope_config_);
  ProductionMppiExecutionReason fallback_reason =
      ProductionMppiExecutionReason::kHorizonSafety;
  if (planning_state == ProductionMppiPlanningState::kNoGuideBrakingHold) {
    fallback_reason = ProductionMppiExecutionReason::kNoGuide;
  } else if (planning_state ==
             ProductionMppiPlanningState::kUnavailableWorldBrakingHold) {
    fallback_reason = ProductionMppiExecutionReason::kUnavailableWorld;
  }
  if (brake_hold.position_hold) {
    const auto hold_duration_ns = static_cast<std::int64_t>(
        std::max(0.2, 2.0 * static_cast<double>(mppi_config_.dynamics.dt_s)) * 1.0e9);
    msg::MppiTrajectoryHorizon horizon =
        make_horizon(now_ns + hold_duration_ns,
                     ProductionMppiExecutionMode::kPositionHold, fallback_reason);
    horizon.stationary_position_hold = true;
    horizon.stationary_hold_position.x = brake_hold.hold_state.x;
    horizon.stationary_hold_position.y = brake_hold.hold_state.y;
    horizon.stationary_hold_position.z = brake_hold.hold_state.z;
    horizon.emergency_braking = true;
    horizon.points.reserve(2U);
    const Point3 hold_position{brake_hold.hold_state.x, brake_hold.hold_state.y,
                               brake_hold.hold_state.z};
    appendStationaryHoldPoint(horizon, hold_position, 0.0F, brake_hold.hold_state.yaw);
    appendStationaryHoldPoint(horizon, hold_position, mppi_config_.dynamics.dt_s,
                              brake_hold.hold_state.yaw);
    execution_horizon_pub_->publish(horizon);
    publication.horizon = {brake_hold.hold_state, brake_hold.hold_state};
    publication.mode = ProductionMppiExecutionMode::kPositionHold;
    publication.reason = fallback_reason;
    publication.published = true;
    return publication;
  }
  std::span<const mppi::State> states{result.horizon};
  std::span<const mppi::Control> controls{result.controls};
  if (braking && !forced_braking_hold) {
    states = safety.fallback_horizon;
    controls = safety.fallback_controls;
  }
  if (states.size() < 2U || controls.empty()) {
    return publication;
  }
  if (!std::ranges::all_of(states, [this](const mppi::State& state) {
        return insideFlightEnvelope(state.z, flight_envelope_config_);
      })) {
    RCLCPP_ERROR(get_logger(),
                 "EXECUTION_HORIZON rejected reason=post_safety_flight_envelope");
    return publication;
  }

  const ProductionMppiExecutionMode execution_mode =
      braking ? ProductionMppiExecutionMode::kBraking
              : ProductionMppiExecutionMode::kPlanned;
  const ProductionMppiExecutionReason execution_reason =
      braking ? fallback_reason : ProductionMppiExecutionReason::kNone;
  msg::MppiTrajectoryHorizon horizon = make_horizon(
      now_ns + static_cast<std::int64_t>(
                   static_cast<double>(controls.size()) *
                   static_cast<double>(mppi_config_.dynamics.dt_s) * 1.0e9),
      execution_mode, execution_reason);
  horizon.emergency_braking = braking;
  horizon.points.reserve(states.size());
  for (std::size_t index = 0U; index < states.size(); ++index) {
    const mppi::State& state = states[index];
    const mppi::Control control = controls[std::min(index, controls.size() - 1U)];
    msg::MppiHorizonPoint point;
    point.time_from_start_s = static_cast<float>(index) * mppi_config_.dynamics.dt_s;
    point.position.x = state.x;
    point.position.y = state.y;
    point.position.z = state.z;
    point.velocity.x = state.vx;
    point.velocity.y = state.vy;
    point.velocity.z = state.vz;
    point.acceleration.x = control.ax;
    point.acceleration.y = control.ay;
    point.acceleration.z = control.az;
    point.yaw_rad = state.yaw;
    point.yaw_rate_radps = state.yaw_rate;
    horizon.points.push_back(point);
  }
  execution_horizon_pub_->publish(horizon);
  publication.horizon.assign(states.begin(), states.end());
  publication.mode = execution_mode;
  publication.reason = execution_reason;
  publication.published = true;
  return publication;
}

} // namespace drone_city_nav
