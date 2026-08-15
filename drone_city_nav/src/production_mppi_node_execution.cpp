#include "drone_city_nav/mppi/finite_execution_path.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <cinttypes>
#include <cmath>
#include <iterator>
#include <limits>
#include <ranges>
#include <span>
#include <vector>

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

[[nodiscard]] std::int64_t
timeToNanoseconds(const builtin_interfaces::msg::Time& time) noexcept {
  return static_cast<std::int64_t>(time.sec) * 1000000000LL +
         static_cast<std::int64_t>(time.nanosec);
}

[[nodiscard]] mppi::TimedExecutionPathPoint
executionPathPoint(const msg::MppiHorizonPoint& point) noexcept {
  return mppi::TimedExecutionPathPoint{
      .time_from_start_s = point.time_from_start_s,
      .state =
          mppi::State{
              .x = static_cast<float>(point.position.x),
              .y = static_cast<float>(point.position.y),
              .z = static_cast<float>(point.position.z),
              .vx = static_cast<float>(point.velocity.x),
              .vy = static_cast<float>(point.velocity.y),
              .vz = static_cast<float>(point.velocity.z),
              .yaw = point.yaw_rad,
              .yaw_rate = point.yaw_rate_radps,
          },
      .control =
          mppi::Control{
              .ax = static_cast<float>(point.acceleration.x),
              .ay = static_cast<float>(point.acceleration.y),
              .az = static_cast<float>(point.acceleration.z),
          },
  };
}

[[nodiscard]] std::vector<mppi::TimedExecutionPathPoint>
executionPathPoints(const msg::MppiTrajectoryHorizon& horizon) {
  std::vector<mppi::TimedExecutionPathPoint> points;
  points.reserve(horizon.points.size());
  std::ranges::transform(horizon.points, std::back_inserter(points),
                         executionPathPoint);
  return points;
}

[[nodiscard]] std::optional<mppi::FiniteExecutionPathTerminalBoundary>
finiteRouteTerminalBoundary(const mppi::MppiTickInput& input,
                            const ProductionMppiPreparedEsdf& esdf) noexcept {
  if (esdf.global_guide_reaches_mission_goal || !input.route.has_value() ||
      !input.route->points || input.route->points->size() < 2U) {
    return std::nullopt;
  }
  const std::vector<mppi::RouteSample3D>& route = *input.route->points;
  const mppi::RouteSample3D& previous = route[route.size() - 2U];
  const mppi::RouteSample3D& endpoint = route.back();
  const Vec3 forward{
      static_cast<double>(endpoint.x_m - previous.x_m),
      static_cast<double>(endpoint.y_m - previous.y_m),
      static_cast<double>(endpoint.z_m - previous.z_m),
  };
  if (std::hypot(std::hypot(forward.x, forward.y), forward.z) <= 1.0e-6) {
    return std::nullopt;
  }
  return mppi::FiniteExecutionPathTerminalBoundary{
      .endpoint = Point3{endpoint.x_m, endpoint.y_m, endpoint.z_m},
      .forward = forward,
  };
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
  const std::shared_ptr<const LatestLidarObstacleSnapshot> latest_lidar_obstacle_scan =
      use_static_map_ ? nullptr
                      : latest_lidar_obstacle_scan_.load(std::memory_order_acquire);
  constexpr std::int64_t kMaximumFutureScanSkewNs{100'000'000LL};
  double latest_lidar_obstacle_age_ms{-1.0};
  bool latest_lidar_obstacle_fresh{false};
  std::span<const Point3> latest_lidar_obstacle_points;
  if (latest_lidar_obstacle_scan &&
      latest_lidar_obstacle_scan->acquisition_stamp_ns > 0) {
    const std::int64_t age_ns =
        now_ns - latest_lidar_obstacle_scan->acquisition_stamp_ns;
    latest_lidar_obstacle_age_ms =
        static_cast<double>(std::max<std::int64_t>(0, age_ns)) * 1.0e-6;
    const auto maximum_age_ns = static_cast<std::int64_t>(
        std::llround(latest_lidar_obstacle_maximum_age_ms_ * 1.0e6));
    latest_lidar_obstacle_fresh =
        age_ns >= -kMaximumFutureScanSkewNs && age_ns <= maximum_age_ns;
    if (latest_lidar_obstacle_fresh) {
      latest_lidar_obstacle_points =
          std::span<const Point3>{latest_lidar_obstacle_scan->hit_points_map_m};
    }
  }
  const OccupancyGrid2D* latest_raw_occupancy =
      latest_raw_world && latest_raw_world->occupancy
          ? latest_raw_world->occupancy.get()
          : nullptr;
  const OccupancyGrid3D* static_occupancy =
      use_static_map_ && static_occupancy_3d_ ? &*static_occupancy_3d_ : nullptr;
  const std::optional<mppi::FiniteExecutionPathTerminalBoundary>
      route_terminal_boundary = finiteRouteTerminalBoundary(input, esdf);
  const mppi::FiniteExecutionPathWorld execution_path_world{
      .flight_envelope = &flight_envelope_config_,
      .dynamics = &mppi_config_.dynamics,
      .altitude_envelope = &mppi_config_.altitude_envelope,
      .footprint = &physical_footprint_config_,
      .static_occupancy = static_occupancy,
      .raw_occupancy = latest_raw_occupancy,
      .latest_lidar_obstacle_points = latest_lidar_obstacle_points,
      .terminal_boundary = route_terminal_boundary,
  };
  constexpr float kArrivalSearchIntervalS{0.5F};
  const std::size_t arrival_search_step_controls = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
              std::ceil(kArrivalSearchIntervalS / mppi_config_.dynamics.dt_s)));
  const std::int64_t finite_path_control_interval_ns =
      mppi::finitePathControlIntervalNanoseconds(mppi_config_.dynamics.dt_s);

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
    return horizon;
  };

  const auto retain_active_finite_path =
      [&](const ProductionMppiExecutionReason replacement_failure_reason)
      -> std::optional<ProductionMppiExecutionPublication> {
    if (!active_finite_execution_path_.has_value()) {
      return std::nullopt;
    }
    ProductionMppiActiveFiniteExecutionPath& active = *active_finite_execution_path_;
    if (active.message.execution_mode !=
            msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED ||
        active.message.stationary_position_hold) {
      active_finite_execution_path_.reset();
      return std::nullopt;
    }
    const std::vector<mppi::TimedExecutionPathPoint> points =
        executionPathPoints(active.message);
    if (points.empty()) {
      active_finite_execution_path_.reset();
      return std::nullopt;
    }
    const std::int64_t original_valid_until_ns =
        timeToNanoseconds(active.message.valid_until);
    mppi::FiniteExecutionPathWorld continuation_world = execution_path_world;
    continuation_world.terminal_boundary = active.terminal_boundary;
    const mppi::FiniteExecutionPathValidation actual_state_validation =
        mppi::validateFiniteExecutionPathContinuation(
            points, timeToNanoseconds(active.message.valid_from),
            original_valid_until_ns, now_ns, input.initial_state,
            input.previous_applied_control.value_or(mppi::Control{}),
            continuation_world);
    const mppi::FiniteExecutionPathValidation trajectory_validation =
        actual_state_validation.accepted()
            ? actual_state_validation
            : mppi::validateFiniteExecutionTrajectoryContinuation(
                  points, timeToNanoseconds(active.message.valid_from),
                  original_valid_until_ns, now_ns, input.initial_state,
                  input.previous_applied_control.value_or(mppi::Control{}),
                  continuation_world);
    const std::size_t trajectory_expected_index =
        std::min(trajectory_validation.first_remaining_point_index, points.size() - 1U);
    const mppi::State& trajectory_expected_state =
        points[trajectory_expected_index].state;
    const double trajectory_tracking_error_m = distance3D(
        Point3{input.initial_state.x, input.initial_state.y, input.initial_state.z},
        Point3{trajectory_expected_state.x, trajectory_expected_state.y,
               trajectory_expected_state.z});
    if (actual_state_validation.accepted()) {
      ProductionMppiExecutionPublication retained = active.publication;
      retained.latest_lidar_obstacle_sequence =
          latest_lidar_obstacle_scan ? latest_lidar_obstacle_scan->sequence : 0U;
      retained.latest_lidar_obstacle_hit_count = latest_lidar_obstacle_points.size();
      retained.latest_lidar_obstacle_age_ms = latest_lidar_obstacle_age_ms;
      retained.latest_lidar_obstacle_fresh = latest_lidar_obstacle_fresh;
      retained.retained_previous_finite_path = true;
      retained.published = false;
      active.publication = retained;
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "FINITE_EXECUTION_PATH retained=true replacement_failure_reason=%s "
          "trajectory_continuation=true actual_state_continuation=true "
          "republished=false "
          "remaining_s=%.3f "
          "tracking_error_m=%.3f vertical_error_m=%.3f valid_until_ns=%" PRId64,
          productionMppiExecutionReasonName(replacement_failure_reason),
          trajectory_validation.remaining_duration_s, trajectory_tracking_error_m,
          static_cast<double>(input.initial_state.z - trajectory_expected_state.z),
          original_valid_until_ns);
      return retained;
    }

    const mppi::RebuiltFiniteExecutionPathContinuation rebuilt =
        mppi::rebuildFiniteExecutionPathContinuation(
            points, timeToNanoseconds(active.message.valid_from),
            original_valid_until_ns, now_ns, input.initial_state,
            input.previous_applied_control.value_or(mppi::Control{}),
            mppi_config_.dynamics, arrival_search_step_controls, finite_horizon_config_,
            continuation_world);
    const std::size_t expected_index =
        std::min(rebuilt.source_control_index, points.size() - 1U);
    const mppi::State& expected_state = points[expected_index].state;
    const double tracking_error_m = distance3D(
        Point3{input.initial_state.x, input.initial_state.y, input.initial_state.z},
        Point3{expected_state.x, expected_state.y, expected_state.z});
    if (!rebuilt.accepted()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "FINITE_EXECUTION_PATH retained=false replacement_failure_reason=%s "
          "rebuild_validation=%s failure_segment=%zu "
          "failure=(%.3f,%.3f,%.3f) current_z=%.3f expected_z=%.3f "
          "tracking_error_m=%.3f trajectory_validation=%s "
          "trajectory_failure_segment=%zu actual_state_validation=%s "
          "actual_state_failure_segment=%zu arrival_shaping_attempts=%zu",
          productionMppiExecutionReasonName(replacement_failure_reason),
          mppi::finiteExecutionPathStatusName(rebuilt.validation.status),
          rebuilt.validation.failure_segment_index, rebuilt.validation.failure_point.x,
          rebuilt.validation.failure_point.y, rebuilt.validation.failure_point.z,
          input.initial_state.z, expected_state.z, tracking_error_m,
          mppi::finiteExecutionPathStatusName(trajectory_validation.status),
          trajectory_validation.failure_segment_index,
          mppi::finiteExecutionPathStatusName(actual_state_validation.status),
          actual_state_validation.failure_segment_index,
          rebuilt.arrival_shaping_attempts);
      active_finite_execution_path_.reset();
      return std::nullopt;
    }

    const mppi::FiniteHorizon& finite_horizon = *rebuilt.horizon;
    msg::MppiTrajectoryHorizon horizon =
        make_horizon(rebuilt.valid_until_ns, ProductionMppiExecutionMode::kPlanned,
                     ProductionMppiExecutionReason::kNone);
    horizon.risk_tier = active.message.risk_tier;
    horizon.points.reserve(finite_horizon.states.size());
    for (std::size_t index = 0U; index < finite_horizon.states.size(); ++index) {
      const mppi::State& state = finite_horizon.states[index];
      const mppi::Control control =
          finite_horizon.controls[std::min(index, finite_horizon.controls.size() - 1U)];
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

    ProductionMppiExecutionPublication retained = active.publication;
    retained.horizon = finite_horizon.states;
    retained.planned_control_count = finite_horizon.controls.size();
    retained.nominal_prefix_control_count = finite_horizon.nominal_prefix_control_count;
    retained.arrival_control_count = finite_horizon.arrival_control_count;
    retained.arrival_shaping_attempts = rebuilt.arrival_shaping_attempts;
    retained.first_control = finite_horizon.controls.front();
    retained.first_control_available = true;
    retained.latest_lidar_obstacle_sequence =
        latest_lidar_obstacle_scan ? latest_lidar_obstacle_scan->sequence : 0U;
    retained.latest_lidar_obstacle_hit_count = latest_lidar_obstacle_points.size();
    retained.latest_lidar_obstacle_age_ms = latest_lidar_obstacle_age_ms;
    retained.latest_lidar_obstacle_fresh = latest_lidar_obstacle_fresh;
    retained.finite_path_validation_backoff = rebuilt.path_validation_backoff;
    retained.latest_lidar_path_validation_backoff =
        rebuilt.latest_lidar_path_validation_backoff;
    retained.retained_previous_finite_path = true;
    retained.terminal_rest_state = true;
    retained.published = true;
    active.message = std::move(horizon);
    active.publication = retained;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_PATH retained=true replacement_failure_reason=%s "
        "rebased_from_actual=true source_trajectory_validation=%s "
        "source_actual_state_validation=%s source_control=%zu tracking_error_m=%.3f "
        "vertical_error_m=%.3f arrival_shaping_attempts=%zu "
        "original_valid_until_ns=%" PRId64 " rebuilt_valid_until_ns=%" PRId64,
        productionMppiExecutionReasonName(replacement_failure_reason),
        mppi::finiteExecutionPathStatusName(trajectory_validation.status),
        mppi::finiteExecutionPathStatusName(actual_state_validation.status),
        rebuilt.source_control_index, tracking_error_m,
        static_cast<double>(input.initial_state.z - expected_state.z),
        rebuilt.arrival_shaping_attempts, original_valid_until_ns,
        rebuilt.valid_until_ns);
    return retained;
  };

  const auto publish_position_hold = [&](const Point3& hold_position,
                                         const ProductionMppiExecutionReason reason) {
    active_finite_execution_path_.reset();
    if (!insideFlightEnvelope(hold_position, flight_envelope_config_)) {
      RCLCPP_ERROR(get_logger(),
                   "EXECUTION_HORIZON rejected reason=hold_outside_flight_envelope "
                   "target_z=%.3f",
                   hold_position.z);
      return publication;
    }
    const auto hold_duration_ns = static_cast<std::int64_t>(
        std::max(0.2, 2.0 * static_cast<double>(mppi_config_.dynamics.dt_s)) * 1.0e9);
    msg::MppiTrajectoryHorizon horizon = make_horizon(
        now_ns + hold_duration_ns, ProductionMppiExecutionMode::kPositionHold, reason);
    horizon.stationary_position_hold = true;
    horizon.stationary_hold_position.x = hold_position.x;
    horizon.stationary_hold_position.y = hold_position.y;
    horizon.stationary_hold_position.z = hold_position.z;
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
  };

  const auto publish_no_executable_path_hold =
      [&](const ProductionMppiExecutionReason reason) {
        if (std::optional<ProductionMppiExecutionPublication> retained =
                retain_active_finite_path(reason);
            retained.has_value()) {
          return *retained;
        }
        if (!no_executable_path_hold_position_.has_value()) {
          no_executable_path_hold_position_ = Point3{
              input.initial_state.x,
              input.initial_state.y,
              clampToFlightEnvelope(input.initial_state.z, flight_envelope_config_)
                  .value_or(flight_envelope_config_.minimum_target_z_m),
          };
          RCLCPP_WARN(
              get_logger(),
              "MPPI_EXECUTION_CONTRACT transition=enter_no_executable_path_hold "
              "reason=%s origin=(%.3f,%.3f,%.3f) "
              "velocity=(%.3f,%.3f,%.3f) previous_acceleration_z=%.3f",
              productionMppiExecutionReasonName(reason),
              no_executable_path_hold_position_->x,
              no_executable_path_hold_position_->y,
              no_executable_path_hold_position_->z, input.initial_state.vx,
              input.initial_state.vy, input.initial_state.vz,
              input.previous_applied_control.value_or(mppi::Control{}).az);
        }
        return publish_position_hold(*no_executable_path_hold_position_, reason);
      };

  const auto publish_explicit_hold = [&](const Point3& hold_position,
                                         const ProductionMppiExecutionReason reason) {
    no_executable_path_hold_position_.reset();
    return publish_position_hold(hold_position, reason);
  };

  if (planning_state == ProductionMppiPlanningState::kMissionGoalPositionHold) {
    return publish_explicit_hold(mission_goal,
                                 ProductionMppiExecutionReason::kGoalCapture);
  }
  if (planning_state == ProductionMppiPlanningState::kMissionCommandPositionHold) {
    return publish_explicit_hold(Point3{input.target.x, input.target.y, input.target.z},
                                 ProductionMppiExecutionReason::kGoalCapture);
  }
  if (planning_state == ProductionMppiPlanningState::kNoExecutableRouteHold) {
    return publish_no_executable_path_hold(
        ProductionMppiExecutionReason::kNoExecutableRoute);
  }
  if (planning_state == ProductionMppiPlanningState::kCooperativeChannelYieldHold) {
    return publish_explicit_hold(
        Point3{input.target.x, input.target.y, input.target.z},
        ProductionMppiExecutionReason::kCooperativeChannelYield);
  }

  const std::span<const mppi::State> states{result.horizon};
  const std::span<const mppi::Control> controls{result.controls};
  if (states.size() < 2U || controls.empty()) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MPPI_EXECUTION_CONTRACT degraded=true classification=%s repair=%s "
        "states=%zu controls=%zu action=hold_no_executable_path",
        mppi::mppiPostUpdateClassificationName(
            result.post_update_classification.classification),
        mppi::mppiPostUpdateRepairName(result.post_update_repair), states.size(),
        controls.size());
    return publish_no_executable_path_hold(
        ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  const auto altitude_violation =
      std::ranges::find_if(states, [this](const mppi::State& state) {
        return !insideFlightEnvelope(state.z, flight_envelope_config_);
      });
  const bool nominal_altitude_violation =
      result.altitude_envelope_violation || altitude_violation != states.end();
  if (nominal_altitude_violation) {
    const auto [minimum_z, maximum_z] = std::ranges::minmax(
        states, {}, [](const mppi::State& state) { return state.z; });
    const std::size_t violation_index = altitude_violation == states.end()
                                            ? states.size()
                                            : static_cast<std::size_t>(std::distance(
                                                  states.begin(), altitude_violation));
    const double violation_z = altitude_violation == states.end()
                                   ? std::numeric_limits<double>::quiet_NaN()
                                   : altitude_violation->z;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EXECUTION_HORIZON nominal_degraded=true "
        "reason=flight_envelope_violation "
        "initial_z=%.3f minimum_z=%.3f maximum_z=%.3f violation_index=%zu "
        "violation_z=%.3f envelope=[%.3f,%.3f) classification=%s repair=%s "
        "action=shape_and_validate_finite_path",
        states.front().z, minimum_z.z, maximum_z.z, violation_index, violation_z,
        flight_envelope_config_.minimum_target_z_m,
        flight_envelope_config_.maximum_target_z_m,
        mppi::mppiPostUpdateClassificationName(
            result.post_update_classification.classification),
        mppi::mppiPostUpdateRepairName(result.post_update_repair));
  }

  const bool nominal_candidate_degraded =
      nominal_altitude_violation || result.raw_collision ||
      result.known_solid_collision || !result.post_update_classification.executable;
  if (nominal_candidate_degraded) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MPPI_EXECUTION_CONTRACT nominal_degraded=true classification=%s repair=%s "
        "altitude_violation=%s raw_collision=%s solid_collision=%s "
        "action=shape_and_validate_finite_path",
        mppi::mppiPostUpdateClassificationName(
            result.post_update_classification.classification),
        mppi::mppiPostUpdateRepairName(result.post_update_repair),
        nominal_altitude_violation ? "true" : "false",
        result.raw_collision ? "true" : "false",
        result.known_solid_collision ? "true" : "false");
  }

  mppi::ValidatedFiniteExecutionPath validated_path =
      mppi::buildValidatedFiniteExecutionPath(
          states, controls, input.previous_applied_control.value_or(mppi::Control{}),
          mppi_config_.dynamics, arrival_search_step_controls, finite_horizon_config_,
          execution_path_world);
  if (!validated_path.accepted()) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_HORIZON executable=false reason=%s attempts=%zu "
        "planned_controls=%zu failure_segment=%zu failure=(%.3f,%.3f,%.3f) "
        "latest_lidar_sequence=%" PRIu64 " latest_lidar_age_ms=%.1f "
        "latest_lidar_hits=%zu action=hold_no_executable_path",
        mppi::finiteExecutionPathStatusName(validated_path.validation.status),
        validated_path.arrival_shaping_attempts, controls.size(),
        validated_path.validation.failure_segment_index,
        validated_path.validation.failure_point.x,
        validated_path.validation.failure_point.y,
        validated_path.validation.failure_point.z,
        latest_lidar_obstacle_scan ? latest_lidar_obstacle_scan->sequence : 0U,
        latest_lidar_obstacle_age_ms, latest_lidar_obstacle_points.size());
    return publish_no_executable_path_hold(
        ProductionMppiExecutionReason::kNoExecutableHorizon);
  }
  mppi::FiniteHorizon executable_path =
      std::move(validated_path.horizon).value_or(mppi::FiniteHorizon{});
  if (no_executable_path_hold_position_.has_value()) {
    const double speed_mps =
        std::hypot(std::hypot(input.initial_state.vx, input.initial_state.vy),
                   input.initial_state.vz);
    RCLCPP_INFO(get_logger(),
                "MPPI_EXECUTION_CONTRACT transition=leave_no_executable_path_hold "
                "reason=executable_path_available pose_revision=%" PRIu64
                " speed_mps=%.3f",
                input.pose_revision, speed_mps);
    no_executable_path_hold_position_.reset();
  }
  if (validated_path.path_validation_backoff || nominal_candidate_degraded) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FINITE_EXECUTION_PATH executable=true terminal_rest_embedded=true "
        "nominal_degraded=%s validation_backoff=%s arrival_shaping_attempts=%zu "
        "nominal_prefix_controls=%zu planned_controls=%zu arrival_controls=%zu "
        "latest_lidar_backoff=%s latest_lidar_sequence=%" PRIu64
        " latest_lidar_age_ms=%.1f latest_lidar_hits=%zu",
        nominal_candidate_degraded ? "true" : "false",
        validated_path.path_validation_backoff ? "true" : "false",
        validated_path.arrival_shaping_attempts,
        executable_path.nominal_prefix_control_count, controls.size(),
        executable_path.arrival_control_count,
        validated_path.latest_lidar_path_validation_backoff ? "true" : "false",
        latest_lidar_obstacle_scan ? latest_lidar_obstacle_scan->sequence : 0U,
        latest_lidar_obstacle_age_ms, latest_lidar_obstacle_points.size());
  }

  const std::span<const mppi::State> execution_states{executable_path.states};
  const std::span<const mppi::Control> execution_controls{executable_path.controls};

  msg::MppiTrajectoryHorizon horizon = make_horizon(
      now_ns + static_cast<std::int64_t>(execution_controls.size()) *
                   finite_path_control_interval_ns,
      ProductionMppiExecutionMode::kPlanned, ProductionMppiExecutionReason::kNone);
  horizon.points.reserve(execution_states.size());
  for (std::size_t index = 0U; index < execution_states.size(); ++index) {
    const mppi::State& state = execution_states[index];
    const mppi::Control control =
        execution_controls[std::min(index, execution_controls.size() - 1U)];
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
  publication.horizon.assign(execution_states.begin(), execution_states.end());
  publication.mode = ProductionMppiExecutionMode::kPlanned;
  publication.reason = ProductionMppiExecutionReason::kNone;
  publication.planned_control_count = controls.size();
  publication.nominal_prefix_control_count =
      executable_path.nominal_prefix_control_count;
  publication.arrival_control_count = executable_path.arrival_control_count;
  publication.arrival_shaping_attempts = validated_path.arrival_shaping_attempts;
  publication.first_control = execution_controls.front();
  publication.first_control_available = true;
  publication.latest_lidar_obstacle_sequence =
      latest_lidar_obstacle_scan ? latest_lidar_obstacle_scan->sequence : 0U;
  publication.latest_lidar_obstacle_hit_count = latest_lidar_obstacle_points.size();
  publication.latest_lidar_obstacle_age_ms = latest_lidar_obstacle_age_ms;
  publication.finite_path_validation_backoff = validated_path.path_validation_backoff;
  publication.latest_lidar_obstacle_fresh = latest_lidar_obstacle_fresh;
  publication.latest_lidar_path_validation_backoff =
      validated_path.latest_lidar_path_validation_backoff;
  publication.terminal_rest_state = true;
  publication.published = true;
  active_finite_execution_path_ = ProductionMppiActiveFiniteExecutionPath{
      .message = std::move(horizon),
      .publication = publication,
      .terminal_boundary = route_terminal_boundary,
  };
  return publication;
}

} // namespace drone_city_nav
