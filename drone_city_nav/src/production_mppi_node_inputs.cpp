#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <numbers>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] bool finitePoint(const geometry_msgs::msg::Point& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finiteVector(const geometry_msgs::msg::Vector3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] double dotProduct(const Point3& first, const Point3& second) noexcept {
  return first.x * second.x + first.y * second.y + first.z * second.z;
}

[[nodiscard]] bool validUnitAxis(const Point3& axis) noexcept {
  constexpr double kUnitTolerance{1.0e-3};
  const double squared_norm = dotProduct(axis, axis);
  return std::isfinite(squared_norm) && std::abs(squared_norm - 1.0) <= kUnitTolerance;
}

[[nodiscard]] bool validBodyFrame(const LidarProjectionBodyFrame& frame) noexcept {
  constexpr double kOrthogonalityTolerance{1.0e-3};
  return std::isfinite(frame.origin_map_m.x) && std::isfinite(frame.origin_map_m.y) &&
         std::isfinite(frame.origin_map_m.z) && validUnitAxis(frame.x_axis_map) &&
         validUnitAxis(frame.y_axis_map) && validUnitAxis(frame.z_axis_map) &&
         std::abs(dotProduct(frame.x_axis_map, frame.y_axis_map)) <=
             kOrthogonalityTolerance &&
         std::abs(dotProduct(frame.x_axis_map, frame.z_axis_map)) <=
             kOrthogonalityTolerance &&
         std::abs(dotProduct(frame.y_axis_map, frame.z_axis_map)) <=
             kOrthogonalityTolerance;
}

[[nodiscard]] std::optional<InterceptGuidanceMode>
guidanceMode(const std::uint8_t value) noexcept {
  switch (value) {
    case msg::NavigationObjective::GUIDANCE_MODE_DIRECT:
      return InterceptGuidanceMode::kDirect;
    case msg::NavigationObjective::GUIDANCE_MODE_ANALYTIC_INTERCEPT:
      return InterceptGuidanceMode::kAnalyticIntercept;
    case msg::NavigationObjective::GUIDANCE_MODE_AHEAD_INTERCEPT:
      return InterceptGuidanceMode::kAheadIntercept;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] const char* radarCadenceReasonName(const std::uint8_t reason) noexcept {
  switch (reason) {
    case msg::RadarTrackModeCommand::REASON_NO_TRACKING_OBJECTIVE:
      return "no_tracking_objective";
    case msg::RadarTrackModeCommand::REASON_OBSERVED_TARGET_OCCLUDED:
      return "observed_target_occluded";
    case msg::RadarTrackModeCommand::REASON_OBSERVED_TARGET_VISIBLE:
      return "observed_target_visible";
    case msg::RadarTrackModeCommand::REASON_WORLD_UNAVAILABLE:
      return "world_unavailable";
    default:
      return "unknown";
  }
}

[[nodiscard]] double pointDistance(const Point3& first, const Point3& second) noexcept {
  return std::hypot(std::hypot(first.x - second.x, first.y - second.y),
                    first.z - second.z);
}

[[nodiscard]] std::optional<Point3>
currentTrackingTarget(const geometry_msgs::msg::Point& observed,
                      const geometry_msgs::msg::Vector3& velocity,
                      const std::int64_t observation_stamp_ns,
                      const std::int64_t objective_stamp_ns,
                      const double vertical_deceleration_mps2,
                      const FlightEnvelopeConfig& flight_envelope) noexcept {
  const double age_s = static_cast<double>(std::max<std::int64_t>(
                           0, objective_stamp_ns - observation_stamp_ns)) *
                       1.0e-9;
  const TargetVerticalPrediction vertical = predictTargetVerticalMotion(
      observed.z, velocity.z, age_s, vertical_deceleration_mps2, flight_envelope);
  if (!vertical.valid) {
    return std::nullopt;
  }
  return Point3{observed.x + velocity.x * age_s, observed.y + velocity.y * age_s,
                vertical.z_m};
}

} // namespace

void ProductionMppiNode::onLocalPosition(
    const px4_msgs::msg::VehicleLocalPosition& message) {
  ProductionMppiNavigation navigation;
  navigation.receive_stamp_ns = get_clock()->now().nanoseconds();
  navigation.valid = message.xy_valid && message.z_valid && message.v_xy_valid &&
                     std::isfinite(message.x) && std::isfinite(message.y) &&
                     std::isfinite(message.z) && std::isfinite(message.vx) &&
                     std::isfinite(message.vy) && std::isfinite(message.vz);
  if (navigation.valid) {
    navigation.state.x = static_cast<float>(message.x + px4_local_origin_.x);
    navigation.state.y = static_cast<float>(message.y + px4_local_origin_.y);
    navigation.state.z = -message.z;
    navigation.state.vx = message.vx;
    navigation.state.vy = message.vy;
    navigation.state.vz = -message.vz;
    if (message.heading_good_for_control && std::isfinite(message.heading)) {
      navigation.state.yaw = message.heading;
    }
  }
  {
    const std::scoped_lock lock{input_mutex_};
    navigation.revision = navigation_.revision + 1U;
    navigation_ = navigation;
    latest_prediction_error_ = {};
    if (previous_predicted_next_state_.has_value() &&
        previous_prediction_stamp_ns_ > 0 && navigation.valid) {
      const mppi::State& predicted = *previous_predicted_next_state_;
      latest_prediction_error_.position_m =
          std::hypot(std::hypot(static_cast<double>(predicted.x - navigation.state.x),
                                static_cast<double>(predicted.y - navigation.state.y)),
                     static_cast<double>(predicted.z - navigation.state.z));
      latest_prediction_error_.velocity_mps = std::hypot(
          std::hypot(static_cast<double>(predicted.vx - navigation.state.vx),
                     static_cast<double>(predicted.vy - navigation.state.vy)),
          static_cast<double>(predicted.vz - navigation.state.vz));
      latest_prediction_error_.yaw_rad = std::abs(
          std::remainder(static_cast<double>(predicted.yaw - navigation.state.yaw),
                         2.0 * std::numbers::pi));
      latest_prediction_error_.valid = true;
    }
  }
  if (navigation.valid && use_static_map_ && vehicle_navigation_ready_.load() &&
      !world_ready_.load()) {
    requestStaticEsdfWork();
  }
}

void ProductionMppiNode::onNavigationReadiness(const std_msgs::msg::Bool& message) {
  vehicle_navigation_ready_.store(message.data, std::memory_order_release);
  if (message.data && use_static_map_ && navigationObjective() &&
      !world_ready_.load(std::memory_order_acquire)) {
    requestStaticEsdfWork();
  }
}

void ProductionMppiNode::onRawObstacleSnapshot(
    msg::RawObstacleSnapshot::ConstSharedPtr message) {
  if (use_static_map_) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  RawObstacleGridUpdate update;
  {
    const std::scoped_lock lock{raw_reconstruction_mutex_};
    update = raw_delta_accumulator_.apply(*message);
    if (update.accepted()) {
      msg::RawObstacleDelta::ConstSharedPtr pending =
          std::exchange(pending_raw_delta_, nullptr);
      if (pending &&
          pending->producer_instance_id == update.state.producer_instance_id &&
          pending->base_snapshot_revision == update.state.base_snapshot_revision &&
          pending->obstacle_snapshot_revision >
              update.state.obstacle_snapshot_revision) {
        const RawObstacleGridUpdate pending_update =
            raw_delta_accumulator_.apply(*pending);
        if (pending_update.accepted()) {
          update = pending_update;
        }
      }
    }
  }
  if (!update.accepted()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "RAW_OBSTACLE_FULL rejected status=%s producer=%" PRIu64 " revision=%" PRIu64,
        rawObstacleGridUpdateStatusName(update.status), message->producer_instance_id,
        message->obstacle_snapshot_revision);
    return;
  }
  const double reconstruction_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - started)
                                       .count();
  queueRawWorld(update.state, reconstruction_ms);
}

void ProductionMppiNode::onRawObstacleDelta(
    msg::RawObstacleDelta::ConstSharedPtr message) {
  if (use_static_map_) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  RawObstacleGridUpdate update;
  {
    const std::scoped_lock lock{raw_reconstruction_mutex_};
    update = raw_delta_accumulator_.apply(*message);
    if (update.status == RawObstacleGridUpdateStatus::kBaseUnavailable &&
        (!pending_raw_delta_ || message->obstacle_snapshot_revision >
                                    pending_raw_delta_->obstacle_snapshot_revision)) {
      pending_raw_delta_ = message;
    }
  }
  if (!update.accepted()) {
    if (update.status == RawObstacleGridUpdateStatus::kInvalidMessage) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "RAW_OBSTACLE_DELTA rejected status=%s producer=%" PRIu64 " base=%" PRIu64
          " revision=%" PRIu64,
          rawObstacleGridUpdateStatusName(update.status), message->producer_instance_id,
          message->base_snapshot_revision, message->obstacle_snapshot_revision);
    }
    return;
  }
  const double reconstruction_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - started)
                                       .count();
  queueRawWorld(update.state, reconstruction_ms);
}

void ProductionMppiNode::queueRawWorld(const RawObstacleGridState& state,
                                       const double reconstruction_ms) {
  auto world =
      std::make_shared<const ProductionMppiRawWorld2D>(ProductionMppiRawWorld2D{
          .producer_instance_id = state.producer_instance_id,
          .base_snapshot_revision = state.base_snapshot_revision,
          .revision = state.obstacle_snapshot_revision,
          .ready_stamp_ns = get_clock()->now().nanoseconds(),
          .reconstruction_ms = reconstruction_ms,
          .occupancy = state.occupancy,
      });
  latest_raw_world_.store(world, std::memory_order_release);
  no_static_raw_updates_.fetch_add(1U, std::memory_order_relaxed);
  {
    const std::scoped_lock lock{raw_queue_mutex_};
    if (pending_raw_world_) {
      ++dropped_raw_snapshots_;
    }
    pending_raw_world_ = std::move(world);
  }
  raw_queue_condition_.notify_all();
}

void ProductionMppiNode::requestStaticEsdfWork(const bool force_refresh) {
  if (!use_static_map_ || !vehicle_navigation_ready_.load(std::memory_order_acquire) ||
      !navigationObjective()) {
    return;
  }
  {
    const std::scoped_lock lock{raw_queue_mutex_};
    if (!force_refresh &&
        (world_ready_.load(std::memory_order_acquire) ||
         static_esdf_work_in_progress_ || pending_static_esdf_work_)) {
      return;
    }
    pending_static_esdf_work_ = true;
  }
  raw_queue_condition_.notify_all();
}

void ProductionMppiNode::completeStaticEsdfWork(const bool world_ready) noexcept {
  {
    const std::scoped_lock lock{raw_queue_mutex_};
    static_esdf_work_in_progress_ = false;
  }
  if (world_ready) {
    world_ready_.store(true, std::memory_order_release);
  }
}

void ProductionMppiNode::publishWorldReadiness(const bool ready) {
  world_ready_.store(ready, std::memory_order_release);
  std_msgs::msg::Bool message;
  message.data = ready;
  world_readiness_pub_->publish(message);
  RCLCPP_INFO(get_logger(), "PLANNER_WORLD_READY ready=%s source=%s",
              ready ? "true" : "false",
              use_static_map_ ? "resident_static_esdf" : "raw_snapshot_esdf");
}

void ProductionMppiNode::onMemoryStatus(const msg::ObstacleMemoryStatus& message) {
  const std::scoped_lock lock{input_mutex_};
  memory_sequence_ = message.sequence;
  memory_receive_stamp_ns_ = get_clock()->now().nanoseconds();
}

void ProductionMppiNode::onLatestLidarSafetyScan(
    const msg::LatestLidarSafetyScan& message) {
  constexpr std::size_t kMaximumSafetyBeamCount{20'000U};
  if (!latest_lidar_safety_enabled_) {
    return;
  }
  const std::int64_t acquisition_stamp_ns = timeNanoseconds(message.header.stamp);
  const LidarProjectionBodyFrame frame{
      .origin_map_m = Point3{message.frame_origin_map.x, message.frame_origin_map.y,
                             message.frame_origin_map.z},
      .x_axis_map = Point3{message.body_x_axis_map.x, message.body_x_axis_map.y,
                           message.body_x_axis_map.z},
      .y_axis_map = Point3{message.body_y_axis_map.x, message.body_y_axis_map.y,
                           message.body_y_axis_map.z},
      .z_axis_map = Point3{message.body_z_axis_map.x, message.body_z_axis_map.y,
                           message.body_z_axis_map.z},
      .valid = true,
  };
  const bool valid_counts =
      message.source_beam_count <= kMaximumSafetyBeamCount &&
      message.hit_points_body_frd.size() <= message.source_beam_count;
  if (message.header.frame_id != frame_id_ || acquisition_stamp_ns <= 0 ||
      !valid_counts || !validBodyFrame(frame)) {
    rejected_lidar_safety_scans_.fetch_add(1U, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LATEST_LIDAR_SAFETY rejected=true reason=invalid_contract sequence=%" PRIu64
        " frame='%s' acquisition_stamp_ns=%" PRId64 " source_beams=%u hit_points=%zu",
        message.sequence, message.header.frame_id.c_str(), acquisition_stamp_ns,
        message.source_beam_count, message.hit_points_body_frd.size());
    return;
  }

  auto snapshot = std::make_shared<LatestLidarSafetySnapshot>();
  snapshot->hit_points_map_m.reserve(message.hit_points_body_frd.size());
  for (const geometry_msgs::msg::Point32& point : message.hit_points_body_frd) {
    const Point3 body_point{point.x, point.y, point.z};
    if (!std::isfinite(body_point.x) || !std::isfinite(body_point.y) ||
        !std::isfinite(body_point.z)) {
      rejected_lidar_safety_scans_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LATEST_LIDAR_SAFETY rejected=true reason=non_finite_hit sequence=%" PRIu64,
          message.sequence);
      return;
    }
    const Point3 map_point = lidarBodyPointToMap(frame, body_point);
    if (!std::isfinite(map_point.x) || !std::isfinite(map_point.y) ||
        !std::isfinite(map_point.z)) {
      rejected_lidar_safety_scans_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    snapshot->hit_points_map_m.push_back(map_point);
  }
  snapshot->acquisition_stamp_ns = acquisition_stamp_ns;
  snapshot->receive_stamp_ns = get_clock()->now().nanoseconds();
  snapshot->sequence = message.sequence;
  snapshot->pose_generation = message.pose_generation;
  snapshot->source_beam_count = message.source_beam_count;
  snapshot->invalid_beam_count = message.invalid_beam_count;
  latest_lidar_safety_scan_.store(std::move(snapshot), std::memory_order_release);
}

void ProductionMppiNode::onAppliedControl(const msg::MppiControlFeedback& message) {
  ProductionMppiAppliedControl feedback;
  feedback.receive_stamp_ns = get_clock()->now().nanoseconds();
  feedback.horizon_sequence = message.horizon_sequence;
  feedback.emergency_braking = message.emergency_braking;
  feedback.valid =
      message.header.frame_id == frame_id_ && std::isfinite(message.acceleration.x) &&
      std::isfinite(message.acceleration.y) && std::isfinite(message.acceleration.z);
  if (feedback.valid) {
    feedback.control.ax = static_cast<float>(message.acceleration.x);
    feedback.control.ay = static_cast<float>(message.acceleration.y);
    feedback.control.az = static_cast<float>(message.acceleration.z);
  }
  const std::scoped_lock lock{input_mutex_};
  applied_control_ = feedback;
}

std::shared_ptr<const ProductionNavigationObjective>
ProductionMppiNode::navigationObjective() const {
  return navigation_objective_.load(std::memory_order_acquire);
}

void ProductionMppiNode::publishRadarTrackModeCommand(
    const ProductionNavigationObjective& objective, const std::uint8_t reason) {
  if (!radar_track_mode_command_pub_) {
    return;
  }
  msg::RadarTrackModeCommand command;
  command.stamp = get_clock()->now();
  command.mission_epoch = objective.mission_epoch;
  command.objective_sample_sequence = objective.sample_sequence;
  command.target_track_id =
      objective.tracking.has_value() ? objective.tracking->target_track_id : 0U;
  command.mode =
      objective.tracking.has_value() && objective.tracking->observed_target_visible
          ? msg::RadarTrackModeCommand::MODE_TRACK
          : msg::RadarTrackModeCommand::MODE_SEARCH;
  command.reason = reason;
  radar_track_mode_command_pub_->publish(command);
}

void ProductionMppiNode::onNavigationObjective(
    const msg::NavigationObjective& message) {
  const bool tracking = message.objective_type ==
                        msg::NavigationObjective::OBJECTIVE_TYPE_TRACKING_PREDICTION;
  const std::optional<InterceptGuidanceMode> guidance_mode =
      guidanceMode(message.guidance_mode);
  const FlightEnvelopeStatus target_altitude_status =
      evaluateFlightEnvelopeAltitude(message.position.z, flight_envelope_config_);
  if (!tracking && target_altitude_status != FlightEnvelopeStatus::kValid) {
    RCLCPP_WARN(get_logger(),
                "NAVIGATION_OBJECTIVE rejected mission_epoch=%" PRIu64
                " sample=%" PRIu64 " reason=flight_envelope_%s target_z=%.3f",
                message.mission_epoch, message.sample_sequence,
                flightEnvelopeStatusName(target_altitude_status), message.position.z);
    return;
  }
  if (!finitePoint(message.position) ||
      message.objective_type >
          msg::NavigationObjective::OBJECTIVE_TYPE_TRACKING_PREDICTION ||
      !guidance_mode.has_value() ||
      message.terminal_policy >
          msg::NavigationObjective::TERMINAL_POLICY_IMMEDIATE_HOLD ||
      (tracking &&
       (!finitePoint(message.observed_target_position) ||
        !finiteVector(message.observed_target_velocity) ||
        !std::isfinite(message.prediction_horizon_s) ||
        message.prediction_horizon_s < 0.0 || message.target_track_id == 0U ||
        timeNanoseconds(message.observation_stamp) <= 0 ||
        message.terminal_policy !=
            msg::NavigationObjective::TERMINAL_POLICY_CONTINUOUS_TRACKING))) {
    RCLCPP_WARN(get_logger(),
                "NAVIGATION_OBJECTIVE rejected mission_epoch=%" PRIu64
                " sample=%" PRIu64 " reason=invalid_payload",
                message.mission_epoch, message.sample_sequence);
    return;
  }
  const std::shared_ptr<const ProductionNavigationObjective> previous =
      navigationObjective();
  if (previous && message.mission_epoch < previous->mission_epoch) {
    return;
  }
  if (previous && message.mission_epoch == previous->mission_epoch &&
      message.sample_sequence <= previous->sample_sequence) {
    return;
  }

  const std::int64_t objective_stamp_ns = timeNanoseconds(message.stamp);
  const std::int64_t observation_stamp_ns = timeNanoseconds(message.observation_stamp);
  const Point3 unconstrained_goal{message.position.x, message.position.y,
                                  message.position.z};
  const std::optional<double> bounded_goal_z =
      clampToFlightEnvelope(unconstrained_goal.z, flight_envelope_config_);
  if (!bounded_goal_z.has_value()) {
    RCLCPP_WARN(get_logger(),
                "NAVIGATION_OBJECTIVE rejected mission_epoch=%" PRIu64
                " sample=%" PRIu64 " reason=invalid_target_altitude target_z=%.3f",
                message.mission_epoch, message.sample_sequence, unconstrained_goal.z);
    return;
  }
  Point3 goal{unconstrained_goal.x, unconstrained_goal.y, *bounded_goal_z};
  std::optional<ProductionTrackingObjective> tracking_objective;
  TrackingLineOfSightUpdate line_of_sight;
  std::uint8_t radar_cadence_reason =
      msg::RadarTrackModeCommand::REASON_NO_TRACKING_OBJECTIVE;
  if (tracking) {
    const Point3 observed{message.observed_target_position.x,
                          message.observed_target_position.y,
                          message.observed_target_position.z};
    const std::optional<double> bounded_observed_z =
        clampToFlightEnvelope(observed.z, flight_envelope_config_);
    if (!bounded_observed_z.has_value()) {
      RCLCPP_WARN(get_logger(),
                  "NAVIGATION_OBJECTIVE rejected mission_epoch=%" PRIu64
                  " sample=%" PRIu64 " reason=invalid_observed_target_altitude",
                  message.mission_epoch, message.sample_sequence);
      return;
    }
    const std::optional<Point3> current_target = currentTrackingTarget(
        message.observed_target_position, message.observed_target_velocity,
        observation_stamp_ns, objective_stamp_ns,
        mppi_config_.dynamics.maximum_vertical_acceleration_mps2,
        flight_envelope_config_);
    if (!current_target.has_value()) {
      RCLCPP_WARN(get_logger(),
                  "NAVIGATION_OBJECTIVE rejected mission_epoch=%" PRIu64
                  " sample=%" PRIu64 " reason=invalid_current_target_prediction",
                  message.mission_epoch, message.sample_sequence);
      return;
    }
    TrackingObjectiveResolution resolution{
        .resolved_position = *current_target,
        .status = TrackingObjectiveResolutionStatus::kWorldUnavailable,
        .resolved_fraction = 0.0,
    };
    DirectTrackingTargetResolution direct_resolution{
        .selected_position = *current_target,
        .status = DirectTrackingTargetStatus::kWorldUnavailable,
    };
    ProductionMppiNavigation navigation;
    {
      const std::scoped_lock lock{input_mutex_};
      navigation = navigation_;
    }
    const Point3 current_position{navigation.state.x, navigation.state.y,
                                  navigation.state.z};
    const SweptFootprintConfig footprint{
        .radius_m = safety_config_.physical_footprint_radius_m,
        .lower_extent_m = safety_config_.physical_footprint_lower_extent_m,
        .upper_extent_m = safety_config_.physical_footprint_upper_extent_m,
        .perimeter_samples = safety_config_.physical_footprint_samples,
        .radial_rings = safety_config_.physical_footprint_radial_rings,
        .axial_samples = safety_config_.physical_footprint_axial_samples,
        .sweep_step_m = tracking_objective_ray_sample_spacing_m_};
    bool world_available = false;
    if (use_static_map_ && static_occupancy_3d_) {
      world_available = true;
      resolution =
          resolveTrackingObjective(*static_occupancy_3d_, *current_target, goal,
                                   tracking_objective_ray_sample_spacing_m_);
      if (navigation.valid) {
        direct_resolution = resolveDirectTrackingTarget(
            *static_occupancy_3d_, current_position, *current_target, goal, footprint);
      }
    } else if (!use_static_map_) {
      const std::shared_ptr<const ProductionMppiRawWorld2D> raw_world =
          latest_raw_world_.load(std::memory_order_acquire);
      if (raw_world && raw_world->occupancy) {
        world_available = true;
        resolution =
            resolveTrackingObjective(*raw_world->occupancy, *current_target, goal,
                                     tracking_objective_ray_sample_spacing_m_);
        if (navigation.valid) {
          direct_resolution =
              resolveDirectTrackingTarget(*raw_world->occupancy, current_position,
                                          *current_target, goal, footprint);
        }
      }
    }
    if (world_available && !navigation.valid) {
      direct_resolution.status = DirectTrackingTargetStatus::kInvalidInput;
    }
    if (resolution.status == TrackingObjectiveResolutionStatus::kInvalidInput) {
      RCLCPP_WARN(get_logger(),
                  "NAVIGATION_OBJECTIVE rejected mission_epoch=%" PRIu64
                  " sample=%" PRIu64 " reason=invalid_tracking_resolution",
                  message.mission_epoch, message.sample_sequence);
      return;
    }
    const bool epoch_changed =
        !previous || previous->mission_epoch != message.mission_epoch;
    const std::uint64_t previous_target_track_id =
        previous
            ? previous->tracking.value_or(ProductionTrackingObjective{}).target_track_id
            : 0U;
    const bool target_track_changed =
        previous_target_track_id != message.target_track_id;
    if (epoch_changed || target_track_changed) {
      tracking_line_of_sight_lifecycle_.reset();
    }
    line_of_sight = tracking_line_of_sight_lifecycle_.update(
        direct_resolution.observed_target_visible);
    goal = line_of_sight.active ? direct_resolution.selected_position
                                : resolution.resolved_position;
    if (!world_available || !navigation.valid) {
      radar_cadence_reason = msg::RadarTrackModeCommand::REASON_WORLD_UNAVAILABLE;
    } else if (direct_resolution.observed_target_visible) {
      radar_cadence_reason = msg::RadarTrackModeCommand::REASON_OBSERVED_TARGET_VISIBLE;
    } else {
      radar_cadence_reason =
          msg::RadarTrackModeCommand::REASON_OBSERVED_TARGET_OCCLUDED;
    }
    tracking_objective = ProductionTrackingObjective{
        .observed_position = observed,
        .current_target_position = *current_target,
        .unconstrained_predicted_position = unconstrained_goal,
        .observed_velocity =
            Vec3{message.observed_target_velocity.x, message.observed_target_velocity.y,
                 message.observed_target_velocity.z},
        .observation_stamp_ns = observation_stamp_ns,
        .prediction_horizon_s = message.prediction_horizon_s,
        .resolved_fraction = line_of_sight.active
                                 ? direct_resolution.selected_prediction_fraction
                                 : resolution.resolved_fraction,
        .guidance_mode = *guidance_mode,
        .resolution_status = resolution.status,
        .direct_target_status = direct_resolution.status,
        .radar_cadence_reason = radar_cadence_reason,
        .vertical_prediction_clipped =
            message.vertical_prediction_limited ||
            std::abs(*bounded_goal_z - unconstrained_goal.z) > 1.0e-9,
        .observed_target_visible = direct_resolution.observed_target_visible,
        .predicted_intercept_path_clear =
            direct_resolution.predicted_intercept_path_clear,
        .direct_interception_active = line_of_sight.active,
        .line_of_sight_generation = line_of_sight.generation,
        .target_track_id = message.target_track_id,
    };
  } else {
    tracking_line_of_sight_lifecycle_.reset();
  }
  const auto objective = std::make_shared<const ProductionNavigationObjective>(
      ProductionNavigationObjective{
          .goal = goal,
          .tracking = tracking_objective,
          .mission_epoch = message.mission_epoch,
          .sample_sequence = message.sample_sequence,
          .stamp_ns = objective_stamp_ns,
          .continuous_tracking =
              message.terminal_policy ==
              msg::NavigationObjective::TERMINAL_POLICY_CONTINUOUS_TRACKING,
          .immediate_hold = message.terminal_policy ==
                            msg::NavigationObjective::TERMINAL_POLICY_IMMEDIATE_HOLD,
      });
  navigation_objective_.store(objective, std::memory_order_release);
  publishRadarTrackModeCommand(*objective, radar_cadence_reason);
  if (use_static_map_ && vehicle_navigation_ready_.load(std::memory_order_acquire) &&
      !world_ready_.load(std::memory_order_acquire)) {
    requestStaticEsdfWork();
  }

  bool request_replan = false;
  bool require_new_tracking_route = false;
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  {
    const std::scoped_lock lock{objective_replan_mutex_};
    const bool epoch_changed =
        !previous || previous->mission_epoch != message.mission_epoch;
    const bool moved = pointDistance(goal, objective_replan_anchor_) >=
                       dynamic_objective_replan_distance_m_;
    const bool period_elapsed =
        objective_replan_stamp_ns_ <= 0 ||
        static_cast<double>(now_ns - objective_replan_stamp_ns_) * 1.0e-9 >=
            dynamic_objective_replan_period_s_;
    const bool previous_direct_interception =
        previous && previous->tracking.value_or(ProductionTrackingObjective{})
                        .direct_interception_active;
    const bool current_direct_interception =
        tracking_objective.value_or(ProductionTrackingObjective{})
            .direct_interception_active;
    const bool direct_interception_lost =
        previous_direct_interception && !current_direct_interception;
    require_new_tracking_route =
        tracking && (epoch_changed || direct_interception_lost);
    request_replan =
        epoch_changed || direct_interception_lost || (moved && period_elapsed);
    if (request_replan) {
      objective_replan_anchor_ = goal;
      objective_replan_stamp_ns_ = now_ns;
    }
  }
  if (require_new_tracking_route) {
    minimum_tracking_route_mission_epoch_.store(message.mission_epoch,
                                                std::memory_order_release);
    minimum_tracking_route_sample_sequence_.store(message.sample_sequence,
                                                  std::memory_order_release);
  } else if (!tracking) {
    minimum_tracking_route_mission_epoch_.store(0U, std::memory_order_release);
    minimum_tracking_route_sample_sequence_.store(0U, std::memory_order_release);
  }
  if (request_replan) {
    requestGuideRelease(GlobalGuideReleaseReason::kObjectiveChanged);
  }
  if (objective->tracking.has_value()) {
    const ProductionTrackingObjective tracking_data =
        objective->tracking.value_or(ProductionTrackingObjective{});
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NAVIGATION_OBJECTIVE accepted mission_epoch=%" PRIu64 " sample=%" PRIu64
        " type=tracking_prediction mode=%s horizon_s=%.3f "
        "observed=(%.2f,%.2f,%.2f) predicted=(%.2f,%.2f,%.2f) "
        "current=(%.2f,%.2f,%.2f) resolved=(%.2f,%.2f,%.2f) resolution=%s "
        "direct_target_status=%s resolved_fraction=%.3f "
        "vertical_prediction_clipped=%s observed_target_visible=%s "
        "predicted_intercept_path_clear=%s direct_interception=%s "
        "los_generation=%" PRIu64 " radar_cadence_reason=%s replan=%s",
        message.mission_epoch, message.sample_sequence,
        interceptGuidanceModeName(tracking_data.guidance_mode),
        tracking_data.prediction_horizon_s, tracking_data.observed_position.x,
        tracking_data.observed_position.y, tracking_data.observed_position.z,
        tracking_data.unconstrained_predicted_position.x,
        tracking_data.unconstrained_predicted_position.y,
        tracking_data.unconstrained_predicted_position.z,
        tracking_data.current_target_position.x,
        tracking_data.current_target_position.y,
        tracking_data.current_target_position.z, goal.x, goal.y, goal.z,
        trackingObjectiveResolutionStatusName(tracking_data.resolution_status),
        directTrackingTargetStatusName(tracking_data.direct_target_status),
        tracking_data.resolved_fraction,
        tracking_data.vertical_prediction_clipped ? "true" : "false",
        tracking_data.observed_target_visible ? "true" : "false",
        tracking_data.predicted_intercept_path_clear ? "true" : "false",
        tracking_data.direct_interception_active ? "true" : "false",
        tracking_data.line_of_sight_generation,
        radarCadenceReasonName(radar_cadence_reason),
        request_replan ? "true" : "false");
  } else {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NAVIGATION_OBJECTIVE accepted mission_epoch=%" PRIu64 " sample=%" PRIu64
        " type=position goal=(%.2f,%.2f,%.2f) policy=%s replan=%s",
        message.mission_epoch, message.sample_sequence, goal.x, goal.y, goal.z,
        objective->immediate_hold ? "immediate_hold" : "position_hold",
        request_replan ? "true" : "false");
  }
}

} // namespace drone_city_nav
