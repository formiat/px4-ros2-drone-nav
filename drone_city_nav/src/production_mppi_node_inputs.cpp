#include <algorithm>
#include <bit>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kLatestLidarWireFingerprintOffset{1469598103934665603ULL};
constexpr std::uint64_t kLatestLidarWireFingerprintPrime{1099511628211ULL};
constexpr std::uint64_t kLatestLidarWireFingerprintDomain{0x4c49444152575233ULL};

void hashLatestLidarWireValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    hash ^= static_cast<std::uint8_t>(value >> (index * 8U));
    hash *= kLatestLidarWireFingerprintPrime;
  }
}

[[nodiscard]] std::uint64_t
latestLidarRawWireFingerprint(const msg::LatestLidarObstacleScan& message) noexcept {
  std::uint64_t hash{kLatestLidarWireFingerprintOffset};
  hashLatestLidarWireValue(hash, kLatestLidarWireFingerprintDomain);
  hashLatestLidarWireValue(hash, static_cast<std::uint32_t>(message.header.stamp.sec));
  hashLatestLidarWireValue(hash, message.header.stamp.nanosec);
  hashLatestLidarWireValue(hash, message.header.frame_id.size());
  for (const char value : message.header.frame_id) {
    hash ^= static_cast<std::uint8_t>(value);
    hash *= kLatestLidarWireFingerprintPrime;
  }
  hashLatestLidarWireValue(hash, message.producer_instance_id);
  hashLatestLidarWireValue(hash, message.sequence);
  hashLatestLidarWireValue(hash, message.pose_generation);
  const auto hash_double = [&hash](const double value) noexcept {
    hashLatestLidarWireValue(hash, std::bit_cast<std::uint64_t>(value));
  };
  hash_double(message.frame_origin_map.x);
  hash_double(message.frame_origin_map.y);
  hash_double(message.frame_origin_map.z);
  hash_double(message.body_x_axis_map.x);
  hash_double(message.body_x_axis_map.y);
  hash_double(message.body_x_axis_map.z);
  hash_double(message.body_y_axis_map.x);
  hash_double(message.body_y_axis_map.y);
  hash_double(message.body_y_axis_map.z);
  hash_double(message.body_z_axis_map.x);
  hash_double(message.body_z_axis_map.y);
  hash_double(message.body_z_axis_map.z);
  hashLatestLidarWireValue(hash, message.hit_points_body_frd.size());
  for (const geometry_msgs::msg::Point32& point : message.hit_points_body_frd) {
    hashLatestLidarWireValue(hash, std::bit_cast<std::uint32_t>(point.x));
    hashLatestLidarWireValue(hash, std::bit_cast<std::uint32_t>(point.y));
    hashLatestLidarWireValue(hash, std::bit_cast<std::uint32_t>(point.z));
  }
  hashLatestLidarWireValue(hash, message.source_beam_count);
  hashLatestLidarWireValue(hash, message.invalid_beam_count);
  return hash == 0U ? 1U : hash;
}

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

void ProductionMppiNode::onVehicleStatus(const px4_msgs::msg::VehicleStatus& message) {
  // PX4 source timestamps are ordered against a local monotonic receipt clock.
  // ROS /clock can pause or jump with the simulator and therefore cannot be
  // the authority for epoch/reacquisition admission.
  const std::int64_t monotonic_receive_stamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  const bool armed =
      message.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  const std::scoped_lock lock{input_mutex_};
  const Px4TimestampEpochAdmissionResult admission = admitPx4TimestampEpoch(
      Px4TimestampEpochAdmissionConfig{
          .maximum_epoch_confirmation_interval_s = 2.0,
          .maximum_post_reset_unprobated_receive_gap_s = 2.0,
          .require_corroborating_timestamp_for_reset = false,
      },
      vehicle_status_timestamp_admission_,
      Px4TimestampEpochObservation{
          .primary_timestamp_us = message.timestamp,
          .corroborating_timestamp_us = 0U,
          .receive_timestamp_ns = monotonic_receive_stamp_ns,
      });
  const bool timestamp_probation_opened =
      vehicle_status_timestamp_admission_.pending_confirmation_count == 0U &&
      admission.next_state.pending_confirmation_count != 0U;
  vehicle_status_timestamp_admission_ = admission.next_state;
  // Derive eligibility from the persisted admission state. Some malformed
  // observations preserve an already-open reset probation even though their
  // individual status is not kPendingEpochReset.
  vehicle_status_epoch_probation_ =
      vehicle_status_timestamp_admission_.pending_confirmation_count != 0U;
  const auto invalidate_vehicle_status = [this]() noexcept {
    if (!vehicle_status_.valid) {
      return;
    }
    if (vehicle_status_.revision != std::numeric_limits<std::uint64_t>::max()) {
      ++vehicle_status_.revision;
      vehicle_status_.valid = false;
    } else {
      // The payload cannot change at an exhausted revision identity. The
      // exhausted latch still makes it ineligible for every publication gate.
      vehicle_status_revision_exhausted_ = true;
    }
  };
  if (timestamp_probation_opened) {
    // Invalidate authority once when reset/reacquisition probation opens. A
    // repeated candidate in the same probation cannot rejuvenate status or
    // flood the monotonic revocation request token.
    invalidate_vehicle_status();
    invalidateAppliedControlWitnessLocked();
    requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
  }
  if (!px4TimestampEpochAdmissionAccepted(admission.status)) {
    const bool same_identity_conflict =
        vehicle_status_.valid && !vehicle_status_revision_exhausted_ &&
        message.timestamp != 0U &&
        message.timestamp == vehicle_status_.source_timestamp_us &&
        armed != vehicle_status_.armed;
    if (same_identity_conflict) {
      invalidate_vehicle_status();
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
    return;
  }
  if (vehicle_status_.revision == std::numeric_limits<std::uint64_t>::max()) {
    // The admitted timestamp high-water still advances, but the payload cannot
    // change without a new revision identity. Retain it and fail closed.
    vehicle_status_revision_exhausted_ = true;
    invalidateAppliedControlWitnessLocked();
    requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    return;
  }
  const bool disarm_transition =
      vehicle_status_.valid && vehicle_status_.armed && !armed;
  if (admission.epoch_reset || disarm_transition) {
    // A PX4 reboot invalidates applied-control evidence immediately. The owner
    // remains as the exact wire witness until the planning thread publishes
    // revoke. An admitted armed-to-disarmed transition has the same barrier.
    invalidateAppliedControlWitnessLocked();
    requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
  }
  vehicle_status_ = ProductionMppiVehicleStatus{
      .receive_stamp_ns = receive_stamp_ns,
      .source_timestamp_us = message.timestamp,
      .revision = vehicle_status_.revision + 1U,
      .armed = armed,
      .valid = receive_stamp_ns > 0 && message.timestamp != 0U,
  };
}

void ProductionMppiNode::onVehicleLandDetected(
    const px4_msgs::msg::VehicleLandDetected& message) {
  const bool contact = message.landed || message.maybe_landed || message.ground_contact;
  vehicle_land_contact_.store(contact, std::memory_order_release);
  if (contact && !launch_support_confirmed_by_land_detector_.exchange(
                     true, std::memory_order_acq_rel)) {
    RCLCPP_INFO(get_logger(),
                "LAUNCH_SUPPORT_EVIDENCE source=vehicle_land_detector state=latched");
  }
  vehicle_land_contact_received_.store(true, std::memory_order_release);
}

void ProductionMppiNode::onNavigationReadiness(const std_msgs::msg::Bool& message) {
  const bool was_ready =
      vehicle_navigation_ready_.exchange(message.data, std::memory_order_acq_rel);
  if (!message.data || !use_static_map_ || !navigationObjective()) {
    return;
  }
  if (!world_ready_.load(std::memory_order_acquire)) {
    requestStaticEsdfWork();
    return;
  }
  if (was_ready) {
    return;
  }

  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  ProductionWorldBuildTelemetry3D world_telemetry;
  const std::shared_ptr<const ExecutionPlan3D> execution_snapshot =
      route_execution_manager_.plan();
  const bool initial_route_required =
      execution_snapshot == nullptr ||
      execution_snapshot->routeGenerationHighWater() == 0U;
  {
    const WorldPipelineResidentSnapshot3D resident =
        world_pipeline_->residentSnapshot();
    if (resident.world && productionWorldGenerationCoherent(*resident.world) &&
        initial_route_required) {
      if (const auto objective = navigationObjective()) {
        transaction = makePlannerSearchTransaction3D(
            resident.world, captureResidentPlannerWorld3D(*resident.world),
            makeStaticRouteObjective(*objective),
            StaticRouteSearchRequestIdentity{
                .kind = StaticRouteSearchRequestKind::kInitial,
            });
        world_telemetry = resident.telemetry;
      }
    }
  }
  bool queued = false;
  if (transaction) {
    const std::scoped_lock lock{route_planning_queue_mutex_};
    if (!pending_route_planning_work_) {
      pending_route_planning_work_ = ProductionRoutePlanningWork3D{
          .transaction = std::move(transaction),
          .world_telemetry = world_telemetry,
          .continuation_session = nullptr,
      };
      queued = true;
    }
  }
  if (queued) {
    route_planning_queue_condition_.notify_all();
    RCLCPP_INFO(get_logger(), "STATIC_ROUTE_SEARCH_REQUEST status=queued_after_takeoff "
                              "resident_esdf_ready=true");
  }
}

void ProductionMppiNode::requestStaticEsdfWork() {
  if (!use_static_map_ || !navigationObjective()) {
    return;
  }
  {
    const std::scoped_lock lock{input_mutex_};
    if (!navigation_.valid) {
      return;
    }
  }
  static_cast<void>(world_pipeline_->requestStaticWork());
}

void ProductionMppiNode::markStaticWorldReady() noexcept {
  world_ready_.store(true, std::memory_order_release);
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

void ProductionMppiNode::onLatestLidarObstacleScan(
    const msg::LatestLidarObstacleScan& message) {
  constexpr std::size_t kMaximumObstacleBeamCount{20'000U};
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  LatestLidarEvidenceClaimResult3D claimed;
  {
    const std::scoped_lock lock{latest_lidar_evidence_commit_mutex_};
    const std::shared_ptr<const VersionedLatestLidarEvidence3D> current =
        latest_lidar_evidence_.load(std::memory_order_acquire);
    claimed = claimLatestLidarEvidenceIdentity3D(
        latest_lidar_evidence_admission_state_, current.get(),
        LatestLidarEvidenceIdentityClaim3D{
            .producer_instance_id = message.producer_instance_id,
            .sequence = message.sequence,
            .raw_wire_fingerprint = latestLidarRawWireFingerprint(message),
            .first_receive_stamp_ns = receive_stamp_ns,
        });
    latest_lidar_evidence_admission_state_ = claimed.next_state;
    latest_lidar_evidence_identity_conflicted_.store(
        latestLidarEvidenceAuthorityQuarantined3D(claimed.next_state),
        std::memory_order_release);
    if (claimed.authority_quarantine_opened) {
      // Active execution publication also locks this admission domain, so the
      // persisted quarantine and its monotonic revocation request are one
      // boundary.
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
  }
  if (!claimed.assess_candidate) {
    if (claimed.status == LatestLidarEvidenceClaimStatus3D::kInstalledReplay) {
      return;
    }
    rejected_lidar_obstacle_scans_.fetch_add(1U, std::memory_order_relaxed);
    const auto status_name = latestLidarEvidenceClaimStatus3DName(claimed.status);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "LATEST_LIDAR_OBSTACLE_SCAN rejected=true reason=claim_%.*s "
                         "producer=%" PRIu64 " sequence=%" PRIu64,
                         static_cast<int>(status_name.size()), status_name.data(),
                         message.producer_instance_id, message.sequence);
    return;
  }

  // The identity is durable before any interpretation of untrusted wire data.
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
  const bool valid_counts = message.source_beam_count > 0U &&
                            message.source_beam_count <= kMaximumObstacleBeamCount &&
                            message.invalid_beam_count < message.source_beam_count &&
                            message.hit_points_body_frd.size() <=
                                message.source_beam_count - message.invalid_beam_count;
  if (message.header.frame_id != frame_id_ || acquisition_stamp_ns <= 0 ||
      message.producer_instance_id == 0U || message.sequence == 0U || !valid_counts ||
      !lidarProjectionBodyFrameIsValid(frame)) {
    rejected_lidar_obstacle_scans_.fetch_add(1U, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LATEST_LIDAR_OBSTACLE_SCAN rejected=true reason=invalid_contract "
        "producer=%" PRIu64 " sequence=%" PRIu64
        " frame='%s' acquisition_stamp_ns=%" PRId64 " source_beams=%u hit_points=%zu",
        message.producer_instance_id, message.sequence, message.header.frame_id.c_str(),
        acquisition_stamp_ns, message.source_beam_count,
        message.hit_points_body_frd.size());
    return;
  }

  LatestLidarEvidenceCapture3D capture{
      .producer_instance_id = message.producer_instance_id,
      .sequence = message.sequence,
      .pose_generation = message.pose_generation,
      .acquisition_stamp_ns = acquisition_stamp_ns,
      .receive_stamp_ns = claimed.claim.first_receive_stamp_ns,
      .source_beam_count = message.source_beam_count,
      .invalid_beam_count = message.invalid_beam_count,
      .hit_points_map_m = {},
  };
  capture.hit_points_map_m.reserve(message.hit_points_body_frd.size());
  for (const geometry_msgs::msg::Point32& point : message.hit_points_body_frd) {
    const Point3 body_point{point.x, point.y, point.z};
    if (!std::isfinite(body_point.x) || !std::isfinite(body_point.y) ||
        !std::isfinite(body_point.z)) {
      rejected_lidar_obstacle_scans_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LATEST_LIDAR_OBSTACLE_SCAN rejected=true reason=non_finite_hit "
          "sequence=%" PRIu64,
          message.sequence);
      return;
    }
    const Point3 map_point = lidarBodyPointToMap(frame, body_point);
    if (!std::isfinite(map_point.x) || !std::isfinite(map_point.y) ||
        !std::isfinite(map_point.z)) {
      rejected_lidar_obstacle_scans_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    capture.hit_points_map_m.push_back(map_point);
  }
  const std::shared_ptr<const VersionedLatestLidarEvidence3D> evidence =
      VersionedLatestLidarEvidence3D::capture(std::move(capture));
  if (evidence == nullptr) {
    rejected_lidar_obstacle_scans_.fetch_add(1U, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LATEST_LIDAR_OBSTACLE_SCAN rejected=true reason=invalid_evidence "
        "producer=%" PRIu64 " sequence=%" PRIu64,
        message.producer_instance_id, message.sequence);
    return;
  }

  LatestLidarEvidenceUpdateStatus3D update_status{
      LatestLidarEvidenceUpdateStatus3D::kRejectedInvalid};
  bool acquisition_epoch_reset{false};
  bool producer_handoff{false};
  std::uint64_t previous_producer_instance_id{0U};
  std::int64_t previous_acquisition_stamp_ns{0};
  {
    const std::scoped_lock lock{latest_lidar_evidence_commit_mutex_};
    const std::shared_ptr<const VersionedLatestLidarEvidence3D> current =
        latest_lidar_evidence_.load(std::memory_order_acquire);
    previous_producer_instance_id =
        current != nullptr ? current->producerInstanceId() : 0U;
    previous_acquisition_stamp_ns =
        current != nullptr ? current->acquisitionStampNs() : 0;
    const LatestLidarEvidenceAdmissionResult3D admission =
        admitClaimedLatestLidarEvidence3D(
            latest_lidar_evidence_admission_state_, current.get(), *evidence,
            claimed.claim, get_clock()->now().nanoseconds(),
            execution_validation_policy_ != nullptr
                ? execution_validation_policy_->latestLidarMaximumAgeMs()
                : 0.0);
    latest_lidar_evidence_admission_state_ = admission.next_state;
    latest_lidar_evidence_identity_conflicted_.store(
        latestLidarEvidenceAuthorityQuarantined3D(admission.next_state),
        std::memory_order_release);
    update_status = admission.status;
    acquisition_epoch_reset = admission.acquisition_epoch_reset;
    producer_handoff = admission.producer_handoff;
    if (admission.install_candidate) {
      latest_lidar_evidence_.store(evidence, std::memory_order_release);
    }
    if (producer_handoff || acquisition_epoch_reset ||
        admission.current_identity_conflict) {
      // Linearize the evidence authority boundary with its admission state.
      // Execution-owner publication takes this mutex and observes the
      // revocation token before it may commit against the replacement evidence.
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
  }
  if (producer_handoff) {
    RCLCPP_WARN(
        get_logger(),
        "LATEST_LIDAR_OBSTACLE_SCAN producer_handoff=true previous_producer=%" PRIu64
        " producer=%" PRIu64 " sequence=%" PRIu64,
        previous_producer_instance_id, evidence->producerInstanceId(),
        evidence->sequence());
  }
  if (acquisition_epoch_reset) {
    RCLCPP_WARN(
        get_logger(),
        "LATEST_LIDAR_OBSTACLE_SCAN acquisition_epoch_reset=true producer=%" PRIu64
        " sequence=%" PRIu64 " previous_acquisition_stamp_ns=%" PRId64
        " acquisition_stamp_ns=%" PRId64,
        evidence->producerInstanceId(), evidence->sequence(),
        previous_acquisition_stamp_ns, evidence->acquisitionStampNs());
  }
  if (update_status == LatestLidarEvidenceUpdateStatus3D::kAcceptedInitial ||
      update_status == LatestLidarEvidenceUpdateStatus3D::kAcceptedNewer) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "LATEST_LIDAR_OBSTACLE_SCAN accepted=true producer=%" PRIu64
                         " sequence=%" PRIu64 " hit_points=%zu",
                         evidence->producerInstanceId(), evidence->sequence(),
                         evidence->hitPointsMapM().size());
  }
  if (update_status == LatestLidarEvidenceUpdateStatus3D::kAcceptedInitial ||
      update_status == LatestLidarEvidenceUpdateStatus3D::kAcceptedNewer ||
      update_status ==
          LatestLidarEvidenceUpdateStatus3D::kAcceptedAcquisitionEpochReset ||
      update_status == LatestLidarEvidenceUpdateStatus3D::kAcceptedProducerHandoff ||
      update_status == LatestLidarEvidenceUpdateStatus3D::kIdempotentDuplicate) {
    return;
  }
  rejected_lidar_obstacle_scans_.fetch_add(1U, std::memory_order_relaxed);
  const auto update_status_name = latestLidarEvidenceUpdateStatus3DName(update_status);
  RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "LATEST_LIDAR_OBSTACLE_SCAN rejected=true reason=%.*s producer=%" PRIu64
      " sequence=%" PRIu64 " acquisition_stamp_ns=%" PRId64,
      static_cast<int>(update_status_name.size()), update_status_name.data(),
      evidence->producerInstanceId(), evidence->sequence(),
      evidence->acquisitionStampNs());
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
        message.prediction_horizon_s < 0.0 || message.assignment_generation == 0U ||
        message.target_detection_id == 0U || message.target_track_id == 0U ||
        timeNanoseconds(message.observation_stamp) <= 0 ||
        message.terminal_policy !=
            msg::NavigationObjective::TERMINAL_POLICY_CONTINUOUS_TRACKING))) {
    RCLCPP_WARN(get_logger(),
                "NAVIGATION_OBJECTIVE rejected mission_epoch=%" PRIu64
                " sample=%" PRIu64 " reason=invalid_payload",
                message.mission_epoch, message.sample_sequence);
    return;
  }
  std::shared_ptr<const ProductionNavigationObjective> previous;
  TrackingLineOfSightLifecycle next_line_of_sight_lifecycle;
  {
    const std::scoped_lock lock{input_mutex_};
    previous = navigation_objective_.load(std::memory_order_acquire);
    next_line_of_sight_lifecycle = tracking_line_of_sight_lifecycle_;
  }
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
        .radius_m = physical_footprint_config_.radius_m,
        .lower_extent_m = physical_footprint_config_.lower_extent_m,
        .upper_extent_m = physical_footprint_config_.upper_extent_m,
        .perimeter_samples = physical_footprint_config_.perimeter_samples,
        .radial_rings = physical_footprint_config_.radial_rings,
        .axial_samples = physical_footprint_config_.axial_samples,
        .sweep_step_m = tracking_objective_ray_sample_spacing_m_};
    bool world_available = false;
    const std::shared_ptr<const OccupancyGrid3D> static_occupancy =
        use_static_map_ ? world_pipeline_->staticOccupancy() : nullptr;
    if (static_occupancy != nullptr) {
      world_available = true;
      resolution = resolveTrackingObjective(*static_occupancy, *current_target, goal,
                                            tracking_objective_ray_sample_spacing_m_);
      if (navigation.valid) {
        direct_resolution = resolveDirectTrackingTarget(
            *static_occupancy, current_position, *current_target, goal, footprint);
      }
    } else if (!use_static_map_) {
      const std::shared_ptr<const ProductionMppiRawWorld3D> raw_world =
          world_pipeline_->latestRawWorld();
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
    const bool assignment_changed =
        !previous || previous->assignment_generation != message.assignment_generation ||
        previous->target_detection_id != message.target_detection_id ||
        previous->target_track_id != message.target_track_id;
    if (epoch_changed || assignment_changed) {
      next_line_of_sight_lifecycle.reset();
    }
    line_of_sight =
        next_line_of_sight_lifecycle.update(direct_resolution.observed_target_visible);
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
    next_line_of_sight_lifecycle.reset();
  }
  const auto objective = std::make_shared<const ProductionNavigationObjective>(
      ProductionNavigationObjective{
          .goal = goal,
          .tracking = tracking_objective,
          .mission_epoch = message.mission_epoch,
          .sample_sequence = message.sample_sequence,
          .assignment_generation = tracking ? message.assignment_generation : 0U,
          .target_detection_id = tracking ? message.target_detection_id : 0U,
          .target_track_id = tracking ? message.target_track_id : 0U,
          .stamp_ns = objective_stamp_ns,
          .continuous_tracking =
              message.terminal_policy ==
              msg::NavigationObjective::TERMINAL_POLICY_CONTINUOUS_TRACKING,
          .immediate_hold = message.terminal_policy ==
                            msg::NavigationObjective::TERMINAL_POLICY_IMMEDIATE_HOLD,
      });
  bool request_replan = false;
  bool require_new_tracking_route = false;
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const bool execution_lineage_changed =
      previous != nullptr &&
      (previous->mission_epoch != objective->mission_epoch ||
       previous->assignment_generation != objective->assignment_generation ||
       previous->target_detection_id != objective->target_detection_id ||
       previous->target_track_id != objective->target_track_id ||
       previous->continuous_tracking != objective->continuous_tracking ||
       previous->immediate_hold != objective->immediate_hold);
  {
    const std::scoped_lock lock{input_mutex_, objective_replan_mutex_};
    if (navigation_objective_.load(std::memory_order_acquire) != previous) {
      return;
    }
    const bool epoch_changed =
        !previous || previous->mission_epoch != message.mission_epoch;
    const bool assignment_changed =
        tracking && (!previous ||
                     previous->assignment_generation != message.assignment_generation ||
                     previous->target_detection_id != message.target_detection_id ||
                     previous->target_track_id != message.target_track_id);
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
        tracking && (epoch_changed || assignment_changed || direct_interception_lost);
    request_replan = epoch_changed || assignment_changed || direct_interception_lost ||
                     (moved && period_elapsed);
    navigation_objective_.store(objective, std::memory_order_release);
    tracking_line_of_sight_lifecycle_ = next_line_of_sight_lifecycle;
    if (request_replan) {
      objective_replan_anchor_ = goal;
      objective_replan_stamp_ns_ = now_ns;
    }
    if (require_new_tracking_route) {
      minimum_tracking_route_sample_sequence_.store(message.sample_sequence,
                                                    std::memory_order_release);
      minimum_tracking_route_mission_epoch_.store(message.mission_epoch,
                                                  std::memory_order_release);
    } else if (!tracking) {
      minimum_tracking_route_sample_sequence_.store(0U, std::memory_order_release);
      minimum_tracking_route_mission_epoch_.store(0U, std::memory_order_release);
    }
    if (execution_lineage_changed) {
      requestExecutionRevocation(ProductionMppiExecutionReason::kNoExecutableHorizon);
    }
  }
  publishRadarTrackModeCommand(*objective, radar_cadence_reason);
  if (use_static_map_ && !world_ready_.load(std::memory_order_acquire)) {
    requestStaticEsdfWork();
  }
  if (request_replan) {
    requestRouteRelease(RouteReleaseReason3D::kObjectiveChanged);
  }
  if (objective->tracking.has_value()) {
    const ProductionTrackingObjective tracking_data =
        objective->tracking.value_or(ProductionTrackingObjective{});
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NAVIGATION_OBJECTIVE accepted mission_epoch=%" PRIu64 " sample=%" PRIu64
        " assignment_generation=%" PRIu64 " detection_id=%" PRIu64 " track_id=%" PRIu64
        " type=tracking_prediction mode=%s horizon_s=%.3f "
        "observed=(%.2f,%.2f,%.2f) predicted=(%.2f,%.2f,%.2f) "
        "current=(%.2f,%.2f,%.2f) resolved=(%.2f,%.2f,%.2f) resolution=%s "
        "direct_target_status=%s resolved_fraction=%.3f "
        "vertical_prediction_clipped=%s observed_target_visible=%s "
        "predicted_intercept_path_clear=%s direct_interception=%s "
        "los_generation=%" PRIu64 " radar_cadence_reason=%s replan=%s",
        message.mission_epoch, message.sample_sequence, message.assignment_generation,
        message.target_detection_id, message.target_track_id,
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
