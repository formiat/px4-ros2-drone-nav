#include "drone_city_nav/mppi/mppi_reference.hpp"

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::onLocalState(const AutopilotLocalState& message) {
  ProductionMppiNavigation navigation;
  navigation.receive_stamp_ns = get_clock()->now().nanoseconds();
  // Autopilot source timestamps and ROS simulation time belong to different
  // clock domains. Timestamp admission uses the local monotonic receive clock;
  // ROS time remains the contract timestamp stored in the navigation sample.
  const std::int64_t monotonic_receive_stamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  // The adapter has already read the sample in the map frame; the contracts
  // here are the validity the autopilot vouches for and finite arithmetic.
  const bool position_velocity_contract =
      message.position_valid && message.altitude_valid && message.velocity_valid &&
      message.vertical_velocity_valid && std::isfinite(message.position.x) &&
      std::isfinite(message.position.y) && std::isfinite(message.position.z) &&
      std::isfinite(message.velocity.x) && std::isfinite(message.velocity.y) &&
      std::isfinite(message.velocity.z);
  const bool heading_contract = message.heading_valid && std::isfinite(message.yaw_rad);
  const double map_yaw = heading_contract ? message.yaw_rad : 0.0;
  navigation.state.x = static_cast<float>(message.position.x);
  navigation.state.y = static_cast<float>(message.position.y);
  navigation.state.z = static_cast<float>(message.position.z);
  navigation.state.vx = static_cast<float>(message.velocity.x);
  navigation.state.vy = static_cast<float>(message.velocity.y);
  navigation.state.vz = static_cast<float>(message.velocity.z);
  navigation.state.yaw = static_cast<float>(map_yaw);
  navigation.xy_reset_counter = message.xy_reset_counter;
  navigation.z_reset_counter = message.z_reset_counter;
  navigation.vxy_reset_counter = message.vxy_reset_counter;
  navigation.vz_reset_counter = message.vz_reset_counter;
  navigation.heading_reset_counter = message.heading_reset_counter;
  const bool converted_state_contract =
      std::isfinite(navigation.state.x) && std::isfinite(navigation.state.y) &&
      std::isfinite(navigation.state.z) && std::isfinite(navigation.state.vx) &&
      std::isfinite(navigation.state.vy) && std::isfinite(navigation.state.vz) &&
      std::isfinite(navigation.state.yaw);
  const bool world_state_contract =
      position_velocity_contract && std::isfinite(navigation.state.x) &&
      std::isfinite(navigation.state.y) && std::isfinite(navigation.state.z) &&
      std::isfinite(navigation.state.vx) && std::isfinite(navigation.state.vy) &&
      std::isfinite(navigation.state.vz);
  const bool authoritative_state_contract =
      world_state_contract && heading_contract && converted_state_contract;
  const std::uint64_t source_payload_fingerprint = message.payload_fingerprint;

  {
    auto lock = evidence_boundary_.releasableInput();
    const NavigationAngularDerivativeEstimate angular_derivative =
        navigation_angular_derivative_estimator_.observe(NavigationAngularObservation{
            .sample_timestamp_us = message.timestamp_sample_us,
            .publication_timestamp_us = message.timestamp_us,
            .receive_timestamp_ns = monotonic_receive_stamp_ns,
            .yaw_rad = map_yaw,
            .source_payload_fingerprint = source_payload_fingerprint,
            .xy_reset_counter = message.xy_reset_counter,
            .z_reset_counter = message.z_reset_counter,
            .vxy_reset_counter = message.vxy_reset_counter,
            .vz_reset_counter = message.vz_reset_counter,
            .heading_reset_counter = message.heading_reset_counter,
            .angular_state_authoritative = authoritative_state_contract,
        });
    if (angular_derivative.status ==
        NavigationAngularUpdateStatus::kIdempotentSourceIdentityDuplicate) {
      return;
    }
    if (angular_derivative.source_identity_conflicted) {
      navigation_.valid = false;
      latest_prediction_error_ = {};
      invalidateAppliedControlWitnessLocked();
      if (angular_derivative.source_identity_conflict) {
        requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LOCAL_POSITION rejected=true reason=%s sample_timestamp_us=%" PRIu64
          " publication_timestamp_us=%" PRIu64,
          navigationAngularUpdateStatusName(angular_derivative.status),
          message.timestamp_sample_us, message.timestamp_us);
      return;
    }
    if (!navigationAngularUpdateAccepted(angular_derivative.status)) {
      const bool timestamp_epoch_probation =
          angular_derivative.status ==
              NavigationAngularUpdateStatus::kRejectedTimestampEpochResetPending ||
          angular_derivative.status ==
              NavigationAngularUpdateStatus::kRejectedTimestampReacquisitionPending;
      if (timestamp_epoch_probation) {
        const bool navigation_was_valid = navigation_.valid;
        navigation_.valid = false;
        latest_prediction_error_ = {};
        invalidateAppliedControlWitnessLocked();
        if (navigation_was_valid) {
          requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
        }
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LOCAL_POSITION rejected=true reason=%s sample_timestamp_us=%" PRIu64
          " publication_timestamp_us=%" PRIu64,
          navigationAngularUpdateStatusName(angular_derivative.status),
          message.timestamp_sample_us, message.timestamp_us);
      return;
    }
    navigation.source_timestamp_us = angular_derivative.source_timestamp_us;
    navigation.source_timestamp_from_sample =
        angular_derivative.provenance == NavigationTimestampProvenance::kSample;
    const NavigationLocalStateResetAssessment state_reset =
        assessNavigationLocalStateReset(
            NavigationLocalStateResetCounters{
                .xy = navigation_.xy_reset_counter,
                .z = navigation_.z_reset_counter,
                .vxy = navigation_.vxy_reset_counter,
                .vz = navigation_.vz_reset_counter,
                .heading = navigation_.heading_reset_counter,
            },
            NavigationLocalStateResetCounters{
                .xy = navigation.xy_reset_counter,
                .z = navigation.z_reset_counter,
                .vxy = navigation.vxy_reset_counter,
                .vz = navigation.vz_reset_counter,
                .heading = navigation.heading_reset_counter,
            },
            navigation_baseline_established_, angular_derivative.timestamp_epoch_reset);
    const bool execution_lineage_discontinuity =
        !navigation_frame_reset_unresolved_ &&
        (state_reset.state_lineage_reset || state_reset.frame_compensation_required);
    if (state_reset.frame_compensation_required &&
        !navigation_frame_reset_unresolved_) {
      // PX4 local-origin continuity cannot be repaired only in this callback:
      // offboard setpoint conversion and world producers use the same frame.
      // Keep navigation unavailable until a coordinated handoff or node restart.
      navigation_frame_reset_unresolved_ = true;
      RCLCPP_ERROR(
          get_logger(),
          "LOCAL_POSITION frame_reset_unresolved=true timestamp_epoch_reset=%s "
          "xy_reset_counter=%u z_reset_counter=%u",
          angular_derivative.timestamp_epoch_reset ? "true" : "false",
          static_cast<unsigned>(navigation.xy_reset_counter),
          static_cast<unsigned>(navigation.z_reset_counter));
    }
    if (angular_derivative.timestamp_epoch_reset) {
      RCLCPP_WARN(
          get_logger(),
          "LOCAL_POSITION timestamp_epoch_reset=true sample_timestamp_us=%" PRIu64
          " publication_timestamp_us=%" PRIu64,
          message.timestamp_sample_us, message.timestamp_us);
    }
    if (navigation_revision_exhausted_) {
      invalidateAppliedControlWitnessLocked();
      return;
    }
    if (navigation_.revision == std::numeric_limits<std::uint64_t>::max()) {
      navigation_revision_exhausted_ = true;
      navigation_.valid = false;
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
      return;
    }
    if (!authoritative_state_contract) {
      navigation.position_velocity_authoritative = world_state_contract;
      navigation.heading_authoritative = false;
      navigation.yaw_rate_authoritative = false;
      navigation.world_state_authoritative =
          world_state_contract && !navigation_frame_reset_unresolved_;
      navigation.full_state_authoritative = false;
      navigation.valid = false;
      navigation.revision = navigation_.revision + 1U;
      navigation_ = navigation;
      latest_prediction_error_ = {};
      if (execution_lineage_discontinuity) {
        invalidateAppliedControlWitnessLocked();
        requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LOCAL_POSITION rejected=true reason=non_authoritative_state "
          "source_timestamp_us=%" PRIu64 " world_state=%s heading=%s",
          navigation.source_timestamp_us, position_velocity_contract ? "true" : "false",
          heading_contract ? "true" : "false");
      if (navigation.world_state_authoritative) {
        lock.release();
        queueLatestObservedWorldForPose(navigation);
        if (config_.world.use_static_map && navigationObjective() &&
            !world_ready_.load()) {
          requestStaticEsdfWork();
        }
      }
      return;
    }

    navigation.position_velocity_authoritative = true;
    navigation.heading_authoritative = true;
    navigation.state.yaw_rate = angular_derivative.yaw_rate_radps;
    navigation.yaw_rate_authoritative = angular_derivative.yaw_rate_authoritative;

    navigation.linear_acceleration_authoritative =
        std::isfinite(message.acceleration.x) &&
        std::isfinite(message.acceleration.y) && std::isfinite(message.acceleration.z);
    if (navigation.linear_acceleration_authoritative) {
      navigation.measured_equivalent_control =
          mppi::equivalentControlFromMeasuredAcceleration(
              navigation.state, static_cast<float>(message.acceleration.x),
              static_cast<float>(message.acceleration.y),
              static_cast<float>(message.acceleration.z),
              config_.control.mppi.dynamics);
    }
    navigation.measured_equivalent_control.yaw_accel =
        angular_derivative.yaw_acceleration_radps2;
    navigation.yaw_acceleration_authoritative =
        angular_derivative.yaw_acceleration_authoritative;
    navigation.full_state_authoritative = navigation.position_velocity_authoritative &&
                                          navigation.heading_authoritative &&
                                          navigation.yaw_rate_authoritative;
    navigation.world_state_authoritative = navigation.position_velocity_authoritative &&
                                           !navigation_frame_reset_unresolved_;
    navigation.measured_acceleration_valid =
        navigation.linear_acceleration_authoritative &&
        navigation.yaw_acceleration_authoritative;
    navigation.valid =
        navigation.full_state_authoritative && !navigation_frame_reset_unresolved_;
    navigation.revision = navigation_.revision + 1U;
    navigation_ = navigation;
    navigation_baseline_established_ = true;
    latest_prediction_error_ = {};
    if (execution_lineage_discontinuity) {
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
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
  if (navigation.world_state_authoritative) {
    queueLatestObservedWorldForPose(navigation);
  }
  if (navigation.world_state_authoritative && config_.world.use_static_map &&
      navigationObjective() && !world_ready_.load()) {
    requestStaticEsdfWork();
  }
}

} // namespace drone_city_nav
