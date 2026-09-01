#include "drone_city_nav/applied_control_admission.hpp"
#include "drone_city_nav/execution_horizon_commit_3d.hpp"
#include "drone_city_nav/execution_horizon_contract_ros.hpp"

#include <cinttypes>
#include <cmath>
#include <limits>
#include <mutex>
#include <string_view>

#include "production_mppi_execution_control.hpp"
#include "production_mppi_node.hpp"

namespace drone_city_nav {

bool vehicleStatusAuthoritativeForExecution(const ProductionMppiVehicleStatus& status,
                                            const bool timestamp_epoch_stable,
                                            const std::int64_t now_ns,
                                            const double maximum_age_ms) noexcept {
  if (!status.valid || !status.armed || !timestamp_epoch_stable ||
      status.source_timestamp_us == 0U || status.receive_stamp_ns <= 0 || now_ns <= 0 ||
      now_ns < status.receive_stamp_ns || !std::isfinite(maximum_age_ms) ||
      !(maximum_age_ms > 0.0)) {
    return false;
  }
  return static_cast<double>(now_ns - status.receive_stamp_ns) * 1.0e-6 <=
         maximum_age_ms;
}

bool appliedControlAuthoritativeForExecution(const AppliedControlEvidence3D& control,
                                             const ExecutionOwnerIdentity3D& owner,
                                             const std::int64_t now_ns,
                                             const double maximum_age_ms) noexcept {
  return appliedControlAuthoritativeForExecution3D(control, owner, now_ns,
                                                   maximum_age_ms);
}

std::optional<FootprintBodyAxis> authoritativeBodyAxisForExecution(
    const AppliedControlEvidence3D& applied_control,
    const ExecutionOwnerIdentity3D& owner, const ProductionMppiNavigation& navigation,
    const std::int64_t now_ns, const double maximum_control_age_ms,
    const double maximum_pose_age_ms) noexcept {
  const auto finite_linear_control = [](const mppi::Control& control) noexcept {
    return std::isfinite(control.ax) && std::isfinite(control.ay) &&
           std::isfinite(control.az);
  };
  if (!navigation.valid || navigation.receive_stamp_ns <= 0 || now_ns < 0 ||
      now_ns < navigation.receive_stamp_ns || !std::isfinite(maximum_pose_age_ms) ||
      !(maximum_pose_age_ms > 0.0) ||
      static_cast<double>(now_ns - navigation.receive_stamp_ns) * 1.0e-6 >
          maximum_pose_age_ms) {
    return std::nullopt;
  }
  if (appliedControlAuthoritativeForExecution(applied_control, owner, now_ns,
                                              maximum_control_age_ms) &&
      finite_linear_control(applied_control.control)) {
    return bodyAxisFromWorldAcceleration(Vec3{applied_control.control.ax,
                                              applied_control.control.ay,
                                              applied_control.control.az});
  }
  if (navigation.linear_acceleration_authoritative &&
      finite_linear_control(navigation.measured_equivalent_control)) {
    return bodyAxisFromWorldAcceleration(
        Vec3{navigation.measured_equivalent_control.ax,
             navigation.measured_equivalent_control.ay,
             navigation.measured_equivalent_control.az});
  }
  return std::nullopt;
}

void ProductionMppiNode::recordAppliedControlDiscontinuityLocked() noexcept {
  if (applied_control_discontinuity_generation_ ==
      std::numeric_limits<std::uint64_t>::max()) {
    applied_control_discontinuity_generation_exhausted_ = true;
  } else {
    ++applied_control_discontinuity_generation_;
  }
}

void ProductionMppiNode::invalidateAppliedControlWitnessLocked() noexcept {
  // Every caller holds input_mutex_. A valid-to-empty transition is a sticky
  // discontinuity even if a later callback reinstalls the exact same horizon
  // tuple before the next planning tick observes it.
  const std::shared_ptr<const CommittedExecutionAuthority3D> expected =
      execution_supervisor_.authority();
  if (expected == nullptr || !expected->control().valid) {
    return;
  }
  if (!execution_supervisor_.clearAppliedControlIfSame(expected)) {
    if (execution_supervisor_.authority() == expected) {
      applied_control_discontinuity_generation_exhausted_ = true;
    }
    return;
  }
  recordAppliedControlDiscontinuityLocked();
}

void ProductionMppiNode::onAppliedControl(const msg::MppiControlFeedback& message) {
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  const ExecutionControlFeedbackAssessment assessment =
      assessExecutionControlFeedback(message, config_.world.frame_id, receive_stamp_ns);
  if (!assessment.valid()) {
    {
      const std::scoped_lock lock{input_mutex_};
      const ExecutionHorizonWitnessAdmissionResult malformed =
          revokeMalformedExecutionHorizonFeedback(
              applied_control_admission_state_, assessment.candidate, receive_stamp_ns);
      if (malformed.state_advanced) {
        applied_control_admission_state_ = malformed.next_state;
      }
      if (malformed.witness_revoked) {
        invalidateAppliedControlWitnessLocked();
      }
    }
    const std::string_view reason =
        executionControlFeedbackStatusName(assessment.status);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "APPLIED_CONTROL rejected=true reason=%.*s horizon=%" PRIu64
                         " producer=%" PRIu64 " receive_stamp_ns=%" PRId64,
                         static_cast<int>(reason.size()), reason.data(),
                         message.horizon_sequence, message.producer_instance_id,
                         receive_stamp_ns);
    return;
  }

  AppliedControlEvidence3D feedback;
  feedback.receive_stamp_ns = assessment.candidate.receive_stamp_ns;
  feedback.source_stamp_ns = assessment.candidate.source_stamp_ns;
  feedback.producer_instance_id = assessment.candidate.offboard_producer_instance_id;
  feedback.horizon_producer_instance_id =
      assessment.candidate.horizon_producer_instance_id;
  feedback.horizon_sequence = assessment.candidate.horizon_sequence;
  feedback.execution_mode =
      static_cast<ExecutionAuthorityMode3D>(message.execution_mode);
  feedback.control_authoritative = assessment.candidate.control_authoritative;
  if (assessment.valid()) {
    feedback.control.ax = static_cast<float>(message.acceleration.x);
    feedback.control.ay = static_cast<float>(message.acceleration.y);
    feedback.control.az = static_cast<float>(message.acceleration.z);
    feedback.control.yaw_accel = message.yaw_acceleration_radps2;
    feedback.yaw_rate_radps = message.yaw_rate_radps;
    feedback.valid = std::isfinite(feedback.control.ax) &&
                     std::isfinite(feedback.control.ay) &&
                     std::isfinite(feedback.control.az) &&
                     std::isfinite(feedback.control.yaw_accel) &&
                     std::isfinite(feedback.yaw_rate_radps);
  }
  feedback.yaw_acceleration_authoritative = feedback.control_authoritative;
  feedback.content_fingerprint = assessment.candidate.content_fingerprint;
  const bool control_payload_valid = assessment.valid() && feedback.valid;
  const bool session_heartbeat = assessment.candidate.heartbeat();
  const std::scoped_lock lock{input_mutex_};
  const std::shared_ptr<const CommittedExecutionAuthority3D> execution_authority =
      execution_supervisor_.authority();
  const ExecutionOwnerIdentity3D execution_owner = execution_authority != nullptr
                                                       ? execution_authority->owner()
                                                       : ExecutionOwnerIdentity3D{};
  const AppliedControlAdmissionResult control_admission = admitAppliedControlEvidence(
      applied_control_admission_state_, assessment.candidate,
      AppliedControlExecutionOwner{
          .offboard_producer_instance_id = execution_owner.target_offboard_instance_id,
          .horizon_producer_instance_id = execution_owner.producer_instance_id,
          .horizon_sequence = execution_owner.sequence,
          .execution_mode =
              static_cast<ExecutionHorizonWitnessMode>(execution_owner.execution_mode),
      },
      control_payload_valid);
  const ExecutionHorizonWitnessAdmissionResult& admission = control_admission.feedback;
  if (!admission.accept) {
    if (admission.state_advanced) {
      applied_control_admission_state_ = admission.next_state;
    }
    if (control_admission.revoke_control) {
      invalidateAppliedControlWitnessLocked();
    }
    if (!admission.replay) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "APPLIED_CONTROL rejected=true reason=%s producer=%" PRIu64
          " horizon_producer=%" PRIu64 " horizon=%" PRIu64 " source_stamp_ns=%" PRId64,
          admission.conflict              ? "identity_conflict"
          : admission.stale               ? "stale"
          : admission.non_current_session ? "non_current_offboard_session"
                                          : "invalid_admission",
          feedback.producer_instance_id, feedback.horizon_producer_instance_id,
          feedback.horizon_sequence, feedback.source_stamp_ns);
    }
    return;
  }
  const bool advances_session_source =
      feedback.source_stamp_ns > offboard_session_admission_.latest_source_stamp_ns;
  applied_control_admission_state_ = admission.next_state;
  offboard_session_admission_ = admission.next_state.offboard_session;
  if (advances_session_source || admission.session_transitioned ||
      admission.witness_updated) {
    offboard_session_receive_stamp_ns_ =
        admission.next_state.latest_session_receive_stamp_ns;
  }

  if (session_heartbeat) {
    // Every newer fallback heartbeat revokes the exact applied-control witness.
    // The execution owner itself survives a same-session heartbeat so only a
    // later exact feedback tuple can make it witnessed again.
    if (control_admission.revoke_control) {
      invalidateAppliedControlWitnessLocked();
      if (admission.session_transitioned) {
        const std::shared_ptr<const CommittedExecutionAuthority3D> current_authority =
            execution_supervisor_.authority();
        if (current_authority != nullptr && current_authority->owner().valid) {
          static_cast<void>(execution_supervisor_.clearLeaseIfSame(current_authority));
        }
      }
      if (!control_payload_valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "APPLIED_CONTROL rejected=true reason=invalid_control_payload "
            "producer=%" PRIu64 " source_stamp_ns=%" PRId64,
            feedback.producer_instance_id, feedback.source_stamp_ns);
      }
    }
    return;
  }

  if (control_admission.revoke_control) {
    invalidateAppliedControlWitnessLocked();
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "APPLIED_CONTROL rejected=true reason=%s producer=%" PRIu64
        " horizon_producer=%" PRIu64 " horizon=%" PRIu64 " expected_producer=%" PRIu64
        " expected_horizon=%" PRIu64,
        control_payload_valid ? "non_owning_horizon" : "invalid_control_payload",
        feedback.producer_instance_id, feedback.horizon_producer_instance_id,
        feedback.horizon_sequence, execution_owner.producer_instance_id,
        execution_owner.sequence);
    return;
  }
  if (!control_admission.install_control) {
    invalidateAppliedControlWitnessLocked();
    return;
  }
  if (!execution_supervisor_.publishAppliedControlIfSame(execution_authority,
                                                         feedback)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "APPLIED_CONTROL rejected=true reason=authority_changed "
                         "horizon_producer=%" PRIu64 " horizon=%" PRIu64,
                         feedback.horizon_producer_instance_id,
                         feedback.horizon_sequence);
  }
}

} // namespace drone_city_nav
