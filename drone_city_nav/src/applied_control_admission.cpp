#include "drone_city_nav/applied_control_admission.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool validMode(const ExecutionHorizonWitnessMode mode) noexcept {
  switch (mode) {
    case ExecutionHorizonWitnessMode::kPlanned:
    case ExecutionHorizonWitnessMode::kPositionHold:
      return true;
  }
  return false;
}

} // namespace

bool AppliedControlExecutionOwner::valid() const noexcept {
  return offboard_producer_instance_id != 0U && horizon_producer_instance_id != 0U &&
         horizon_sequence != 0U && validMode(execution_mode);
}

bool AppliedControlExecutionOwner::matches(
    const ExecutionHorizonFeedbackCandidate& candidate) const noexcept {
  return valid() && candidate.horizonFeedback() &&
         offboard_producer_instance_id == candidate.offboard_producer_instance_id &&
         horizon_producer_instance_id == candidate.horizon_producer_instance_id &&
         horizon_sequence == candidate.horizon_sequence &&
         execution_mode == candidate.execution_mode;
}

AppliedControlAdmissionResult
admitAppliedControlEvidence(const ExecutionHorizonWitnessState& state,
                            const ExecutionHorizonFeedbackCandidate& candidate,
                            const AppliedControlExecutionOwner& owner,
                            const bool control_payload_valid) noexcept {
  AppliedControlAdmissionResult result{
      .feedback =
          admitExecutionHorizonFeedbackPayload(state, candidate, control_payload_valid),
  };
  result.revoke_control = result.feedback.witness_revoked ||
                          result.feedback.session_transitioned || !state.valid();
  if (!result.feedback.accept) {
    return result;
  }
  if (!control_payload_valid) {
    result.revoke_control = true;
    return result;
  }
  if (result.feedback.heartbeat) {
    return result;
  }

  result.install_control = owner.matches(candidate);
  result.revoke_control = !result.install_control;
  return result;
}

} // namespace drone_city_nav
