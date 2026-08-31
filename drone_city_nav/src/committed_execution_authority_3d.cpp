#include "drone_city_nav/committed_execution_authority_3d.hpp"

#include <cmath>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finitePoint(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool validMode(const ExecutionAuthorityMode3D mode) noexcept {
  switch (mode) {
    case ExecutionAuthorityMode3D::kPlanned:
    case ExecutionAuthorityMode3D::kPositionHold:
    case ExecutionAuthorityMode3D::kRevoked:
      return true;
  }
  return false;
}

[[nodiscard]] bool validReason(const ExecutionAuthorityReason3D reason) noexcept {
  switch (reason) {
    case ExecutionAuthorityReason3D::kNone:
    case ExecutionAuthorityReason3D::kNoExecutableHorizon:
    case ExecutionAuthorityReason3D::kCooperativePassageYield:
    case ExecutionAuthorityReason3D::kGoalCapture:
    case ExecutionAuthorityReason3D::kNoExecutableRoute:
    case ExecutionAuthorityReason3D::kUnavailableWorld:
      return true;
  }
  return false;
}

[[nodiscard]] bool finiteControl(const MotionControl3D& control) noexcept {
  return std::isfinite(control.ax) && std::isfinite(control.ay) &&
         std::isfinite(control.az) && std::isfinite(control.yaw_accel);
}

[[nodiscard]] const VersionedExecutionInput3D*
planExecutionInput(const ExecutionPlan3D& plan) noexcept {
  if (const FiniteExecutionState3D* const execution = plan.finiteExecution()) {
    return execution->execution_input.get();
  }
  if (const DirectTrackingFiniteExecution3D* const execution =
          plan.directTrackingExecution()) {
    return execution->execution_input.get();
  }
  if (const StationaryExecutionHold3D* const hold = plan.stationaryHold()) {
    return hold->terminal_execution_input.get();
  }
  return nullptr;
}

} // namespace

bool ExecutionOwnerIdentity3D::empty() const noexcept {
  return !valid && route_target.x == 0.0 && route_target.y == 0.0 &&
         route_target.z == 0.0 && stationary_hold_position.x == 0.0 &&
         stationary_hold_position.y == 0.0 && stationary_hold_position.z == 0.0 &&
         valid_from_ns == 0 && valid_until_ns == 0 && producer_instance_id == 0U &&
         target_offboard_instance_id == 0U && sequence == 0U &&
         execution_owner_epoch == 0U &&
         execution_mode == ExecutionAuthorityMode3D::kPositionHold &&
         execution_reason == ExecutionAuthorityReason3D::kNone &&
         !stationary_position_hold;
}

bool ExecutionOwnerIdentity3D::validFor(const ExecutionPlan3D& plan) const noexcept {
  const bool mode_semantics_valid =
      execution_mode == ExecutionAuthorityMode3D::kPlanned
          ? !stationary_position_hold &&
                execution_reason == ExecutionAuthorityReason3D::kNone
          : execution_mode == ExecutionAuthorityMode3D::kPositionHold &&
                stationary_position_hold &&
                execution_reason != ExecutionAuthorityReason3D::kNone;
  return valid && plan.valid() && finitePoint(route_target) &&
         finitePoint(stationary_hold_position) && valid_from_ns > 0 &&
         valid_until_ns > valid_from_ns && producer_instance_id != 0U &&
         target_offboard_instance_id != 0U && sequence != 0U &&
         execution_owner_epoch != 0U &&
         execution_owner_epoch == plan.execution_owner_epoch &&
         validMode(execution_mode) &&
         execution_mode != ExecutionAuthorityMode3D::kRevoked &&
         validReason(execution_reason) && mode_semantics_valid;
}

bool AppliedControlEvidence3D::empty() const noexcept {
  return !valid && control.ax == 0.0F && control.ay == 0.0F && control.az == 0.0F &&
         control.yaw_accel == 0.0F && yaw_rate_radps == 0.0F && source_stamp_ns == 0 &&
         receive_stamp_ns == 0 && producer_instance_id == 0U &&
         horizon_producer_instance_id == 0U && horizon_sequence == 0U &&
         content_fingerprint == 0U &&
         execution_mode == ExecutionAuthorityMode3D::kPositionHold &&
         !yaw_acceleration_authoritative && !control_authoritative;
}

bool AppliedControlEvidence3D::validFor(
    const ExecutionOwnerIdentity3D& owner) const noexcept {
  const bool authority_semantics_valid =
      execution_mode == ExecutionAuthorityMode3D::kPlanned
          ? control_authoritative && yaw_acceleration_authoritative
          : execution_mode == ExecutionAuthorityMode3D::kPositionHold &&
                !control_authoritative && !yaw_acceleration_authoritative;
  return valid && owner.valid && finiteControl(control) &&
         std::isfinite(yaw_rate_radps) && source_stamp_ns > 0 && receive_stamp_ns > 0 &&
         producer_instance_id != 0U && horizon_producer_instance_id != 0U &&
         horizon_sequence != 0U && content_fingerprint != 0U &&
         validMode(execution_mode) &&
         execution_mode != ExecutionAuthorityMode3D::kRevoked &&
         producer_instance_id == owner.target_offboard_instance_id &&
         horizon_producer_instance_id == owner.producer_instance_id &&
         horizon_sequence == owner.sequence && execution_mode == owner.execution_mode &&
         authority_semantics_valid;
}

CommittedExecutionAuthority3D::CommittedExecutionAuthority3D(
    CaptureToken /*capture_token*/, const std::uint64_t revision,
    std::shared_ptr<const ExecutionPlan3D> plan, const ExecutionOwnerIdentity3D& owner,
    std::shared_ptr<const VersionedExecutionInput3D> input,
    const AppliedControlEvidence3D& control)
    : revision_{revision},
      plan_{std::move(plan)},
      owner_{owner},
      input_{std::move(input)},
      control_{control} {
}

std::uint64_t CommittedExecutionAuthority3D::revision() const noexcept {
  return revision_;
}

const std::shared_ptr<const ExecutionPlan3D>&
CommittedExecutionAuthority3D::plan() const noexcept {
  return plan_;
}

const ExecutionOwnerIdentity3D& CommittedExecutionAuthority3D::owner() const noexcept {
  return owner_;
}

const std::shared_ptr<const VersionedExecutionInput3D>&
CommittedExecutionAuthority3D::input() const noexcept {
  return input_;
}

const AppliedControlEvidence3D&
CommittedExecutionAuthority3D::control() const noexcept {
  return control_;
}

bool CommittedExecutionAuthority3D::valid() const noexcept {
  if (revision_ == 0U || plan_ == nullptr || !plan_->valid()) {
    return false;
  }
  if (!owner_.valid) {
    return owner_.empty() && input_ == nullptr && control_.empty();
  }
  if (!owner_.validFor(*plan_) || input_ == nullptr || !input_->valid() ||
      planExecutionInput(*plan_) != input_.get()) {
    return false;
  }
  return control_.valid ? control_.validFor(owner_) : control_.empty();
}

} // namespace drone_city_nav
