#include "production_mppi_execution_control.hpp"

#include "drone_city_nav/execution_horizon_commit_3d.hpp"

#include <cmath>

#include "production_mppi_node_types.hpp"

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

const char* productionMppiHorizonSupersessionDecisionName(
    const ProductionMppiHorizonSupersessionDecision decision) noexcept {
  switch (decision) {
    case ProductionMppiHorizonSupersessionDecision::kAllowedNoPlannedOwner:
      return "no_planned_owner";
    case ProductionMppiHorizonSupersessionDecision::kAllowedWitnessedOwner:
      return "witnessed_owner";
    case ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgedPredecessor:
      return "acknowledged_predecessor";
    case ProductionMppiHorizonSupersessionDecision::kAllowedAcknowledgementGraceElapsed:
      return "acknowledgement_grace_elapsed";
    case ProductionMppiHorizonSupersessionDecision::kDeferredAwaitingAcknowledgement:
      return "awaiting_acknowledgement";
    case ProductionMppiHorizonSupersessionDecision::kRejectedOwnerNotCurrent:
      return "owner_not_current";
  }
  return "unknown";
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
  const auto finite_linear_control = [](const MotionControl3D& control) noexcept {
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

} // namespace drone_city_nav
