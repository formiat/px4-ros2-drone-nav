#include "drone_city_nav/certified_route_splice_3d.hpp"
#include "drone_city_nav/execution_route_transitions_3d.hpp"

#include <memory>
#include <utility>

#include "execution_route_snapshot_3d_internal.hpp"

namespace drone_city_nav {
namespace {

using namespace execution_route_snapshot_3d_internal;

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current, ActivateCertifiedRouteCommand3D command) {
  return applyActivateCertifiedRouteCommand3D(
      current, command.expected_snapshot_version, std::move(command.candidate),
      std::move(command.candidate_execution));
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current, AdvanceCertifiedRouteCommand3D command) {
  return applyAdvanceCertifiedRouteCommand3D(
      current, command.guard, command.observation, std::move(command.execution_input),
      std::move(command.observed_raw_world));
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current,
             ReplaceFiniteExecutionPlanCommand3D command) {
  return applyReplaceFiniteExecutionPlanCommand3D(current, command.guard,
                                                  std::move(command.execution));
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current, CompleteCertifiedRouteCommand3D command) {
  return applyCompleteCertifiedRouteCommand3D(current, command.guard, command.event);
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current, ReplaceCertifiedRouteCommand3D command) {
  if (command.splice == nullptr) {
    return transitionFailure(
        ExecutionRouteTransitionStatus3D::kInvalidCandidate,
        ExecutionRouteTransitionDetail3D::kReplacementWithoutSplice);
  }
  return applyReplaceCertifiedRouteCommand3D(
      current, command.guard, std::move(command.successor),
      std::move(command.successor_execution), *command.splice);
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current,
             ReplaceCertifiedRouteAtHandoffCommand3D command) {
  return applyReplaceCertifiedRouteAtHandoffCommand3D(
      current, command.guard, std::move(command.successor),
      std::move(command.successor_execution));
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current,
             TransferToDirectTrackingCommand3D command) {
  return applyTransferToDirectTrackingCommand3D(
      current, command.expected_snapshot_version, std::move(command.direct_execution));
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current,
             ReplaceDirectTrackingExecutionCommand3D command) {
  return applyReplaceDirectTrackingExecutionCommand3D(
      current, command.expected_snapshot_version, std::move(command.direct_execution));
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current,
             TransferDirectTrackingToCertifiedRouteCommand3D command) {
  return applyTransferDirectTrackingToCertifiedRouteCommand3D(
      current, command.expected_snapshot_version, std::move(command.successor),
      std::move(command.successor_execution));
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current, TransferToExecutionHoldCommand3D command) {
  return applyTransferToExecutionHoldCommand3D(
      current, command.expected_snapshot_version, std::move(command.certification));
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current,
             ArmStationaryCaptureHoldCommand3D command) {
  return applyArmStationaryCaptureHoldCommand3D(
      current, command.expected_snapshot_version, std::move(command.certification));
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current, EnterStopExecutionCommand3D command) {
  return applyEnterStopExecutionCommand3D(current, command.expected_snapshot_version,
                                          std::move(command.certification),
                                          command.certification_report);
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current, RevokeExecutionCommand3D command) {
  return applyRevokeExecutionCommand3D(current, command.expected_snapshot_version);
}

[[nodiscard]] ExecutionRouteTransitionResult3D
applyCommand(const ExecutionPlan3D& current, SuspendFiniteExecutionCommand3D command) {
  return applySuspendFiniteExecutionCommand3D(current,
                                              command.expected_snapshot_version);
}

} // namespace

ExecutionRouteTransitionResult3D
reduceExecutionPlan3D(const ExecutionPlan3D& current,
                      ExecutionPlanTransitionCommand3D command) {
  return std::visit(
      [&current](auto&& concrete_command) {
        return applyCommand(current,
                            std::forward<decltype(concrete_command)>(concrete_command));
      },
      std::move(command));
}

ExecutionRouteTransitionResult3D activateCertifiedRoute3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D candidate, FiniteExecutionPlan3D candidate_execution) {
  return reduceExecutionPlan3D(
      current, ActivateCertifiedRouteCommand3D{
                   .expected_snapshot_version = expected_snapshot_version,
                   .candidate = std::move(candidate),
                   .candidate_execution = std::move(candidate_execution),
               });
}

ExecutionRouteTransitionResult3D advanceCertifiedRoute3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    RouteExecutionObservation3D observation,
    std::shared_ptr<const VersionedExecutionInput3D> execution_input,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world) {
  return reduceExecutionPlan3D(current,
                               AdvanceCertifiedRouteCommand3D{
                                   .guard = guard,
                                   .observation = observation,
                                   .execution_input = std::move(execution_input),
                                   .observed_raw_world = std::move(observed_raw_world),
                               });
}

ExecutionRouteTransitionResult3D
replaceFiniteExecutionPlan3D(const ExecutionPlan3D& current,
                             const ExecutionRouteTransitionGuard3D& guard,
                             FiniteExecutionPlan3D execution) {
  return reduceExecutionPlan3D(current, ReplaceFiniteExecutionPlanCommand3D{
                                            .guard = guard,
                                            .execution = std::move(execution),
                                        });
}

ExecutionRouteTransitionResult3D
completeCertifiedRoute3D(const ExecutionPlan3D& current,
                         const ExecutionRouteTransitionGuard3D& guard,
                         const RouteLifecycleEvent3D& event) {
  return reduceExecutionPlan3D(current, CompleteCertifiedRouteCommand3D{
                                            .guard = guard,
                                            .event = event,
                                        });
}

ExecutionRouteTransitionResult3D replaceCertifiedRoute3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution,
    const CertifiedRouteSplice3D& splice) {
  return reduceExecutionPlan3D(
      current, ReplaceCertifiedRouteCommand3D{
                   .guard = guard,
                   .successor = std::move(successor),
                   .successor_execution = std::move(successor_execution),
                   .splice = std::make_shared<const CertifiedRouteSplice3D>(splice),
               });
}

ExecutionRouteTransitionResult3D replaceCertifiedRouteAtHandoff3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution) {
  return reduceExecutionPlan3D(
      current, ReplaceCertifiedRouteAtHandoffCommand3D{
                   .guard = guard,
                   .successor = std::move(successor),
                   .successor_execution = std::move(successor_execution),
               });
}

ExecutionRouteTransitionResult3D
transferToDirectTracking3D(const ExecutionPlan3D& current,
                           const std::uint64_t expected_snapshot_version,
                           DirectTrackingFiniteExecution3D direct_execution) {
  return reduceExecutionPlan3D(
      current, TransferToDirectTrackingCommand3D{
                   .expected_snapshot_version = expected_snapshot_version,
                   .direct_execution = std::move(direct_execution),
               });
}

ExecutionRouteTransitionResult3D
replaceDirectTrackingExecution3D(const ExecutionPlan3D& current,
                                 const std::uint64_t expected_snapshot_version,
                                 DirectTrackingFiniteExecution3D direct_execution) {
  return reduceExecutionPlan3D(
      current, ReplaceDirectTrackingExecutionCommand3D{
                   .expected_snapshot_version = expected_snapshot_version,
                   .direct_execution = std::move(direct_execution),
               });
}

ExecutionRouteTransitionResult3D transferDirectTrackingToCertifiedRoute3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D successor, FiniteExecutionPlan3D successor_execution) {
  return reduceExecutionPlan3D(
      current, TransferDirectTrackingToCertifiedRouteCommand3D{
                   .expected_snapshot_version = expected_snapshot_version,
                   .successor = std::move(successor),
                   .successor_execution = std::move(successor_execution),
               });
}

ExecutionRouteTransitionResult3D
transferToExecutionHold3D(const ExecutionPlan3D& current,
                          const std::uint64_t expected_snapshot_version,
                          StationaryExecutionHoldCertification3D certification) {
  return reduceExecutionPlan3D(
      current, TransferToExecutionHoldCommand3D{
                   .expected_snapshot_version = expected_snapshot_version,
                   .certification = std::move(certification),
               });
}

ExecutionRouteTransitionResult3D
armStationaryCaptureHold3D(const ExecutionPlan3D& current,
                           const std::uint64_t expected_snapshot_version,
                           StationaryExecutionHoldCertification3D certification) {
  return reduceExecutionPlan3D(
      current, ArmStationaryCaptureHoldCommand3D{
                   .expected_snapshot_version = expected_snapshot_version,
                   .certification = std::move(certification),
               });
}

ExecutionRouteTransitionResult3D
enterStopExecution3D(const ExecutionPlan3D& current,
                     const std::uint64_t expected_snapshot_version,
                     StopExecutionCertification3D certification,
                     StopCertificationResult3D* const certification_report) {
  return reduceExecutionPlan3D(
      current, EnterStopExecutionCommand3D{
                   .expected_snapshot_version = expected_snapshot_version,
                   .certification = std::move(certification),
                   .certification_report = certification_report,
               });
}

ExecutionRouteTransitionResult3D
revokeExecution3D(const ExecutionPlan3D& current,
                  const std::uint64_t expected_snapshot_version) {
  return reduceExecutionPlan3D(
      current,
      RevokeExecutionCommand3D{.expected_snapshot_version = expected_snapshot_version});
}

ExecutionRouteTransitionResult3D
suspendFiniteExecution3D(const ExecutionPlan3D& current,
                         const std::uint64_t expected_snapshot_version) {
  return reduceExecutionPlan3D(
      current, SuspendFiniteExecutionCommand3D{
                   .expected_snapshot_version = expected_snapshot_version,
               });
}

ExecutionRouteTransitionResult3D composeExecutionPlanTransition3D(
    const ExecutionPlan3D& resident,
    const ExecutionRouteTransitionResult3D& prepared_progress,
    const ExecutionRouteTransitionResult3D& prepared_execution_plan) {
  if (!prepared_progress.applied() || prepared_progress.predecessor != &resident ||
      prepared_progress.next == nullptr || !prepared_execution_plan.applied() ||
      prepared_execution_plan.predecessor != prepared_progress.next.get() ||
      prepared_execution_plan.next == nullptr ||
      !prepared_execution_plan.next->publishable()) {
    return execution_route_snapshot_3d_internal::transitionFailure(
        ExecutionRouteTransitionStatus3D::kInvalidCandidate,
        ExecutionRouteTransitionDetail3D::kProgressCompositionMismatch);
  }
  return execution_route_snapshot_3d_internal::finishTransition(
      resident, *prepared_execution_plan.next);
}

} // namespace drone_city_nav
