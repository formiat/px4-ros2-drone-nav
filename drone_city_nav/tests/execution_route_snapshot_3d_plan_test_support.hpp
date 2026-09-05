#pragma once

#include "execution_route_snapshot_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

FiniteExecutionPlan3D SnapshotFixture3D::finitePlanForRoute(
    const ExecutionPlan3D& snapshot, const CertifiedRouteSuffix3D& suffix,
    const FiniteExecutionKind3D command_kind, const std::uint64_t trajectory_revision,
    const std::uint64_t source_navigation_revision,
    const std::size_t extra_stationary_control_count, const double begin_station_m) {
  // A stop owns no route and no finite execution, yet its evidence is the
  // newest the plan has seen: a successor certified over it must not regress.
  const StopExecution3D* const stop = snapshot.stopExecution();
  FiniteExecutionCertification3D command = finiteCertificationForRoute(
      suffix, command_kind, trajectory_revision, source_navigation_revision,
      extra_stationary_control_count, begin_station_m,
      snapshot.route() != nullptr ? snapshot.route()->progress.execution_input.get()
      : stop != nullptr           ? stop->execution_input.get()
                                  : nullptr,
      snapshot.finiteExecution() != nullptr
          ? snapshot.finiteExecution()->latest_lidar_evidence.get()
      : stop != nullptr ? stop->latest_lidar_evidence.get()
                        : nullptr);
  const std::optional<FiniteMotionHorizon3D> braking = buildFiniteBrakingHorizon3D(
      command.horizon.states.front(), command.horizon.controls.size(),
      suffix.validation_policy->dynamics(), command.execution_input->previousControl());
  if (!braking.has_value()) {
    throw std::logic_error{"failed to build finite braking fixture"};
  }
  FiniteExecutionPlanCertificationResult3D certified =
      certifyFiniteExecutionPlan3DDetailed(snapshot, suffix,
                                           FiniteExecutionPlanCertification3D{
                                               .command_horizon = std::move(command),
                                               .braking_tail = braking.value(),
                                           });
  if (!certified.certified() || !certified.plan.has_value()) {
    throw std::logic_error{"valid finite execution plan fixture was rejected"};
  }
  return certified.plan.value();
}

// Test-only adapter for constructing intentionally valid or tampered command
// horizons while the production API accepts only a complete atomic plan.
[[nodiscard, maybe_unused]] FiniteExecutionPlan3D
testExecutionPlanForCommand(const ExecutionPlan3D& current,
                            const CertifiedRouteSuffix3D& target_route,
                            FiniteExecutionState3D command_horizon) {
  const auto invalid_plan = [&command_horizon] {
    return FiniteExecutionPlan3D{.command_horizon = command_horizon,
                                 .braking_tail = command_horizon};
  };
  if (command_horizon.horizon == nullptr || command_horizon.horizon->states.empty() ||
      command_horizon.execution_input == nullptr ||
      command_horizon.latest_lidar_evidence == nullptr ||
      target_route.validation_policy == nullptr) {
    return invalid_plan();
  }
  const std::optional<FiniteMotionHorizon3D> braking_horizon =
      buildFiniteBrakingHorizon3D(command_horizon.horizon->states.front(),
                                  command_horizon.horizon->controls.size(),
                                  target_route.validation_policy->dynamics(),
                                  command_horizon.execution_input->previousControl());
  if (!braking_horizon.has_value()) {
    return invalid_plan();
  }
  const std::optional<FiniteExecutionState3D> braking_tail = certifyFiniteExecution3D(
      current, target_route,
      FiniteExecutionCertification3D{
          .trajectory_revision = command_horizon.trajectory_revision,
          .horizon = braking_horizon.value(),
          .execution_input = command_horizon.execution_input,
          .latest_lidar_evidence = command_horizon.latest_lidar_evidence,
          .valid_from_ns = command_horizon.valid_from_ns,
          .kind = FiniteExecutionKind3D::kEmergencyBrakeTail,
      });
  return braking_tail.has_value()
             ? FiniteExecutionPlan3D{.command_horizon = std::move(command_horizon),
                                     .braking_tail = braking_tail.value()}
             : invalid_plan();
}

[[nodiscard, maybe_unused]] ExecutionRouteTransitionResult3D activateCertifiedRoute3D(
    const ExecutionPlan3D& current, const std::uint64_t expected_snapshot_version,
    CertifiedRouteSuffix3D candidate, FiniteExecutionState3D candidate_execution) {
  FiniteExecutionPlan3D plan =
      testExecutionPlanForCommand(current, candidate, std::move(candidate_execution));
  return ::drone_city_nav::activateCertifiedRoute3D(
      current, expected_snapshot_version, std::move(candidate), std::move(plan));
}

[[nodiscard, maybe_unused]] ExecutionRouteTransitionResult3D
replaceFiniteExecution3D(const ExecutionPlan3D& current,
                         const ExecutionRouteTransitionGuard3D& guard,
                         std::optional<FiniteExecutionState3D> candidate_execution) {
  const CertifiedRouteSuffix3D* const route = current.route();
  if (!candidate_execution.has_value() || route == nullptr) {
    return ::drone_city_nav::replaceFiniteExecutionPlan3D(current, guard,
                                                          FiniteExecutionPlan3D{});
  }
  return ::drone_city_nav::replaceFiniteExecutionPlan3D(
      current, guard,
      testExecutionPlanForCommand(current, *route,
                                  std::move(candidate_execution.value())));
}

[[nodiscard, maybe_unused]] ExecutionRouteTransitionResult3D
replaceCertifiedRoute3D(const ExecutionPlan3D& current,
                        const ExecutionRouteTransitionGuard3D& guard,
                        CertifiedRouteSuffix3D successor,
                        std::optional<FiniteExecutionState3D> successor_execution,
                        const CertifiedRouteSplice3D& splice) {
  FiniteExecutionPlan3D plan =
      successor_execution.has_value()
          ? testExecutionPlanForCommand(current, successor,
                                        std::move(successor_execution.value()))
          : FiniteExecutionPlan3D{};
  return ::drone_city_nav::replaceCertifiedRoute3D(current, guard, std::move(successor),
                                                   std::move(plan), splice);
}

[[nodiscard, maybe_unused]] ExecutionRouteTransitionResult3D
replaceCertifiedRouteAtHandoff3D(
    const ExecutionPlan3D& current, const ExecutionRouteTransitionGuard3D& guard,
    CertifiedRouteSuffix3D successor,
    std::optional<FiniteExecutionState3D> successor_execution) {
  FiniteExecutionPlan3D plan =
      successor_execution.has_value()
          ? testExecutionPlanForCommand(current, successor,
                                        std::move(successor_execution.value()))
          : FiniteExecutionPlan3D{};
  return ::drone_city_nav::replaceCertifiedRouteAtHandoff3D(
      current, guard, std::move(successor), std::move(plan));
}

[[nodiscard, maybe_unused]] ExecutionRouteTransitionResult3D
transferDirectTrackingToCertifiedRoute3D(const ExecutionPlan3D& current,
                                         const std::uint64_t expected_snapshot_version,
                                         CertifiedRouteSuffix3D successor,
                                         FiniteExecutionState3D successor_execution) {
  FiniteExecutionPlan3D plan =
      testExecutionPlanForCommand(current, successor, std::move(successor_execution));
  return ::drone_city_nav::transferDirectTrackingToCertifiedRoute3D(
      current, expected_snapshot_version, std::move(successor), std::move(plan));
}

} // namespace
} // namespace drone_city_nav
