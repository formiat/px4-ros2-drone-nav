#pragma once

#include "drone_city_nav/finite_execution_path_3d.hpp"
#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

namespace drone_city_nav::mppi {

using TimedExecutionPathPoint = drone_city_nav::TimedExecutionPathPoint3D;
using FiniteExecutionPathStatus = drone_city_nav::FiniteExecutionPathStatus3D;
using FiniteExecutionPathTerminalBoundary =
    drone_city_nav::FiniteExecutionPathTerminalBoundary3D;
using FiniteExecutionPathValidation = drone_city_nav::FiniteExecutionPathValidation3D;
using FiniteExecutionPathWorld = drone_city_nav::FiniteExecutionPathWorld3D;
using RebuiltFiniteExecutionPathContinuation =
    drone_city_nav::RebuiltFiniteExecutionPathContinuation3D;
using ValidatedFiniteExecutionPath = drone_city_nav::ValidatedFiniteExecutionPath3D;
using FiniteExecutionPathCandidateValidator =
    drone_city_nav::FiniteExecutionPathCandidateValidator3D;
using FiniteExecutionPathBudget = drone_city_nav::FiniteExecutionPathBudget3D;

[[nodiscard]] inline FiniteExecutionPathValidation validateCompleteFiniteExecutionPath(
    const std::span<const TimedExecutionPathPoint> points,
    const Control& previous_applied_control,
    const FiniteExecutionPathWorld& world) noexcept {
  return validateCompleteFiniteExecutionPath3D(points, previous_applied_control, world);
}

[[nodiscard]] inline ValidatedFiniteExecutionPath buildValidatedFiniteExecutionPath(
    const std::span<const State> planned_states,
    const std::span<const Control> planned_controls,
    const Control& previous_applied_control, const DynamicsConfig& dynamics,
    const std::size_t arrival_search_step_controls,
    const FiniteHorizonConfig& finite_horizon_config,
    const FiniteExecutionPathWorld& world,
    FiniteExecutionPathCandidateValidator candidate_validator = {},
    const FiniteExecutionPathBudget3D& budget = {}) {
  return buildValidatedFiniteExecutionPath3D(
      planned_states, planned_controls, previous_applied_control, dynamics,
      arrival_search_step_controls, finite_horizon_config, world,
      std::move(candidate_validator), budget);
}

[[nodiscard]] inline FiniteExecutionPathValidation
validateFiniteExecutionTrajectoryContinuation(
    const std::span<const TimedExecutionPathPoint> points,
    const std::int64_t valid_from_ns, const std::int64_t valid_until_ns,
    const std::int64_t now_ns, const State& current_state,
    const Control& current_control, const FiniteExecutionPathWorld& world) noexcept {
  return validateFiniteExecutionTrajectoryContinuation3D(
      points, valid_from_ns, valid_until_ns, now_ns, current_state, current_control,
      world);
}

[[nodiscard]] inline FiniteExecutionPathValidation
validateFiniteExecutionPathContinuation(
    const std::span<const TimedExecutionPathPoint> points,
    const std::int64_t valid_from_ns, const std::int64_t valid_until_ns,
    const std::int64_t now_ns, const State& current_state,
    const Control& current_control, const FiniteExecutionPathWorld& world) noexcept {
  return validateFiniteExecutionPathContinuation3D(
      points, valid_from_ns, valid_until_ns, now_ns, current_state, current_control,
      world);
}

[[nodiscard]] inline RebuiltFiniteExecutionPathContinuation
rebuildFiniteExecutionPathContinuation(
    const std::span<const TimedExecutionPathPoint> points,
    const std::int64_t valid_from_ns, const std::int64_t valid_until_ns,
    const std::int64_t now_ns, const State& current_state,
    const Control& current_control,
    const std::size_t source_nominal_prefix_control_count,
    const std::size_t source_preserved_prefix_control_count,
    const DynamicsConfig& dynamics, const std::size_t arrival_search_step_controls,
    const FiniteHorizonConfig& finite_horizon_config,
    const FiniteExecutionPathWorld& world,
    FiniteExecutionPathCandidateValidator candidate_validator = {}) {
  return rebuildFiniteExecutionPathContinuation3D(
      points, valid_from_ns, valid_until_ns, now_ns, current_state, current_control,
      source_nominal_prefix_control_count, source_preserved_prefix_control_count,
      dynamics, arrival_search_step_controls, finite_horizon_config, world,
      std::move(candidate_validator));
}

[[nodiscard]] inline const char*
finiteExecutionPathStatusName(const FiniteExecutionPathStatus status) noexcept {
  return finiteExecutionPathStatus3DName(status);
}

} // namespace drone_city_nav::mppi
