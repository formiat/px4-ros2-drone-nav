#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/motion_dynamics_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <span>

namespace drone_city_nav {

struct TimedExecutionPathPoint3D {
  double time_from_start_s{0.0};
  MotionState3D state{};
  // Arrival convention: point zero carries the exact previously applied
  // control; every later point carries the control of the interval ending at
  // that point.
  MotionControl3D control{};
};

enum class FiniteExecutionPathStatus3D {
  kValid,
  kInvalidContract,
  kDynamicsInconsistent,
  kCandidateRejected,
  kNotActive,
  kRouteEndpointExceeded,
  kFlightEnvelopeViolation,
  kDynamicFlightEnvelopeViolation,
  kRawWorldUnavailable,
  kRawCollision,
  kLatestLidarRawCollision,
};

struct FiniteExecutionPathTerminalBoundary3D {
  Point3 endpoint{};
  Vec3 forward{};
  double tolerance_m{0.5};
  double activation_distance_m{std::numeric_limits<double>::infinity()};
  double maximum_cross_track_m{std::numeric_limits<double>::infinity()};
  std::span<const ControlRouteSample3D> activation_route;
  float initial_route_station_m{0.0F};
  float activation_route_station_m{0.0F};
};

struct FiniteExecutionPathValidation3D {
  FiniteExecutionPathStatus3D status{FiniteExecutionPathStatus3D::kInvalidContract};
  // Which dynamics law the rejected step broke; kConsistent whenever the
  // status is not kDynamicsInconsistent.
  MotionDynamicsConsistency3D dynamics_consistency{
      MotionDynamicsConsistency3D::kConsistent};
  // How many leading points are known clear of occupied evidence after this
  // call, including the ones whose sweep the caller had already discharged. A
  // later call on a path sharing that many leading points may discharge them
  // in turn.
  std::size_t physically_validated_point_count{0U};
  std::size_t first_remaining_point_index{0U};
  std::size_t failure_segment_index{0U};
  Point3 failure_point{};
  double remaining_duration_s{0.0};

  [[nodiscard]] bool accepted() const noexcept {
    return status == FiniteExecutionPathStatus3D::kValid;
  }
};

struct FiniteExecutionPathWorld3D {
  const FlightEnvelopeConfig* flight_envelope{nullptr};
  const MotionDynamicsConfig3D* dynamics{nullptr};
  const MotionAltitudeEnvelopeConfig3D* altitude_envelope{nullptr};
  const SweptFootprintConfig* footprint{nullptr};
  const OccupancyGrid3D* static_occupancy{nullptr};
  const ObservedOccupancyGrid3D* observed_occupancy{nullptr};
  const LaunchSupportContact3D* launch_support_contact{nullptr};
  const ProprioceptiveFreeSpaceSeed3D* proprioceptive_free_space_seed{nullptr};
  const OccupancyGrid2D* raw_occupancy{nullptr};
  std::span<const Point3> latest_lidar_obstacle_points;
  std::optional<FiniteExecutionPathTerminalBoundary3D> terminal_boundary;
};

struct RebuiltFiniteExecutionPathContinuation3D {
  std::optional<FiniteMotionHorizon3D> horizon;
  FiniteExecutionPathValidation3D validation{};
  std::size_t source_control_index{0U};
  std::size_t arrival_shaping_attempts{0U};
  std::int64_t valid_until_ns{0};
  bool path_validation_backoff{false};
  bool persistent_raw_path_validation_backoff{false};
  bool latest_lidar_path_validation_backoff{false};

  [[nodiscard]] bool accepted() const noexcept {
    return horizon.has_value() && validation.accepted();
  }

  [[nodiscard]] bool physicalObstacleValidationBackoff() const noexcept {
    return persistent_raw_path_validation_backoff ||
           latest_lidar_path_validation_backoff;
  }
};

struct ValidatedFiniteExecutionPath3D {
  std::optional<FiniteMotionHorizon3D> horizon;
  FiniteExecutionPathValidation3D validation{};
  // Names the first failing input precondition when no candidate was built at
  // all; "none" once the arrival search ran.
  const char* rejected_precondition{"none"};
  std::size_t arrival_shaping_attempts{0U};
  bool path_validation_backoff{false};
  FiniteExecutionPathStatus3D first_failed_validation_status{
      FiniteExecutionPathStatus3D::kValid};
  // The validation of the first rejected candidate: the arrival search then
  // shortens the preserved prefix, so `validation` describes the last attempt
  // while this one names where the longest candidate failed.
  FiniteExecutionPathValidation3D first_failed_validation{
      .status = FiniteExecutionPathStatus3D::kValid};
  bool persistent_raw_path_validation_backoff{false};
  bool latest_lidar_path_validation_backoff{false};
  // The arrival-shaping search ran out of its wall-clock budget before any
  // candidate was accepted.
  bool arrival_shaping_budget_exhausted{false};

  [[nodiscard]] bool accepted() const noexcept {
    return horizon.has_value() && validation.accepted();
  }

  [[nodiscard]] bool physicalObstacleValidationBackoff() const noexcept {
    return persistent_raw_path_validation_backoff ||
           latest_lidar_path_validation_backoff;
  }
};

using FiniteExecutionPathCandidateValidator3D =
    std::function<bool(const FiniteMotionHorizon3D&)>;

// `discharged_leading_point_count` names how many leading points of `points`
// need no swept-footprint check from this call, because the caller has already
// discharged it. Two callers do:
//
//   - the arrival-shaping search, which rebuilds the same horizon with a
//     shorter and shorter nominal prefix; every candidate shares that prefix
//     bit for bit with the longer one before it, and a point once proved clear
//     stays clear for the rest of the search;
//   - the commit-time revalidation, which asks whether a published horizon is
//     still executable; the part of it the vehicle has already flown will not
//     be flown again.
//
// The sweep is by far the most expensive part of this validation, and
// re-running it made horizon assembly and the commit the two largest costs of
// the planning cycle. Everything else is still checked for every point: the
// contract, the dynamics, the flight envelope and the terminal boundary.
[[nodiscard]] FiniteExecutionPathValidation3D validateCompleteFiniteExecutionPath3D(
    std::span<const TimedExecutionPathPoint3D> points,
    const MotionControl3D& previous_applied_control,
    const FiniteExecutionPathWorld3D& world,
    std::size_t discharged_leading_point_count = 0U) noexcept;

// A wall-clock bound on the whole arrival-shaping search. The search rebuilds
// and revalidates the horizon once per shortened prefix, and each rebuild
// solves an arrival profile by damped Newton; on a bad tick that ran the
// planning cycle several times past its period, which delays every horizon the
// vehicle receives. Past the deadline the search stops shortening and returns
// what it has, so a slow tick degrades into a hold instead of into a late
// horizon. Absent means unbounded, which is what the tests and offline tools
// want.
struct FiniteExecutionPathBudget3D {
  std::optional<std::chrono::steady_clock::time_point> deadline;

  [[nodiscard]] bool expired() const noexcept {
    return deadline.has_value() && std::chrono::steady_clock::now() >= *deadline;
  }
};

[[nodiscard]] ValidatedFiniteExecutionPath3D buildValidatedFiniteExecutionPath3D(
    std::span<const MotionState3D> planned_states,
    std::span<const MotionControl3D> planned_controls,
    const MotionControl3D& previous_applied_control,
    const MotionDynamicsConfig3D& dynamics, std::size_t arrival_search_step_controls,
    const FiniteMotionHorizonConfig3D& finite_horizon_config,
    const FiniteExecutionPathWorld3D& world,
    FiniteExecutionPathCandidateValidator3D candidate_validator = {},
    const FiniteExecutionPathBudget3D& budget = {});

[[nodiscard]] FiniteExecutionPathValidation3D
validateFiniteExecutionTrajectoryContinuation3D(
    std::span<const TimedExecutionPathPoint3D> points, std::int64_t valid_from_ns,
    std::int64_t valid_until_ns, std::int64_t now_ns,
    const MotionState3D& current_state, const MotionControl3D& current_control,
    const FiniteExecutionPathWorld3D& world) noexcept;

[[nodiscard]] FiniteExecutionPathValidation3D validateFiniteExecutionPathContinuation3D(
    std::span<const TimedExecutionPathPoint3D> points, std::int64_t valid_from_ns,
    std::int64_t valid_until_ns, std::int64_t now_ns,
    const MotionState3D& current_state, const MotionControl3D& current_control,
    const FiniteExecutionPathWorld3D& world) noexcept;

// The preserved prefix may include an already shaped arrival tail, while the
// nominal prefix retains its original phase classification.
[[nodiscard]] RebuiltFiniteExecutionPathContinuation3D
rebuildFiniteExecutionPathContinuation3D(
    std::span<const TimedExecutionPathPoint3D> points, std::int64_t valid_from_ns,
    std::int64_t valid_until_ns, std::int64_t now_ns,
    const MotionState3D& current_state, const MotionControl3D& current_control,
    std::size_t source_nominal_prefix_control_count,
    std::size_t source_preserved_prefix_control_count,
    const MotionDynamicsConfig3D& dynamics, std::size_t arrival_search_step_controls,
    const FiniteMotionHorizonConfig3D& finite_horizon_config,
    const FiniteExecutionPathWorld3D& world,
    FiniteExecutionPathCandidateValidator3D candidate_validator = {});

[[nodiscard]] const char*
finiteExecutionPathStatus3DName(FiniteExecutionPathStatus3D status) noexcept;

} // namespace drone_city_nav
