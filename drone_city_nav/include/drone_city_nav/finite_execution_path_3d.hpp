#pragma once

#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

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

[[nodiscard]] FiniteExecutionPathValidation3D
validateCompleteFiniteExecutionPath3D(std::span<const TimedExecutionPathPoint3D> points,
                                      const MotionControl3D& previous_applied_control,
                                      const FiniteExecutionPathWorld3D& world) noexcept;

[[nodiscard]] ValidatedFiniteExecutionPath3D buildValidatedFiniteExecutionPath3D(
    std::span<const MotionState3D> planned_states,
    std::span<const MotionControl3D> planned_controls,
    const MotionControl3D& previous_applied_control,
    const MotionDynamicsConfig3D& dynamics, std::size_t arrival_search_step_controls,
    const FiniteMotionHorizonConfig3D& finite_horizon_config,
    const FiniteExecutionPathWorld3D& world,
    FiniteExecutionPathCandidateValidator3D candidate_validator = {});

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
