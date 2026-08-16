#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_finite_horizon.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace drone_city_nav::mppi {

struct TimedExecutionPathPoint {
  double time_from_start_s{0.0};
  State state{};
  Control control{};
};

enum class FiniteExecutionPathStatus {
  kValid,
  kInvalidContract,
  kNotActive,
  kRouteEndpointExceeded,
  kFlightEnvelopeViolation,
  kDynamicFlightEnvelopeViolation,
  kRawWorldUnavailable,
  kRawCollision,
  kLatestLidarRawCollision,
};

struct FiniteExecutionPathTerminalBoundary {
  Point3 endpoint{};
  Vec3 forward{};
  double tolerance_m{0.5};
  double activation_distance_m{std::numeric_limits<double>::infinity()};
  double maximum_cross_track_m{std::numeric_limits<double>::infinity()};
};

struct FiniteExecutionPathValidation {
  FiniteExecutionPathStatus status{FiniteExecutionPathStatus::kInvalidContract};
  std::size_t first_remaining_point_index{0U};
  std::size_t failure_segment_index{0U};
  Point3 failure_point{};
  double remaining_duration_s{0.0};

  [[nodiscard]] bool accepted() const noexcept {
    return status == FiniteExecutionPathStatus::kValid;
  }
};

struct FiniteExecutionPathWorld {
  const FlightEnvelopeConfig* flight_envelope{nullptr};
  const DynamicsConfig* dynamics{nullptr};
  const AltitudeEnvelopeConfig* altitude_envelope{nullptr};
  const SweptFootprintConfig* footprint{nullptr};
  const OccupancyGrid3D* static_occupancy{nullptr};
  const OccupancyGrid2D* raw_occupancy{nullptr};
  std::span<const Point3> latest_lidar_obstacle_points;
  std::optional<FiniteExecutionPathTerminalBoundary> terminal_boundary;
};

struct RebuiltFiniteExecutionPathContinuation {
  std::optional<FiniteHorizon> horizon;
  FiniteExecutionPathValidation validation{};
  std::size_t source_control_index{0U};
  std::size_t arrival_shaping_attempts{0U};
  std::int64_t valid_until_ns{0};
  bool path_validation_backoff{false};
  bool latest_lidar_path_validation_backoff{false};

  [[nodiscard]] bool accepted() const noexcept {
    return horizon.has_value() && validation.accepted();
  }
};

struct ValidatedFiniteExecutionPath {
  std::optional<FiniteHorizon> horizon;
  FiniteExecutionPathValidation validation{};
  std::size_t arrival_shaping_attempts{0U};
  bool path_validation_backoff{false};
  bool latest_lidar_path_validation_backoff{false};

  [[nodiscard]] bool accepted() const noexcept {
    return horizon.has_value() && validation.accepted();
  }
};

[[nodiscard]] FiniteExecutionPathValidation
validateCompleteFiniteExecutionPath(std::span<const TimedExecutionPathPoint> points,
                                    const Control& previous_applied_control,
                                    const FiniteExecutionPathWorld& world) noexcept;

[[nodiscard]] ValidatedFiniteExecutionPath buildValidatedFiniteExecutionPath(
    std::span<const State> planned_states, std::span<const Control> planned_controls,
    const Control& previous_applied_control, const DynamicsConfig& dynamics,
    std::size_t arrival_search_step_controls,
    const FiniteHorizonConfig& finite_horizon_config,
    const FiniteExecutionPathWorld& world);

[[nodiscard]] FiniteExecutionPathValidation
validateFiniteExecutionTrajectoryContinuation(
    std::span<const TimedExecutionPathPoint> points, std::int64_t valid_from_ns,
    std::int64_t valid_until_ns, std::int64_t now_ns, const State& current_state,
    const Control& current_control, const FiniteExecutionPathWorld& world) noexcept;

[[nodiscard]] FiniteExecutionPathValidation validateFiniteExecutionPathContinuation(
    std::span<const TimedExecutionPathPoint> points, std::int64_t valid_from_ns,
    std::int64_t valid_until_ns, std::int64_t now_ns, const State& current_state,
    const Control& current_control, const FiniteExecutionPathWorld& world) noexcept;

[[nodiscard]] RebuiltFiniteExecutionPathContinuation
rebuildFiniteExecutionPathContinuation(std::span<const TimedExecutionPathPoint> points,
                                       std::int64_t valid_from_ns,
                                       std::int64_t valid_until_ns, std::int64_t now_ns,
                                       const State& current_state,
                                       const Control& current_control,
                                       const DynamicsConfig& dynamics,
                                       std::size_t arrival_search_step_controls,
                                       const FiniteHorizonConfig& finite_horizon_config,
                                       const FiniteExecutionPathWorld& world);

[[nodiscard]] const char*
finiteExecutionPathStatusName(FiniteExecutionPathStatus status) noexcept;

} // namespace drone_city_nav::mppi
