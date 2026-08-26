#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace drone_city_nav::mppi {

enum class StaticRouteHandoffStatus : std::uint8_t {
  kNotAttempted,
  kAccepted,
  kInvalidInput,
  kInvalidProjection,
  kExcessiveCrossTrack,
  kNoRouteConvergentFiniteHorizon,
  kAltitudeEnvelopeViolation,
  kRawCollision,
};

struct StaticRouteHandoffResult {
  StaticRouteHandoffStatus status{StaticRouteHandoffStatus::kNotAttempted};
  float cross_track_m{0.0F};
  float minimum_clearance_m{0.0F};
  float critical_exposure_m{0.0F};
  float planning_exposure_m{0.0F};
  float terminal_cross_track_m{-1.0F};
  std::size_t arrival_shaping_attempts{0U};
  std::size_t nominal_prefix_control_count{0U};
  bool accepted{false};
};

[[nodiscard]] StaticRouteHandoffResult validateStaticRouteHandoff(
    const State& current_state, Control previous_applied_control,
    std::span<const RouteSample3D> candidate_route, float reference_speed_mps,
    float maximum_cross_track_m, float terminal_cross_track_tolerance_m,
    const BenchmarkConfig& config, const EsdfGrid& grid, std::span<const float> esdf_m);

[[nodiscard]] const char*
staticRouteHandoffStatusName(StaticRouteHandoffStatus status) noexcept;

} // namespace drone_city_nav::mppi
