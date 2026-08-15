#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <cstdint>
#include <span>

namespace drone_city_nav::mppi {

enum class StaticRouteHandoffStatus : std::uint8_t {
  kAccepted,
  kInvalidInput,
  kInvalidProjection,
  kExcessiveCrossTrack,
  kAltitudeEnvelopeViolation,
  kRawCollision,
};

struct StaticRouteHandoffResult {
  StaticRouteHandoffStatus status{StaticRouteHandoffStatus::kInvalidInput};
  float cross_track_m{0.0F};
  float minimum_clearance_m{0.0F};
  float critical_exposure_m{0.0F};
  float planning_exposure_m{0.0F};
  bool accepted{false};
};

[[nodiscard]] StaticRouteHandoffResult
validateStaticRouteHandoff(const State& current_state, Control previous_applied_control,
                           std::span<const RouteSample3D> candidate_route,
                           float reference_speed_mps, float maximum_cross_track_m,
                           const BenchmarkConfig& config, const EsdfGrid& grid,
                           std::span<const float> esdf_m);

[[nodiscard]] const char*
staticRouteHandoffStatusName(StaticRouteHandoffStatus status) noexcept;

} // namespace drone_city_nav::mppi
