#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/esdf_grid_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace drone_city_nav {

enum class DynamicHandoffStatus3D : std::uint8_t {
  kNotAttempted,
  kAccepted,
  kInvalidInput,
  kInvalidProjection,
  kExcessiveCrossTrack,
  kNoRouteConvergentFiniteHorizon,
  kAltitudeEnvelopeViolation,
};

struct DynamicHandoffResult3D {
  DynamicHandoffStatus3D status{DynamicHandoffStatus3D::kNotAttempted};
  float cross_track_m{0.0F};
  float minimum_clearance_m{0.0F};
  float critical_exposure_m{0.0F};
  float planning_exposure_m{0.0F};
  float terminal_cross_track_m{-1.0F};
  std::size_t arrival_shaping_attempts{0U};
  std::size_t nominal_prefix_control_count{0U};

  [[nodiscard]] bool accepted() const noexcept {
    return status == DynamicHandoffStatus3D::kAccepted;
  }
};

// The request owns every derived resource needed by a controller adapter.
// Handoff validation is synchronous, but shared ownership prevents an adapter
// from observing a trajectory or ESDF detached from the captured transaction.
struct DynamicHandoffRequest3D {
  MotionState3D current_state{};
  MotionControl3D previous_applied_control{};
  std::shared_ptr<const CompiledTrajectory3D> candidate_trajectory;
  float reference_speed_mps{0.0F};
  float maximum_cross_track_m{0.0F};
  float terminal_cross_track_tolerance_m{0.0F};
  EsdfGrid3D grid{};
  std::shared_ptr<const std::vector<float>> derived_distances_m;
};

using DynamicHandoffValidator3D =
    std::function<DynamicHandoffResult3D(const DynamicHandoffRequest3D&)>;

[[nodiscard]] inline const char*
dynamicHandoffStatus3DName(const DynamicHandoffStatus3D status) noexcept {
  switch (status) {
    case DynamicHandoffStatus3D::kNotAttempted:
      return "not_attempted";
    case DynamicHandoffStatus3D::kAccepted:
      return "accepted";
    case DynamicHandoffStatus3D::kInvalidInput:
      return "invalid_input";
    case DynamicHandoffStatus3D::kInvalidProjection:
      return "invalid_projection";
    case DynamicHandoffStatus3D::kExcessiveCrossTrack:
      return "excessive_cross_track";
    case DynamicHandoffStatus3D::kNoRouteConvergentFiniteHorizon:
      return "no_route_convergent_finite_horizon";
    case DynamicHandoffStatus3D::kAltitudeEnvelopeViolation:
      return "altitude_envelope_violation";
  }
  return "unknown";
}

} // namespace drone_city_nav
