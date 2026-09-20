#pragma once

#include "drone_city_nav/motion_state_3d.hpp"
#include "drone_city_nav/stopping_capability.hpp"
#include "drone_city_nav/translational_speed_limit_3d.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace drone_city_nav {

// Controller-neutral dynamics used to certify finite execution. A controller
// backend may adapt this value, but route execution owns no backend config.
struct MotionDynamicsConfig3D {
  float dt_s{0.05F};
  float linear_drag_1ps{0.08F};
  float maximum_horizontal_acceleration_mps2{4.0F};
  float maximum_vertical_acceleration_mps2{4.0F};
  float maximum_horizontal_speed_mps{10.0F};
  float maximum_vertical_speed_mps{5.0F};
  // The bound over every direction: the largest entry of the table below.
  float maximum_translational_speed_mps{std::numeric_limits<float>::max()};
  TranslationalSpeedLimitByVerticalShare3D
      translational_speed_limit_by_vertical_share{};
  float maximum_yaw_acceleration_radps2{2.0F};
  float maximum_yaw_rate_radps{1.5F};
  float maximum_control_jerk_mps3{12.0F};
};

// The translational speed the dynamics admit along the direction of the
// velocity (vx, vy, vz): the table entry for its vertical share, never above
// the bound over every direction. One law for the host and device rollouts,
// the overspeed cost and the reference clamp.
[[nodiscard]] DRONE_CITY_NAV_TRANSLATIONAL_SPEED_LIMIT_FN float
translationalSpeedLimitAlong3D(const MotionDynamicsConfig3D& dynamics, const float vx,
                               const float vy, const float vz) noexcept {
  const float speed = std::hypot(std::hypot(vx, vy), vz);
  if (!(speed > 0.0F)) {
    return dynamics.maximum_translational_speed_mps;
  }
  return std::fmin(
      dynamics.maximum_translational_speed_mps,
      translationalSpeedLimitForVerticalShare3D(
          dynamics.translational_speed_limit_by_vertical_share, std::fabs(vz) / speed));
}

[[nodiscard]] inline bool translationalSpeedLimitByVerticalShareValid3D(
    const MotionDynamicsConfig3D& dynamics) noexcept {
  return translationalSpeedLimitByVerticalShareValid3D(
      dynamics.translational_speed_limit_by_vertical_share,
      dynamics.maximum_translational_speed_mps);
}

struct MotionAltitudeEnvelopeConfig3D {
  float minimum_z_m{-std::numeric_limits<float>::max()};
  float maximum_z_m{std::numeric_limits<float>::max()};
  float guaranteed_vertical_deceleration_mps2{4.0F};
  float reaction_latency_s{0.0F};
};

enum class ControlRouteRiskTier3D : std::uint8_t {
  kPreferred = 0,
  kPlanning = 1,
  kCritical = 2,
  kCollision = 3,
};

// Sampled route input shared by finite-path validation and controller adapters.
// It is a derived view and carries no independent route identity.
struct ControlRouteSample3D {
  float x_m{0.0F};
  float y_m{0.0F};
  float z_m{0.0F};
  float tangent_x{0.0F};
  float tangent_y{0.0F};
  float tangent_z{0.0F};
  float station_m{0.0F};
  float reference_speed_mps{0.0F};
  ControlRouteRiskTier3D required_risk_tier{ControlRouteRiskTier3D::kPreferred};
};

// Terminal rest has one definition across the whole execution path: the
// builder that shapes the arrival, the certificate that admits the horizon and
// the wire contract the offboard accepts all read these two tolerances. A
// horizon rests when its last state is slower than the velocity tolerance and
// its last control commands no acceleration beyond the control tolerance.
// Keeping them here is what stops one stage from producing a rest another
// stage refuses.
inline constexpr float kTerminalRestVelocityToleranceMps{1.0e-3F};
inline constexpr float kTerminalRestControlToleranceMps2{1.0e-6F};

struct FiniteMotionHorizonConfig3D {
  float terminal_velocity_tolerance_mps{kTerminalRestVelocityToleranceMps};
  // Where the heading turns to while an arrival brings the vehicle to rest and
  // keeps it there, for a vehicle whose obstacle sensor looks forward. Without
  // it the arrival brings the yaw rate to rest with everything else: a horizon
  // shortened to nothing, because its motion is one the vehicle does not face,
  // then held the very heading that kept it from moving (30 s at a time, the
  // heading unchanged to a degree, r530 to r533 and the probes between them).
  std::optional<float> rest_gaze_heading_rad;
  StoppingCapability stopping_capability{
      .maximum_commanded_horizontal_deceleration_mps2 =
          std::numeric_limits<double>::max(),
      .guaranteed_horizontal_deceleration_mps2 = std::numeric_limits<double>::max(),
      .guaranteed_vertical_deceleration_mps2 = std::numeric_limits<double>::max(),
      .reaction_latency_s = 0.0,
  };
};

struct FiniteMotionHorizon3D {
  std::vector<MotionState3D> states;
  std::vector<MotionControl3D> controls;
  std::size_t nominal_prefix_control_count{0U};
  std::size_t arrival_control_count{0U};
};

struct RouteConvergentFiniteMotionHorizon3D {
  std::optional<FiniteMotionHorizon3D> horizon;
  std::size_t arrival_shaping_attempts{0U};
  std::size_t nominal_prefix_control_count{0U};
  float closest_terminal_cross_track_m{-1.0F};

  [[nodiscard]] bool accepted() const noexcept {
    return horizon.has_value();
  }
};

} // namespace drone_city_nav
