#pragma once

#include "drone_city_nav/motion_state_3d.hpp"
#include "drone_city_nav/stopping_capability.hpp"

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
  float maximum_translational_speed_mps{std::numeric_limits<float>::max()};
  float maximum_yaw_acceleration_radps2{2.0F};
  float maximum_yaw_rate_radps{1.5F};
  float maximum_control_jerk_mps3{12.0F};
};

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

struct FiniteMotionHorizonConfig3D {
  float terminal_velocity_tolerance_mps{1.0e-3F};
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
