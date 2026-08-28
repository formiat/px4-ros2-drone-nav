#pragma once

#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace drone_city_nav {

// Shared kinematic limits for strategic search, route compilation, and
// controller-facing speed profiles. Horizontal and vertical limits remain
// physically distinct, but both contribute to one three-dimensional time
// objective.
struct FlightTimeModel3D {
  double maximum_horizontal_speed_mps{5.0};
  double maximum_vertical_speed_mps{3.0};
  double maximum_translational_speed_mps{std::numeric_limits<double>::max()};
  double maximum_horizontal_acceleration_mps2{4.0};
  double maximum_vertical_acceleration_mps2{4.0};
  double maximum_control_jerk_mps3{12.0};
  double maximum_yaw_acceleration_radps2{2.0};
  double maximum_yaw_rate_radps{1.5};

  [[nodiscard]] bool valid() const noexcept;
};

struct FlightPathTimeProfile3D {
  bool valid{false};
  double travel_time_s{0.0};
  double translation_time_s{0.0};
  double stationary_turn_time_s{0.0};
  std::vector<double> reference_speeds_mps;
};

// Admissible anisotropic travel-time lower bound used by the persistent graph.
// The complete path profile below adds acceleration, jerk, and stop-turn time.
[[nodiscard]] double
minimumFlightTranslationTime3D(const Point3& first, const Point3& second,
                               const FlightTimeModel3D& model) noexcept;

// Returns true when the three-dimensional tangent change requires the same
// full translational stop used by route compilation. The threshold is the
// minimum normalized tangent alignment that may remain continuous.
[[nodiscard]] bool
requiresFlightStopAndTurn3D(const Vec3& incoming, const Vec3& outgoing,
                            double minimum_continuous_alignment) noexcept;

// Incremental delay, relative to translation at the directional speed limit,
// for one jerk-limited transition between rest and that speed. Acceleration
// and braking are symmetric under the shared zero-boundary-acceleration model.
[[nodiscard]] double
estimatedFlightRestTransitionDelay3D(const Vec3& tangent,
                                     const FlightTimeModel3D& model) noexcept;

// Incremental strategic cost of a mandatory stop-and-turn: jerk-limited
// braking, physically bounded stationary yaw, and jerk-limited restart. The
// full path parameterizer below remains authoritative for the final ETA.
[[nodiscard]] double
estimatedFlightStopAndTurnDelay3D(const Vec3& incoming, const Vec3& outgoing,
                                  const FlightTimeModel3D& model) noexcept;

// Time-parameterizes one spatial path. speed_limits_mps and stop_turn_flags
// are point-aligned; a nonzero flag requires zero translational speed and a
// physically bounded stationary yaw turn at that point.
[[nodiscard]] FlightPathTimeProfile3D parameterizeFlightPathTime3D(
    std::span<const Point3> points, std::span<const double> speed_limits_mps,
    std::span<const std::uint8_t> stop_turn_flags, const Vec3& initial_velocity,
    bool terminal_stop, const FlightTimeModel3D& model);

} // namespace drone_city_nav
