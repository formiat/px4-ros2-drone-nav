#pragma once

#include "drone_city_nav/trajectory.hpp"
#include "drone_city_nav/types.hpp"
#include "drone_city_nav/velocity_control_config.hpp"

#include <limits>
#include <span>

namespace drone_city_nav {

enum class ControlProjectionSmoothingMode {
  kNone,
  kStraight,
  kCurve,
};

struct ControlProjectionSmoothingDiagnostics {
  bool applied{false};
  ControlProjectionSmoothingMode mode{ControlProjectionSmoothingMode::kNone};
  Point2 raw_tangent{};
  double heading_span_rad{std::numeric_limits<double>::quiet_NaN()};
  double max_abs_curvature_1pm{std::numeric_limits<double>::quiet_NaN()};
  double curvature_feedforward_context_scale{1.0};
  double window_start_s_m{std::numeric_limits<double>::quiet_NaN()};
  double window_end_s_m{std::numeric_limits<double>::quiet_NaN()};
};

struct SmoothedControlProjection {
  TrajectoryProjection projection{};
  ControlProjectionSmoothingDiagnostics diagnostics{};
};

[[nodiscard]] const char*
controlProjectionSmoothingModeName(ControlProjectionSmoothingMode mode) noexcept;

[[nodiscard]] SmoothedControlProjection
smoothControlProjection(std::span<const TrajectoryPointSample> samples,
                        const TrajectoryProjection& raw_projection,
                        const VelocityFollowerConfig& config);

} // namespace drone_city_nav
