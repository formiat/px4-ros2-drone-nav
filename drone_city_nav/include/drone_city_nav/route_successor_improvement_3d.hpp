#pragma once

#include "drone_city_nav/compiled_trajectory_3d.hpp"

#include <cstdint>
#include <string_view>

namespace drone_city_nav {

// Both thresholds are mandatory. This prevents small absolute changes on long
// routes and large percentage changes on already-short remainders from causing
// route churn.
struct RouteSuccessorImprovementConfig3D {
  double minimum_absolute_improvement_s{1.0};
  double minimum_relative_improvement{0.05};

  [[nodiscard]] bool valid() const noexcept;
};

enum class RouteSuccessorImprovementStatus3D : std::uint8_t {
  kNotAssessed,
  kAccepted,
  kInvalidConfig,
  kInvalidTrajectory,
  kIncomparableEndpointSemantics,
  kInvalidStation,
  kInsufficientAbsoluteImprovement,
  kInsufficientRelativeImprovement,
};

struct RouteSuccessorImprovementAssessment3D {
  RouteSuccessorImprovementStatus3D status{
      RouteSuccessorImprovementStatus3D::kNotAssessed};
  double resident_remaining_time_s{0.0};
  double candidate_remaining_time_s{0.0};
  double absolute_improvement_s{0.0};
  double relative_improvement{0.0};

  [[nodiscard]] bool accepted() const noexcept;
};

// Compares two sealed trajectories at their independently projected current
// stations. Mission/intent comparability is decided by the caller; this pure
// execution policy additionally requires identical endpoint semantics.
[[nodiscard]] RouteSuccessorImprovementAssessment3D assessRouteSuccessorImprovement3D(
    const CompiledTrajectory3D& resident, double resident_station_m,
    const CompiledTrajectory3D& candidate, double candidate_station_m,
    const RouteSuccessorImprovementConfig3D& config) noexcept;

[[nodiscard]] std::string_view routeSuccessorImprovementStatus3DName(
    RouteSuccessorImprovementStatus3D status) noexcept;

} // namespace drone_city_nav
