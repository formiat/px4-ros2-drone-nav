#include "drone_city_nav/route_successor_improvement_3d.hpp"

#include "drone_city_nav/compiled_trajectory_views_3d.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace drone_city_nav {

bool RouteSuccessorImprovementConfig3D::valid() const noexcept {
  return std::isfinite(minimum_absolute_improvement_s) &&
         minimum_absolute_improvement_s > 0.0 &&
         std::isfinite(minimum_relative_improvement) &&
         minimum_relative_improvement > 0.0 && minimum_relative_improvement < 1.0;
}

bool RouteSuccessorImprovementAssessment3D::accepted() const noexcept {
  return status == RouteSuccessorImprovementStatus3D::kAccepted;
}

RouteSuccessorImprovementAssessment3D assessRouteSuccessorImprovement3D(
    const CompiledTrajectory3D& resident, const double resident_station_m,
    const CompiledTrajectory3D& candidate, const double candidate_station_m,
    const RouteSuccessorImprovementConfig3D& config) noexcept {
  using Status = RouteSuccessorImprovementStatus3D;
  if (!config.valid()) {
    return {.status = Status::kInvalidConfig};
  }
  if (!compiledTrajectoryResourcesValid3D(resident) ||
      !compiledTrajectoryResourcesValid3D(candidate) ||
      resident.compiled_trajectory_revision == 0U ||
      candidate.compiled_trajectory_revision == 0U ||
      compiledTrajectoryRevision3D(resident) != resident.compiled_trajectory_revision ||
      compiledTrajectoryRevision3D(candidate) !=
          candidate.compiled_trajectory_revision) {
    return {.status = Status::kInvalidTrajectory};
  }
  if (resident.endpoint_semantics != candidate.endpoint_semantics) {
    return {.status = Status::kIncomparableEndpointSemantics};
  }
  const std::optional<double> resident_remaining =
      remainingCompiledTrajectoryTime3D(resident, resident_station_m);
  const std::optional<double> candidate_remaining =
      remainingCompiledTrajectoryTime3D(candidate, candidate_station_m);
  if (!resident_remaining.has_value() || !candidate_remaining.has_value()) {
    return {.status = Status::kInvalidStation};
  }
  const double resident_remaining_time_s = *resident_remaining;
  const double candidate_remaining_time_s = *candidate_remaining;
  if (resident_remaining_time_s <= 0.0 || candidate_remaining_time_s < 0.0) {
    return {.status = Status::kInvalidStation};
  }

  RouteSuccessorImprovementAssessment3D result{
      .status = Status::kInsufficientAbsoluteImprovement,
      .resident_remaining_time_s = resident_remaining_time_s,
      .candidate_remaining_time_s = candidate_remaining_time_s,
      .absolute_improvement_s = resident_remaining_time_s - candidate_remaining_time_s,
  };
  result.relative_improvement =
      result.absolute_improvement_s / result.resident_remaining_time_s;
  const double tolerance = 1.0e-9 * std::max({1.0, result.resident_remaining_time_s,
                                              result.candidate_remaining_time_s});
  if (result.absolute_improvement_s <= tolerance ||
      result.absolute_improvement_s + tolerance <
          config.minimum_absolute_improvement_s) {
    return result;
  }
  if (result.relative_improvement + 1.0e-9 < config.minimum_relative_improvement) {
    result.status = Status::kInsufficientRelativeImprovement;
    return result;
  }
  result.status = Status::kAccepted;
  return result;
}

std::string_view routeSuccessorImprovementStatus3DName(
    const RouteSuccessorImprovementStatus3D status) noexcept {
  switch (status) {
    case RouteSuccessorImprovementStatus3D::kNotAssessed:
      return "not_assessed";
    case RouteSuccessorImprovementStatus3D::kAccepted:
      return "accepted";
    case RouteSuccessorImprovementStatus3D::kInvalidConfig:
      return "invalid_config";
    case RouteSuccessorImprovementStatus3D::kInvalidTrajectory:
      return "invalid_trajectory";
    case RouteSuccessorImprovementStatus3D::kIncomparableEndpointSemantics:
      return "incomparable_endpoint_semantics";
    case RouteSuccessorImprovementStatus3D::kInvalidStation:
      return "invalid_station";
    case RouteSuccessorImprovementStatus3D::kInsufficientAbsoluteImprovement:
      return "insufficient_absolute_improvement";
    case RouteSuccessorImprovementStatus3D::kInsufficientRelativeImprovement:
      return "insufficient_relative_improvement";
  }
  return "unknown";
}

} // namespace drone_city_nav
