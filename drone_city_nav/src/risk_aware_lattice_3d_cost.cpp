#include "risk_aware_lattice_3d_cost.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav::detail {
namespace {

[[nodiscard]] double horizontalAngle(const Vec3& first, const Vec3& second) noexcept {
  const double first_norm = std::hypot(first.x, first.y);
  const double second_norm = std::hypot(second.x, second.y);
  if (!(first_norm > 1.0e-9) || !(second_norm > 1.0e-9)) {
    return 0.0;
  }
  const double cosine =
      std::clamp((first.x * second.x + first.y * second.y) / (first_norm * second_norm),
                 -1.0, 1.0);
  return std::acos(cosine);
}

[[nodiscard]] double verticalFlightPathAngle(const Vec3& direction) noexcept {
  return std::atan2(direction.z, std::hypot(direction.x, direction.y));
}

[[nodiscard]] bool directionValid(const Vec3& direction) noexcept {
  return std::hypot(std::hypot(direction.x, direction.y), direction.z) > 1.0e-9;
}

} // namespace

Vec3 lattice3DUnitDirection(const Point3& first, const Point3& second) noexcept {
  const double length = distance3D(first, second);
  return length > 1.0e-9
             ? Vec3{(second.x - first.x) / length, (second.y - first.y) / length,
                    (second.z - first.z) / length}
             : Vec3{};
}

Lattice3DCostMetrics evaluateLattice3DEdgeCost(
    const Point3& first, const Point3& second, const Vec3& incoming_direction,
    const Vec3& preferred_direction, const Lattice3DEdgeEvaluation& exposure,
    const RiskAwareLattice3DConfig& config, const bool charge_shape_turn) noexcept {
  Lattice3DCostMetrics result;
  result.route_length_m = distance3D(first, second);
  const double horizontal_m = std::hypot(second.x - first.x, second.y - first.y);
  const double vertical_m = std::abs(second.z - first.z);
  const double horizontal_time_s =
      horizontal_m / std::max(1.0e-6, config.nominal_horizontal_speed_mps);
  result.vertical_alignment_time_s =
      vertical_m / std::max(1.0e-6, config.nominal_vertical_speed_mps);
  result.travel_time_s = std::max(horizontal_time_s, result.vertical_alignment_time_s);
  result.planning_exposure_m = exposure.planning_exposure_m;
  result.critical_exposure_m = exposure.critical_exposure_m;

  const Vec3 outgoing = lattice3DUnitDirection(first, second);
  if (charge_shape_turn && directionValid(incoming_direction) &&
      directionValid(outgoing)) {
    const double horizontal_turn = config.route_shape_turn_cost_per_rad *
                                   horizontalAngle(incoming_direction, outgoing);
    const double vertical_turn = config.route_shape_vertical_turn_cost_per_rad *
                                 std::abs(verticalFlightPathAngle(incoming_direction) -
                                          verticalFlightPathAngle(outgoing));
    result.turn_cost = horizontal_turn + vertical_turn;
  }
  const bool first_maneuver = !directionValid(incoming_direction);
  const double heading_cost = first_maneuver
                                  ? config.heading_bias_cost_per_rad *
                                        horizontalAngle(preferred_direction, outgoing)
                                  : 0.0;
  result.objective_cost =
      result.travel_time_s +
      config.vertical_alignment_cost_weight * result.vertical_alignment_time_s +
      config.planning_exposure_cost_per_m * result.planning_exposure_m +
      config.critical_exposure_cost_per_m * result.critical_exposure_m +
      result.turn_cost + heading_cost;
  return result;
}

void accumulateLattice3DCost(Lattice3DCostMetrics& target,
                             const Lattice3DCostMetrics& addition) noexcept {
  target.objective_cost += addition.objective_cost;
  target.route_length_m += addition.route_length_m;
  target.travel_time_s += addition.travel_time_s;
  target.vertical_alignment_time_s += addition.vertical_alignment_time_s;
  target.planning_exposure_m += addition.planning_exposure_m;
  target.critical_exposure_m += addition.critical_exposure_m;
  target.turn_cost += addition.turn_cost;
}

double lattice3DTravelHeuristic(const Point3& point, const Point3& goal,
                                const RiskAwareLattice3DConfig& config) noexcept {
  const double horizontal_m = std::hypot(goal.x - point.x, goal.y - point.y);
  const double vertical_m = std::abs(goal.z - point.z);
  const double horizontal_time_s =
      horizontal_m / std::max(1.0e-6, config.nominal_horizontal_speed_mps);
  const double vertical_time_s =
      vertical_m / std::max(1.0e-6, config.nominal_vertical_speed_mps);
  return std::max(horizontal_time_s, vertical_time_s) +
         config.vertical_alignment_cost_weight * vertical_time_s;
}

} // namespace drone_city_nav::detail
