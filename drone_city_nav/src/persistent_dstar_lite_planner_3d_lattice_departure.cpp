#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <ranges>
#include <span>
#include <tuple>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

namespace drone_city_nav::detail {

std::vector<PersistentPlannerNode3D>
PlannerLattice3D::admissibleAnchors(const Point3& point, const bool start_anchor,
                                    DepartureDiagnostics3D* const diagnostics) const {
  const PersistentPlannerNode3D center = nearestNode(point);
  const auto radius = static_cast<int>(config_->connector_search_radius_cells);
  std::vector<PersistentPlannerNode3D> candidates;
  const int diameter = 2 * radius + 1;
  const auto diameter_size = static_cast<std::size_t>(diameter);
  candidates.reserve(diameter_size * diameter_size * diameter_size);
  for (int z_offset = -radius; z_offset <= radius; ++z_offset) {
    for (int y_offset = -radius; y_offset <= radius; ++y_offset) {
      for (int x_offset = -radius; x_offset <= radius; ++x_offset) {
        const PersistentPlannerNode3D candidate{
            center.x + x_offset, center.y + y_offset, center.z + z_offset};
        if (nodeInside(candidate)) {
          candidates.push_back(candidate);
        }
      }
    }
  }
  std::ranges::sort(candidates, [&](const PersistentPlannerNode3D& first,
                                    const PersistentPlannerNode3D& second) {
    const double first_distance = distance3D(point, pointFor(first));
    const double second_distance = distance3D(point, pointFor(second));
    return first_distance == second_distance ? nodeLess(first, second)
                                             : first_distance < second_distance;
  });
  std::vector<PersistentPlannerNode3D> anchors;
  if (diagnostics != nullptr) {
    diagnostics->candidate_nodes += candidates.size();
  }
  for (const PersistentPlannerNode3D candidate : candidates) {
    const Point3 anchor = pointFor(candidate);
    if (!nodeValid(candidate)) {
      continue;
    }
    if (diagnostics != nullptr) {
      ++diagnostics->valid_nodes;
    }
    bool connector_valid{false};
    if (start_anchor) {
      const OccupiedCollisionResult3D validation =
          departureSegmentValidation(point, anchor);
      connector_valid = validation.clear();
      if (!connector_valid && diagnostics != nullptr) {
        ++diagnostics->rejected_legs;
        if (!diagnostics->first_leg_failure_available) {
          diagnostics->first_leg_failure = validation;
          diagnostics->first_leg_failure_available = true;
        }
      }
    } else {
      connector_valid = rawSegmentValid(anchor, point);
    }
    if (connector_valid) {
      anchors.push_back(candidate);
    }
  }
  return anchors;
}

std::optional<PersistentPlannerNode3D>
PlannerLattice3D::selectAnchor(const Point3& point, const bool start_anchor) const {
  const std::vector<PersistentPlannerNode3D> anchors =
      admissibleAnchors(point, start_anchor);
  return anchors.empty() ? std::nullopt
                         : std::optional<PersistentPlannerNode3D>{anchors.front()};
}

std::size_t PlannerLattice3D::departureConnectionCount(const Point3& start) const {
  return admissibleAnchors(start, true).size();
}

PlannerLattice3D::DepartureConnection3D PlannerLattice3D::selectDepartureConnection(
    const Point3& start, const std::size_t skipped_connections,
    const std::optional<PersistentPlannerNode3D> preferred) const {
  DepartureConnection3D result;
  const std::vector<PersistentPlannerNode3D> anchors =
      admissibleAnchors(start, true, &result.diagnostics);
  if (!anchors.empty()) {
    if (skipped_connections == 0U && preferred.has_value() &&
        std::ranges::find(anchors, *preferred) != anchors.end()) {
      result.anchor = *preferred;
      return result;
    }
    result.anchor = anchors[skipped_connections % anchors.size()];
    return result;
  }
  if (config_->departure_refinement_subdivisions == 0U) {
    return result;
  }
  // No node in the connector radius is reachable in one segment. Probe a grid
  // finer than the lattice around the vehicle for a free point it can reach,
  // and from which a node is reachable; the nearest such point wins, so the
  // detour stays as short as the geometry allows.
  const auto subdivisions =
      static_cast<int>(config_->departure_refinement_subdivisions);
  const double horizontal_step_m =
      config_->minimum_horizontal_step_m / static_cast<double>(subdivisions);
  const double vertical_step_m =
      config_->minimum_vertical_step_m / static_cast<double>(subdivisions);
  const auto radius = static_cast<int>(config_->connector_search_radius_cells);
  const int span = radius * subdivisions;
  std::vector<Point3> waypoints;
  const auto span_size = static_cast<std::size_t>(2 * span + 1);
  waypoints.reserve(span_size * span_size * span_size);
  for (int z_offset = -span; z_offset <= span; ++z_offset) {
    for (int y_offset = -span; y_offset <= span; ++y_offset) {
      for (int x_offset = -span; x_offset <= span; ++x_offset) {
        if (x_offset == 0 && y_offset == 0 && z_offset == 0) {
          continue;
        }
        const Point3 waypoint{
            start.x + static_cast<double>(x_offset) * horizontal_step_m,
            start.y + static_cast<double>(y_offset) * horizontal_step_m,
            start.z + static_cast<double>(z_offset) * vertical_step_m};
        if (pointInsideFlightEnvelope(waypoint)) {
          waypoints.push_back(waypoint);
        }
      }
    }
  }
  std::ranges::sort(waypoints, [&](const Point3& first, const Point3& second) {
    const double first_distance = distance3D(start, first);
    const double second_distance = distance3D(start, second);
    if (first_distance != second_distance) {
      return first_distance < second_distance;
    }
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  std::size_t probes{0U};
  for (const Point3& waypoint : waypoints) {
    if (probes >= config_->maximum_departure_refinement_probes) {
      break;
    }
    ++probes;
    result.diagnostics.refinement_probes = probes;
    if (!departureSegmentValid(start, waypoint)) {
      continue;
    }
    ++result.diagnostics.refinement_reachable;
    const std::optional<PersistentPlannerNode3D> anchor = selectAnchor(waypoint, false);
    if (anchor.has_value()) {
      result.anchor = anchor;
      result.waypoints = {waypoint};
      return result;
    }
  }
  return result;
}

PlannerLattice3D::GoalConnection3D
PlannerLattice3D::selectGoalConnection(const Point3& goal,
                                       const double tolerance_m) const {
  GoalConnection3D result{.endpoint = goal};
  result.anchor = selectAnchor(goal, false);
  if (result.available() || config_->departure_refinement_subdivisions == 0U ||
      !std::isfinite(tolerance_m) || !(tolerance_m > 0.0)) {
    return result;
  }
  const auto subdivisions =
      static_cast<int>(config_->departure_refinement_subdivisions);
  const double horizontal_step_m =
      config_->minimum_horizontal_step_m / static_cast<double>(subdivisions);
  const double vertical_step_m =
      config_->minimum_vertical_step_m / static_cast<double>(subdivisions);
  const int horizontal_span =
      static_cast<int>(std::ceil(tolerance_m / horizontal_step_m));
  const int vertical_span = static_cast<int>(std::ceil(tolerance_m / vertical_step_m));
  std::vector<Point3> probes;
  for (int z_offset = -vertical_span; z_offset <= vertical_span; ++z_offset) {
    for (int y_offset = -horizontal_span; y_offset <= horizontal_span; ++y_offset) {
      for (int x_offset = -horizontal_span; x_offset <= horizontal_span; ++x_offset) {
        if (x_offset == 0 && y_offset == 0 && z_offset == 0) {
          continue;
        }
        const Point3 probe{goal.x + static_cast<double>(x_offset) * horizontal_step_m,
                           goal.y + static_cast<double>(y_offset) * horizontal_step_m,
                           goal.z + static_cast<double>(z_offset) * vertical_step_m};
        if (distance3D(goal, probe) <= tolerance_m &&
            pointInsideFlightEnvelope(probe)) {
          probes.push_back(probe);
        }
      }
    }
  }
  std::ranges::sort(probes, [&](const Point3& first, const Point3& second) {
    const double first_distance = distance3D(goal, first);
    const double second_distance = distance3D(goal, second);
    if (first_distance != second_distance) {
      return first_distance < second_distance;
    }
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  std::size_t attempts{0U};
  for (const Point3& probe : probes) {
    if (attempts >= config_->maximum_departure_refinement_probes) {
      break;
    }
    ++attempts;
    if (!rawSegmentValid(probe, probe)) {
      continue;
    }
    const std::optional<PersistentPlannerNode3D> anchor = selectAnchor(probe, false);
    if (anchor.has_value()) {
      result.anchor = anchor;
      result.endpoint = probe;
      result.refined = true;
      return result;
    }
  }
  return result;
}

bool PlannerLattice3D::departureReachable(const Point3& start,
                                          const std::span<const Point3> waypoints,
                                          const Point3& target) const {
  // The leg leaving the vehicle carries the departure exemption; every later
  // leg is ordinary raw evidence.
  Point3 previous = start;
  for (std::size_t index = 0U; index < waypoints.size(); ++index) {
    const bool valid = index == 0U ? departureSegmentValid(previous, waypoints[index])
                                   : rawSegmentValid(previous, waypoints[index]);
    if (!valid) {
      return false;
    }
    previous = waypoints[index];
  }
  return waypoints.empty() ? departureSegmentValid(previous, target)
                           : rawSegmentValid(previous, target);
}

} // namespace drone_city_nav::detail
