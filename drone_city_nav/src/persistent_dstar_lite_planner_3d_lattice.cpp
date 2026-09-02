#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <tuple>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

namespace drone_city_nav::detail {
namespace {

constexpr double kGeometryTolerance{1.0e-9};

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  return std::abs(first.origin_x - second.origin_x) <= kGeometryTolerance &&
         std::abs(first.origin_y - second.origin_y) <= kGeometryTolerance &&
         std::abs(first.origin_z - second.origin_z) <= kGeometryTolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= kGeometryTolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

[[nodiscard]] bool nodeLess(const PersistentPlannerNode3D& first,
                            const PersistentPlannerNode3D& second) noexcept {
  return std::tuple{first.z, first.y, first.x} <
         std::tuple{second.z, second.y, second.x};
}

[[nodiscard]] PersistentPlannerEdge3D
canonicalEdge(const PersistentPlannerNode3D first,
              const PersistentPlannerNode3D second) noexcept {
  if (nodeLess(first, second)) {
    return PersistentPlannerEdge3D{first, second};
  }
  return PersistentPlannerEdge3D{second, first};
}

} // namespace

bool PlannerLattice3D::sameGridGeometry(const GridBounds3D& bounds) const noexcept {
  return width_ > 0 && height_ > 0 && depth_ > 0 && sameBounds(raw_bounds_, bounds);
}

void PlannerLattice3D::configureGridGeometry(const GridBounds3D& bounds) {
  raw_bounds_ = bounds;
  const double width_m = static_cast<double>(bounds.width_cells) * bounds.resolution_m;
  const double height_m =
      static_cast<double>(bounds.height_cells) * bounds.resolution_m;
  const double depth_m = static_cast<double>(bounds.depth_cells) * bounds.resolution_m;
  width_ = std::max(
      1, static_cast<int>(std::floor(width_m / config_->minimum_horizontal_step_m)));
  height_ = std::max(
      1, static_cast<int>(std::floor(height_m / config_->minimum_horizontal_step_m)));
  depth_ = std::max(
      1, static_cast<int>(std::floor(depth_m / config_->minimum_vertical_step_m)));
}

void PlannerLattice3D::installWorld(const PersistentPlannerWorld3D& world) {
  resident_collision_oracle_.emplace(OccupiedCollisionWorld3D{
      .observed_occupancy = world.observed_occupancy.get(),
      .static_occupancy = world.static_occupancy.get(),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = config_->physical_footprint,
      .flight_envelope = config_->flight_envelope,
  });
  departure_collision_oracle_.emplace(OccupiedCollisionWorld3D{
      .observed_occupancy = world.observed_occupancy.get(),
      .static_occupancy = world.static_occupancy.get(),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = world.launch_support_contact
                                    ? std::addressof(*world.launch_support_contact)
                                    : nullptr,
      .footprint = config_->physical_footprint,
      .flight_envelope = config_->flight_envelope,
  });
}

bool PlannerLattice3D::nodeInside(const PersistentPlannerNode3D node) const noexcept {
  return node.x >= 0 && node.y >= 0 && node.z >= 0 && node.x < width_ &&
         node.y < height_ && node.z < depth_;
}

int PlannerLattice3D::maximumScale() const noexcept {
  return 1 << config_->maximum_adaptive_lattice_level;
}

std::size_t
PlannerLattice3D::level(const PersistentPlannerNode3D first,
                        const PersistentPlannerNode3D second) const noexcept {
  const int scale =
      std::max({std::abs(first.x - second.x), std::abs(first.y - second.y),
                std::abs(first.z - second.z)});
  if (scale <= 1) {
    return 0U;
  }
  return static_cast<std::size_t>(std::countr_zero(static_cast<unsigned int>(scale)));
}

Point3 PlannerLattice3D::pointFor(const PersistentPlannerNode3D node) const noexcept {
  return Point3{
      raw_bounds_.origin_x +
          (static_cast<double>(node.x) + 0.5) * config_->minimum_horizontal_step_m,
      raw_bounds_.origin_y +
          (static_cast<double>(node.y) + 0.5) * config_->minimum_horizontal_step_m,
      raw_bounds_.origin_z +
          (static_cast<double>(node.z) + 0.5) * config_->minimum_vertical_step_m,
  };
}

PersistentPlannerNode3D
PlannerLattice3D::nearestNode(const Point3& point) const noexcept {
  const auto clamp_index = [](const double coordinate, const double origin,
                              const double step, const int size) {
    const int index = static_cast<int>(std::floor((coordinate - origin) / step));
    return std::clamp(index, 0, size - 1);
  };
  return PersistentPlannerNode3D{
      clamp_index(point.x, raw_bounds_.origin_x, config_->minimum_horizontal_step_m,
                  width_),
      clamp_index(point.y, raw_bounds_.origin_y, config_->minimum_horizontal_step_m,
                  height_),
      clamp_index(point.z, raw_bounds_.origin_z, config_->minimum_vertical_step_m,
                  depth_),
  };
}

std::optional<PersistentPlannerNode3D>
PlannerLattice3D::selectAnchor(const Point3& point, const bool start_anchor) const {
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
  for (const PersistentPlannerNode3D candidate : candidates) {
    const Point3 anchor = pointFor(candidate);
    if (!nodeValid(candidate)) {
      continue;
    }
    const bool connector_valid = start_anchor ? departureSegmentValid(point, anchor)
                                              : rawSegmentValid(anchor, point);
    if (connector_valid) {
      return candidate;
    }
  }
  return std::nullopt;
}

bool PlannerLattice3D::pointInsideFlightEnvelope(const Point3& point) const noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
         evaluateFlightEnvelopeAltitude(point.z, config_->flight_envelope) ==
             FlightEnvelopeStatus::kValid;
}

bool PlannerLattice3D::rawSegmentValid(const Point3& first,
                                       const Point3& second) const {
  // The resident graph represents persistent raw occupancy only. A moving
  // proprioceptive seed and launch support are local execution evidence; if
  // they changed graph edge costs, every pose refresh would invalidate the
  // complete backward search and its edge cache.
  return resident_collision_oracle_.has_value() &&
         resident_collision_oracle_
             ->validateSegment(first, FootprintBodyAxis{}, second, FootprintBodyAxis{})
             .clear();
}

bool PlannerLattice3D::departureSegmentValid(const Point3& first,
                                             const Point3& second) const {
  return departure_collision_oracle_.has_value() &&
         departure_collision_oracle_
             ->validateSegment(first, FootprintBodyAxis{}, second, FootprintBodyAxis{})
             .clear();
}

bool PlannerLattice3D::nodeValid(const PersistentPlannerNode3D node) const {
  return nodeInside(node) && rawSegmentValid(pointFor(node), pointFor(node));
}

std::vector<PersistentPlannerNode3D>
PlannerLattice3D::adjacentNodes(const PersistentPlannerNode3D node) const {
  std::vector<PersistentPlannerNode3D> result;
  result.reserve(26U * (config_->maximum_adaptive_lattice_level + 1U));
  forEachAdjacentNode(node, [&result](const PersistentPlannerNode3D candidate) {
    result.push_back(candidate);
  });
  return result;
}

double
PlannerLattice3D::heuristic(const PersistentPlannerNode3D first,
                            const PersistentPlannerNode3D second) const noexcept {
  // Every horizontal graph edge is axial or diagonal. Its obstacle-free
  // lattice distance is therefore an admissible, consistent lower bound that
  // is strictly tighter than continuous Euclidean distance for non-45-degree
  // missions. Adaptive edges preserve the same per-metre cost.
  const double delta_x_m = static_cast<double>(std::abs(first.x - second.x)) *
                           config_->minimum_horizontal_step_m;
  const double delta_y_m = static_cast<double>(std::abs(first.y - second.y)) *
                           config_->minimum_horizontal_step_m;
  const double horizontal_diagonal_m =
      std::max(delta_x_m, delta_y_m) +
      (std::numbers::sqrt2 - 1.0) * std::min(delta_x_m, delta_y_m);
  const double vertical_m = static_cast<double>(std::abs(first.z - second.z)) *
                            config_->minimum_vertical_step_m;
  const double euclidean_m = std::hypot(std::hypot(delta_x_m, delta_y_m), vertical_m);
  return std::max(
      {horizontal_diagonal_m / config_->time_model.maximum_horizontal_speed_mps,
       vertical_m / config_->time_model.maximum_vertical_speed_mps,
       horizontal_diagonal_m / config_->time_model.maximum_translational_speed_mps,
       euclidean_m / config_->time_model.maximum_translational_speed_mps});
}

double PlannerLattice3D::rawEdgeCost(const PersistentPlannerNode3D first,
                                     const PersistentPlannerNode3D second) {
  ++edge_queries_;
  if (!nodeInside(first) || !nodeInside(second) || first == second) {
    return std::numeric_limits<double>::infinity();
  }
  const std::size_t queried_level = level(first, second);
  if (queried_level > 0U) {
    ++adaptive_edge_queries_;
    maximum_queried_level_ = std::max(maximum_queried_level_, queried_level);
  }
  const PersistentPlannerEdge3D edge = canonicalEdge(first, second);
  if (const auto found = edge_cost_cache_.find(edge); found != edge_cost_cache_.end()) {
    return found->second;
  }
  const Point3 first_point = pointFor(first);
  const Point3 second_point = pointFor(second);
  ++raw_edge_validation_checks_;
  const double cost = rawSegmentValid(first_point, second_point)
                          ? minimumFlightTranslationTime3D(first_point, second_point,
                                                           config_->time_model)
                          : std::numeric_limits<double>::infinity();
  edge_cost_cache_.emplace(edge, cost);
  return cost;
}

void PlannerLattice3D::reset() noexcept {
  raw_bounds_ = {};
  width_ = 0;
  height_ = 0;
  depth_ = 0;
  resetEdgeEvidence();
}

void PlannerLattice3D::resetEdgeEvidence() noexcept {
  edge_cost_cache_.clear();
  resetEdgeStatistics();
}

void PlannerLattice3D::resetEdgeStatistics() noexcept {
  edge_queries_ = 0U;
  raw_edge_validation_checks_ = 0U;
  adaptive_edge_queries_ = 0U;
  maximum_queried_level_ = 0U;
}

const GridBounds3D& PlannerLattice3D::bounds() const noexcept {
  return raw_bounds_;
}

bool PlannerLattice3D::edgeTraversable(const PersistentPlannerNode3D first,
                                       const PersistentPlannerNode3D second) {
  return std::isfinite(rawEdgeCost(first, second));
}

bool PlannerLattice3D::pathTraversable(const std::vector<Point3>& path) const {
  if (path.size() < 2U || !std::ranges::all_of(path, [&](const Point3& point) {
        return pointInsideFlightEnvelope(point);
      })) {
    return false;
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const bool segment_valid =
        index == 1U ? departureSegmentValid(path[index - 1U], path[index])
                    : rawSegmentValid(path[index - 1U], path[index]);
    if (!segment_valid) {
      return false;
    }
  }
  return true;
}

std::size_t PlannerLattice3D::nodeSpan() const noexcept {
  return static_cast<std::size_t>(width_) + static_cast<std::size_t>(height_) +
         static_cast<std::size_t>(depth_);
}

bool PlannerLattice3D::forgetEdgeCost(const PersistentPlannerEdge3D& edge) {
  return edge_cost_cache_.erase(edge) != 0U;
}

bool PlannerLattice3D::configured() const noexcept {
  return width_ > 0 && height_ > 0 && depth_ > 0;
}

std::size_t PlannerLattice3D::edgeQueries() const noexcept {
  return edge_queries_;
}

std::size_t PlannerLattice3D::rawEdgeValidationChecks() const noexcept {
  return raw_edge_validation_checks_;
}

std::size_t PlannerLattice3D::adaptiveEdgeQueries() const noexcept {
  return adaptive_edge_queries_;
}

std::size_t PlannerLattice3D::maximumQueriedLevel() const noexcept {
  return maximum_queried_level_;
}

} // namespace drone_city_nav::detail
