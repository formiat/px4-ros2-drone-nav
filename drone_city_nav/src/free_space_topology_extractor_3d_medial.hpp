#pragma once

#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/free_space_topology_extractor_3d.hpp"
#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <numbers>
#include <numeric>
#include <queue>
#include <ranges>
#include <set>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <utility>
#include <vector>

#include "free_space_topology_extractor_3d_geometry.hpp"
#include "free_space_topology_extractor_3d_graph.hpp"

namespace drone_city_nav::topology_extractor_detail {

// The extraction's working types and the medial axis: the portals of a
// constrained component, its medial tree, the tree's compression into
// segments and the centreline of each.

constexpr double kEpsilon{1.0e-9};

class ExtractionCancelled final : public std::runtime_error {
public:
  ExtractionCancelled()
      : std::runtime_error{"topology extraction cancelled"} {
  }
};

inline void throwIfCancelled(const std::stop_token stop_token) {
  if (stop_token.stop_requested()) {
    throw ExtractionCancelled{};
  }
}

struct Component {
  std::uint32_t label{0U};
  std::size_t minimum_cell{0U};
  std::vector<std::size_t> cells;
  std::size_t representative_cell{0U};
  double maximum_clearance_m{0.0};
};

struct PortalPatch {
  std::uint32_t open_region_label{0U};
  std::vector<std::size_t> cells;
};

struct ProjectedPoint {
  double u{0.0};
  double v{0.0};
};

struct SegmentDraft {
  std::size_t first_cell{0U};
  std::size_t second_cell{0U};
  std::vector<std::size_t> cells;
};

class ClearanceView {
public:
  ClearanceView(const std::span<const float> center_distances_m,
                const double voxel_half_diagonal_m) noexcept
      : center_distances_m_{center_distances_m},
        voxel_half_diagonal_m_{voxel_half_diagonal_m} {
  }

  [[nodiscard]] std::size_t size() const noexcept {
    return center_distances_m_.size();
  }

  [[nodiscard]] float operator[](const std::size_t index) const noexcept {
    const float center_distance_m = center_distances_m_[index];
    if (!std::isfinite(center_distance_m)) {
      return center_distance_m;
    }
    return static_cast<float>(
        std::max(0.0, static_cast<double>(center_distance_m) - voxel_half_diagonal_m_));
  }

private:
  std::span<const float> center_distances_m_;
  double voxel_half_diagonal_m_{0.0};
};

[[nodiscard]] inline std::pair<Vec3, Vec3> portalAxes(const Vec3& normal) noexcept {
  const Vec3 reference =
      std::abs(normal.z) < 0.9 ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0};
  Vec3 u = normalized(cross(reference, normal));
  if (std::hypot(std::hypot(u.x, u.y), u.z) <= kEpsilon) {
    u = Vec3{0.0, 1.0, 0.0};
  }
  return {u, normalized(cross(normal, u))};
}

[[nodiscard]] inline double hullCross(const ProjectedPoint& origin,
                                      const ProjectedPoint& first,
                                      const ProjectedPoint& second) noexcept {
  return (first.u - origin.u) * (second.v - origin.v) -
         (first.v - origin.v) * (second.u - origin.u);
}

template<typename Occupancy>
[[nodiscard]] std::vector<Point3>
portalPolygon(const Point3& center, const Vec3& u_axis, const Vec3& v_axis,
              const std::vector<GridIndex3D>& patch, const Occupancy& occupancy) {
  std::vector<ProjectedPoint> points;
  points.reserve(patch.size());
  for (const GridIndex3D cell : patch) {
    const Point3 point = occupancy.cellCenter(cell);
    const Vec3 relative{point.x - center.x, point.y - center.y, point.z - center.z};
    points.push_back(
        ProjectedPoint{.u = dot(relative, u_axis), .v = dot(relative, v_axis)});
  }
  std::ranges::sort(points,
                    [](const ProjectedPoint& first, const ProjectedPoint& second) {
                      return std::tie(first.u, first.v) < std::tie(second.u, second.v);
                    });
  points.erase(
      std::unique(points.begin(), points.end(),
                  [](const ProjectedPoint& first, const ProjectedPoint& second) {
                    return std::abs(first.u - second.u) <= 1.0e-6 &&
                           std::abs(first.v - second.v) <= 1.0e-6;
                  }),
      points.end());
  std::vector<ProjectedPoint> hull;
  if (points.size() >= 3U) {
    hull.resize(points.size() * 2U);
    std::size_t size = 0U;
    for (const ProjectedPoint& point : points) {
      while (size >= 2U && hullCross(hull[size - 2U], hull[size - 1U], point) <= 0.0) {
        --size;
      }
      hull[size++] = point;
    }
    const std::size_t lower_size = size;
    for (auto iterator = points.rbegin() + 1; iterator != points.rend(); ++iterator) {
      while (size > lower_size &&
             hullCross(hull[size - 2U], hull[size - 1U], *iterator) <= 0.0) {
        --size;
      }
      hull[size++] = *iterator;
    }
    if (size > 1U) {
      --size;
    }
    hull.resize(size);
  }
  if (hull.size() < 3U) {
    const double half = 0.5 * occupancy.bounds().resolution_m;
    hull = {{-half, -half}, {half, -half}, {half, half}, {-half, half}};
  }
  std::vector<Point3> polygon;
  polygon.reserve(hull.size());
  for (const ProjectedPoint& point : hull) {
    polygon.push_back(translated(center, u_axis, point.u, v_axis, point.v));
  }
  return polygon;
}

template<typename Occupancy>
[[nodiscard]] PassagePortal
makePortal(const Occupancy& occupancy, const Component& constrained_component,
           const PortalPatch& patch, const FreeSpaceRegionId& region_id,
           const PassagePortalId& portal_id, const ClearanceView& clearance,
           const std::vector<std::uint32_t>& open_labels) {
  const GridBounds3D& bounds = occupancy.bounds();
  Point3 center{};
  Vec3 normal_sum{};
  double minimum_clearance_m = std::numeric_limits<double>::infinity();
  double maximum_clearance_m = 0.0;
  double clearance_sum_m = 0.0;
  std::vector<GridIndex3D> surface_voxels;
  surface_voxels.reserve(patch.cells.size());
  std::size_t anchor_cell = patch.cells.front();
  for (const std::size_t linear : patch.cells) {
    const GridIndex3D cell = cellFor(bounds, linear);
    const Point3 point = occupancy.cellCenter(cell);
    center.x += point.x;
    center.y += point.y;
    center.z += point.z;
    surface_voxels.push_back(cell);
    const double cell_clearance_m = static_cast<double>(clearance[linear]);
    minimum_clearance_m = std::min(minimum_clearance_m, cell_clearance_m);
    maximum_clearance_m = std::max(maximum_clearance_m, cell_clearance_m);
    clearance_sum_m += cell_clearance_m;
    if (clearance[linear] > clearance[anchor_cell]) {
      anchor_cell = linear;
    }
    for (const GridIndex3D direction : neighbors6()) {
      const GridIndex3D neighbor = offset(cell, direction);
      if (!contains(bounds, neighbor) ||
          open_labels[linearIndex(bounds, neighbor)] != patch.open_region_label) {
        continue;
      }
      normal_sum.x += static_cast<double>(direction.x);
      normal_sum.y += static_cast<double>(direction.y);
      normal_sum.z += static_cast<double>(direction.z);
    }
  }
  const double inverse_count = 1.0 / static_cast<double>(patch.cells.size());
  center.x *= inverse_count;
  center.y *= inverse_count;
  center.z *= inverse_count;
  Vec3 normal = normalized(normal_sum);
  if (std::hypot(std::hypot(normal.x, normal.y), normal.z) <= kEpsilon) {
    const Point3 component_center = occupancy.cellCenter(
        cellFor(bounds, constrained_component.representative_cell));
    normal =
        normalized(Vec3{center.x - component_center.x, center.y - component_center.y,
                        center.z - component_center.z});
  }
  const auto [u_axis, v_axis] = portalAxes(normal);
  return PassagePortal{
      .id = portal_id,
      .region_id = region_id,
      .center = occupancy.cellCenter(cellFor(bounds, anchor_cell)),
      .outward_normal = normal,
      .opening_polygon =
          portalPolygon(center, u_axis, v_axis, surface_voxels, occupancy),
      .surface_voxels = std::move(surface_voxels),
      .traversable_anchors = {occupancy.cellCenter(cellFor(bounds, anchor_cell))},
      .local_u_axis = u_axis,
      .local_v_axis = v_axis,
      .minimum_clearance_m = minimum_clearance_m,
      .mean_clearance_m = clearance_sum_m * inverse_count,
      .maximum_clearance_m = maximum_clearance_m,
  };
}

[[nodiscard]] inline double neighborStepM(const GridIndex3D direction,
                                          const double resolution_m) noexcept {
  return std::sqrt(static_cast<double>(direction.x * direction.x +
                                       direction.y * direction.y +
                                       direction.z * direction.z)) *
         resolution_m;
}

[[nodiscard]] inline std::map<std::size_t, std::set<std::size_t>>
buildMedialTree(const GridBounds3D& bounds, const Component& component,
                const std::vector<std::size_t>& anchors, const ClearanceView& clearance,
                const std::vector<std::uint32_t>& constrained_labels,
                const FreeSpaceTopologyExtractorConfig& config,
                const std::stop_token stop_token) {
  std::map<std::size_t, std::set<std::size_t>> tree;
  if (anchors.size() < 2U) {
    return tree;
  }
  const std::size_t count = clearance.size();
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> distances(count, infinity);
  std::vector<std::size_t> predecessors(count, count);
  using QueueEntry = std::pair<double, std::size_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> pending;
  distances[anchors.front()] = 0.0;
  pending.emplace(0.0, anchors.front());
  std::set<std::size_t> remaining(anchors.begin() + 1, anchors.end());
  std::size_t visited_queue_entries = 0U;
  while (!pending.empty() && !remaining.empty()) {
    if (visited_queue_entries++ % 4096U == 0U) {
      throwIfCancelled(stop_token);
    }
    const auto [distance, current] = pending.top();
    pending.pop();
    if (distance > distances[current] + 1.0e-9) {
      continue;
    }
    remaining.erase(current);
    const GridIndex3D current_cell = cellFor(bounds, current);
    for (const GridIndex3D direction : neighbors26()) {
      const GridIndex3D neighbor = offset(current_cell, direction);
      if (!contains(bounds, neighbor)) {
        continue;
      }
      const std::size_t neighbor_linear = linearIndex(bounds, neighbor);
      if (constrained_labels[neighbor_linear] != component.label) {
        continue;
      }
      const double local_clearance_m =
          std::max(0.05, static_cast<double>(clearance[neighbor_linear]));
      const double edge_cost =
          neighborStepM(direction, bounds.resolution_m) *
          (1.0 + config.medial_clearance_weight / local_clearance_m);
      const double candidate = distance + edge_cost;
      if (candidate + 1.0e-9 >= distances[neighbor_linear]) {
        continue;
      }
      distances[neighbor_linear] = candidate;
      predecessors[neighbor_linear] = current;
      pending.emplace(candidate, neighbor_linear);
    }
  }
  for (auto anchor = anchors.begin() + 1; anchor != anchors.end(); ++anchor) {
    if (predecessors[*anchor] == count) {
      return {};
    }
    std::size_t current = *anchor;
    while (current != anchors.front()) {
      const std::size_t previous = predecessors[current];
      if (previous == count) {
        return {};
      }
      tree[current].insert(previous);
      tree[previous].insert(current);
      current = previous;
    }
  }
  return tree;
}

[[nodiscard]] inline std::vector<SegmentDraft>
compressMedialTree(const std::map<std::size_t, std::set<std::size_t>>& tree,
                   const std::set<std::size_t>& portal_anchors) {
  std::set<std::size_t> significant;
  for (const auto& [cell, neighbors] : tree) {
    if (neighbors.size() != 2U || portal_anchors.contains(cell)) {
      significant.insert(cell);
    }
  }
  std::set<std::pair<std::size_t, std::size_t>> visited_edges;
  std::vector<SegmentDraft> drafts;
  for (const std::size_t start : significant) {
    for (const std::size_t neighbor : tree.at(start)) {
      const auto first_edge = std::minmax(start, neighbor);
      if (visited_edges.contains(first_edge)) {
        continue;
      }
      SegmentDraft draft{.first_cell = start, .cells = {start}};
      std::size_t previous = start;
      std::size_t current = neighbor;
      visited_edges.insert(first_edge);
      while (true) {
        draft.cells.push_back(current);
        if (significant.contains(current)) {
          draft.second_cell = current;
          break;
        }
        const std::set<std::size_t>& neighbors = tree.at(current);
        const auto next = std::ranges::find_if(
            neighbors, [previous](const std::size_t cell) { return cell != previous; });
        if (next == neighbors.end()) {
          draft.second_cell = current;
          break;
        }
        const std::size_t next_cell = *next;
        visited_edges.insert(std::minmax(current, next_cell));
        previous = current;
        current = next_cell;
      }
      if (draft.cells.size() >= 2U) {
        drafts.push_back(std::move(draft));
      }
    }
  }
  std::ranges::sort(drafts, [](const SegmentDraft& first, const SegmentDraft& second) {
    return std::tie(first.first_cell, first.second_cell) <
           std::tie(second.first_cell, second.second_cell);
  });
  return drafts;
}

template<typename Occupancy>
[[nodiscard]] std::vector<RouteSample3D>
makeCenterline(const Occupancy& occupancy, const std::vector<std::size_t>& cells,
               const double speed_limit_mps) {
  std::vector<RouteSample3D> centerline;
  centerline.reserve(cells.size());
  double station_m = 0.0;
  for (std::size_t index = 0U; index < cells.size(); ++index) {
    const Point3 point =
        occupancy.cellCenter(cellFor(occupancy.bounds(), cells[index]));
    if (index > 0U) {
      station_m += distance3D(centerline.back().position, point);
    }
    centerline.push_back(RouteSample3D{.position = point,
                                       .station_m = station_m,
                                       .reference_speed_mps = speed_limit_mps});
  }
  for (std::size_t index = 0U; index < centerline.size(); ++index) {
    const Point3& first = centerline[index == 0U ? 0U : index - 1U].position;
    const Point3& second =
        centerline[index + 1U < centerline.size() ? index + 1U : index].position;
    centerline[index].tangent =
        normalized(Vec3{second.x - first.x, second.y - first.y, second.z - first.z});
  }
  return centerline;
}

template<typename Occupancy>
[[nodiscard]] bool rawSafeCenterline(const Occupancy& occupancy,
                                     const std::vector<RouteSample3D>& centerline,
                                     const SweptFootprintConfig& footprint) noexcept {
  if (centerline.size() < 2U) {
    return false;
  }
  const OccupiedCollisionOracle3D oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = nullptr,
      .static_occupancy = std::addressof(occupancy),
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = footprint,
      .flight_envelope = std::nullopt,
  }};
  for (std::size_t index = 1U; index < centerline.size(); ++index) {
    if (!oracle
             .validateSegment(centerline[index - 1U].position, FootprintBodyAxis{},
                              centerline[index].position, FootprintBodyAxis{})
             .clear()) {
      return false;
    }
  }
  return true;
}

} // namespace drone_city_nav::topology_extractor_detail
