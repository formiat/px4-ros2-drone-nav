#include "drone_city_nav/free_space_topology_extractor_3d.hpp"

#include "drone_city_nav/distance_field_3d.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <numbers>
#include <numeric>
#include <queue>
#include <ranges>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "free_space_topology_extractor_3d_geometry.hpp"

namespace drone_city_nav {
namespace {

using topology_extractor_detail::cellFor;
using topology_extractor_detail::contains;
using topology_extractor_detail::cross;
using topology_extractor_detail::dot;
using topology_extractor_detail::indexedId;
using topology_extractor_detail::linearIndex;
using topology_extractor_detail::normalized;
using topology_extractor_detail::offset;
using topology_extractor_detail::translated;
using topology_extractor_detail::voxelCount;

constexpr std::uint8_t kBlocked{0U};
constexpr std::uint8_t kConstrained{1U};
constexpr std::uint8_t kOpen{2U};
constexpr double kEpsilon{1.0e-9};

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

[[nodiscard]] bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] const std::vector<GridIndex3D>& neighbors26() {
  static const std::vector<GridIndex3D> directions = [] {
    std::vector<GridIndex3D> result;
    result.reserve(26U);
    for (int z = -1; z <= 1; ++z) {
      for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
          if (x == 0 && y == 0 && z == 0) {
            continue;
          }
          result.push_back(GridIndex3D{x, y, z});
        }
      }
    }
    return result;
  }();
  return directions;
}

[[nodiscard]] constexpr std::array<GridIndex3D, 6U> neighbors6() noexcept {
  return {GridIndex3D{-1, 0, 0}, GridIndex3D{1, 0, 0},  GridIndex3D{0, -1, 0},
          GridIndex3D{0, 1, 0},  GridIndex3D{0, 0, -1}, GridIndex3D{0, 0, 1}};
}

[[nodiscard]] const std::vector<GridIndex3D>& antipodalNeighborDirections() {
  static const std::vector<GridIndex3D> directions = [] {
    std::vector<GridIndex3D> result;
    result.reserve(13U);
    for (const GridIndex3D direction : neighbors26()) {
      if (direction.z > 0 ||
          (direction.z == 0 &&
           (direction.y > 0 || (direction.y == 0 && direction.x > 0)))) {
        result.push_back(direction);
      }
    }
    return result;
  }();
  return directions;
}

[[nodiscard]] bool
footprintInsideBounds(const Point3& center, const GridBounds3D& bounds,
                      const SweptFootprintConfig& footprint) noexcept {
  const double maximum_x =
      bounds.origin_x + bounds.resolution_m * static_cast<double>(bounds.width_cells);
  const double maximum_y =
      bounds.origin_y + bounds.resolution_m * static_cast<double>(bounds.height_cells);
  const double maximum_z =
      bounds.origin_z + bounds.resolution_m * static_cast<double>(bounds.depth_cells);
  return center.x - footprint.radius_m >= bounds.origin_x &&
         center.x + footprint.radius_m <= maximum_x &&
         center.y - footprint.radius_m >= bounds.origin_y &&
         center.y + footprint.radius_m <= maximum_y &&
         center.z - footprint.lower_extent_m >= bounds.origin_z &&
         center.z + footprint.upper_extent_m <= maximum_z;
}

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  constexpr double tolerance = 1.0e-6;
  return std::abs(first.origin_x - second.origin_x) <= tolerance &&
         std::abs(first.origin_y - second.origin_y) <= tolerance &&
         std::abs(first.origin_z - second.origin_z) <= tolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= tolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

[[nodiscard]] bool centerInsideExtractionEnvelope(
    const Point3& center, const FreeSpaceTopologyExtractorConfig& config) noexcept {
  return (!config.minimum_center_z_m.has_value() ||
          center.z + kEpsilon >= *config.minimum_center_z_m) &&
         (!config.maximum_center_z_m.has_value() ||
          center.z < *config.maximum_center_z_m - kEpsilon);
}

void classifyVoxels(const OccupancyGrid3D& occupancy,
                    const FreeSpaceTopologyExtractorConfig& config,
                    const ClearanceView& clearance,
                    std::vector<std::uint8_t>& classification,
                    FreeSpaceTopologyExtractionStats& stats) {
  const GridBounds3D& bounds = occupancy.bounds();
  const double bounding_radius_m =
      std::hypot(config.footprint.radius_m, std::max(config.footprint.lower_extent_m,
                                                     config.footprint.upper_extent_m));
  const int chunk_size = static_cast<int>(config.chunk_size_cells);
  for (int core_z = 0; core_z < bounds.depth_cells; core_z += chunk_size) {
    for (int core_y = 0; core_y < bounds.height_cells; core_y += chunk_size) {
      for (int core_x = 0; core_x < bounds.width_cells; core_x += chunk_size) {
        ++stats.processed_chunks;
        const int end_x = std::min(bounds.width_cells, core_x + chunk_size);
        const int end_y = std::min(bounds.height_cells, core_y + chunk_size);
        const int end_z = std::min(bounds.depth_cells, core_z + chunk_size);
        for (int z = core_z; z < end_z; ++z) {
          for (int y = core_y; y < end_y; ++y) {
            for (int x = core_x; x < end_x; ++x) {
              const GridIndex3D cell{x, y, z};
              const std::size_t linear = linearIndex(bounds, cell);
              const double cell_clearance_m = static_cast<double>(clearance[linear]);
              // The topology compiler only needs the finite ESDF neighborhood of
              // physical geometry. Capped infinity is known wide-open space.
              if (!std::isfinite(cell_clearance_m)) {
                continue;
              }
              if (occupancy.isOccupied(cell)) {
                continue;
              }
              ++stats.free_voxels;
              const Point3 center = occupancy.cellCenter(cell);
              if (!centerInsideExtractionEnvelope(center, config) ||
                  !footprintInsideBounds(center, bounds, config.footprint)) {
                continue;
              }
              const bool clearance_proves_feasible =
                  cell_clearance_m + 1.0e-6 >= bounding_radius_m;
              if (!clearance_proves_feasible &&
                  !validateRawFootprintAt(occupancy, center, FootprintBodyAxis{},
                                          config.footprint)
                       .accepted()) {
                continue;
              }
              ++stats.footprint_feasible_voxels;
              if (cell_clearance_m + 1.0e-6 >= config.open_space_clearance_m) {
                classification[linear] = kOpen;
                ++stats.open_space_voxels;
              } else {
                classification[linear] = kConstrained;
                ++stats.constrained_voxels;
              }
            }
          }
        }
      }
    }
  }
}

void retainMedialBand(const GridBounds3D& bounds,
                      const FreeSpaceTopologyExtractorConfig& config,
                      const ClearanceView& clearance,
                      std::vector<std::uint8_t>& classification,
                      FreeSpaceTopologyExtractionStats& stats) {
  std::vector<std::uint8_t> medial(classification.size(), kBlocked);
  std::vector<std::uint16_t> distance_cells(classification.size(),
                                            std::numeric_limits<std::uint16_t>::max());
  std::queue<std::size_t> pending;
  const double tolerance_m = 0.05 * bounds.resolution_m;
  for (std::size_t linear = 0U; linear < classification.size(); ++linear) {
    if (classification[linear] == kOpen) {
      medial[linear] = kOpen;
      continue;
    }
    if (classification[linear] != kConstrained) {
      continue;
    }
    const GridIndex3D cell = cellFor(bounds, linear);
    const double center_clearance_m = static_cast<double>(clearance[linear]);
    const bool ridge = std::ranges::any_of(
        antipodalNeighborDirections(), [&](const GridIndex3D direction) {
          const GridIndex3D first = offset(cell, direction);
          const GridIndex3D second =
              offset(cell, GridIndex3D{-direction.x, -direction.y, -direction.z});
          if (!contains(bounds, first) || !contains(bounds, second)) {
            return false;
          }
          const std::size_t first_linear = linearIndex(bounds, first);
          const std::size_t second_linear = linearIndex(bounds, second);
          if (classification[first_linear] == kBlocked ||
              classification[second_linear] == kBlocked) {
            return false;
          }
          const double first_clearance_m = static_cast<double>(clearance[first_linear]);
          const double second_clearance_m =
              static_cast<double>(clearance[second_linear]);
          if (!std::isfinite(first_clearance_m) || !std::isfinite(second_clearance_m)) {
            return false;
          }
          return std::max(first_clearance_m, second_clearance_m) <=
                     center_clearance_m + tolerance_m &&
                 center_clearance_m - std::min(first_clearance_m, second_clearance_m) +
                         tolerance_m >=
                     config.medial_ridge_prominence_m;
        });
    if (!ridge) {
      continue;
    }
    medial[linear] = kConstrained;
    distance_cells[linear] = 0U;
    pending.push(linear);
    ++stats.medial_ridge_voxels;
  }

  while (!pending.empty()) {
    const std::size_t current = pending.front();
    pending.pop();
    const std::uint16_t current_distance = distance_cells[current];
    if (static_cast<std::size_t>(current_distance) >= config.medial_band_radius_cells) {
      continue;
    }
    const GridIndex3D current_cell = cellFor(bounds, current);
    for (const GridIndex3D direction : neighbors26()) {
      const GridIndex3D neighbor = offset(current_cell, direction);
      if (!contains(bounds, neighbor)) {
        continue;
      }
      const std::size_t neighbor_linear = linearIndex(bounds, neighbor);
      if (classification[neighbor_linear] != kConstrained ||
          distance_cells[neighbor_linear] !=
              std::numeric_limits<std::uint16_t>::max()) {
        continue;
      }
      distance_cells[neighbor_linear] =
          static_cast<std::uint16_t>(current_distance + 1U);
      medial[neighbor_linear] = kConstrained;
      pending.push(neighbor_linear);
    }
  }
  stats.medial_band_voxels = static_cast<std::size_t>(
      std::ranges::count(medial, static_cast<std::uint8_t>(kConstrained)));
  classification = std::move(medial);
}

[[nodiscard]] std::vector<Component>
labelComponents(const GridBounds3D& bounds, std::vector<std::uint8_t>& classification,
                const std::uint8_t target, const std::size_t minimum_voxels,
                const ClearanceView& clearance, std::vector<std::uint32_t>& labels) {
  std::vector<Component> components;
  std::vector<bool> visited(classification.size(), false);
  std::queue<std::size_t> pending;
  for (std::size_t seed = 0U; seed < classification.size(); ++seed) {
    if (classification[seed] != target || visited[seed]) {
      continue;
    }
    std::vector<std::size_t> cells;
    visited[seed] = true;
    pending.push(seed);
    while (!pending.empty()) {
      const std::size_t current = pending.front();
      pending.pop();
      cells.push_back(current);
      const GridIndex3D current_cell = cellFor(bounds, current);
      for (const GridIndex3D direction : neighbors26()) {
        const GridIndex3D neighbor = offset(current_cell, direction);
        if (!contains(bounds, neighbor)) {
          continue;
        }
        const std::size_t neighbor_linear = linearIndex(bounds, neighbor);
        if (classification[neighbor_linear] != target || visited[neighbor_linear]) {
          continue;
        }
        visited[neighbor_linear] = true;
        pending.push(neighbor_linear);
      }
    }
    if (cells.size() < minimum_voxels) {
      if (target == kOpen) {
        for (const std::size_t cell : cells) {
          classification[cell] = kConstrained;
        }
      }
      continue;
    }
    const auto representative = std::ranges::max_element(
        cells, {}, [&clearance](const std::size_t cell) { return clearance[cell]; });
    const std::size_t representative_cell = *representative;
    Component component{
        .label = static_cast<std::uint32_t>(components.size() + 1U),
        .minimum_cell = *std::ranges::min_element(cells),
        .cells = std::move(cells),
        .representative_cell = representative_cell,
        .maximum_clearance_m = static_cast<double>(clearance[representative_cell]),
    };
    for (const std::size_t cell : component.cells) {
      labels[cell] = component.label;
    }
    components.push_back(std::move(component));
  }
  return components;
}

[[nodiscard]] std::vector<PortalPatch>
derivePortalPatches(const GridBounds3D& bounds, const Component& component,
                    const std::vector<std::uint32_t>& open_labels,
                    const std::vector<std::uint32_t>& constrained_labels,
                    const std::size_t minimum_portal_voxels) {
  std::map<std::uint32_t, std::set<std::size_t>> boundary_by_region;
  for (const std::size_t linear : component.cells) {
    const GridIndex3D cell = cellFor(bounds, linear);
    for (const GridIndex3D direction : neighbors6()) {
      const GridIndex3D neighbor = offset(cell, direction);
      if (!contains(bounds, neighbor)) {
        continue;
      }
      const std::uint32_t open_label = open_labels[linearIndex(bounds, neighbor)];
      if (open_label != 0U) {
        boundary_by_region[open_label].insert(linear);
      }
    }
  }

  std::vector<PortalPatch> patches;
  for (const auto& [open_label, candidates] : boundary_by_region) {
    std::set<std::size_t> remaining = candidates;
    while (!remaining.empty()) {
      std::vector<std::size_t> cells;
      std::queue<std::size_t> pending;
      const std::size_t seed = *remaining.begin();
      remaining.erase(seed);
      pending.push(seed);
      while (!pending.empty()) {
        const std::size_t current = pending.front();
        pending.pop();
        cells.push_back(current);
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
          const auto found = remaining.find(neighbor_linear);
          if (found == remaining.end()) {
            continue;
          }
          remaining.erase(found);
          pending.push(neighbor_linear);
        }
      }
      if (cells.size() >= minimum_portal_voxels) {
        std::ranges::sort(cells);
        patches.push_back(
            PortalPatch{.open_region_label = open_label, .cells = std::move(cells)});
      }
    }
  }
  std::ranges::sort(patches, [](const PortalPatch& first, const PortalPatch& second) {
    return std::tie(first.open_region_label, first.cells.front()) <
           std::tie(second.open_region_label, second.cells.front());
  });
  return patches;
}

[[nodiscard]] std::pair<Vec3, Vec3> portalAxes(const Vec3& normal) noexcept {
  const Vec3 reference =
      std::abs(normal.z) < 0.9 ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0};
  Vec3 u = normalized(cross(reference, normal));
  if (std::hypot(std::hypot(u.x, u.y), u.z) <= kEpsilon) {
    u = Vec3{0.0, 1.0, 0.0};
  }
  return {u, normalized(cross(normal, u))};
}

[[nodiscard]] double hullCross(const ProjectedPoint& origin,
                               const ProjectedPoint& first,
                               const ProjectedPoint& second) noexcept {
  return (first.u - origin.u) * (second.v - origin.v) -
         (first.v - origin.v) * (second.u - origin.u);
}

[[nodiscard]] std::vector<Point3> portalPolygon(const Point3& center,
                                                const Vec3& u_axis, const Vec3& v_axis,
                                                const std::vector<GridIndex3D>& patch,
                                                const OccupancyGrid3D& occupancy) {
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

[[nodiscard]] PassagePortal
makePortal(const OccupancyGrid3D& occupancy, const Component& constrained_component,
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

[[nodiscard]] double neighborStepM(const GridIndex3D direction,
                                   const double resolution_m) noexcept {
  return std::sqrt(static_cast<double>(direction.x * direction.x +
                                       direction.y * direction.y +
                                       direction.z * direction.z)) *
         resolution_m;
}

[[nodiscard]] std::map<std::size_t, std::set<std::size_t>>
buildMedialTree(const GridBounds3D& bounds, const Component& component,
                const std::vector<std::size_t>& anchors, const ClearanceView& clearance,
                const std::vector<std::uint32_t>& constrained_labels,
                const FreeSpaceTopologyExtractorConfig& config) {
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
  while (!pending.empty() && !remaining.empty()) {
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

[[nodiscard]] std::vector<SegmentDraft>
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

[[nodiscard]] std::vector<RouteSample3D>
makeCenterline(const OccupancyGrid3D& occupancy, const std::vector<std::size_t>& cells,
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

[[nodiscard]] bool rawSafeCenterline(const OccupancyGrid3D& occupancy,
                                     const std::vector<RouteSample3D>& centerline,
                                     const SweptFootprintConfig& footprint) noexcept {
  if (centerline.size() < 2U) {
    return false;
  }
  for (std::size_t index = 1U; index < centerline.size(); ++index) {
    if (!validateRawSweptFootprint(occupancy, centerline[index - 1U].position,
                                   FootprintBodyAxis{}, centerline[index].position,
                                   FootprintBodyAxis{}, footprint)
             .accepted()) {
      return false;
    }
  }
  return true;
}

} // namespace

bool freeSpaceTopologyExtractorConfigIsValid(
    const FreeSpaceTopologyExtractorConfig& config) noexcept {
  const bool valid_minimum_z = !config.minimum_center_z_m.has_value() ||
                               std::isfinite(*config.minimum_center_z_m);
  const bool valid_maximum_z = !config.maximum_center_z_m.has_value() ||
                               std::isfinite(*config.maximum_center_z_m);
  const bool valid_z_interval = !config.minimum_center_z_m.has_value() ||
                                !config.maximum_center_z_m.has_value() ||
                                *config.minimum_center_z_m < *config.maximum_center_z_m;
  return finitePositive(config.maximum_clearance_m) &&
         finitePositive(config.open_space_clearance_m) &&
         config.maximum_clearance_m > config.open_space_clearance_m &&
         finitePositive(config.speed_limit_mps) &&
         std::isfinite(config.medial_clearance_weight) &&
         config.medial_clearance_weight >= 0.0 &&
         finitePositive(config.medial_ridge_prominence_m) &&
         config.medial_band_radius_cells <=
             static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max()) &&
         config.chunk_size_cells > 0U && config.minimum_open_region_voxels > 0U &&
         config.minimum_constrained_component_voxels > 0U &&
         config.minimum_portal_voxels > 0U &&
         std::isfinite(config.footprint.radius_m) && config.footprint.radius_m >= 0.0 &&
         std::isfinite(config.footprint.lower_extent_m) &&
         config.footprint.lower_extent_m >= 0.0 &&
         std::isfinite(config.footprint.upper_extent_m) &&
         config.footprint.upper_extent_m >= 0.0 && valid_minimum_z && valid_maximum_z &&
         valid_z_interval;
}

ExtractedFreeSpaceTopology3D
extractFreeSpaceTopology3D(const OccupancyGrid3D& occupancy,
                           const DistanceField3D& clearance_field,
                           const FreeSpaceTopologyExtractorConfig& config) {
  const auto started = std::chrono::steady_clock::now();
  if (!freeSpaceTopologyExtractorConfigIsValid(config)) {
    throw std::invalid_argument{"invalid FreeSpaceTopologyExtractor3D config"};
  }
  if (!sameBounds(occupancy.bounds(), clearance_field.bounds()) ||
      clearance_field.maximumDistanceM() + 1.0e-6 < config.maximum_clearance_m ||
      clearance_field.distancesM().size() != voxelCount(occupancy.bounds())) {
    throw std::invalid_argument{
        "clearance field does not match FreeSpaceTopologyExtractor3D input"};
  }
  ExtractedFreeSpaceTopology3D result;
  const GridBounds3D& bounds = occupancy.bounds();
  result.stats.clearance_ms = clearance_field.stats().duration_ms;
  const ClearanceView clearance{clearance_field.distancesM(),
                                0.5 * std::numbers::sqrt3_v<double> *
                                    occupancy.bounds().resolution_m};
  std::vector<std::uint8_t> classification(clearance.size(), kBlocked);
  const auto classification_started = std::chrono::steady_clock::now();
  classifyVoxels(occupancy, config, clearance, classification, result.stats);
  retainMedialBand(bounds, config, clearance, classification, result.stats);
  result.stats.classification_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                classification_started)
          .count();

  const auto component_labeling_started = std::chrono::steady_clock::now();
  std::vector<std::uint32_t> open_labels(clearance.size(), 0U);
  std::vector<Component> open_components =
      labelComponents(bounds, classification, kOpen, config.minimum_open_region_voxels,
                      clearance, open_labels);
  std::ranges::sort(open_components, {}, &Component::minimum_cell);
  std::fill(open_labels.begin(), open_labels.end(), 0U);
  for (std::size_t index = 0U; index < open_components.size(); ++index) {
    open_components[index].label = static_cast<std::uint32_t>(index + 1U);
    for (const std::size_t cell : open_components[index].cells) {
      open_labels[cell] = open_components[index].label;
    }
  }
  result.stats.open_space_components = open_components.size();

  std::vector<std::uint32_t> constrained_labels(clearance.size(), 0U);
  std::vector<Component> constrained_components = labelComponents(
      bounds, classification, kConstrained, config.minimum_constrained_component_voxels,
      clearance, constrained_labels);
  std::ranges::sort(constrained_components, {}, &Component::minimum_cell);
  std::fill(constrained_labels.begin(), constrained_labels.end(), 0U);
  for (std::size_t index = 0U; index < constrained_components.size(); ++index) {
    constrained_components[index].label = static_cast<std::uint32_t>(index + 1U);
    for (const std::size_t cell : constrained_components[index].cells) {
      constrained_labels[cell] = constrained_components[index].label;
    }
  }
  result.stats.constrained_components = constrained_components.size();
  result.stats.component_labeling_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                component_labeling_started)
          .count();

  const auto topology_build_started = std::chrono::steady_clock::now();
  std::map<std::uint32_t, std::vector<PassagePortalId>> region_portals;
  std::size_t portal_number = 0U;
  std::size_t segment_number = 0U;
  for (const Component& component : constrained_components) {
    std::vector<PortalPatch> patches =
        derivePortalPatches(bounds, component, open_labels, constrained_labels,
                            config.minimum_portal_voxels);
    result.stats.portal_patches += patches.size();
    if (patches.size() < 2U) {
      ++result.stats.rejected_constrained_components;
      ++result.stats.rejected_for_insufficient_portals;
      continue;
    }

    std::vector<PassagePortal> component_portals;
    std::vector<std::size_t> anchors;
    component_portals.reserve(patches.size());
    anchors.reserve(patches.size());
    for (const PortalPatch& patch : patches) {
      const std::size_t open_index =
          static_cast<std::size_t>(patch.open_region_label - 1U);
      if (open_index >= open_components.size()) {
        throw std::logic_error{"invalid open-space component label"};
      }
      const FreeSpaceRegionId region_id{indexedId("region", open_index)};
      const PassagePortalId portal_id{indexedId("portal", portal_number++)};
      PassagePortal portal = makePortal(occupancy, component, patch, region_id,
                                        portal_id, clearance, open_labels);
      const std::optional<GridIndex3D> anchor =
          occupancy.worldToCell(portal.traversable_anchors.front());
      if (!anchor.has_value()) {
        throw std::logic_error{"portal anchor outside Occupancy3D"};
      }
      anchors.push_back(linearIndex(bounds, *anchor));
      component_portals.push_back(std::move(portal));
    }

    const std::map<std::size_t, std::set<std::size_t>> medial_tree = buildMedialTree(
        bounds, component, anchors, clearance, constrained_labels, config);
    const std::set<std::size_t> portal_anchors(anchors.begin(), anchors.end());
    const std::vector<SegmentDraft> drafts =
        compressMedialTree(medial_tree, portal_anchors);
    if (drafts.empty()) {
      ++result.stats.rejected_constrained_components;
      ++result.stats.rejected_for_disconnected_medial_graph;
      continue;
    }

    std::map<std::size_t, std::vector<PassagePortalId>> portals_by_anchor;
    for (std::size_t index = 0U; index < component_portals.size(); ++index) {
      portals_by_anchor[anchors[index]].push_back(component_portals[index].id);
    }
    std::vector<PassageSegment> component_segments;
    std::map<std::size_t, std::vector<std::size_t>> segment_indices_by_endpoint;
    for (const SegmentDraft& draft : drafts) {
      PassageSegment segment{
          .id = PassageSegmentId{indexedId("segment", segment_number++)},
          .centerline = makeCenterline(occupancy, draft.cells, config.speed_limit_mps),
          .endpoint_portal_ids = {},
          .first_endpoint_neighbors = {},
          .second_endpoint_neighbors = {},
          .minimum_clearance_m = std::numeric_limits<double>::infinity(),
          .speed_limit_mps = config.speed_limit_mps,
      };
      if (!rawSafeCenterline(occupancy, segment.centerline, config.footprint)) {
        ++result.stats.raw_unsafe_segments;
        continue;
      }
      for (const std::size_t cell : draft.cells) {
        segment.minimum_clearance_m =
            std::min(segment.minimum_clearance_m, static_cast<double>(clearance[cell]));
      }
      if (const auto portals = portals_by_anchor.find(draft.first_cell);
          portals != portals_by_anchor.end()) {
        segment.endpoint_portal_ids.insert(segment.endpoint_portal_ids.end(),
                                           portals->second.begin(),
                                           portals->second.end());
      }
      if (const auto portals = portals_by_anchor.find(draft.second_cell);
          portals != portals_by_anchor.end()) {
        segment.endpoint_portal_ids.insert(segment.endpoint_portal_ids.end(),
                                           portals->second.begin(),
                                           portals->second.end());
      }
      const std::size_t segment_index = component_segments.size();
      component_segments.push_back(std::move(segment));
      segment_indices_by_endpoint[draft.first_cell].push_back(segment_index);
      segment_indices_by_endpoint[draft.second_cell].push_back(segment_index);
    }
    if (component_segments.empty()) {
      ++result.stats.rejected_constrained_components;
      ++result.stats.rejected_for_no_safe_segments;
      continue;
    }
    for (const auto& [endpoint, segment_indices] : segment_indices_by_endpoint) {
      static_cast<void>(endpoint);
      for (const std::size_t segment_index : segment_indices) {
        PassageSegment& segment = component_segments[segment_index];
        const bool first_endpoint =
            occupancy.worldToCell(segment.centerline.front().position) ==
            std::optional<GridIndex3D>{cellFor(bounds, endpoint)};
        std::vector<PassageSegmentId>& neighbors =
            first_endpoint ? segment.first_endpoint_neighbors
                           : segment.second_endpoint_neighbors;
        for (const std::size_t other_index : segment_indices) {
          if (other_index != segment_index) {
            neighbors.push_back(component_segments[other_index].id);
          }
        }
        std::ranges::sort(neighbors);
      }
    }
    for (std::size_t portal_index = 0U; portal_index < component_portals.size();
         ++portal_index) {
      PassagePortal& portal = component_portals[portal_index];
      region_portals[patches[portal_index].open_region_label].push_back(portal.id);
      result.portals.push_back(std::move(portal));
    }
    result.segments.insert(result.segments.end(),
                           std::make_move_iterator(component_segments.begin()),
                           std::make_move_iterator(component_segments.end()));
  }

  for (const Component& open_component : open_components) {
    const auto portal_ids = region_portals.find(open_component.label);
    if (portal_ids == region_portals.end()) {
      continue;
    }
    const std::size_t open_index = static_cast<std::size_t>(open_component.label - 1U);
    result.regions.push_back(FreeSpaceRegion{
        .id = FreeSpaceRegionId{indexedId("region", open_index)},
        .representative =
            occupancy.cellCenter(cellFor(bounds, open_component.representative_cell)),
        .maximum_clearance_m = open_component.maximum_clearance_m,
        .portal_ids = portal_ids->second,
    });
  }
  result.stats.topology_build_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                topology_build_started)
          .count();
  result.stats.duration_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
  return result;
}

ExtractedFreeSpaceTopology3D
extractFreeSpaceTopology3D(const OccupancyGrid3D& occupancy,
                           const FreeSpaceTopologyExtractorConfig& config) {
  if (!freeSpaceTopologyExtractorConfigIsValid(config)) {
    throw std::invalid_argument{"invalid FreeSpaceTopologyExtractor3D config"};
  }
  const auto started = std::chrono::steady_clock::now();
  const DistanceField3D clearance_field =
      DistanceField3D::build(occupancy, config.maximum_clearance_m);
  ExtractedFreeSpaceTopology3D result =
      extractFreeSpaceTopology3D(occupancy, clearance_field, config);
  result.stats.clearance_ms = clearance_field.stats().duration_ms;
  result.stats.duration_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
  return result;
}

} // namespace drone_city_nav
