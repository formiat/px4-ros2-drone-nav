#include "drone_city_nav/reachable_observation_domain_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <ranges>
#include <tuple>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::uint64_t cellKey(const GridBounds3D& bounds,
                                    const GridIndex3D cell) noexcept {
  return (static_cast<std::uint64_t>(cell.z) *
              static_cast<std::uint64_t>(bounds.height_cells) +
          static_cast<std::uint64_t>(cell.y)) *
             static_cast<std::uint64_t>(bounds.width_cells) +
         static_cast<std::uint64_t>(cell.x);
}

[[nodiscard]] std::array<GridIndex3D, 6U> neighbors(const GridIndex3D cell) noexcept {
  return {
      GridIndex3D{cell.x - 1, cell.y, cell.z}, GridIndex3D{cell.x + 1, cell.y, cell.z},
      GridIndex3D{cell.x, cell.y - 1, cell.z}, GridIndex3D{cell.x, cell.y + 1, cell.z},
      GridIndex3D{cell.x, cell.y, cell.z - 1}, GridIndex3D{cell.x, cell.y, cell.z + 1},
  };
}

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  constexpr double kTolerance{1.0e-9};
  return std::abs(first.origin_x - second.origin_x) <= kTolerance &&
         std::abs(first.origin_y - second.origin_y) <= kTolerance &&
         std::abs(first.origin_z - second.origin_z) <= kTolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= kTolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

void appendUnique(std::vector<Point3>& points, const Point3& point) {
  if (points.empty() || distance3D(points.back(), point) > 1.0e-9) {
    points.push_back(point);
  }
}

[[nodiscard]] double pathLength(const std::span<const Point3> points) noexcept {
  double result = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    result += distance3D(points[index - 1U], points[index]);
  }
  return result;
}

struct PendingCell {
  double distance_m{0.0};
  std::uint64_t key{0U};

  [[nodiscard]] bool operator>(const PendingCell& other) const noexcept {
    return std::tie(distance_m, key) > std::tie(other.distance_m, other.key);
  }
};

} // namespace

bool reachableObservationDomain3DConfigIsValid(
    const ReachableObservationDomain3DConfig& config) noexcept {
  return std::isfinite(config.maximum_fresh_extension_m) &&
         config.maximum_fresh_extension_m >= 0.0 &&
         std::isfinite(config.footprint.radius_m) && config.footprint.radius_m >= 0.0 &&
         std::isfinite(config.footprint.lower_extent_m) &&
         config.footprint.lower_extent_m >= 0.0 &&
         std::isfinite(config.footprint.upper_extent_m) &&
         config.footprint.upper_extent_m >= 0.0 &&
         std::isfinite(config.footprint.sweep_step_m) &&
         config.footprint.sweep_step_m > 0.0 &&
         config.footprint.perimeter_samples > 0U &&
         config.footprint.radial_rings > 0U && config.footprint.axial_samples > 0U;
}

ReachableObservationDomain3D buildReachableObservationDomain3D(
    const IncrementalTopologyGraph3DSnapshot& graph,
    const ObservedOccupancyGrid3D& occupancy,
    const std::span<const IncrementalTopologyNodeId> reachable_nodes,
    const ReachableObservationDomain3DConfig& config) {
  ReachableObservationDomain3D result;
  if (!reachableObservationDomain3DConfigIsValid(config) ||
      !sameBounds(graph.bounds(), occupancy.bounds())) {
    return result;
  }
  result.graph_ = &graph;
  result.occupancy_ = &occupancy;
  result.config_ = config;
  result.reachable_nodes_.insert(reachable_nodes.begin(), reachable_nodes.end());

  std::unordered_set<std::uint64_t> graph_sample_keys;
  graph_sample_keys.reserve(graph.samples().size());
  std::priority_queue<PendingCell, std::vector<PendingCell>, std::greater<>> pending;
  for (const IncrementalTopologySample3D& sample : graph.samples()) {
    const std::uint64_t key = cellKey(graph.bounds(), sample.cell);
    graph_sample_keys.insert(key);
    if (!result.reachable_nodes_.contains(sample.node)) {
      continue;
    }
    result.candidate_cells_.push_back(sample.cell);
    const IncrementalTopologyNode3D* const node = graph.findNode(sample.node);
    if (node == nullptr || !node->unknown_boundary_exposure) {
      continue;
    }
    result.extension_cells_.try_emplace(key,
                                        ReachableObservationDomain3D::ExtensionCell{
                                            .cell = sample.cell,
                                            .parent_key = key,
                                            .root_key = key,
                                            .root_node = sample.node,
                                        });
    pending.push(PendingCell{.key = key});
  }

  const double step_m = occupancy.bounds().resolution_m;
  while (!pending.empty()) {
    const PendingCell current = pending.top();
    pending.pop();
    const auto current_found = result.extension_cells_.find(current.key);
    if (current_found == result.extension_cells_.end() ||
        current.distance_m > current_found->second.distance_from_graph_m + 1.0e-9) {
      continue;
    }
    const Point3 current_position = occupancy.cellCenter(current_found->second.cell);
    for (const GridIndex3D next : neighbors(current_found->second.cell)) {
      if (!occupancy.contains(next) || !occupancy.isKnownFree(next)) {
        continue;
      }
      const std::uint64_t next_key = cellKey(occupancy.bounds(), next);
      if (graph_sample_keys.contains(next_key)) {
        continue;
      }
      const double next_distance_m = current.distance_m + step_m;
      if (next_distance_m > config.maximum_fresh_extension_m + 1.0e-9) {
        continue;
      }
      const Point3 next_position = occupancy.cellCenter(next);
      const SweptFootprintResult evidence = validateObservedSweptFootprint(
          occupancy, current_position, FootprintBodyAxis{}, next_position,
          FootprintBodyAxis{}, config.footprint, config.validation_policy);
      if (!evidence.accepted()) {
        continue;
      }
      const auto found = result.extension_cells_.find(next_key);
      if (found != result.extension_cells_.end() &&
          found->second.distance_from_graph_m <= next_distance_m + 1.0e-9) {
        continue;
      }
      result.extension_cells_.insert_or_assign(
          next_key, ReachableObservationDomain3D::ExtensionCell{
                        .cell = next,
                        .parent_key = current.key,
                        .root_key = current_found->second.root_key,
                        .root_node = current_found->second.root_node,
                        .distance_from_graph_m = next_distance_m,
                    });
      pending.push(PendingCell{.distance_m = next_distance_m, .key = next_key});
    }
  }

  for (const auto& [key, extension] : result.extension_cells_) {
    if (extension.parent_key != key) {
      result.candidate_cells_.push_back(extension.cell);
    }
  }
  std::ranges::sort(result.candidate_cells_, [](const GridIndex3D first,
                                                const GridIndex3D second) {
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  const auto unique_end =
      std::unique(result.candidate_cells_.begin(), result.candidate_cells_.end());
  result.candidate_cells_.erase(unique_end, result.candidate_cells_.end());
  return result;
}

std::span<const GridIndex3D>
ReachableObservationDomain3D::candidateCells() const noexcept {
  return candidate_cells_;
}

std::optional<IncrementalTopologyConnector3D>
ReachableObservationDomain3D::connect(const Point3& position,
                                      const double maximum_distance_m) const {
  if (graph_ == nullptr || occupancy_ == nullptr ||
      !std::isfinite(maximum_distance_m) || maximum_distance_m < 0.0) {
    return std::nullopt;
  }
  if (std::optional<IncrementalTopologyConnector3D> connector =
          graph_->connectObserved(*occupancy_, position, maximum_distance_m,
                                  config_.footprint, config_.validation_policy);
      connector.has_value() && reachable_nodes_.contains(connector->node)) {
    return connector;
  }

  struct Candidate {
    std::uint64_t key{0U};
    double distance_m{0.0};
  };

  std::vector<Candidate> candidates;
  candidates.reserve(extension_cells_.size());
  for (const auto& [key, extension] : extension_cells_) {
    if (extension.parent_key == key) {
      continue;
    }
    const double distance_m =
        distance3D(position, occupancy_->cellCenter(extension.cell));
    if (distance_m <= maximum_distance_m) {
      candidates.push_back(Candidate{.key = key, .distance_m = distance_m});
    }
  }
  std::ranges::sort(candidates, [](const Candidate& first, const Candidate& second) {
    return std::tie(first.distance_m, first.key) <
           std::tie(second.distance_m, second.key);
  });
  constexpr std::size_t kMaximumConnectorCandidates{64U};
  if (candidates.size() > kMaximumConnectorCandidates) {
    candidates.resize(kMaximumConnectorCandidates);
  }

  std::optional<IncrementalTopologyConnector3D> best;
  for (const Candidate& candidate : candidates) {
    const ExtensionCell& extension = extension_cells_.at(candidate.key);
    const Point3 extension_position = occupancy_->cellCenter(extension.cell);
    const SweptFootprintResult initial = validateObservedSweptFootprint(
        *occupancy_, position, FootprintBodyAxis{}, extension_position,
        FootprintBodyAxis{}, config_.footprint, config_.validation_policy);
    if (!initial.accepted()) {
      continue;
    }
    std::vector<Point3> polyline;
    appendUnique(polyline, position);
    std::uint64_t current_key = candidate.key;
    bool complete = false;
    for (std::size_t guard = 0U; guard <= extension_cells_.size(); ++guard) {
      const auto found = extension_cells_.find(current_key);
      if (found == extension_cells_.end()) {
        break;
      }
      appendUnique(polyline, occupancy_->cellCenter(found->second.cell));
      if (found->second.parent_key == current_key) {
        complete = true;
        break;
      }
      current_key = found->second.parent_key;
    }
    if (!complete) {
      continue;
    }
    const Point3 root_position = polyline.back();
    const std::optional<IncrementalTopologyConnector3D> root_connector =
        graph_->connectObserved(*occupancy_, root_position,
                                occupancy_->bounds().resolution_m * 0.5,
                                config_.footprint, config_.validation_policy);
    if (!root_connector.has_value() || root_connector->node != extension.root_node ||
        !reachable_nodes_.contains(root_connector->node)) {
      continue;
    }
    for (const Point3& point : root_connector->polyline) {
      appendUnique(polyline, point);
    }
    const double length_m = pathLength(polyline);
    if (length_m > maximum_distance_m + 1.0e-9) {
      continue;
    }
    if (!best.has_value() || length_m + 1.0e-9 < best->length_m ||
        (std::abs(length_m - best->length_m) <= 1.0e-9 &&
         root_connector->node < best->node)) {
      best = IncrementalTopologyConnector3D{
          .node = root_connector->node,
          .polyline = std::move(polyline),
          .length_m = length_m,
          .validated_through_revision = graph_->revision(),
      };
    }
  }
  return best;
}

} // namespace drone_city_nav
