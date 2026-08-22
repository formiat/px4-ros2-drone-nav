#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <tuple>
#include <vector>

#include "incremental_topology_graph_3d_internal.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] Point3 pointForKey(const GridBounds3D& bounds,
                                 const std::uint64_t key) noexcept {
  const GridIndex3D cell = incremental_topology_detail::sampleCellForKey(bounds, key);
  return Point3{
      bounds.origin_x + (static_cast<double>(cell.x) + 0.5) * bounds.resolution_m,
      bounds.origin_y + (static_cast<double>(cell.y) + 0.5) * bounds.resolution_m,
      bounds.origin_z + (static_cast<double>(cell.z) + 0.5) * bounds.resolution_m,
  };
}

void appendUnique(std::vector<Point3>& output, const Point3& point) {
  if (output.empty() || distance3D(output.back(), point) > 1.0e-9) {
    output.push_back(point);
  }
}

[[nodiscard]] double pathLength(const std::span<const Point3> points) noexcept {
  double result = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    result += distance3D(points[index - 1U], points[index]);
  }
  return result;
}

} // namespace

std::uint64_t IncrementalTopologyGraph3DSnapshot::revision() const noexcept {
  return revision_;
}

const GridBounds3D& IncrementalTopologyGraph3DSnapshot::bounds() const noexcept {
  return bounds_;
}

std::span<const IncrementalTopologyNode3D>
IncrementalTopologyGraph3DSnapshot::nodes() const noexcept {
  return nodes_;
}

std::span<const IncrementalTopologyEdge3D>
IncrementalTopologyGraph3DSnapshot::edges() const noexcept {
  return edges_;
}

std::span<const IncrementalTopologyBlockCoverage3D>
IncrementalTopologyGraph3DSnapshot::blockCoverage() const noexcept {
  return block_coverage_;
}

std::span<const IncrementalTopologySample3D>
IncrementalTopologyGraph3DSnapshot::samples() const noexcept {
  return samples_;
}

std::size_t IncrementalTopologyGraph3DSnapshot::pendingBlockCount() const noexcept {
  return pending_block_count_;
}

const IncrementalTopologyNode3D* IncrementalTopologyGraph3DSnapshot::findNode(
    const IncrementalTopologyNodeId id) const noexcept {
  const auto found = node_indices_.find(id);
  return found == node_indices_.end() ? nullptr : &nodes_[found->second];
}

std::optional<IncrementalTopologyNodeId>
IncrementalTopologyGraph3DSnapshot::nodeForSampleCell(
    const GridIndex3D cell) const noexcept {
  const auto found = sample_cell_nodes_.find(
      incremental_topology_detail::sampleCellKey(bounds_, cell));
  return found == sample_cell_nodes_.end()
             ? std::nullopt
             : std::optional<IncrementalTopologyNodeId>{found->second};
}

std::optional<IncrementalTopologyNodeId>
IncrementalTopologyGraph3DSnapshot::nearestNode(
    const Point3& position, const double maximum_distance_m) const noexcept {
  if (!(maximum_distance_m >= 0.0) || !std::isfinite(maximum_distance_m)) {
    return std::nullopt;
  }
  std::optional<IncrementalTopologyNodeId> best;
  double best_distance = maximum_distance_m;
  for (const IncrementalTopologyNode3D& node : nodes_) {
    const double candidate_distance = distance3D(position, node.representative);
    if (candidate_distance + 1.0e-9 < best_distance ||
        (std::abs(candidate_distance - best_distance) <= 1.0e-9 &&
         (!best.has_value() || node.id < *best))) {
      best = node.id;
      best_distance = candidate_distance;
    }
  }
  return best;
}

std::optional<IncrementalTopologyConnector3D>
IncrementalTopologyGraph3DSnapshot::connectObserved(
    const ObservedOccupancyGrid3D& occupancy, const Point3& position,
    const double maximum_distance_m, const SweptFootprintConfig& footprint,
    const ObservedSpaceValidationPolicy validation_policy) const {
  if (!(maximum_distance_m >= 0.0) || !std::isfinite(maximum_distance_m) ||
      revision_ == 0U) {
    return std::nullopt;
  }

  struct Candidate {
    std::uint64_t cell_key{0U};
    IncrementalTopologyNodeId node{};
    double distance_m{0.0};
  };

  std::vector<Candidate> candidates;
  candidates.reserve(sample_cell_nodes_.size());
  for (const auto& [cell_key, node] : sample_cell_nodes_) {
    const double distance_m = distance3D(position, pointForKey(bounds_, cell_key));
    if (distance_m <= maximum_distance_m) {
      candidates.push_back(Candidate{cell_key, node, distance_m});
    }
  }
  std::ranges::sort(candidates, [](const Candidate& first, const Candidate& second) {
    return std::tie(first.distance_m, first.node.value, first.cell_key) <
           std::tie(second.distance_m, second.node.value, second.cell_key);
  });
  constexpr std::size_t kMaximumCandidates{64U};
  if (candidates.size() > kMaximumCandidates) {
    candidates.resize(kMaximumCandidates);
  }

  std::optional<IncrementalTopologyConnector3D> best;
  for (const Candidate& candidate : candidates) {
    std::vector<Point3> polyline;
    appendUnique(polyline, position);
    std::uint64_t current = candidate.cell_key;
    bool complete = false;
    for (std::size_t guard = 0U; guard <= sample_cell_parents_.size(); ++guard) {
      appendUnique(polyline,
                   occupancy.cellCenter(incremental_topology_detail::sampleCellForKey(
                       bounds_, current)));
      const auto parent = sample_cell_parents_.find(current);
      if (parent == sample_cell_parents_.end()) {
        break;
      }
      if (parent->second == current) {
        complete = true;
        break;
      }
      current = parent->second;
    }
    if (!complete) {
      continue;
    }
    bool valid = true;
    for (std::size_t index = 1U; index < polyline.size(); ++index) {
      const SweptFootprintResult evidence = validateObservedSweptFootprint(
          occupancy, polyline[index - 1U], FootprintBodyAxis{}, polyline[index],
          FootprintBodyAxis{}, footprint, validation_policy);
      if (!evidence.accepted()) {
        valid = false;
        break;
      }
    }
    if (!valid) {
      continue;
    }
    const double length_m = pathLength(polyline);
    if (!best.has_value() || length_m + 1.0e-9 < best->length_m ||
        (std::abs(length_m - best->length_m) <= 1.0e-9 &&
         candidate.node < best->node)) {
      best = IncrementalTopologyConnector3D{
          .node = candidate.node,
          .polyline = std::move(polyline),
          .length_m = length_m,
          .validated_through_revision = revision_,
      };
    }
  }
  return best;
}

} // namespace drone_city_nav
