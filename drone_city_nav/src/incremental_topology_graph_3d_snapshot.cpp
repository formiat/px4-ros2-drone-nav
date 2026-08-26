#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <tuple>
#include <utility>
#include <vector>

#include "incremental_topology_graph_3d_internal.hpp"

namespace drone_city_nav {
namespace {

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

[[nodiscard]] std::vector<Point3>
splitLongSegments(const std::span<const Point3> polyline,
                  const double maximum_segment_length_m) {
  std::vector<Point3> result;
  if (polyline.empty()) {
    return result;
  }
  result.reserve(polyline.size());
  appendUnique(result, polyline.front());
  for (std::size_t index = 1U; index < polyline.size(); ++index) {
    const Point3& first = polyline[index - 1U];
    const Point3& second = polyline[index];
    const double length_m = distance3D(first, second);
    const std::size_t interval_count = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(length_m / maximum_segment_length_m)));
    for (std::size_t interval = 1U; interval <= interval_count; ++interval) {
      const double ratio =
          static_cast<double>(interval) / static_cast<double>(interval_count);
      appendUnique(result, Point3{.x = std::lerp(first.x, second.x, ratio),
                                  .y = std::lerp(first.y, second.y, ratio),
                                  .z = std::lerp(first.z, second.z, ratio)});
    }
  }
  return result;
}

[[nodiscard]] double distanceToInterval(const double value, const double minimum,
                                        const double maximum) noexcept {
  if (value < minimum) {
    return minimum - value;
  }
  if (value > maximum) {
    return value - maximum;
  }
  return 0.0;
}

[[nodiscard]] double
bucketDistanceLowerBound(const GridBounds3D& bounds,
                         const IncrementalTopologyBlockIndex3D bucket,
                         const int bucket_size_cells, const Point3& position) noexcept {
  const auto interval = [&](const int bucket_coordinate, const int cell_count,
                            const double origin) {
    const int minimum_cell = bucket_coordinate * bucket_size_cells;
    const int maximum_cell = std::min(cell_count, minimum_cell + bucket_size_cells) - 1;
    return std::pair{
        origin + (static_cast<double>(minimum_cell) + 0.5) * bounds.resolution_m,
        origin + (static_cast<double>(maximum_cell) + 0.5) * bounds.resolution_m};
  };
  const auto [minimum_x, maximum_x] =
      interval(bucket.x, bounds.width_cells, bounds.origin_x);
  const auto [minimum_y, maximum_y] =
      interval(bucket.y, bounds.height_cells, bounds.origin_y);
  const auto [minimum_z, maximum_z] =
      interval(bucket.z, bounds.depth_cells, bounds.origin_z);
  return std::hypot(distanceToInterval(position.x, minimum_x, maximum_x),
                    distanceToInterval(position.y, minimum_y, maximum_y),
                    distanceToInterval(position.z, minimum_z, maximum_z));
}

[[nodiscard]] const IncrementalTopologySampleBlock3D* findSampleBlock(
    const std::span<const std::shared_ptr<const IncrementalTopologySampleBlock3D>>
        blocks,
    const IncrementalTopologyBlockIndex3D index) noexcept {
  const auto found = std::ranges::lower_bound(
      blocks, index, {}, [](const auto& block) { return block->block; });
  return found != blocks.end() && (*found)->block == index ? found->get() : nullptr;
}

[[nodiscard]] const IncrementalTopologySampleRecord3D*
findSampleRecord(const IncrementalTopologySampleBlock3D& block,
                 const GridIndex3D cell) noexcept {
  const auto found = std::lower_bound(
      block.records.begin(), block.records.end(), cell,
      [](const IncrementalTopologySampleRecord3D& record, const GridIndex3D candidate) {
        return incremental_topology_detail::cellLess(record.cell, candidate);
      });
  return found != block.records.end() && found->cell == cell ? &*found : nullptr;
}

struct SampleTreePath3D {
  IncrementalTopologyNodeId node{};
  std::vector<Point3> polyline;
};

[[nodiscard]] std::optional<SampleTreePath3D> reconstructSampleTree(
    const GridBounds3D& bounds, const IncrementalTopologySampleBlock3D& block,
    const IncrementalTopologySampleRecord3D& sample,
    const std::optional<std::chrono::steady_clock::time_point> deadline =
        std::nullopt) {
  SampleTreePath3D result{.node = sample.node, .polyline = {}};
  const IncrementalTopologySampleRecord3D* current = &sample;
  bool complete = false;
  for (std::size_t guard = 0U; guard <= block.records.size(); ++guard) {
    if (deadline.has_value() && std::chrono::steady_clock::now() >= *deadline) {
      return std::nullopt;
    }
    appendUnique(result.polyline,
                 incremental_topology_detail::cellCenter(bounds, current->cell));
    if (current->parent_cell == current->cell) {
      complete = true;
      break;
    }
    current = findSampleRecord(block, current->parent_cell);
    if (current == nullptr || current->node != result.node) {
      break;
    }
  }
  if (!complete ||
      std::abs(pathLength(result.polyline) - sample.distance_to_node_m) > 1.0e-6) {
    return std::nullopt;
  }
  return result;
}

[[nodiscard]] std::optional<IncrementalTopologyConnector3D> makeObservedConnector(
    const ObservedOccupancyGrid3D& occupancy, const IncrementalTopologyNode3D& node,
    std::vector<Point3> polyline, const SweptFootprintConfig& footprint,
    const ObservedSpaceValidationPolicy validation_policy,
    const std::uint64_t graph_revision,
    const std::optional<std::chrono::steady_clock::time_point> deadline =
        std::nullopt) {
  polyline = splitLongSegments(
      polyline, std::max(occupancy.bounds().resolution_m, footprint.sweep_step_m));
  bool unknown_exposure = false;
  for (std::size_t index = 1U; index < polyline.size(); ++index) {
    if (deadline.has_value() && std::chrono::steady_clock::now() >= *deadline) {
      return std::nullopt;
    }
    const SweptFootprintResult evidence =
        validateRawSweptFootprint(occupancy, polyline[index - 1U], FootprintBodyAxis{},
                                  polyline[index], FootprintBodyAxis{}, footprint);
    const bool unknown_rejected =
        validation_policy == ObservedSpaceValidationPolicy::kRequireKnownFree &&
        evidence.evidence.unknown_exposure;
    if (evidence.evidence.raw_collision || evidence.evidence.outside_grid_exposure ||
        evidence.evidence.invalid_esdf_exposure || unknown_rejected) {
      return std::nullopt;
    }
    unknown_exposure = unknown_exposure || evidence.evidence.unknown_exposure;
  }
  const double length_m = pathLength(polyline);
  const std::uint64_t validated_through_revision =
      std::min(graph_revision, node.validated_through_revision);
  const std::uint64_t lineage_id =
      incremental_topology_detail::makeTransitionLineage(node.id.value, polyline);
  const IncrementalTopologyTransitionEvidence3D transition_evidence{
      .kind = unknown_exposure ? IncrementalTopologyTransitionKind3D::kOptimisticUnknown
                               : IncrementalTopologyTransitionKind3D::kObservedFree,
      .support_segment_count = polyline.empty() ? 0U : polyline.size() - 1U,
      .validated_through_revision = validated_through_revision,
      .complete_through_revision = node.complete_through_revision,
      .lineage_id = lineage_id,
      .unknown_exposure = unknown_exposure,
  };
  return IncrementalTopologyConnector3D{
      .node = node.id,
      .polyline = std::move(polyline),
      .length_m = length_m,
      .evidence = transition_evidence,
  };
}

} // namespace

std::uint64_t IncrementalTopologyGraph3DSnapshot::revision() const noexcept {
  return revision_;
}

std::uint64_t IncrementalTopologyGraph3DSnapshot::sourceSeenRevision() const noexcept {
  return source_seen_revision_;
}

std::uint64_t
IncrementalTopologyGraph3DSnapshot::materializedRevision() const noexcept {
  return materialized_revision_;
}

std::uint64_t
IncrementalTopologyGraph3DSnapshot::coverageCompleteThroughRevision() const noexcept {
  return coverage_complete_through_revision_;
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

std::size_t IncrementalTopologyGraph3DSnapshot::pendingBlockCount() const noexcept {
  return pending_block_count_;
}

std::size_t IncrementalTopologyGraph3DSnapshot::sampleBlockCount() const noexcept {
  return sample_blocks_.size();
}

std::size_t IncrementalTopologyGraph3DSnapshot::sampleCount() const noexcept {
  return sample_count_;
}

const IncrementalTopologyNode3D* IncrementalTopologyGraph3DSnapshot::findNode(
    const IncrementalTopologyNodeId id) const noexcept {
  const auto found = node_indices_.find(id);
  return found == node_indices_.end() ? nullptr : &nodes_[found->second];
}

std::optional<IncrementalTopologyNodeId>
IncrementalTopologyGraph3DSnapshot::nodeForSampleCell(
    const GridIndex3D cell) const noexcept {
  if (cell.x < 0 || cell.y < 0 || cell.z < 0 || cell.x >= bounds_.width_cells ||
      cell.y >= bounds_.height_cells || cell.z >= bounds_.depth_cells) {
    return std::nullopt;
  }
  const IncrementalTopologySampleBlock3D* const block = findSampleBlock(
      sample_blocks_,
      incremental_topology_detail::blockForCell(cell, sample_block_size_cells_));
  if (block == nullptr) {
    return std::nullopt;
  }
  const IncrementalTopologySampleRecord3D* const record =
      findSampleRecord(*block, cell);
  return record == nullptr ? std::nullopt
                           : std::optional<IncrementalTopologyNodeId>{record->node};
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
    const ObservedSpaceValidationPolicy validation_policy,
    const std::optional<std::chrono::steady_clock::time_point> deadline) const {
  if (!(maximum_distance_m >= 0.0) || !std::isfinite(maximum_distance_m) ||
      revision_ == 0U ||
      !incremental_topology_detail::sameBounds(bounds_, occupancy.bounds())) {
    return std::nullopt;
  }

  struct Candidate {
    const IncrementalTopologySampleBlock3D* block{nullptr};
    std::size_t sample_index{0U};
    IncrementalTopologyNodeId node{};
    double distance_m{0.0};
    double connector_length_m{0.0};
  };

  const auto candidate_less = [](const Candidate& first, const Candidate& second) {
    const GridIndex3D& first_cell = first.block->records[first.sample_index].cell;
    const GridIndex3D& second_cell = second.block->records[second.sample_index].cell;
    return std::tie(first.distance_m, first.node.value, first_cell.z, first_cell.y,
                    first_cell.x) < std::tie(second.distance_m, second.node.value,
                                             second_cell.z, second_cell.y,
                                             second_cell.x);
  };

  struct BucketCandidate {
    const IncrementalTopologySampleBlock3D* block{nullptr};
    double distance_lower_bound_m{0.0};
  };

  std::vector<BucketCandidate> nearby_buckets;
  nearby_buckets.reserve(sample_blocks_.size());
  for (const auto& block : sample_blocks_) {
    const double lower_bound_m = bucketDistanceLowerBound(
        bounds_, block->block, sample_block_size_cells_, position);
    if (lower_bound_m <= maximum_distance_m) {
      nearby_buckets.push_back(BucketCandidate{
          .block = block.get(),
          .distance_lower_bound_m = lower_bound_m,
      });
    }
  }
  std::ranges::sort(
      nearby_buckets, [](const BucketCandidate& first, const BucketCandidate& second) {
        return std::tie(first.distance_lower_bound_m, first.block->block) <
               std::tie(second.distance_lower_bound_m, second.block->block);
      });

  std::vector<Candidate> candidates;
  constexpr std::size_t kMaximumCandidates{64U};
  candidates.reserve(kMaximumCandidates);
  for (const BucketCandidate& bucket : nearby_buckets) {
    if (candidates.size() == kMaximumCandidates &&
        bucket.distance_lower_bound_m > candidates.front().distance_m + 1.0e-9) {
      break;
    }
    for (std::size_t sample_index = 0U; sample_index < bucket.block->records.size();
         ++sample_index) {
      const IncrementalTopologySampleRecord3D& sample =
          bucket.block->records[sample_index];
      const double distance_m = distance3D(position, occupancy.cellCenter(sample.cell));
      if (distance_m > maximum_distance_m) {
        continue;
      }
      const Candidate candidate{
          .block = bucket.block,
          .sample_index = sample_index,
          .node = sample.node,
          .distance_m = distance_m,
          .connector_length_m = distance_m + sample.distance_to_node_m,
      };
      if (candidates.size() < kMaximumCandidates) {
        candidates.push_back(candidate);
        std::ranges::push_heap(candidates, candidate_less);
      } else if (candidate_less(candidate, candidates.front())) {
        std::ranges::pop_heap(candidates, candidate_less);
        candidates.back() = candidate;
        std::ranges::push_heap(candidates, candidate_less);
      }
    }
  }
  std::ranges::sort(candidates, [](const Candidate& first, const Candidate& second) {
    const GridIndex3D& first_cell = first.block->records[first.sample_index].cell;
    const GridIndex3D& second_cell = second.block->records[second.sample_index].cell;
    return std::tie(first.connector_length_m, first.node.value, first_cell.z,
                    first_cell.y, first_cell.x) <
           std::tie(second.connector_length_m, second.node.value, second_cell.z,
                    second_cell.y, second_cell.x);
  });

  for (const Candidate& candidate : candidates) {
    if (deadline.has_value() && std::chrono::steady_clock::now() >= *deadline) {
      break;
    }
    const std::optional<SampleTreePath3D> tree = reconstructSampleTree(
        bounds_, *candidate.block, candidate.block->records[candidate.sample_index],
        deadline);
    const IncrementalTopologyNode3D* const node = findNode(candidate.node);
    if (!tree.has_value() || tree->node != candidate.node || node == nullptr) {
      continue;
    }
    std::vector<Point3> polyline;
    appendUnique(polyline, position);
    for (const Point3& point : tree->polyline) {
      appendUnique(polyline, point);
    }
    std::optional<IncrementalTopologyConnector3D> connector =
        makeObservedConnector(occupancy, *node, std::move(polyline), footprint,
                              validation_policy, revision_, deadline);
    if (connector.has_value()) {
      return connector;
    }
  }
  return std::nullopt;
}

std::optional<IncrementalTopologyConnector3D>
IncrementalTopologyGraph3DSnapshot::connectObservedSample(
    const ObservedOccupancyGrid3D& occupancy, const GridIndex3D cell,
    const SweptFootprintConfig& footprint,
    const ObservedSpaceValidationPolicy validation_policy) const {
  if (revision_ == 0U ||
      !incremental_topology_detail::sameBounds(bounds_, occupancy.bounds())) {
    return std::nullopt;
  }
  const IncrementalTopologySampleBlock3D* const block = findSampleBlock(
      sample_blocks_,
      incremental_topology_detail::blockForCell(cell, sample_block_size_cells_));
  const IncrementalTopologySampleRecord3D* const sample =
      block != nullptr ? findSampleRecord(*block, cell) : nullptr;
  if (block == nullptr || sample == nullptr) {
    return std::nullopt;
  }
  const std::optional<SampleTreePath3D> tree =
      reconstructSampleTree(bounds_, *block, *sample);
  const IncrementalTopologyNode3D* const node = findNode(sample->node);
  if (!tree.has_value() || node == nullptr || tree->node != node->id) {
    return std::nullopt;
  }
  return makeObservedConnector(occupancy, *node, tree->polyline, footprint,
                               validation_policy, revision_);
}

} // namespace drone_city_nav
