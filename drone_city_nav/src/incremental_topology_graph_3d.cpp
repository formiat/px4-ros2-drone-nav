#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <vector>

#include "incremental_topology_graph_3d_impl.hpp"

namespace drone_city_nav {

using incremental_topology_detail::sameBounds;

IncrementalTopologyGraph3D::IncrementalTopologyGraph3D(
    IncrementalTopologyGraph3DConfig config)
    : impl_{std::make_unique<Impl>(config)} {
}

IncrementalTopologyGraph3D::~IncrementalTopologyGraph3D() = default;
IncrementalTopologyGraph3D::IncrementalTopologyGraph3D(
    IncrementalTopologyGraph3D&&) noexcept = default;
IncrementalTopologyGraph3D&
IncrementalTopologyGraph3D::operator=(IncrementalTopologyGraph3D&&) noexcept = default;

IncrementalTopologyGraph3DUpdate IncrementalTopologyGraph3D::update(
    const ObservedOccupancyGrid3D& occupancy, const std::uint64_t revision,
    const std::span<const OccupancyChunkIndex3D> dirty_chunks, const bool full_reset,
    const std::optional<IncrementalTopologyBuildPriority3D> priority,
    const std::optional<std::chrono::steady_clock::time_point> deadline) {
  if (revision == 0U) {
    throw std::invalid_argument{"incremental topology revision must be non-zero"};
  }
  const bool geometry_changed =
      impl_->graph_revision != 0U && !sameBounds(impl_->bounds, occupancy.bounds());
  const bool reset_required = impl_->graph_revision == 0U || geometry_changed;
  const bool continue_pending_revision = !reset_required &&
                                         revision == impl_->graph_revision &&
                                         impl_->observed_blocks.pendingCount() > 0U;
  if (!reset_required &&
      (revision < impl_->graph_revision ||
       (revision == impl_->graph_revision && !continue_pending_revision))) {
    return IncrementalTopologyGraph3DUpdate{
        .revision = impl_->graph_revision,
        .node_count = impl_->nodes.size(),
        .edge_count = impl_->edges.size(),
    };
  }
  impl_->bounds = occupancy.bounds();
  const std::vector<OccupancyChunkIndex3D> complete_snapshot_chunks =
      !reset_required && !continue_pending_revision && full_reset
          ? impl_->observed_blocks.completeSnapshotChunks(occupancy)
          : std::vector<OccupancyChunkIndex3D>{};
  const std::span<const OccupancyChunkIndex3D> effective_dirty_chunks =
      !complete_snapshot_chunks.empty()
          ? std::span<const OccupancyChunkIndex3D>{complete_snapshot_chunks}
          : dirty_chunks;
  const auto discovery_started = std::chrono::steady_clock::now();
  std::vector<IncrementalTopologyBlockIndex3D> dirty_blocks;
  std::vector<IncrementalTopologyBlockIndex3D> observation_evidence_blocks;
  if (continue_pending_revision) {
    // Dirty discovery already ran for this immutable raw revision. Continue the
    // bounded pending-block queue without requiring another sensor update.
  } else if (reset_required) {
    dirty_blocks = impl_->observed_blocks.allObservedBlocks(occupancy);
  } else {
    incremental_topology_detail::ObservedBlockChanges3D changes =
        impl_->observed_blocks.dirtyObservedBlocks(occupancy, effective_dirty_chunks);
    dirty_blocks = std::move(changes.geometry);
    observation_evidence_blocks = std::move(changes.observation_evidence);
    for (const IncrementalTopologyBlockIndex3D block : observation_evidence_blocks) {
      if (!impl_->block_validated_through.contains(block)) {
        dirty_blocks.push_back(block);
      }
    }
    std::ranges::sort(dirty_blocks);
    const auto unique_end = std::unique(dirty_blocks.begin(), dirty_blocks.end());
    dirty_blocks.erase(unique_end, dirty_blocks.end());
  }
  const double dirty_block_discovery_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                discovery_started)
          .count();
  const std::size_t discovered_dirty_blocks = dirty_blocks.size();
  incremental_topology_detail::ObservedBlockSelection3D selection;
  if (reset_required) {
    impl_->observed_blocks.clearPending();
    impl_->observed_blocks.enqueue(dirty_blocks);
    selection = impl_->observed_blocks.takePending(impl_->bounds, priority, false);
  } else if (continue_pending_revision) {
    selection = impl_->observed_blocks.takePending(impl_->bounds, priority, true);
  } else {
    impl_->observed_blocks.enqueue(dirty_blocks);
    selection = impl_->observed_blocks.takePending(impl_->bounds, priority, true);
  }
  std::vector<IncrementalTopologyBlockIndex3D> rebuilt_blocks =
      std::move(selection.blocks);
  const std::vector<IncrementalTopologyBlockIndex3D> selected_blocks = rebuilt_blocks;
  const auto rebuild_started = std::chrono::steady_clock::now();
  std::vector<IncrementalTopologyBlockIndex3D> deferred_blocks;
  IncrementalTopologyGraph3DUpdate result = impl_->rebuild(
      occupancy, revision, std::move(rebuilt_blocks), effective_dirty_chunks.size(),
      reset_required, deadline, std::addressof(deferred_blocks));
  if (!deferred_blocks.empty()) {
    impl_->observed_blocks.enqueue(deferred_blocks);
  }
  const std::vector<IncrementalTopologyBlockIndex3D> rebuilt_block_copy =
      [&result, &selected_blocks]() {
        std::vector<IncrementalTopologyBlockIndex3D> completed = selected_blocks;
        completed.resize(result.rebuilt_blocks);
        return completed;
      }();
  if (!reset_required && !impl_->config.require_known_free_space) {
    result.refreshed_observation_blocks = impl_->refreshObservationEvidence(
        occupancy, observation_evidence_blocks, rebuilt_block_copy, revision);
    if (result.refreshed_observation_blocks > 0U) {
      impl_->classifyNodes(revision);
    }
  }
  result.dirty_block_discovery_ms = dirty_block_discovery_ms;
  result.graph_rebuild_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - rebuild_started)
                                .count();
  result.discovered_dirty_blocks = discovered_dirty_blocks;
  result.pending_blocks = impl_->observed_blocks.pendingCount();
  result.scheduled_block_budget = selection.budget;
  result.local_priority_blocks = selection.local_priority_blocks;
  result.forward_corridor_blocks = selection.forward_corridor_blocks;
  result.oldest_preserved_blocks = selection.oldest_preserved_blocks;
  result.backlog_boosted = selection.backlog_boosted;
  result.deadline_exhausted = !deferred_blocks.empty();
  if (reset_required) {
    impl_->observed_blocks.replaceObservedSnapshot(occupancy.chunks());
  }
  return result;
}

IncrementalTopologyGraph3DUpdate
IncrementalTopologyGraph3D::reset(const OccupancyGrid3D& occupancy,
                                  const std::uint64_t revision) {
  if (revision == 0U) {
    throw std::invalid_argument{"incremental topology revision must be non-zero"};
  }
  impl_->bounds = occupancy.bounds();
  impl_->observed_blocks.clearPending();
  impl_->observed_blocks.clearObservedSnapshot();
  return impl_->rebuild(occupancy, revision,
                        impl_->observed_blocks.allStaticBlocks(impl_->bounds), 0U,
                        true);
}

IncrementalTopologyGraph3DSnapshot IncrementalTopologyGraph3D::snapshot() const {
  IncrementalTopologyGraph3DSnapshot result;
  result.revision_ = impl_->graph_revision;
  result.bounds_ = impl_->bounds;
  result.nodes_.reserve(impl_->nodes.size());
  for (const auto& [node_id, node] : impl_->nodes) {
    static_cast<void>(node_id);
    result.nodes_.push_back(node);
  }
  std::ranges::sort(result.nodes_, {}, &IncrementalTopologyNode3D::id);
  for (std::size_t index = 0U; index < result.nodes_.size(); ++index) {
    result.node_indices_.emplace(result.nodes_[index].id, index);
  }
  result.edges_.reserve(impl_->edges.size());
  for (const auto& [edge_id, edge] : impl_->edges) {
    static_cast<void>(edge_id);
    result.edges_.push_back(edge);
  }
  std::ranges::sort(result.edges_, {}, &IncrementalTopologyEdge3D::id);
  const auto& pending_blocks = impl_->observed_blocks.pending();
  result.block_coverage_.reserve(impl_->block_validated_through.size() +
                                 pending_blocks.size());
  for (const auto& [block, validated_through] : impl_->block_validated_through) {
    result.block_coverage_.push_back(IncrementalTopologyBlockCoverage3D{
        .block = block,
        .validated_through_revision = validated_through,
        .complete_through_revision = impl_->block_complete_through.at(block),
        .pending_rebuild = pending_blocks.contains(block),
    });
  }
  for (const IncrementalTopologyBlockIndex3D block : pending_blocks) {
    if (!impl_->block_validated_through.contains(block)) {
      result.block_coverage_.push_back(IncrementalTopologyBlockCoverage3D{
          .block = block,
          .pending_rebuild = true,
      });
    }
  }
  std::ranges::sort(result.block_coverage_, {},
                    &IncrementalTopologyBlockCoverage3D::block);
  result.pending_block_count_ = pending_blocks.size();
  result.sample_block_size_cells_ = impl_->config.block_size_cells;
  result.sample_blocks_.reserve(impl_->blocks.size());
  for (const auto& [block, data] : impl_->blocks) {
    static_cast<void>(block);
    if (data.snapshot_samples) {
      result.sample_count_ += data.snapshot_samples->records.size();
      result.sample_blocks_.push_back(data.snapshot_samples);
    }
  }
  std::ranges::sort(result.sample_blocks_, {},
                    [](const auto& block) { return block->block; });
  return result;
}

const IncrementalTopologyGraph3DConfig&
IncrementalTopologyGraph3D::config() const noexcept {
  return impl_->config;
}

} // namespace drone_city_nav
