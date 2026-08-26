#pragma once

namespace drone_city_nav {

template<typename Occupancy>
[[nodiscard]] IncrementalTopologyGraph3DUpdate
IncrementalTopologyGraph3D::Impl::rebuild(
    const Occupancy& occupancy, const std::uint64_t update_revision,
    std::vector<IncrementalTopologyBlockIndex3D> rebuilt_blocks,
    const std::size_t requested_dirty_chunks, const bool full_reset,
    const std::optional<std::chrono::steady_clock::time_point> deadline,
    std::vector<IncrementalTopologyBlockIndex3D>* const deferred_blocks) {
  IncrementalTopologyGraph3DUpdate stats{
      .revision = update_revision,
      .requested_dirty_chunks = requested_dirty_chunks,
      .rebuilt_blocks = 0U,
      .full_reset = full_reset,
  };
  if (full_reset) {
    stats.retired_nodes = nodes.size();
    blocks.clear();
    nodes.clear();
    edges.clear();
    sample_cell_nodes.clear();
    sample_cell_parents.clear();
    block_validated_through.clear();
    block_complete_through.clear();
    issued_node_ids.clear();
    edge_created_on_revision.clear();
  }
  std::vector<BuiltBlock> replacements;
  replacements.reserve(rebuilt_blocks.size());
  auto stage_started = std::chrono::steady_clock::now();
  std::size_t built_block_count{0U};
  for (const IncrementalTopologyBlockIndex3D block : rebuilt_blocks) {
    const bool guarantee_first_block = built_block_count == 0U && deadline.has_value();
    if (!guarantee_first_block && deadline.has_value() &&
        std::chrono::steady_clock::now() >= *deadline) {
      break;
    }
    try {
      replacements.push_back(buildBlock(
          occupancy, block,
          guarantee_first_block ? std::optional<std::chrono::steady_clock::time_point>{}
                                : deadline));
    } catch (const BuildCancelled&) {
      break;
    }
    ++built_block_count;
    stats.adaptively_refined_blocks += replacements.back().adaptively_refined ? 1U : 0U;
    for (const BlockComponent& component : replacements.back().components) {
      stats.sampled_navigable_cells += component.cells.size();
    }
  }
  if (deferred_blocks != nullptr && built_block_count < rebuilt_blocks.size()) {
    deferred_blocks->insert(deferred_blocks->end(),
                            rebuilt_blocks.begin() +
                                static_cast<std::ptrdiff_t>(built_block_count),
                            rebuilt_blocks.end());
  }
  rebuilt_blocks.resize(built_block_count);
  stats.rebuilt_blocks = built_block_count;
  stats.minimum_progress_guaranteed =
      deadline.has_value() && !rebuilt_blocks.empty() && built_block_count > 0U;
  stats.block_build_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - stage_started)
                             .count();
  stage_started = std::chrono::steady_clock::now();
  for (std::size_t index = 0U; index < rebuilt_blocks.size(); ++index) {
    replaceBlock(rebuilt_blocks[index], std::move(replacements[index]), update_revision,
                 stats);
  }
  stats.block_replace_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - stage_started)
                               .count();
  stage_started = std::chrono::steady_clock::now();
  connectRebuiltBlocks(occupancy, rebuilt_blocks, update_revision);
  stats.block_connect_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - stage_started)
                               .count();
  stage_started = std::chrono::steady_clock::now();
  classifyNodes(update_revision);
  stats.node_classification_ms = std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - stage_started)
                                     .count();
  graph_revision = update_revision;
  stats.node_count = nodes.size();
  stats.edge_count = edges.size();
  return stats;
}

} // namespace drone_city_nav
