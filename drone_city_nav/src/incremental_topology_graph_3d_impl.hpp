#pragma once

#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <numbers>
#include <ranges>
#include <span>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "incremental_topology_graph_3d_internal.hpp"
#include "incremental_topology_observed_blocks_3d.hpp"

namespace drone_city_nav {

using incremental_topology_detail::appendUniquePoint;
using incremental_topology_detail::blockForCell;
using incremental_topology_detail::cardinalNeighbors;
using incremental_topology_detail::cellCenter;
using incremental_topology_detail::cellLess;
using incremental_topology_detail::firstAlignedCell;
using incremental_topology_detail::makeEdgeId;
using incremental_topology_detail::makeNodeIdValue;
using incremental_topology_detail::makeTransitionEvidence;
using incremental_topology_detail::makeTransitionLineage;
using incremental_topology_detail::navigableAt;
using incremental_topology_detail::navigableBetween;
using incremental_topology_detail::polylineLength;
using incremental_topology_detail::sampleCellKey;

struct IncrementalTopologyGraph3D::Impl {
  struct BlockComponent {
    IncrementalTopologyNodeId id{};
    std::vector<GridIndex3D> cells;
    GridIndex3D representative_cell{};
    Point3 representative{};
    std::unordered_map<std::uint64_t, GridIndex3D> parent_by_cell;
    bool unknown_boundary_exposure{false};
  };

  struct BlockData {
    std::vector<BlockComponent> components;
    int sample_stride_cells{1};
    std::shared_ptr<const IncrementalTopologySampleBlock3D> snapshot_samples;
  };

  struct BuiltBlock {
    std::vector<BlockComponent> components;
    int sample_stride_cells{1};
    bool adaptively_refined{false};
  };

  explicit Impl(const IncrementalTopologyGraph3DConfig& requested_config)
      : config{requested_config},
        observed_blocks{requested_config} {
    if (!incrementalTopologyGraph3DConfigIsValid(config)) {
      throw std::invalid_argument{"invalid incremental 3d topology configuration"};
    }
  }

  template<typename Occupancy>
  [[nodiscard]] bool rawFreeAt(const Occupancy& occupancy,
                               const GridIndex3D cell) const noexcept {
    if (!occupancy.contains(cell)) {
      return false;
    }
    if constexpr (std::is_same_v<Occupancy, ObservedOccupancyGrid3D>) {
      return occupancy.isKnownFree(cell);
    }
    return !occupancy.isOccupied(cell);
  }

  template<typename Occupancy>
  [[nodiscard]] bool knownOccupiedAt(const Occupancy& occupancy,
                                     const GridIndex3D cell) const noexcept {
    if (!occupancy.contains(cell)) {
      return false;
    }
    if constexpr (std::is_same_v<Occupancy, ObservedOccupancyGrid3D>) {
      return occupancy.state(cell) == ObservedVoxelState::kOccupied;
    }
    return occupancy.isOccupied(cell);
  }

  template<typename Occupancy>
  [[nodiscard]] bool
  blockIsNearKnownObstacle(const Occupancy& occupancy,
                           const IncrementalTopologyBlockIndex3D block) const {
    if (config.refined_sample_stride_cells == config.coarse_sample_stride_cells) {
      return false;
    }
    const int minimum_x = block.x * config.block_size_cells;
    const int minimum_y = block.y * config.block_size_cells;
    const int minimum_z = block.z * config.block_size_cells;
    const int maximum_x =
        std::min(bounds.width_cells, minimum_x + config.block_size_cells);
    const int maximum_y =
        std::min(bounds.height_cells, minimum_y + config.block_size_cells);
    const int maximum_z =
        std::min(bounds.depth_cells, minimum_z + config.block_size_cells);
    bool contains_free_evidence = false;
    for (int z = minimum_z; z < maximum_z && !contains_free_evidence; ++z) {
      for (int y = minimum_y; y < maximum_y && !contains_free_evidence; ++y) {
        for (int x = minimum_x; x < maximum_x; ++x) {
          if (rawFreeAt(occupancy, {x, y, z})) {
            contains_free_evidence = true;
            break;
          }
        }
      }
    }
    if (!contains_free_evidence) {
      return false;
    }

    const double maximum_extent_m =
        std::max({config.footprint.radius_m, config.footprint.lower_extent_m,
                  config.footprint.upper_extent_m});
    const int footprint_halo_cells =
        static_cast<int>(std::ceil(maximum_extent_m / bounds.resolution_m));
    const int refinement_halo_cells =
        footprint_halo_cells + config.coarse_sample_stride_cells;
    for (int z = minimum_z - refinement_halo_cells;
         z < maximum_z + refinement_halo_cells; ++z) {
      for (int y = minimum_y - refinement_halo_cells;
           y < maximum_y + refinement_halo_cells; ++y) {
        for (int x = minimum_x - refinement_halo_cells;
             x < maximum_x + refinement_halo_cells; ++x) {
          const GridIndex3D cell{x, y, z};
          if (knownOccupiedAt(occupancy, cell)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  template<typename Occupancy>
  [[nodiscard]] BuiltBlock
  buildBlock(const Occupancy& occupancy,
             const IncrementalTopologyBlockIndex3D block) const {
    const int minimum_x = block.x * config.block_size_cells;
    const int minimum_y = block.y * config.block_size_cells;
    const int minimum_z = block.z * config.block_size_cells;
    const int maximum_x =
        std::min(bounds.width_cells, minimum_x + config.block_size_cells);
    const int maximum_y =
        std::min(bounds.height_cells, minimum_y + config.block_size_cells);
    const int maximum_z =
        std::min(bounds.depth_cells, minimum_z + config.block_size_cells);
    bool adaptively_refined = blockIsNearKnownObstacle(occupancy, block);
    int sample_stride_cells = adaptively_refined ? config.refined_sample_stride_cells
                                                 : config.coarse_sample_stride_cells;
    const auto sample_navigable_cells = [&](const int stride_cells) {
      std::vector<GridIndex3D> result;
      for (int z = firstAlignedCell(minimum_z, stride_cells); z < maximum_z;
           z += stride_cells) {
        for (int y = firstAlignedCell(minimum_y, stride_cells); y < maximum_y;
             y += stride_cells) {
          for (int x = firstAlignedCell(minimum_x, stride_cells); x < maximum_x;
               x += stride_cells) {
            const GridIndex3D cell{x, y, z};
            if (navigableAt(occupancy, occupancy.cellCenter(cell), config.footprint,
                            config.require_known_free_space)) {
              result.push_back(cell);
            }
          }
        }
      }
      return result;
    };
    std::vector<GridIndex3D> navigable_cells =
        sample_navigable_cells(sample_stride_cells);
    if (navigable_cells.empty() && !adaptively_refined &&
        config.refined_sample_stride_cells < config.coarse_sample_stride_cells) {
      std::vector<GridIndex3D> refined_cells =
          sample_navigable_cells(config.refined_sample_stride_cells);
      if (!refined_cells.empty()) {
        adaptively_refined = true;
        sample_stride_cells = config.refined_sample_stride_cells;
        navigable_cells = std::move(refined_cells);
      }
    }
    std::unordered_map<std::uint64_t, std::size_t> cell_indices;
    for (std::size_t index = 0U; index < navigable_cells.size(); ++index) {
      cell_indices.emplace(sampleCellKey(bounds, navigable_cells[index]), index);
    }

    std::vector<bool> visited(navigable_cells.size(), false);
    std::vector<BlockComponent> components;
    for (std::size_t seed = 0U; seed < navigable_cells.size(); ++seed) {
      if (visited[seed]) {
        continue;
      }
      BlockComponent component;
      std::deque<std::size_t> pending;
      pending.push_back(seed);
      visited[seed] = true;
      component.parent_by_cell.emplace(sampleCellKey(bounds, navigable_cells[seed]),
                                       navigable_cells[seed]);
      while (!pending.empty()) {
        const std::size_t index = pending.front();
        pending.pop_front();
        const GridIndex3D cell = navigable_cells[index];
        component.cells.push_back(cell);
        for (const GridIndex3D neighbor :
             cardinalNeighbors(cell, sample_stride_cells)) {
          const auto found = cell_indices.find(sampleCellKey(bounds, neighbor));
          if (found == cell_indices.end() || visited[found->second]) {
            continue;
          }
          if (!navigableBetween(occupancy, occupancy.cellCenter(cell),
                                occupancy.cellCenter(neighbor), config.footprint,
                                config.require_known_free_space)) {
            continue;
          }
          visited[found->second] = true;
          component.parent_by_cell.emplace(sampleCellKey(bounds, neighbor), cell);
          pending.push_back(found->second);
        }
      }
      std::ranges::sort(component.cells, cellLess);
      component.representative_cell = representativeCellFor(occupancy, component.cells);
      component.representative = occupancy.cellCenter(component.representative_cell);
      rerootParentTree(component);
      if constexpr (std::is_same_v<Occupancy, ObservedOccupancyGrid3D>) {
        component.unknown_boundary_exposure =
            componentTouchesUnknown(occupancy, component.cells, sample_stride_cells);
      }
      components.push_back(std::move(component));
    }
    std::ranges::sort(components,
                      [](const BlockComponent& first, const BlockComponent& second) {
                        return cellLess(first.cells.front(), second.cells.front());
                      });
    return BuiltBlock{.components = std::move(components),
                      .sample_stride_cells = sample_stride_cells,
                      .adaptively_refined = adaptively_refined};
  }

  [[nodiscard]] static bool
  componentTouchesUnknown(const ObservedOccupancyGrid3D& occupancy,
                          const std::span<const GridIndex3D> cells,
                          const int stride_cells) noexcept {
    for (const GridIndex3D cell : cells) {
      if (!occupancy.isKnownFree(cell)) {
        continue;
      }
      for (const GridIndex3D neighbor : cardinalNeighbors(cell, stride_cells)) {
        if (occupancy.contains(neighbor) &&
            occupancy.state(neighbor) == ObservedVoxelState::kUnknown) {
          return true;
        }
      }
    }
    return false;
  }

  [[nodiscard]] std::size_t refreshObservationEvidence(
      const ObservedOccupancyGrid3D& occupancy,
      const std::span<const IncrementalTopologyBlockIndex3D> evidence_blocks,
      const std::span<const IncrementalTopologyBlockIndex3D> rebuilt_blocks,
      const std::uint64_t update_revision) {
    std::unordered_set<IncrementalTopologyBlockIndex3D,
                       IncrementalTopologyBlockIndex3DHash>
        rebuilt{rebuilt_blocks.begin(), rebuilt_blocks.end()};
    std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash>
        refreshed_nodes;
    std::size_t refreshed_blocks = 0U;
    for (const IncrementalTopologyBlockIndex3D block : evidence_blocks) {
      if (rebuilt.contains(block)) {
        continue;
      }
      const auto block_found = blocks.find(block);
      if (block_found == blocks.end()) {
        continue;
      }
      ++refreshed_blocks;
      block_validated_through[block] =
          std::max(block_validated_through[block], update_revision);
      for (BlockComponent& component : block_found->second.components) {
        component.unknown_boundary_exposure = componentTouchesUnknown(
            occupancy, component.cells, block_found->second.sample_stride_cells);
        auto node = nodes.find(component.id);
        if (node == nodes.end()) {
          continue;
        }
        node->second.unknown_boundary_exposure = component.unknown_boundary_exposure;
        node->second.validated_through_revision =
            std::max(node->second.validated_through_revision, update_revision);
        node->second.classification_revision = update_revision;
        refreshed_nodes.insert(component.id);
      }
    }
    if (refreshed_nodes.empty()) {
      return refreshed_blocks;
    }
    for (auto& [edge_id, edge] : edges) {
      static_cast<void>(edge_id);
      if (!refreshed_nodes.contains(edge.first) &&
          !refreshed_nodes.contains(edge.second)) {
        continue;
      }
      const std::uint64_t support =
          std::min(block_validated_through.at(nodes.at(edge.first).block),
                   block_validated_through.at(nodes.at(edge.second).block));
      const std::uint64_t validated_through_revision =
          std::max(edge.evidence.validated_through_revision, support);
      const std::uint64_t complete_through_revision =
          std::min(block_complete_through.at(nodes.at(edge.first).block),
                   block_complete_through.at(nodes.at(edge.second).block));
      const std::uint64_t lineage_id =
          makeTransitionLineage(edge.id.value, edge.polyline);
      edge.evidence = makeTransitionEvidence(occupancy, edge.polyline, config.footprint,
                                             validated_through_revision,
                                             complete_through_revision, lineage_id);
    }
    return refreshed_blocks;
  }

  template<typename Occupancy>
  [[nodiscard]] GridIndex3D
  representativeCellFor(const Occupancy& occupancy,
                        const std::span<const GridIndex3D> cells) const {
    Point3 mean{};
    for (const GridIndex3D cell : cells) {
      const Point3 center = occupancy.cellCenter(cell);
      mean.x += center.x;
      mean.y += center.y;
      mean.z += center.z;
    }
    const double inverse_count = 1.0 / static_cast<double>(cells.size());
    mean.x *= inverse_count;
    mean.y *= inverse_count;
    mean.z *= inverse_count;
    GridIndex3D best = cells.front();
    double best_distance = squaredDistance(occupancy.cellCenter(best), mean);
    for (const GridIndex3D cell : cells.subspan(1U)) {
      const double candidate_distance =
          squaredDistance(occupancy.cellCenter(cell), mean);
      if (candidate_distance + 1.0e-12 < best_distance ||
          (std::abs(candidate_distance - best_distance) <= 1.0e-12 &&
           cellLess(cell, best))) {
        best = cell;
        best_distance = candidate_distance;
      }
    }
    return best;
  }

  void rerootParentTree(BlockComponent& component) const {
    GridIndex3D current = component.representative_cell;
    GridIndex3D replacement_parent = current;
    for (std::size_t guard = 0U; guard <= component.cells.size(); ++guard) {
      const auto parent = component.parent_by_cell.find(sampleCellKey(bounds, current));
      if (parent == component.parent_by_cell.end()) {
        throw std::logic_error{"topology discovery tree is missing its representative"};
      }
      const GridIndex3D previous_parent = parent->second;
      parent->second = replacement_parent;
      if (previous_parent == current) {
        return;
      }
      replacement_parent = current;
      current = previous_parent;
    }
    throw std::logic_error{"topology discovery tree contains a cycle"};
  }

  [[nodiscard]] std::vector<Point3>
  pathFromRepresentative(const BlockComponent& component,
                         const GridIndex3D target) const {
    std::vector<GridIndex3D> reverse_cells;
    reverse_cells.reserve(component.cells.size());
    GridIndex3D current = target;
    for (std::size_t guard = 0U; guard <= component.cells.size(); ++guard) {
      reverse_cells.push_back(current);
      if (current == component.representative_cell) {
        break;
      }
      const auto parent = component.parent_by_cell.find(sampleCellKey(bounds, current));
      if (parent == component.parent_by_cell.end() || parent->second == current) {
        return {};
      }
      current = parent->second;
    }
    if (reverse_cells.back() != component.representative_cell) {
      return {};
    }
    std::ranges::reverse(reverse_cells);
    std::vector<Point3> result;
    result.reserve(reverse_cells.size());
    for (const GridIndex3D cell : reverse_cells) {
      result.push_back(cellCenter(bounds, cell));
    }
    return result;
  }

  [[nodiscard]] const BlockComponent*
  componentForNode(const IncrementalTopologyNodeId id) const noexcept {
    const auto node = nodes.find(id);
    if (node == nodes.end()) {
      return nullptr;
    }
    const auto block = blocks.find(node->second.block);
    if (block == blocks.end()) {
      return nullptr;
    }
    const auto component =
        std::ranges::find(block->second.components, id, &BlockComponent::id);
    return component == block->second.components.end() ? nullptr : &*component;
  }

  [[nodiscard]] IncrementalTopologyNodeId
  allocateNodeId(const IncrementalTopologyBlockIndex3D block,
                 const GridIndex3D anchor) const {
    for (std::uint64_t salt = 0U;; ++salt) {
      const IncrementalTopologyNodeId candidate{makeNodeIdValue(block, anchor, salt)};
      if (!issued_node_ids.contains(candidate)) {
        return candidate;
      }
    }
  }

  void removeIncidentEdges(
      const std::unordered_set<IncrementalTopologyNodeId,
                               IncrementalTopologyNodeIdHash>& node_ids) {
    std::erase_if(edges, [&node_ids](const auto& entry) {
      return node_ids.contains(entry.second.first) ||
             node_ids.contains(entry.second.second);
    });
  }

  void replaceBlock(const IncrementalTopologyBlockIndex3D block, BuiltBlock replacement,
                    const std::uint64_t update_revision,
                    IncrementalTopologyGraph3DUpdate& stats) {
    std::vector<BlockComponent>& components = replacement.components;
    std::vector<BlockComponent> previous;
    if (const auto found = blocks.find(block); found != blocks.end()) {
      previous = std::move(found->second.components);
      blocks.erase(found);
    }
    std::unordered_map<std::uint64_t, IncrementalTopologyNodeId> previous_cells;
    std::unordered_map<IncrementalTopologyNodeId, IncrementalTopologyNode3D,
                       IncrementalTopologyNodeIdHash>
        previous_nodes;
    std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash>
        previous_ids;
    for (const BlockComponent& component : previous) {
      previous_ids.insert(component.id);
      previous_nodes.emplace(component.id, nodes.at(component.id));
      for (const GridIndex3D cell : component.cells) {
        const std::uint64_t key = sampleCellKey(bounds, cell);
        previous_cells.emplace(key, component.id);
        sample_cell_nodes.erase(key);
        sample_cell_parents.erase(key);
      }
      nodes.erase(component.id);
    }
    removeIncidentEdges(previous_ids);

    using Overlaps = std::unordered_map<IncrementalTopologyNodeId, std::size_t,
                                        IncrementalTopologyNodeIdHash>;
    std::vector<Overlaps> component_overlaps(components.size());
    std::unordered_map<IncrementalTopologyNodeId, std::vector<std::size_t>,
                       IncrementalTopologyNodeIdHash>
        successor_indices;
    for (std::size_t index = 0U; index < components.size(); ++index) {
      BlockComponent& component = components[index];
      Overlaps& overlaps = component_overlaps[index];
      for (const GridIndex3D cell : component.cells) {
        if (const auto found = previous_cells.find(sampleCellKey(bounds, cell));
            found != previous_cells.end()) {
          ++overlaps[found->second];
        }
      }
      if (overlaps.empty()) {
        const double maximum_match_distance_m =
            std::numbers::sqrt3 *
            static_cast<double>(config.coarse_sample_stride_cells) *
            bounds.resolution_m;
        const BlockComponent* closest_previous{nullptr};
        double closest_distance_m = maximum_match_distance_m;
        for (const BlockComponent& previous_component : previous) {
          const double candidate_distance_m =
              distance3D(previous_component.representative, component.representative);
          if (candidate_distance_m + 1.0e-9 < closest_distance_m ||
              (std::abs(candidate_distance_m - closest_distance_m) <= 1.0e-9 &&
               (closest_previous == nullptr ||
                previous_component.id < closest_previous->id))) {
            closest_distance_m = candidate_distance_m;
            closest_previous = &previous_component;
          }
        }
        if (closest_previous != nullptr) {
          overlaps.emplace(closest_previous->id, 1U);
        }
      }
      for (const auto& [predecessor, overlap] : overlaps) {
        static_cast<void>(overlap);
        successor_indices[predecessor].push_back(index);
      }
    }

    std::unordered_map<IncrementalTopologyNodeId, std::size_t,
                       IncrementalTopologyNodeIdHash>
        inheriting_successor;
    for (const auto& [predecessor, successors] : successor_indices) {
      const auto winner =
          std::ranges::max_element(successors, {}, [&](const auto index) {
            return std::tuple{component_overlaps[index].at(predecessor),
                              std::numeric_limits<std::size_t>::max() - index};
          });
      inheriting_successor.emplace(predecessor, *winner);
    }

    std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash>
        retained_ids;
    for (std::size_t index = 0U; index < components.size(); ++index) {
      BlockComponent& component = components[index];
      const Overlaps& overlaps = component_overlaps[index];
      std::vector<IncrementalTopologyNodeId> predecessors;
      predecessors.reserve(overlaps.size());
      for (const auto& [predecessor, overlap] : overlaps) {
        static_cast<void>(overlap);
        predecessors.push_back(predecessor);
      }
      std::ranges::sort(predecessors);

      std::optional<IncrementalTopologyNodeId> inherited;
      for (const IncrementalTopologyNodeId predecessor : predecessors) {
        if (inheriting_successor.at(predecessor) != index) {
          continue;
        }
        if (!inherited.has_value() ||
            component_overlaps[index].at(predecessor) >
                component_overlaps[index].at(*inherited) ||
            (component_overlaps[index].at(predecessor) ==
                 component_overlaps[index].at(*inherited) &&
             predecessor < *inherited)) {
          inherited = predecessor;
        }
      }
      const bool split = std::ranges::any_of(predecessors, [&](const auto predecessor) {
        return successor_indices.at(predecessor).size() > 1U;
      });
      const bool merge = predecessors.size() > 1U;
      IncrementalTopologyLineageEvent3D lineage_event{
          IncrementalTopologyLineageEvent3D::kCreated};
      if (merge) {
        lineage_event = IncrementalTopologyLineageEvent3D::kMerge;
      } else if (split) {
        lineage_event = IncrementalTopologyLineageEvent3D::kSplit;
      } else if (inherited.has_value()) {
        lineage_event = IncrementalTopologyLineageEvent3D::kRetained;
      }

      std::uint64_t generation = 0U;
      std::uint64_t created_on_revision = update_revision;
      if (inherited.has_value() && !retained_ids.contains(*inherited)) {
        component.id = *inherited;
        retained_ids.insert(component.id);
        const IncrementalTopologyNode3D& prior = previous_nodes.at(component.id);
        created_on_revision = prior.created_on_revision;
        generation = prior.generation + (split || merge ? 1U : 0U);
        ++stats.retained_node_ids;
      } else {
        component.id = allocateNodeId(block, component.representative_cell);
        for (const IncrementalTopologyNodeId predecessor : predecessors) {
          generation =
              std::max(generation, previous_nodes.at(predecessor).generation + 1U);
        }
        ++stats.created_nodes;
      }
      issued_node_ids.insert(component.id);
      IncrementalTopologyNode3D node{
          .id = component.id,
          .block = block,
          .representative = component.representative,
          .support_cell_count = component.cells.size(),
          .created_on_revision = created_on_revision,
          .validated_through_revision = update_revision,
          .complete_through_revision = update_revision,
          .generation = generation,
          .classification_revision = update_revision,
          .lineage_event = lineage_event,
          .predecessors = std::move(predecessors),
          .unknown_boundary_exposure = component.unknown_boundary_exposure,
      };
      nodes.emplace(node.id, node);
      for (const GridIndex3D cell : component.cells) {
        const std::uint64_t key = sampleCellKey(bounds, cell);
        sample_cell_nodes[key] = component.id;
        sample_cell_parents[key] =
            sampleCellKey(bounds, component.parent_by_cell.at(key));
      }
    }
    stats.retired_nodes += previous.size() - retained_ids.size();
    block_validated_through[block] = update_revision;
    block_complete_through[block] = update_revision;
    if (!components.empty()) {
      auto snapshot_samples = std::make_shared<IncrementalTopologySampleBlock3D>();
      snapshot_samples->block = block;
      std::size_t sample_count{0U};
      for (const BlockComponent& component : components) {
        sample_count += component.cells.size();
      }
      snapshot_samples->records.reserve(sample_count);
      for (const BlockComponent& component : components) {
        for (const GridIndex3D cell : component.cells) {
          const std::uint64_t key = sampleCellKey(bounds, cell);
          snapshot_samples->records.push_back(IncrementalTopologySampleRecord3D{
              .cell = cell,
              .parent_cell = component.parent_by_cell.at(key),
              .node = component.id,
          });
        }
      }
      std::ranges::sort(snapshot_samples->records,
                        [](const IncrementalTopologySampleRecord3D& first,
                           const IncrementalTopologySampleRecord3D& second) {
                          return cellLess(first.cell, second.cell);
                        });
      blocks.emplace(block,
                     BlockData{.components = std::move(components),
                               .sample_stride_cells = replacement.sample_stride_cells,
                               .snapshot_samples = std::move(snapshot_samples)});
    }
  }

  template<typename Occupancy>
  void connectRebuiltBlocks(
      const Occupancy& occupancy,
      const std::span<const IncrementalTopologyBlockIndex3D> rebuilt_blocks,
      const std::uint64_t update_revision) {
    struct ContactCandidate {
      IncrementalTopologyNodeId first{};
      IncrementalTopologyNodeId second{};
      GridIndex3D first_cell{};
      GridIndex3D second_cell{};
      double estimated_length_m{std::numeric_limits<double>::infinity()};
    };

    std::unordered_map<IncrementalTopologyEdgeId, ContactCandidate,
                       IncrementalTopologyEdgeIdHash>
        contacts;
    std::unordered_set<IncrementalTopologyBlockIndex3D,
                       IncrementalTopologyBlockIndex3DHash>
        rebuilt{rebuilt_blocks.begin(), rebuilt_blocks.end()};
    constexpr std::array<IncrementalTopologyBlockIndex3D, 6U> directions{
        IncrementalTopologyBlockIndex3D{-1, 0, 0},
        IncrementalTopologyBlockIndex3D{1, 0, 0},
        IncrementalTopologyBlockIndex3D{0, -1, 0},
        IncrementalTopologyBlockIndex3D{0, 1, 0},
        IncrementalTopologyBlockIndex3D{0, 0, -1},
        IncrementalTopologyBlockIndex3D{0, 0, 1},
    };
    const auto near_shared_face = [&](const GridIndex3D cell,
                                      const IncrementalTopologyBlockIndex3D block,
                                      const IncrementalTopologyBlockIndex3D direction,
                                      const int stride) noexcept {
      const GridIndex3D minimum{block.x * config.block_size_cells,
                                block.y * config.block_size_cells,
                                block.z * config.block_size_cells};
      const GridIndex3D maximum{minimum.x + config.block_size_cells,
                                minimum.y + config.block_size_cells,
                                minimum.z + config.block_size_cells};
      if (direction.x < 0) {
        return cell.x - minimum.x < stride;
      }
      if (direction.x > 0) {
        return maximum.x - cell.x <= stride;
      }
      if (direction.y < 0) {
        return cell.y - minimum.y < stride;
      }
      if (direction.y > 0) {
        return maximum.y - cell.y <= stride;
      }
      if (direction.z < 0) {
        return cell.z - minimum.z < stride;
      }
      return maximum.z - cell.z <= stride;
    };
    const auto offset_cell = [](const GridIndex3D cell,
                                const IncrementalTopologyBlockIndex3D direction,
                                const int normal, const int first_transverse,
                                const int second_transverse) noexcept {
      GridIndex3D result{cell.x + direction.x * normal, cell.y + direction.y * normal,
                         cell.z + direction.z * normal};
      if (direction.x != 0) {
        result.y += first_transverse;
        result.z += second_transverse;
      } else if (direction.y != 0) {
        result.x += first_transverse;
        result.z += second_transverse;
      } else {
        result.x += first_transverse;
        result.y += second_transverse;
      }
      return result;
    };

    for (const IncrementalTopologyBlockIndex3D block : rebuilt_blocks) {
      const auto block_found = blocks.find(block);
      if (block_found == blocks.end()) {
        continue;
      }
      for (const IncrementalTopologyBlockIndex3D direction : directions) {
        const IncrementalTopologyBlockIndex3D neighbor_block{
            block.x + direction.x, block.y + direction.y, block.z + direction.z};
        if (rebuilt.contains(neighbor_block) && neighbor_block < block) {
          continue;
        }
        const auto neighbor_block_found = blocks.find(neighbor_block);
        if (neighbor_block_found == blocks.end()) {
          continue;
        }
        const int local_stride = block_found->second.sample_stride_cells;
        const int neighbor_stride = neighbor_block_found->second.sample_stride_cells;
        const int connection_stride = std::max(local_stride, neighbor_stride);
        const int transverse_tolerance =
            connection_stride - std::min(local_stride, neighbor_stride);
        for (const BlockComponent& component : block_found->second.components) {
          for (const GridIndex3D cell : component.cells) {
            if (!near_shared_face(cell, block, direction, connection_stride)) {
              continue;
            }
            for (int normal = 1; normal <= connection_stride; ++normal) {
              for (int first_transverse = -transverse_tolerance;
                   first_transverse <= transverse_tolerance; ++first_transverse) {
                for (int second_transverse = -transverse_tolerance;
                     second_transverse <= transverse_tolerance; ++second_transverse) {
                  const GridIndex3D neighbor = offset_cell(
                      cell, direction, normal, first_transverse, second_transverse);
                  if (!occupancy.contains(neighbor) ||
                      blockForCell(neighbor, config.block_size_cells) !=
                          neighbor_block) {
                    continue;
                  }
                  const auto neighbor_found =
                      sample_cell_nodes.find(sampleCellKey(bounds, neighbor));
                  if (neighbor_found == sample_cell_nodes.end() ||
                      neighbor_found->second == component.id) {
                    continue;
                  }
                  const BlockComponent* const neighbor_component =
                      componentForNode(neighbor_found->second);
                  if (neighbor_component == nullptr ||
                      !navigableBetween(occupancy, occupancy.cellCenter(cell),
                                        occupancy.cellCenter(neighbor),
                                        config.footprint,
                                        config.require_known_free_space)) {
                    continue;
                  }
                  IncrementalTopologyNodeId first = component.id;
                  IncrementalTopologyNodeId second = neighbor_component->id;
                  GridIndex3D first_cell = cell;
                  GridIndex3D second_cell = neighbor;
                  const Point3 local_contact = occupancy.cellCenter(cell);
                  const Point3 remote_contact = occupancy.cellCenter(neighbor);
                  const double estimated_length_m =
                      distance3D(component.representative, local_contact) +
                      distance3D(local_contact, remote_contact) +
                      distance3D(remote_contact, neighbor_component->representative);
                  if (second < first) {
                    std::swap(first, second);
                    std::swap(first_cell, second_cell);
                  }
                  const IncrementalTopologyEdgeId edge_id = makeEdgeId(first, second);
                  const auto existing = contacts.find(edge_id);
                  const auto candidate_key = std::tuple{
                      estimated_length_m, first_cell.z,  first_cell.y, first_cell.x,
                      second_cell.z,      second_cell.y, second_cell.x};
                  const auto existing_key =
                      existing == contacts.end()
                          ? std::tuple{std::numeric_limits<double>::infinity(),
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0}
                          : std::tuple{existing->second.estimated_length_m,
                                       existing->second.first_cell.z,
                                       existing->second.first_cell.y,
                                       existing->second.first_cell.x,
                                       existing->second.second_cell.z,
                                       existing->second.second_cell.y,
                                       existing->second.second_cell.x};
                  if (existing == contacts.end() || candidate_key < existing_key) {
                    contacts.insert_or_assign(
                        edge_id,
                        ContactCandidate{.first = first,
                                         .second = second,
                                         .first_cell = first_cell,
                                         .second_cell = second_cell,
                                         .estimated_length_m = estimated_length_m});
                  }
                }
              }
            }
          }
        }
      }
    }

    std::vector<IncrementalTopologyEdgeId> ordered_edges;
    ordered_edges.reserve(contacts.size());
    for (const auto& [edge_id, unused] : contacts) {
      static_cast<void>(unused);
      ordered_edges.push_back(edge_id);
    }
    std::ranges::sort(ordered_edges);
    for (const IncrementalTopologyEdgeId edge_id : ordered_edges) {
      const ContactCandidate& contact = contacts.at(edge_id);
      const BlockComponent* const first_component = componentForNode(contact.first);
      const BlockComponent* const second_component = componentForNode(contact.second);
      if (first_component == nullptr || second_component == nullptr) {
        continue;
      }
      std::vector<Point3> polyline =
          pathFromRepresentative(*first_component, contact.first_cell);
      std::vector<Point3> second_path =
          pathFromRepresentative(*second_component, contact.second_cell);
      if (polyline.empty() || second_path.empty()) {
        continue;
      }
      const Point3 first_contact = occupancy.cellCenter(contact.first_cell);
      const Point3 second_contact = occupancy.cellCenter(contact.second_cell);
      if (!navigableBetween(occupancy, first_contact, second_contact, config.footprint,
                            config.require_known_free_space)) {
        continue;
      }
      appendUniquePoint(polyline, second_contact);
      std::ranges::reverse(second_path);
      for (const Point3& point : second_path) {
        appendUniquePoint(polyline, point);
      }
      const double length_m = polylineLength(polyline);
      const IncrementalTopologyBlockIndex3D first_block = nodes.at(contact.first).block;
      const IncrementalTopologyBlockIndex3D second_block =
          nodes.at(contact.second).block;
      const std::uint64_t validated_through_revision =
          std::min(block_validated_through.at(first_block),
                   block_validated_through.at(second_block));
      const std::uint64_t complete_through_revision =
          std::min(block_complete_through.at(first_block),
                   block_complete_through.at(second_block));
      const std::uint64_t created_on_revision =
          edge_created_on_revision.try_emplace(edge_id, update_revision).first->second;
      const std::uint64_t lineage_id = makeTransitionLineage(edge_id.value, polyline);
      const IncrementalTopologyTransitionEvidence3D evidence = makeTransitionEvidence(
          occupancy, polyline, config.footprint, validated_through_revision,
          complete_through_revision, lineage_id);
      edges[edge_id] = IncrementalTopologyEdge3D{
          .id = edge_id,
          .first = contact.first,
          .second = contact.second,
          .first_contact = first_contact,
          .second_contact = second_contact,
          .polyline = std::move(polyline),
          .length_m = length_m,
          .created_on_revision = created_on_revision,
          .evidence = evidence,
      };
    }
  }

  void classifyNodes(const std::uint64_t update_revision) {
    std::unordered_map<IncrementalTopologyNodeId,
                       std::vector<IncrementalTopologyNodeId>,
                       IncrementalTopologyNodeIdHash>
        adjacency;
    for (const auto& [edge_id, edge] : edges) {
      static_cast<void>(edge_id);
      adjacency[edge.first].push_back(edge.second);
      adjacency[edge.second].push_back(edge.first);
    }
    for (auto& [node_id, node] : nodes) {
      const std::vector<IncrementalTopologyNodeId>& neighbors = adjacency[node_id];
      IncrementalTopologyNodeTraits3D traits;
      traits.terminal = neighbors.size() <= 1U && !node.unknown_boundary_exposure;
      for (const IncrementalTopologyNodeId neighbor_id : neighbors) {
        const Point3& neighbor = nodes.at(neighbor_id).representative;
        const double horizontal = std::hypot(neighbor.x - node.representative.x,
                                             neighbor.y - node.representative.y);
        if (std::abs(neighbor.z - node.representative.z) > horizontal * 0.5) {
          traits.vertical_connector = true;
        }
      }
      node.degree = neighbors.size();
      node.traits = traits;
      node.classification_revision = update_revision;
    }
  }

  template<typename Occupancy>
  [[nodiscard]] IncrementalTopologyGraph3DUpdate
  rebuild(const Occupancy& occupancy, const std::uint64_t update_revision,
          std::vector<IncrementalTopologyBlockIndex3D> rebuilt_blocks,
          const std::size_t requested_dirty_chunks, const bool full_reset) {
    IncrementalTopologyGraph3DUpdate stats{
        .revision = update_revision,
        .requested_dirty_chunks = requested_dirty_chunks,
        .rebuilt_blocks = rebuilt_blocks.size(),
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
    for (const IncrementalTopologyBlockIndex3D block : rebuilt_blocks) {
      replacements.push_back(buildBlock(occupancy, block));
      stats.adaptively_refined_blocks +=
          replacements.back().adaptively_refined ? 1U : 0U;
      for (const BlockComponent& component : replacements.back().components) {
        stats.sampled_navigable_cells += component.cells.size();
      }
    }
    stats.block_build_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - stage_started)
                               .count();
    stage_started = std::chrono::steady_clock::now();
    for (std::size_t index = 0U; index < rebuilt_blocks.size(); ++index) {
      replaceBlock(rebuilt_blocks[index], std::move(replacements[index]),
                   update_revision, stats);
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

  IncrementalTopologyGraph3DConfig config{};
  GridBounds3D bounds{};
  std::uint64_t graph_revision{0U};
  std::unordered_map<IncrementalTopologyBlockIndex3D, BlockData,
                     IncrementalTopologyBlockIndex3DHash>
      blocks;
  std::unordered_map<IncrementalTopologyNodeId, IncrementalTopologyNode3D,
                     IncrementalTopologyNodeIdHash>
      nodes;
  std::unordered_map<IncrementalTopologyEdgeId, IncrementalTopologyEdge3D,
                     IncrementalTopologyEdgeIdHash>
      edges;
  std::unordered_map<std::uint64_t, IncrementalTopologyNodeId> sample_cell_nodes;
  std::unordered_map<std::uint64_t, std::uint64_t> sample_cell_parents;
  std::unordered_map<IncrementalTopologyBlockIndex3D, std::uint64_t,
                     IncrementalTopologyBlockIndex3DHash>
      block_validated_through;
  std::unordered_map<IncrementalTopologyBlockIndex3D, std::uint64_t,
                     IncrementalTopologyBlockIndex3DHash>
      block_complete_through;
  std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash>
      issued_node_ids;
  std::unordered_map<IncrementalTopologyEdgeId, std::uint64_t,
                     IncrementalTopologyEdgeIdHash>
      edge_created_on_revision;
  incremental_topology_detail::ObservedBlockLifecycle3D observed_blocks;
};

} // namespace drone_city_nav
