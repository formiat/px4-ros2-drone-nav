#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

void hashInteger(std::uint64_t& hash, const int value) noexcept {
  const auto encoded = static_cast<std::uint32_t>(value);
  for (std::size_t shift = 0U; shift < sizeof(encoded); ++shift) {
    hash ^= static_cast<std::uint8_t>(encoded >> (shift * 8U));
    hash *= kFnvPrime;
  }
}

void hashUnsigned(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t shift = 0U; shift < sizeof(value); ++shift) {
    hash ^= static_cast<std::uint8_t>(value >> (shift * 8U));
    hash *= kFnvPrime;
  }
}

[[nodiscard]] std::uint64_t sampleCellKey(const GridBounds3D& bounds,
                                          const GridIndex3D cell) noexcept {
  if (cell.x < 0 || cell.y < 0 || cell.z < 0 || cell.x >= bounds.width_cells ||
      cell.y >= bounds.height_cells || cell.z >= bounds.depth_cells) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return (static_cast<std::uint64_t>(cell.z) *
              static_cast<std::uint64_t>(bounds.height_cells) +
          static_cast<std::uint64_t>(cell.y)) *
             static_cast<std::uint64_t>(bounds.width_cells) +
         static_cast<std::uint64_t>(cell.x);
}

[[nodiscard]] bool cellLess(const GridIndex3D first,
                            const GridIndex3D second) noexcept {
  return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
}

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  constexpr double tolerance{1.0e-9};
  return std::abs(first.origin_x - second.origin_x) <= tolerance &&
         std::abs(first.origin_y - second.origin_y) <= tolerance &&
         std::abs(first.origin_z - second.origin_z) <= tolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= tolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

[[nodiscard]] IncrementalTopologyTileIndex3D
tileForCell(const GridIndex3D cell, const int tile_size_cells) noexcept {
  return IncrementalTopologyTileIndex3D{
      cell.x / tile_size_cells, cell.y / tile_size_cells, cell.z / tile_size_cells};
}

[[nodiscard]] int firstAlignedCell(const int minimum, const int stride) noexcept {
  const int remainder = minimum % stride;
  return remainder == 0 ? minimum : minimum + stride - remainder;
}

[[nodiscard]] std::uint64_t makeNodeIdValue(const IncrementalTopologyTileIndex3D tile,
                                            const GridIndex3D anchor,
                                            const std::uint64_t salt) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashInteger(hash, tile.x);
  hashInteger(hash, tile.y);
  hashInteger(hash, tile.z);
  hashInteger(hash, anchor.x);
  hashInteger(hash, anchor.y);
  hashInteger(hash, anchor.z);
  hashUnsigned(hash, salt);
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] IncrementalTopologyEdgeId
makeEdgeId(IncrementalTopologyNodeId first, IncrementalTopologyNodeId second) noexcept {
  if (second < first) {
    std::swap(first, second);
  }
  std::uint64_t hash{kFnvOffset};
  hashUnsigned(hash, first.value);
  hashUnsigned(hash, second.value);
  return IncrementalTopologyEdgeId{hash == 0U ? 1U : hash};
}

[[nodiscard]] std::array<GridIndex3D, 6U> cardinalNeighbors(const GridIndex3D cell,
                                                            const int stride) noexcept {
  return {
      GridIndex3D{cell.x - stride, cell.y, cell.z},
      GridIndex3D{cell.x + stride, cell.y, cell.z},
      GridIndex3D{cell.x, cell.y - stride, cell.z},
      GridIndex3D{cell.x, cell.y + stride, cell.z},
      GridIndex3D{cell.x, cell.y, cell.z - stride},
      GridIndex3D{cell.x, cell.y, cell.z + stride},
  };
}

template<typename Occupancy>
[[nodiscard]] bool navigableAt(const Occupancy& occupancy, const Point3& position,
                               const SweptFootprintConfig& footprint) noexcept {
  return validateRawFootprintAt(occupancy, position, FootprintBodyAxis{}, footprint)
      .accepted();
}

template<typename Occupancy>
[[nodiscard]] bool navigableBetween(const Occupancy& occupancy, const Point3& first,
                                    const Point3& second,
                                    const SweptFootprintConfig& footprint) noexcept {
  return validateRawSweptFootprint(occupancy, first, FootprintBodyAxis{}, second,
                                   FootprintBodyAxis{}, footprint)
      .accepted();
}

[[nodiscard]] double squaredNorm(const Vec3& vector) noexcept {
  return vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
}

} // namespace

std::size_t IncrementalTopologyNodeIdHash::operator()(
    const IncrementalTopologyNodeId id) const noexcept {
  return std::hash<std::uint64_t>{}(id.value);
}

std::size_t IncrementalTopologyEdgeIdHash::operator()(
    const IncrementalTopologyEdgeId id) const noexcept {
  return std::hash<std::uint64_t>{}(id.value);
}

std::size_t IncrementalTopologyTileIndex3DHash::operator()(
    const IncrementalTopologyTileIndex3D index) const noexcept {
  std::uint64_t hash{kFnvOffset};
  hashInteger(hash, index.x);
  hashInteger(hash, index.y);
  hashInteger(hash, index.z);
  return static_cast<std::size_t>(hash);
}

bool incrementalTopologyGraph3DConfigIsValid(
    const IncrementalTopologyGraph3DConfig& config) noexcept {
  return config.tile_size_cells > 0 && config.sample_stride_cells > 0 &&
         config.sample_stride_cells <= config.tile_size_cells &&
         config.maximum_frontier_evaluations_per_component > 0U &&
         std::isfinite(config.footprint.radius_m) &&
         std::isfinite(config.footprint.lower_extent_m) &&
         std::isfinite(config.footprint.upper_extent_m) &&
         std::isfinite(config.footprint.sweep_step_m) &&
         config.footprint.radius_m >= 0.0 && config.footprint.lower_extent_m >= 0.0 &&
         config.footprint.upper_extent_m >= 0.0 &&
         config.footprint.perimeter_samples > 0U &&
         config.footprint.radial_rings > 0U && config.footprint.axial_samples > 0U &&
         config.footprint.sweep_step_m > 0.0 &&
         config.observability.maximum_observation_range_m > 0.0 &&
         config.observability.minimum_known_free_ray_m >= 0.0 &&
         config.observability.minimum_supporting_rays > 0U &&
         config.observability.minimum_information_gain_voxels > 0U;
}

struct IncrementalTopologyGraph3D::Impl {
  struct TileComponent {
    IncrementalTopologyNodeId id{};
    std::vector<GridIndex3D> cells;
    Point3 representative{};
    std::optional<ObservationFrontier> frontier;
  };

  struct TileData {
    std::vector<TileComponent> components;
  };

  explicit Impl(const IncrementalTopologyGraph3DConfig& requested_config)
      : config{requested_config} {
    if (!incrementalTopologyGraph3DConfigIsValid(config)) {
      throw std::invalid_argument{"invalid incremental 3d topology configuration"};
    }
    config.observability.footprint = config.footprint;
  }

  template<typename Occupancy>
  [[nodiscard]] std::vector<TileComponent>
  buildTile(const Occupancy& occupancy, const IncrementalTopologyTileIndex3D tile,
            const std::uint64_t update_revision) const {
    const int minimum_x = tile.x * config.tile_size_cells;
    const int minimum_y = tile.y * config.tile_size_cells;
    const int minimum_z = tile.z * config.tile_size_cells;
    const int maximum_x =
        std::min(bounds.width_cells, minimum_x + config.tile_size_cells);
    const int maximum_y =
        std::min(bounds.height_cells, minimum_y + config.tile_size_cells);
    const int maximum_z =
        std::min(bounds.depth_cells, minimum_z + config.tile_size_cells);
    std::vector<GridIndex3D> navigable_cells;
    std::unordered_map<std::uint64_t, std::size_t> cell_indices;
    for (int z = firstAlignedCell(minimum_z, config.sample_stride_cells); z < maximum_z;
         z += config.sample_stride_cells) {
      for (int y = firstAlignedCell(minimum_y, config.sample_stride_cells);
           y < maximum_y; y += config.sample_stride_cells) {
        for (int x = firstAlignedCell(minimum_x, config.sample_stride_cells);
             x < maximum_x; x += config.sample_stride_cells) {
          const GridIndex3D cell{x, y, z};
          const Point3 center = occupancy.cellCenter(cell);
          if (!navigableAt(occupancy, center, config.footprint)) {
            continue;
          }
          cell_indices.emplace(sampleCellKey(bounds, cell), navigable_cells.size());
          navigable_cells.push_back(cell);
        }
      }
    }

    std::vector<bool> visited(navigable_cells.size(), false);
    std::vector<TileComponent> components;
    for (std::size_t seed = 0U; seed < navigable_cells.size(); ++seed) {
      if (visited[seed]) {
        continue;
      }
      TileComponent component;
      std::deque<std::size_t> pending;
      pending.push_back(seed);
      visited[seed] = true;
      while (!pending.empty()) {
        const std::size_t index = pending.front();
        pending.pop_front();
        const GridIndex3D cell = navigable_cells[index];
        component.cells.push_back(cell);
        for (const GridIndex3D neighbor :
             cardinalNeighbors(cell, config.sample_stride_cells)) {
          const auto found = cell_indices.find(sampleCellKey(bounds, neighbor));
          if (found == cell_indices.end() || visited[found->second]) {
            continue;
          }
          if (!navigableBetween(occupancy, occupancy.cellCenter(cell),
                                occupancy.cellCenter(neighbor), config.footprint)) {
            continue;
          }
          visited[found->second] = true;
          pending.push_back(found->second);
        }
      }
      std::ranges::sort(component.cells, cellLess);
      component.representative = representativeFor(occupancy, component.cells);
      if constexpr (std::is_same_v<Occupancy, ObservedOccupancyGrid3D>) {
        component.frontier =
            selectFrontier(occupancy, component.cells, update_revision);
      }
      components.push_back(std::move(component));
    }
    std::ranges::sort(components,
                      [](const TileComponent& first, const TileComponent& second) {
                        return cellLess(first.cells.front(), second.cells.front());
                      });
    return components;
  }

  template<typename Occupancy>
  [[nodiscard]] Point3
  representativeFor(const Occupancy& occupancy,
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
    return occupancy.cellCenter(best);
  }

  [[nodiscard]] std::optional<ObservationFrontier>
  selectFrontier(const ObservedOccupancyGrid3D& occupancy,
                 const std::span<const GridIndex3D> cells,
                 const std::uint64_t update_revision) const {
    std::vector<GridIndex3D> candidates{cells.begin(), cells.end()};
    std::ranges::sort(candidates,
                      [this](const GridIndex3D first, const GridIndex3D second) {
                        std::uint64_t first_hash{kFnvOffset};
                        std::uint64_t second_hash{kFnvOffset};
                        hashUnsigned(first_hash, sampleCellKey(bounds, first));
                        hashUnsigned(second_hash, sampleCellKey(bounds, second));
                        return std::tie(first_hash, first.z, first.y, first.x) <
                               std::tie(second_hash, second.z, second.y, second.x);
                      });
    const std::size_t evaluation_count =
        std::min(candidates.size(), config.maximum_frontier_evaluations_per_component);
    std::optional<ObservationFrontier> best;
    for (std::size_t index = 0U; index < evaluation_count; ++index) {
      const ObservationFrontierEvaluation evaluation = evaluateObservationFrontier(
          occupancy, occupancy.cellCenter(candidates[index]), update_revision,
          config.observability);
      if (!evaluation.accepted()) {
        continue;
      }
      if (!best.has_value() ||
          std::tie(evaluation.frontier.information_gain_voxels,
                   evaluation.frontier.supporting_rays) >
              std::tie(best->information_gain_voxels, best->supporting_rays) ||
          (evaluation.frontier.information_gain_voxels ==
               best->information_gain_voxels &&
           evaluation.frontier.supporting_rays == best->supporting_rays &&
           evaluation.frontier.id < best->id)) {
        best = evaluation.frontier;
      }
    }
    return best;
  }

  [[nodiscard]] IncrementalTopologyNodeId
  allocateNodeId(const IncrementalTopologyTileIndex3D tile,
                 const GridIndex3D anchor) const {
    for (std::uint64_t salt = 0U;; ++salt) {
      const IncrementalTopologyNodeId candidate{makeNodeIdValue(tile, anchor, salt)};
      if (!nodes.contains(candidate)) {
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

  void replaceTile(const IncrementalTopologyTileIndex3D tile,
                   std::vector<TileComponent> components,
                   const std::uint64_t update_revision,
                   IncrementalTopologyGraph3DUpdate& stats) {
    std::vector<TileComponent> previous;
    if (const auto found = tiles.find(tile); found != tiles.end()) {
      previous = std::move(found->second.components);
      tiles.erase(found);
    }
    std::unordered_map<std::uint64_t, IncrementalTopologyNodeId> previous_cells;
    std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash>
        previous_ids;
    for (const TileComponent& component : previous) {
      previous_ids.insert(component.id);
      for (const GridIndex3D cell : component.cells) {
        previous_cells.emplace(sampleCellKey(bounds, cell), component.id);
        sample_cell_nodes.erase(sampleCellKey(bounds, cell));
      }
      nodes.erase(component.id);
    }
    removeIncidentEdges(previous_ids);

    std::unordered_set<IncrementalTopologyNodeId, IncrementalTopologyNodeIdHash>
        retained_ids;
    for (TileComponent& component : components) {
      std::unordered_map<IncrementalTopologyNodeId, std::size_t,
                         IncrementalTopologyNodeIdHash>
          overlaps;
      for (const GridIndex3D cell : component.cells) {
        if (const auto found = previous_cells.find(sampleCellKey(bounds, cell));
            found != previous_cells.end() && !retained_ids.contains(found->second)) {
          ++overlaps[found->second];
        }
      }
      if (!overlaps.empty()) {
        const auto best = std::ranges::max_element(overlaps, {}, [](const auto& entry) {
          return std::tuple{entry.second, std::numeric_limits<std::uint64_t>::max() -
                                              entry.first.value};
        });
        component.id = best->first;
        retained_ids.insert(component.id);
        ++stats.retained_node_ids;
      } else {
        component.id = allocateNodeId(tile, component.cells.front());
        ++stats.created_nodes;
      }
      IncrementalTopologyNode3D node{
          .id = component.id,
          .tile = tile,
          .representative = component.representative,
          .support_cell_count = component.cells.size(),
          .geometry_revision = update_revision,
          .classification_revision = update_revision,
          .observation_frontier = component.frontier,
      };
      nodes.emplace(node.id, node);
      for (const GridIndex3D cell : component.cells) {
        sample_cell_nodes[sampleCellKey(bounds, cell)] = component.id;
      }
    }
    stats.retired_nodes += previous.size() - retained_ids.size();
    if (!components.empty()) {
      tiles.emplace(tile, TileData{.components = std::move(components)});
    }
  }

  template<typename Occupancy>
  void connectRebuiltTiles(
      const Occupancy& occupancy,
      const std::span<const IncrementalTopologyTileIndex3D> rebuilt_tiles,
      const std::uint64_t update_revision) {
    for (const IncrementalTopologyTileIndex3D tile : rebuilt_tiles) {
      const auto tile_found = tiles.find(tile);
      if (tile_found == tiles.end()) {
        continue;
      }
      for (const TileComponent& component : tile_found->second.components) {
        for (const GridIndex3D cell : component.cells) {
          for (const GridIndex3D neighbor :
               cardinalNeighbors(cell, config.sample_stride_cells)) {
            if (tileForCell(neighbor, config.tile_size_cells) == tile) {
              continue;
            }
            const auto neighbor_found =
                sample_cell_nodes.find(sampleCellKey(bounds, neighbor));
            if (neighbor_found == sample_cell_nodes.end() ||
                neighbor_found->second == component.id ||
                !navigableBetween(occupancy, occupancy.cellCenter(cell),
                                  occupancy.cellCenter(neighbor), config.footprint)) {
              continue;
            }
            IncrementalTopologyNodeId first = component.id;
            IncrementalTopologyNodeId second = neighbor_found->second;
            Point3 first_contact = occupancy.cellCenter(cell);
            Point3 second_contact = occupancy.cellCenter(neighbor);
            if (second < first) {
              std::swap(first, second);
              std::swap(first_contact, second_contact);
            }
            const IncrementalTopologyNode3D& first_node = nodes.at(first);
            const IncrementalTopologyNode3D& second_node = nodes.at(second);
            const double length_m =
                distance3D(first_node.representative, first_contact) +
                distance3D(first_contact, second_contact) +
                distance3D(second_contact, second_node.representative);
            const IncrementalTopologyEdgeId edge_id = makeEdgeId(first, second);
            const auto existing = edges.find(edge_id);
            if (existing != edges.end() &&
                existing->second.length_m <= length_m + 1.0e-9) {
              existing->second.supporting_revision = update_revision;
              continue;
            }
            edges[edge_id] = IncrementalTopologyEdge3D{
                .id = edge_id,
                .first = first,
                .second = second,
                .first_contact = first_contact,
                .second_contact = second_contact,
                .length_m = length_m,
                .supporting_revision = update_revision,
            };
          }
        }
      }
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
      traits.frontier = node.observation_frontier.has_value();
      traits.junction = neighbors.size() >= 3U;
      traits.terminal = neighbors.size() <= 1U && !traits.frontier;
      for (const IncrementalTopologyNodeId neighbor_id : neighbors) {
        const Point3& neighbor = nodes.at(neighbor_id).representative;
        const double horizontal = std::hypot(neighbor.x - node.representative.x,
                                             neighbor.y - node.representative.y);
        if (std::abs(neighbor.z - node.representative.z) > horizontal * 0.5) {
          traits.vertical_connector = true;
        }
      }
      if (neighbors.size() == 2U) {
        const Point3& first = nodes.at(neighbors[0]).representative;
        const Point3& second = nodes.at(neighbors[1]).representative;
        const Vec3 first_direction{first.x - node.representative.x,
                                   first.y - node.representative.y,
                                   first.z - node.representative.z};
        const Vec3 second_direction{second.x - node.representative.x,
                                    second.y - node.representative.y,
                                    second.z - node.representative.z};
        const double denominator =
            std::sqrt(squaredNorm(first_direction) * squaredNorm(second_direction));
        const double cosine = denominator > 1.0e-9
                                  ? (first_direction.x * second_direction.x +
                                     first_direction.y * second_direction.y +
                                     first_direction.z * second_direction.z) /
                                        denominator
                                  : -1.0;
        traits.turn = cosine > -0.8660254037844386;
      }
      node.degree = neighbors.size();
      node.traits = traits;
      node.classification_revision = update_revision;
    }
  }

  [[nodiscard]] std::vector<IncrementalTopologyTileIndex3D>
  allObservedTiles(const ObservedOccupancyGrid3D& occupancy) const {
    std::unordered_set<IncrementalTopologyTileIndex3D,
                       IncrementalTopologyTileIndex3DHash>
        unique;
    for (const auto& [chunk, data] : occupancy.chunks()) {
      static_cast<void>(data);
      const int minimum_x = chunk.x * ObservedOccupancyGrid3D::kChunkSize;
      const int minimum_y = chunk.y * ObservedOccupancyGrid3D::kChunkSize;
      const int minimum_z = chunk.z * ObservedOccupancyGrid3D::kChunkSize;
      const int maximum_x = std::min(bounds.width_cells,
                                     minimum_x + ObservedOccupancyGrid3D::kChunkSize) -
                            1;
      const int maximum_y = std::min(bounds.height_cells,
                                     minimum_y + ObservedOccupancyGrid3D::kChunkSize) -
                            1;
      const int maximum_z = std::min(bounds.depth_cells,
                                     minimum_z + ObservedOccupancyGrid3D::kChunkSize) -
                            1;
      addTileRange(unique, {minimum_x, minimum_y, minimum_z},
                   {maximum_x, maximum_y, maximum_z});
    }
    return sortedTiles(unique);
  }

  [[nodiscard]] std::vector<IncrementalTopologyTileIndex3D>
  dirtyObservedTiles(const std::span<const OccupancyChunkIndex3D> dirty_chunks) const {
    std::unordered_set<IncrementalTopologyTileIndex3D,
                       IncrementalTopologyTileIndex3DHash>
        unique;
    const double maximum_extent_m =
        std::max({config.footprint.radius_m, config.footprint.lower_extent_m,
                  config.footprint.upper_extent_m});
    const int halo_cells =
        static_cast<int>(std::ceil(maximum_extent_m / bounds.resolution_m)) +
        config.sample_stride_cells;
    for (const OccupancyChunkIndex3D chunk : dirty_chunks) {
      const GridIndex3D minimum{
          std::max(0, chunk.x * ObservedOccupancyGrid3D::kChunkSize - halo_cells),
          std::max(0, chunk.y * ObservedOccupancyGrid3D::kChunkSize - halo_cells),
          std::max(0, chunk.z * ObservedOccupancyGrid3D::kChunkSize - halo_cells)};
      const GridIndex3D maximum{
          std::min(bounds.width_cells - 1,
                   (chunk.x + 1) * ObservedOccupancyGrid3D::kChunkSize - 1 +
                       halo_cells),
          std::min(bounds.height_cells - 1,
                   (chunk.y + 1) * ObservedOccupancyGrid3D::kChunkSize - 1 +
                       halo_cells),
          std::min(bounds.depth_cells - 1,
                   (chunk.z + 1) * ObservedOccupancyGrid3D::kChunkSize - 1 +
                       halo_cells)};
      addTileRange(unique, minimum, maximum);
    }
    return sortedTiles(unique);
  }

  [[nodiscard]] std::vector<IncrementalTopologyTileIndex3D> allStaticTiles() const {
    std::unordered_set<IncrementalTopologyTileIndex3D,
                       IncrementalTopologyTileIndex3DHash>
        unique;
    addTileRange(
        unique, {0, 0, 0},
        {bounds.width_cells - 1, bounds.height_cells - 1, bounds.depth_cells - 1});
    return sortedTiles(unique);
  }

  void addTileRange(std::unordered_set<IncrementalTopologyTileIndex3D,
                                       IncrementalTopologyTileIndex3DHash>& output,
                    const GridIndex3D minimum, const GridIndex3D maximum) const {
    if (maximum.x < minimum.x || maximum.y < minimum.y || maximum.z < minimum.z) {
      return;
    }
    const IncrementalTopologyTileIndex3D first =
        tileForCell(minimum, config.tile_size_cells);
    const IncrementalTopologyTileIndex3D last =
        tileForCell(maximum, config.tile_size_cells);
    for (int z = first.z; z <= last.z; ++z) {
      for (int y = first.y; y <= last.y; ++y) {
        for (int x = first.x; x <= last.x; ++x) {
          output.insert({x, y, z});
        }
      }
    }
  }

  [[nodiscard]] static std::vector<IncrementalTopologyTileIndex3D>
  sortedTiles(const std::unordered_set<IncrementalTopologyTileIndex3D,
                                       IncrementalTopologyTileIndex3DHash>& unique) {
    std::vector<IncrementalTopologyTileIndex3D> result{unique.begin(), unique.end()};
    std::ranges::sort(result, [](const IncrementalTopologyTileIndex3D first,
                                 const IncrementalTopologyTileIndex3D second) {
      return std::tie(first.z, first.y, first.x) <
             std::tie(second.z, second.y, second.x);
    });
    return result;
  }

  template<typename Occupancy>
  [[nodiscard]] IncrementalTopologyGraph3DUpdate
  rebuild(const Occupancy& occupancy, const std::uint64_t update_revision,
          std::vector<IncrementalTopologyTileIndex3D> rebuilt_tiles,
          const std::size_t requested_dirty_chunks, const bool full_reset) {
    IncrementalTopologyGraph3DUpdate stats{
        .revision = update_revision,
        .requested_dirty_chunks = requested_dirty_chunks,
        .rebuilt_tiles = rebuilt_tiles.size(),
        .full_reset = full_reset,
    };
    if (full_reset) {
      stats.retired_nodes = nodes.size();
      tiles.clear();
      nodes.clear();
      edges.clear();
      sample_cell_nodes.clear();
    }
    std::vector<std::vector<TileComponent>> replacements;
    replacements.reserve(rebuilt_tiles.size());
    for (const IncrementalTopologyTileIndex3D tile : rebuilt_tiles) {
      replacements.push_back(buildTile(occupancy, tile, update_revision));
    }
    for (std::size_t index = 0U; index < rebuilt_tiles.size(); ++index) {
      replaceTile(rebuilt_tiles[index], std::move(replacements[index]), update_revision,
                  stats);
    }
    connectRebuiltTiles(occupancy, rebuilt_tiles, update_revision);
    classifyNodes(update_revision);
    graph_revision = update_revision;
    stats.node_count = nodes.size();
    stats.edge_count = edges.size();
    return stats;
  }

  IncrementalTopologyGraph3DConfig config{};
  GridBounds3D bounds{};
  std::uint64_t graph_revision{0U};
  std::unordered_map<IncrementalTopologyTileIndex3D, TileData,
                     IncrementalTopologyTileIndex3DHash>
      tiles;
  std::unordered_map<IncrementalTopologyNodeId, IncrementalTopologyNode3D,
                     IncrementalTopologyNodeIdHash>
      nodes;
  std::unordered_map<IncrementalTopologyEdgeId, IncrementalTopologyEdge3D,
                     IncrementalTopologyEdgeIdHash>
      edges;
  std::unordered_map<std::uint64_t, IncrementalTopologyNodeId> sample_cell_nodes;
};

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
    const std::span<const OccupancyChunkIndex3D> dirty_chunks, const bool full_reset) {
  if (revision == 0U) {
    throw std::invalid_argument{"incremental topology revision must be non-zero"};
  }
  const bool geometry_changed =
      impl_->graph_revision != 0U && !sameBounds(impl_->bounds, occupancy.bounds());
  const bool reset_required =
      full_reset || impl_->graph_revision == 0U || geometry_changed;
  if (!reset_required && revision <= impl_->graph_revision) {
    return IncrementalTopologyGraph3DUpdate{
        .revision = impl_->graph_revision,
        .node_count = impl_->nodes.size(),
        .edge_count = impl_->edges.size(),
    };
  }
  impl_->bounds = occupancy.bounds();
  std::vector<IncrementalTopologyTileIndex3D> rebuilt_tiles =
      reset_required ? impl_->allObservedTiles(occupancy)
                     : impl_->dirtyObservedTiles(dirty_chunks);
  return impl_->rebuild(occupancy, revision, std::move(rebuilt_tiles),
                        dirty_chunks.size(), reset_required);
}

IncrementalTopologyGraph3DUpdate
IncrementalTopologyGraph3D::reset(const OccupancyGrid3D& occupancy,
                                  const std::uint64_t revision) {
  if (revision == 0U) {
    throw std::invalid_argument{"incremental topology revision must be non-zero"};
  }
  impl_->bounds = occupancy.bounds();
  return impl_->rebuild(occupancy, revision, impl_->allStaticTiles(), 0U, true);
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
  result.sample_cell_nodes_ = impl_->sample_cell_nodes;
  return result;
}

const IncrementalTopologyGraph3DConfig&
IncrementalTopologyGraph3D::config() const noexcept {
  return impl_->config;
}

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

const IncrementalTopologyNode3D* IncrementalTopologyGraph3DSnapshot::findNode(
    const IncrementalTopologyNodeId id) const noexcept {
  const auto found = node_indices_.find(id);
  return found == node_indices_.end() ? nullptr : &nodes_[found->second];
}

std::optional<IncrementalTopologyNodeId>
IncrementalTopologyGraph3DSnapshot::nodeForSampleCell(
    const GridIndex3D cell) const noexcept {
  const auto found = sample_cell_nodes_.find(sampleCellKey(bounds_, cell));
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

} // namespace drone_city_nav
