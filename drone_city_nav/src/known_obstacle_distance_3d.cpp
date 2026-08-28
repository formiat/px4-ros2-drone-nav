#include "drone_city_nav/known_obstacle_distance_3d.hpp"

#include "drone_city_nav/bounded_worker_pool.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "known_obstacle_distance_3d_internal.hpp"

namespace drone_city_nav {
namespace {

constexpr double kGeometryTolerance{1.0e-9};
constexpr std::uint64_t kFnvOffsetBasis{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

[[nodiscard]] int alignedCellOffset(const double local_origin,
                                    const double world_origin,
                                    const double resolution_m) {
  const double offset = (local_origin - world_origin) / resolution_m;
  const double rounded = std::round(offset);
  if (std::abs(offset - rounded) > 1.0e-6) {
    throw std::invalid_argument{"known obstacle distance bounds are not cell aligned"};
  }
  return static_cast<int>(rounded);
}

[[nodiscard]] bool localBoundsInsideWorld(const GridBounds3D& world,
                                          const GridBounds3D& local) {
  if (!(world.resolution_m > 0.0) || world.width_cells <= 0 ||
      world.height_cells <= 0 || world.depth_cells <= 0 || local.width_cells <= 0 ||
      local.height_cells <= 0 || local.depth_cells <= 0 ||
      std::abs(world.resolution_m - local.resolution_m) > kGeometryTolerance) {
    return false;
  }
  const int offset_x =
      alignedCellOffset(local.origin_x, world.origin_x, world.resolution_m);
  const int offset_y =
      alignedCellOffset(local.origin_y, world.origin_y, world.resolution_m);
  const int offset_z =
      alignedCellOffset(local.origin_z, world.origin_z, world.resolution_m);
  return offset_x >= 0 && offset_y >= 0 && offset_z >= 0 &&
         offset_x + local.width_cells <= world.width_cells &&
         offset_y + local.height_cells <= world.height_cells &&
         offset_z + local.depth_cells <= world.depth_cells;
}

[[nodiscard]] bool localCellInside(const GridBounds3D& bounds,
                                   const GridIndex3D cell) noexcept {
  return cell.x >= 0 && cell.y >= 0 && cell.z >= 0 && cell.x < bounds.width_cells &&
         cell.y < bounds.height_cells && cell.z < bounds.depth_cells;
}

[[nodiscard]] int axisCoordinate(const GridIndex3D cell, const int axis) noexcept {
  switch (axis) {
    case 0:
      return cell.x;
    case 1:
      return cell.y;
    default:
      return cell.z;
  }
}

[[nodiscard]] double
squaredDistanceBetweenIntervals(const int first_minimum, const int first_maximum,
                                const int second_minimum,
                                const int second_maximum) noexcept {
  if (first_maximum < second_minimum) {
    const double delta = static_cast<double>(second_minimum - first_maximum);
    return delta * delta;
  }
  if (second_maximum < first_minimum) {
    const double delta = static_cast<double>(first_minimum - second_maximum);
    return delta * delta;
  }
  return 0.0;
}

struct SourceChunkExtent3D {
  GridIndex3D minimum{};
  GridIndex3D maximum{};
  bool initialized{false};
};

} // namespace

GridBounds3D knownObstacleDistanceSourceBounds3D(const GridBounds3D& world_bounds,
                                                 const GridBounds3D& local_bounds,
                                                 const double maximum_distance_m) {
  if (!localBoundsInsideWorld(world_bounds, local_bounds) ||
      !std::isfinite(maximum_distance_m) || maximum_distance_m <= 0.0) {
    throw std::invalid_argument{"invalid known obstacle distance source bounds"};
  }
  const int radius_cells =
      static_cast<int>(std::ceil(maximum_distance_m / world_bounds.resolution_m));
  const int local_x = alignedCellOffset(local_bounds.origin_x, world_bounds.origin_x,
                                        world_bounds.resolution_m);
  const int local_y = alignedCellOffset(local_bounds.origin_y, world_bounds.origin_y,
                                        world_bounds.resolution_m);
  const int local_z = alignedCellOffset(local_bounds.origin_z, world_bounds.origin_z,
                                        world_bounds.resolution_m);
  const int minimum_x = std::max(0, local_x - radius_cells);
  const int minimum_y = std::max(0, local_y - radius_cells);
  const int minimum_z = std::max(0, local_z - radius_cells);
  const int maximum_x = std::min(world_bounds.width_cells,
                                 local_x + local_bounds.width_cells + radius_cells);
  const int maximum_y = std::min(world_bounds.height_cells,
                                 local_y + local_bounds.height_cells + radius_cells);
  const int maximum_z = std::min(world_bounds.depth_cells,
                                 local_z + local_bounds.depth_cells + radius_cells);
  return GridBounds3D{
      .origin_x = world_bounds.origin_x +
                  static_cast<double>(minimum_x) * world_bounds.resolution_m,
      .origin_y = world_bounds.origin_y +
                  static_cast<double>(minimum_y) * world_bounds.resolution_m,
      .origin_z = world_bounds.origin_z +
                  static_cast<double>(minimum_z) * world_bounds.resolution_m,
      .resolution_m = world_bounds.resolution_m,
      .width_cells = maximum_x - minimum_x,
      .height_cells = maximum_y - minimum_y,
      .depth_cells = maximum_z - minimum_z,
  };
}

KnownObstacleDistance3D::KnownObstacleDistance3D(
    const ConstructionKey construction_key,
    std::shared_ptr<const detail::KnownObstacleDistanceStorage3D> storage)
    : storage_{std::move(storage)} {
  static_cast<void>(construction_key);
}

std::shared_ptr<const KnownObstacleDistance3D> KnownObstacleDistance3D::create(
    std::shared_ptr<const detail::KnownObstacleDistanceStorage3D> storage) {
  return std::make_shared<const KnownObstacleDistance3D>(ConstructionKey{},
                                                         std::move(storage));
}

bool KnownObstacleDistance3D::valid() const noexcept {
  return storage_ != nullptr && storage_->source_fingerprint != 0U &&
         storage_->maximum_distance_m > 0.0;
}

const GridBounds3D& KnownObstacleDistance3D::bounds() const noexcept {
  return storage_->output_bounds;
}

const GridBounds3D& KnownObstacleDistance3D::sourceBounds() const noexcept {
  return storage_->source_bounds;
}

double KnownObstacleDistance3D::maximumDistanceM() const noexcept {
  return storage_->maximum_distance_m;
}

std::uint64_t KnownObstacleDistance3D::sourceFingerprint() const noexcept {
  return storage_->source_fingerprint;
}

std::size_t KnownObstacleDistance3D::sourceVoxelCount() const noexcept {
  return storage_->source_voxels;
}

std::size_t KnownObstacleDistance3D::sourceChunkCount() const noexcept {
  return storage_->source_chunks.size();
}

std::size_t KnownObstacleDistance3D::storedDistanceChunkCount() const noexcept {
  return storage_->distance_chunks.size();
}

std::size_t KnownObstacleDistance3D::finiteDistanceVoxelCount() const noexcept {
  return storage_->finite_distance_voxels;
}

float KnownObstacleDistance3D::distanceAt(const GridIndex3D local_cell) const noexcept {
  if (!valid() || !localCellInside(storage_->output_bounds, local_cell)) {
    return std::numeric_limits<float>::infinity();
  }
  const OccupancyChunkIndex3D chunk = detail::knownObstacleOutputChunk3D(local_cell);
  const auto found = storage_->distance_chunks.find(chunk);
  if (found == storage_->distance_chunks.end()) {
    return std::numeric_limits<float>::infinity();
  }
  return found->second->distances_m.at(
      detail::knownObstacleChunkBitIndex3D(local_cell));
}

std::shared_ptr<const std::vector<float>>
KnownObstacleDistance3D::materializeDense() const {
  if (!valid()) {
    return {};
  }
  auto dense = std::make_shared<std::vector<float>>(
      detail::knownObstacleVoxelCount3D(storage_->output_bounds),
      std::numeric_limits<float>::infinity());
  for (const auto& [chunk_index, chunk] : storage_->distance_chunks) {
    const KnownObstacleDistanceRegion3D region =
        detail::knownObstacleChunkRegion3D(storage_->output_bounds, chunk_index);
    for (int z = region.minimum_z; z < region.maximum_z_exclusive; ++z) {
      for (int y = region.minimum_y; y < region.maximum_y_exclusive; ++y) {
        for (int x = region.minimum_x; x < region.maximum_x_exclusive; ++x) {
          const GridIndex3D local{x, y, z};
          dense->at(
              detail::knownObstacleLocalLinearIndex3D(storage_->output_bounds, local)) =
              chunk->distances_m.at(detail::knownObstacleChunkBitIndex3D(local));
        }
      }
    }
  }
  return dense;
}

const char* knownObstacleDistance3DBuildModeName(
    const KnownObstacleDistance3DBuildMode mode) noexcept {
  switch (mode) {
    case KnownObstacleDistance3DBuildMode::kFull:
      return "full";
    case KnownObstacleDistance3DBuildMode::kIncremental:
      return "incremental";
    case KnownObstacleDistance3DBuildMode::kReused:
      return "reused";
  }
  return "invalid";
}

namespace detail {

bool sameKnownObstacleBounds3D(const GridBounds3D& first,
                               const GridBounds3D& second) noexcept {
  return std::abs(first.origin_x - second.origin_x) <= kGeometryTolerance &&
         std::abs(first.origin_y - second.origin_y) <= kGeometryTolerance &&
         std::abs(first.origin_z - second.origin_z) <= kGeometryTolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= kGeometryTolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

std::size_t knownObstacleVoxelCount3D(const GridBounds3D& bounds) {
  const auto width = static_cast<std::size_t>(bounds.width_cells);
  const auto height = static_cast<std::size_t>(bounds.height_cells);
  const auto depth = static_cast<std::size_t>(bounds.depth_cells);
  if (height != 0U && width > std::numeric_limits<std::size_t>::max() / height) {
    throw std::overflow_error{"known obstacle distance dimensions overflow"};
  }
  const std::size_t plane = width * height;
  if (depth != 0U && plane > std::numeric_limits<std::size_t>::max() / depth) {
    throw std::overflow_error{"known obstacle distance dimensions overflow"};
  }
  return plane * depth;
}

std::size_t knownObstacleLocalLinearIndex3D(const GridBounds3D& bounds,
                                            const GridIndex3D local_cell) noexcept {
  return (static_cast<std::size_t>(local_cell.z) *
              static_cast<std::size_t>(bounds.height_cells) +
          static_cast<std::size_t>(local_cell.y)) *
             static_cast<std::size_t>(bounds.width_cells) +
         static_cast<std::size_t>(local_cell.x);
}

std::size_t knownObstacleChunkBitIndex3D(const GridIndex3D local_cell) noexcept {
  constexpr int kChunkSize{KnownObstacleDistance3D::kChunkSize};
  const int local_x = local_cell.x % kChunkSize;
  const int local_y = local_cell.y % kChunkSize;
  const int local_z = local_cell.z % kChunkSize;
  return (static_cast<std::size_t>(local_z) * static_cast<std::size_t>(kChunkSize) +
          static_cast<std::size_t>(local_y)) *
             static_cast<std::size_t>(kChunkSize) +
         static_cast<std::size_t>(local_x);
}

bool knownObstacleSourceCanInfluence3D(const KnownObstacleDistanceStorage3D& storage,
                                       const GridIndex3D global_cell) noexcept {
  if (global_cell.x < storage.source_minimum_x ||
      global_cell.x >= storage.source_maximum_x_exclusive ||
      global_cell.y < storage.source_minimum_y ||
      global_cell.y >= storage.source_maximum_y_exclusive ||
      global_cell.z < storage.source_minimum_z ||
      global_cell.z >= storage.source_maximum_z_exclusive) {
    return false;
  }
  const int output_maximum_x =
      storage.output_offset_x + storage.output_bounds.width_cells - 1;
  const int output_maximum_y =
      storage.output_offset_y + storage.output_bounds.height_cells - 1;
  const int output_maximum_z =
      storage.output_offset_z + storage.output_bounds.depth_cells - 1;
  const double minimum_squared_distance =
      squaredDistanceBetweenIntervals(global_cell.x, global_cell.x,
                                      storage.output_offset_x, output_maximum_x) +
      squaredDistanceBetweenIntervals(global_cell.y, global_cell.y,
                                      storage.output_offset_y, output_maximum_y) +
      squaredDistanceBetweenIntervals(global_cell.z, global_cell.z,
                                      storage.output_offset_z, output_maximum_z);
  return minimum_squared_distance <= storage.maximum_squared_cells;
}

std::uint64_t knownObstacleSourceKey3D(const GridBounds3D& world_bounds,
                                       const GridIndex3D global_cell) noexcept {
  return (static_cast<std::uint64_t>(global_cell.z) *
              static_cast<std::uint64_t>(world_bounds.height_cells) +
          static_cast<std::uint64_t>(global_cell.y)) *
             static_cast<std::uint64_t>(world_bounds.width_cells) +
         static_cast<std::uint64_t>(global_cell.x);
}

OccupancyChunkIndex3D
knownObstacleOutputChunk3D(const GridIndex3D local_cell) noexcept {
  constexpr int kChunkSize{KnownObstacleDistance3D::kChunkSize};
  return OccupancyChunkIndex3D{local_cell.x / kChunkSize, local_cell.y / kChunkSize,
                               local_cell.z / kChunkSize};
}

KnownObstacleSourceIndex3D::KnownObstacleSourceIndex3D(
    const KnownObstacleSourceChunkMap3D& source_chunks) {
  std::vector<KnownObstacleSourcePoint3D> sources =
      flattenKnownObstacleSources3D(source_chunks);
  nodes_.reserve(sources.size());
  root_ = build(sources, 0U, sources.size(), 0);
}

KnownObstacleSourceIndex3D::KnownObstacleSourceIndex3D(
    const std::span<const KnownObstacleSourcePoint3D> sources) {
  std::vector<KnownObstacleSourcePoint3D> mutable_sources{sources.begin(),
                                                          sources.end()};
  nodes_.reserve(mutable_sources.size());
  root_ = build(mutable_sources, 0U, mutable_sources.size(), 0);
}

int KnownObstacleSourceIndex3D::build(std::vector<KnownObstacleSourcePoint3D>& sources,
                                      const std::size_t begin, const std::size_t end,
                                      const int depth) {
  if (begin >= end) {
    return -1;
  }
  const int axis = depth % 3;
  const std::size_t middle = begin + (end - begin) / 2U;
  std::nth_element(std::next(sources.begin(), static_cast<std::ptrdiff_t>(begin)),
                   std::next(sources.begin(), static_cast<std::ptrdiff_t>(middle)),
                   std::next(sources.begin(), static_cast<std::ptrdiff_t>(end)),
                   [axis](const KnownObstacleSourcePoint3D& first,
                          const KnownObstacleSourcePoint3D& second) {
                     const int first_coordinate = axisCoordinate(first.cell, axis);
                     const int second_coordinate = axisCoordinate(second.cell, axis);
                     return first_coordinate == second_coordinate
                                ? first.key < second.key
                                : first_coordinate < second_coordinate;
                   });
  const int node_index = static_cast<int>(nodes_.size());
  nodes_.push_back(Node{.source = sources.at(middle), .axis = axis});
  const int left = build(sources, begin, middle, depth + 1);
  const int right = build(sources, middle + 1U, end, depth + 1);
  nodes_.at(static_cast<std::size_t>(node_index)).left = left;
  nodes_.at(static_cast<std::size_t>(node_index)).right = right;
  return node_index;
}

KnownObstacleNearestSource3D
KnownObstacleSourceIndex3D::nearest(const GridIndex3D global_cell,
                                    const double maximum_squared_cells) const noexcept {
  KnownObstacleNearestSource3D result{
      .key = kNoKnownObstacleSource3D,
      .squared_cells = maximum_squared_cells,
  };
  nearestRecursive(root_, global_cell, result);
  return result;
}

void KnownObstacleSourceIndex3D::nearestRecursive(
    const int node_index, const GridIndex3D cell,
    KnownObstacleNearestSource3D& nearest_source) const noexcept {
  if (node_index < 0) {
    return;
  }
  const Node& node = nodes_.at(static_cast<std::size_t>(node_index));
  const double dx =
      static_cast<double>(cell.x) - static_cast<double>(node.source.cell.x);
  const double dy =
      static_cast<double>(cell.y) - static_cast<double>(node.source.cell.y);
  const double dz =
      static_cast<double>(cell.z) - static_cast<double>(node.source.cell.z);
  const double squared_cells = dx * dx + dy * dy + dz * dz;
  if (squared_cells < nearest_source.squared_cells ||
      (squared_cells == nearest_source.squared_cells &&
       (nearest_source.key == kNoKnownObstacleSource3D ||
        node.source.key < nearest_source.key))) {
    nearest_source = {.key = node.source.key, .squared_cells = squared_cells};
  }
  const int delta =
      axisCoordinate(cell, node.axis) - axisCoordinate(node.source.cell, node.axis);
  const int near_child = delta < 0 ? node.left : node.right;
  const int far_child = delta < 0 ? node.right : node.left;
  nearestRecursive(near_child, cell, nearest_source);
  if (static_cast<double>(delta) * static_cast<double>(delta) <=
      nearest_source.squared_cells) {
    nearestRecursive(far_child, cell, nearest_source);
  }
}

std::size_t KnownObstacleSourceIndex3D::size() const noexcept {
  return nodes_.size();
}

KnownObstacleSourceChunk3D collectKnownObstacleSourceChunk3D(
    const ObservedOccupancyGrid3D& occupancy,
    const KnownObstacleDistanceStorage3D& storage,
    const OccupancyChunkIndex3D chunk_index,
    const std::unordered_set<std::uint64_t>& suppressed_keys) {
  KnownObstacleSourceChunk3D result;
  const ObservedOccupancyGrid3D::Chunk* const chunk = occupancy.findChunk(chunk_index);
  if (chunk == nullptr) {
    return result;
  }
  for (std::size_t word_index = 0U; word_index < chunk->occupied.size(); ++word_index) {
    std::uint64_t occupied_bits = chunk->occupied.at(word_index);
    while (occupied_bits != 0U) {
      const int bit_offset = std::countr_zero(occupied_bits);
      const std::size_t bit_index =
          word_index * 64U + static_cast<std::size_t>(bit_offset);
      const int local_z = static_cast<int>(
          bit_index / static_cast<std::size_t>(ObservedOccupancyGrid3D::kChunkSize *
                                               ObservedOccupancyGrid3D::kChunkSize));
      const int local_y =
          static_cast<int>((bit_index / ObservedOccupancyGrid3D::kChunkSize) %
                           ObservedOccupancyGrid3D::kChunkSize);
      const int local_x = static_cast<int>(
          bit_index % static_cast<std::size_t>(ObservedOccupancyGrid3D::kChunkSize));
      const GridIndex3D cell{
          chunk_index.x * ObservedOccupancyGrid3D::kChunkSize + local_x,
          chunk_index.y * ObservedOccupancyGrid3D::kChunkSize + local_y,
          chunk_index.z * ObservedOccupancyGrid3D::kChunkSize + local_z,
      };
      const std::uint64_t key = knownObstacleSourceKey3D(storage.world_bounds, cell);
      if (knownObstacleSourceCanInfluence3D(storage, cell) &&
          !suppressed_keys.contains(key)) {
        result.push_back({.cell = cell, .key = key});
      }
      occupied_bits &= occupied_bits - 1U;
    }
  }
  std::ranges::sort(result, {}, &KnownObstacleSourcePoint3D::key);
  return result;
}

KnownObstacleSourceChunkMap3D collectKnownObstacleSourceChunks3D(
    const ObservedOccupancyGrid3D& occupancy,
    const KnownObstacleDistanceStorage3D& storage,
    const std::unordered_set<std::uint64_t>& suppressed_keys) {
  KnownObstacleSourceChunkMap3D result;
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  const OccupancyChunkIndex3D first{storage.source_minimum_x / kChunkSize,
                                    storage.source_minimum_y / kChunkSize,
                                    storage.source_minimum_z / kChunkSize};
  const OccupancyChunkIndex3D last{
      (storage.source_maximum_x_exclusive - 1) / kChunkSize,
      (storage.source_maximum_y_exclusive - 1) / kChunkSize,
      (storage.source_maximum_z_exclusive - 1) / kChunkSize};
  for (int z = first.z; z <= last.z; ++z) {
    for (int y = first.y; y <= last.y; ++y) {
      for (int x = first.x; x <= last.x; ++x) {
        const OccupancyChunkIndex3D chunk_index{x, y, z};
        KnownObstacleSourceChunk3D sources = collectKnownObstacleSourceChunk3D(
            occupancy, storage, chunk_index, suppressed_keys);
        if (!sources.empty()) {
          result.emplace(
              chunk_index,
              std::make_shared<const KnownObstacleSourceChunk3D>(std::move(sources)));
        }
      }
    }
  }
  return result;
}

std::vector<KnownObstacleSourcePoint3D>
flattenKnownObstacleSources3D(const KnownObstacleSourceChunkMap3D& source_chunks) {
  std::size_t count{0U};
  for (const auto& [chunk_index, sources] : source_chunks) {
    static_cast<void>(chunk_index);
    count += sources->size();
  }
  std::vector<KnownObstacleSourcePoint3D> result;
  result.reserve(count);
  for (const auto& [chunk_index, sources] : source_chunks) {
    static_cast<void>(chunk_index);
    result.insert(result.end(), sources->begin(), sources->end());
  }
  std::ranges::sort(result, {}, &KnownObstacleSourcePoint3D::key);
  return result;
}

std::uint64_t
knownObstacleSourceFingerprint3D(const KnownObstacleDistanceStorage3D& storage,
                                 const KnownObstacleSourceChunkMap3D& source_chunks) {
  std::uint64_t hash = kFnvOffsetBasis;
  const auto combine = [&hash](const std::uint64_t value) {
    hash ^= value;
    hash *= kFnvPrime;
  };
  combine(std::bit_cast<std::uint64_t>(storage.world_bounds.origin_x));
  combine(std::bit_cast<std::uint64_t>(storage.world_bounds.origin_y));
  combine(std::bit_cast<std::uint64_t>(storage.world_bounds.origin_z));
  combine(std::bit_cast<std::uint64_t>(storage.world_bounds.resolution_m));
  combine(static_cast<std::uint64_t>(storage.world_bounds.width_cells));
  combine(static_cast<std::uint64_t>(storage.world_bounds.height_cells));
  combine(static_cast<std::uint64_t>(storage.world_bounds.depth_cells));
  combine(std::bit_cast<std::uint64_t>(storage.output_bounds.origin_x));
  combine(std::bit_cast<std::uint64_t>(storage.output_bounds.origin_y));
  combine(std::bit_cast<std::uint64_t>(storage.output_bounds.origin_z));
  combine(std::bit_cast<std::uint64_t>(storage.output_bounds.resolution_m));
  combine(static_cast<std::uint64_t>(storage.output_bounds.width_cells));
  combine(static_cast<std::uint64_t>(storage.output_bounds.height_cells));
  combine(static_cast<std::uint64_t>(storage.output_bounds.depth_cells));
  combine(static_cast<std::uint64_t>(storage.source_minimum_x));
  combine(static_cast<std::uint64_t>(storage.source_minimum_y));
  combine(static_cast<std::uint64_t>(storage.source_minimum_z));
  combine(static_cast<std::uint64_t>(storage.source_maximum_x_exclusive));
  combine(static_cast<std::uint64_t>(storage.source_maximum_y_exclusive));
  combine(static_cast<std::uint64_t>(storage.source_maximum_z_exclusive));
  combine(std::bit_cast<std::uint64_t>(storage.maximum_distance_m));
  for (const KnownObstacleSourcePoint3D source :
       flattenKnownObstacleSources3D(source_chunks)) {
    combine(source.key);
  }
  return hash == 0U ? 1U : hash;
}

std::vector<OccupancyChunkIndex3D> knownObstacleOutputChunksAffectedBySources3D(
    const KnownObstacleDistanceStorage3D& storage,
    const std::span<const KnownObstacleSourcePoint3D> sources) {
  std::unordered_map<OccupancyChunkIndex3D, SourceChunkExtent3D,
                     OccupancyChunkIndex3DHash>
      source_extents;
  for (const KnownObstacleSourcePoint3D source : sources) {
    const OccupancyChunkIndex3D source_chunk =
        ObservedOccupancyGrid3D::chunkIndex(source.cell);
    SourceChunkExtent3D& extent = source_extents[source_chunk];
    if (!extent.initialized) {
      extent.minimum = source.cell;
      extent.maximum = source.cell;
      extent.initialized = true;
    } else {
      extent.minimum.x = std::min(extent.minimum.x, source.cell.x);
      extent.minimum.y = std::min(extent.minimum.y, source.cell.y);
      extent.minimum.z = std::min(extent.minimum.z, source.cell.z);
      extent.maximum.x = std::max(extent.maximum.x, source.cell.x);
      extent.maximum.y = std::max(extent.maximum.y, source.cell.y);
      extent.maximum.z = std::max(extent.maximum.z, source.cell.z);
    }
  }

  std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash> chunks;
  constexpr int kChunkSize{KnownObstacleDistance3D::kChunkSize};
  for (const auto& [source_chunk, extent] : source_extents) {
    static_cast<void>(source_chunk);
    const int minimum_local_x =
        std::max(0, extent.minimum.x - storage.output_offset_x - storage.radius_cells);
    const int minimum_local_y =
        std::max(0, extent.minimum.y - storage.output_offset_y - storage.radius_cells);
    const int minimum_local_z =
        std::max(0, extent.minimum.z - storage.output_offset_z - storage.radius_cells);
    const int maximum_local_x =
        std::min(storage.output_bounds.width_cells - 1,
                 extent.maximum.x - storage.output_offset_x + storage.radius_cells);
    const int maximum_local_y =
        std::min(storage.output_bounds.height_cells - 1,
                 extent.maximum.y - storage.output_offset_y + storage.radius_cells);
    const int maximum_local_z =
        std::min(storage.output_bounds.depth_cells - 1,
                 extent.maximum.z - storage.output_offset_z + storage.radius_cells);
    if (minimum_local_x > maximum_local_x || minimum_local_y > maximum_local_y ||
        minimum_local_z > maximum_local_z) {
      continue;
    }
    for (int chunk_z = minimum_local_z / kChunkSize;
         chunk_z <= maximum_local_z / kChunkSize; ++chunk_z) {
      for (int chunk_y = minimum_local_y / kChunkSize;
           chunk_y <= maximum_local_y / kChunkSize; ++chunk_y) {
        for (int chunk_x = minimum_local_x / kChunkSize;
             chunk_x <= maximum_local_x / kChunkSize; ++chunk_x) {
          const int output_minimum_x = storage.output_offset_x + chunk_x * kChunkSize;
          const int output_minimum_y = storage.output_offset_y + chunk_y * kChunkSize;
          const int output_minimum_z = storage.output_offset_z + chunk_z * kChunkSize;
          const int output_maximum_x =
              storage.output_offset_x +
              std::min(storage.output_bounds.width_cells, (chunk_x + 1) * kChunkSize) -
              1;
          const int output_maximum_y =
              storage.output_offset_y +
              std::min(storage.output_bounds.height_cells, (chunk_y + 1) * kChunkSize) -
              1;
          const int output_maximum_z =
              storage.output_offset_z +
              std::min(storage.output_bounds.depth_cells, (chunk_z + 1) * kChunkSize) -
              1;
          const double minimum_squared_distance =
              squaredDistanceBetweenIntervals(extent.minimum.x, extent.maximum.x,
                                              output_minimum_x, output_maximum_x) +
              squaredDistanceBetweenIntervals(extent.minimum.y, extent.maximum.y,
                                              output_minimum_y, output_maximum_y) +
              squaredDistanceBetweenIntervals(extent.minimum.z, extent.maximum.z,
                                              output_minimum_z, output_maximum_z);
          if (minimum_squared_distance <= storage.maximum_squared_cells) {
            chunks.insert({chunk_x, chunk_y, chunk_z});
          }
        }
      }
    }
  }
  std::vector<OccupancyChunkIndex3D> result{chunks.begin(), chunks.end()};
  std::ranges::sort(result, {}, [](const OccupancyChunkIndex3D index) {
    return std::tuple{index.z, index.y, index.x};
  });
  return result;
}

std::shared_ptr<const KnownObstacleDistanceChunk3D>
computeKnownObstacleDistanceChunk3D(const KnownObstacleDistanceStorage3D& storage,
                                    const KnownObstacleSourceIndex3D& source_index,
                                    const OccupancyChunkIndex3D output_chunk) {
  auto chunk = std::make_shared<KnownObstacleDistanceChunk3D>();
  chunk->distances_m.fill(std::numeric_limits<float>::infinity());
  const KnownObstacleDistanceRegion3D region =
      knownObstacleChunkRegion3D(storage.output_bounds, output_chunk);
  for (int z = region.minimum_z; z < region.maximum_z_exclusive; ++z) {
    for (int y = region.minimum_y; y < region.maximum_y_exclusive; ++y) {
      for (int x = region.minimum_x; x < region.maximum_x_exclusive; ++x) {
        const GridIndex3D local{x, y, z};
        const GridIndex3D global{x + storage.output_offset_x,
                                 y + storage.output_offset_y,
                                 z + storage.output_offset_z};
        const KnownObstacleNearestSource3D nearest =
            source_index.nearest(global, storage.maximum_squared_cells);
        if (!nearest.found() || nearest.squared_cells > storage.maximum_squared_cells) {
          continue;
        }
        const std::size_t bit_index = knownObstacleChunkBitIndex3D(local);
        chunk->distances_m.at(bit_index) = static_cast<float>(
            std::sqrt(nearest.squared_cells) * storage.output_bounds.resolution_m);
        ++chunk->finite_voxels;
      }
    }
  }
  return chunk->finite_voxels == 0U ? nullptr : std::move(chunk);
}

std::vector<std::shared_ptr<const KnownObstacleDistanceChunk3D>>
computeKnownObstacleDistanceChunks3D(
    const KnownObstacleDistanceStorage3D& storage,
    const KnownObstacleSourceIndex3D& source_index,
    const std::span<const OccupancyChunkIndex3D> output_chunks,
    BoundedWorkerPool* const worker_pool) {
  std::vector<std::shared_ptr<const KnownObstacleDistanceChunk3D>> result(
      output_chunks.size());
  const auto compute = [&](const std::size_t index) {
    result.at(index) = computeKnownObstacleDistanceChunk3D(storage, source_index,
                                                           output_chunks[index]);
  };
  if (worker_pool != nullptr) {
    worker_pool->parallelFor(output_chunks.size(), WorkerTaskLane::kWorldUpdate,
                             compute);
  } else {
    for (std::size_t index = 0U; index < output_chunks.size(); ++index) {
      compute(index);
    }
  }
  return result;
}

KnownObstacleDistanceRegion3D
knownObstacleChunkRegion3D(const GridBounds3D& bounds,
                           const OccupancyChunkIndex3D output_chunk) noexcept {
  constexpr int kChunkSize{KnownObstacleDistance3D::kChunkSize};
  return KnownObstacleDistanceRegion3D{
      .minimum_x = output_chunk.x * kChunkSize,
      .minimum_y = output_chunk.y * kChunkSize,
      .minimum_z = output_chunk.z * kChunkSize,
      .maximum_x_exclusive =
          std::min(bounds.width_cells, (output_chunk.x + 1) * kChunkSize),
      .maximum_y_exclusive =
          std::min(bounds.height_cells, (output_chunk.y + 1) * kChunkSize),
      .maximum_z_exclusive =
          std::min(bounds.depth_cells, (output_chunk.z + 1) * kChunkSize),
  };
}

std::size_t
knownObstacleRegionVoxelCount3D(const KnownObstacleDistanceRegion3D& region) noexcept {
  return static_cast<std::size_t>(region.maximum_x_exclusive - region.minimum_x) *
         static_cast<std::size_t>(region.maximum_y_exclusive - region.minimum_y) *
         static_cast<std::size_t>(region.maximum_z_exclusive - region.minimum_z);
}

std::unordered_set<std::uint64_t>
knownObstacleSuppressedKeys3D(const GridBounds3D& world_bounds,
                              const std::span<const GridIndex3D> suppressed_cells) {
  std::unordered_set<std::uint64_t> result;
  result.reserve(suppressed_cells.size());
  for (const GridIndex3D cell : suppressed_cells) {
    if (cell.x >= 0 && cell.y >= 0 && cell.z >= 0 &&
        cell.x < world_bounds.width_cells && cell.y < world_bounds.height_cells &&
        cell.z < world_bounds.depth_cells) {
      result.insert(knownObstacleSourceKey3D(world_bounds, cell));
    }
  }
  return result;
}

std::shared_ptr<KnownObstacleDistanceStorage3D> makeKnownObstacleDistanceStorage3D(
    const ObservedOccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
    const double maximum_distance_m,
    const std::span<const GridIndex3D> suppressed_cells) {
  const GridBounds3D& world_bounds = occupancy.bounds();
  const GridBounds3D source_bounds = knownObstacleDistanceSourceBounds3D(
      world_bounds, local_bounds, maximum_distance_m);
  auto storage = std::make_shared<KnownObstacleDistanceStorage3D>();
  storage->world_bounds = world_bounds;
  storage->output_bounds = local_bounds;
  storage->source_bounds = source_bounds;
  storage->output_offset_x = alignedCellOffset(
      local_bounds.origin_x, world_bounds.origin_x, world_bounds.resolution_m);
  storage->output_offset_y = alignedCellOffset(
      local_bounds.origin_y, world_bounds.origin_y, world_bounds.resolution_m);
  storage->output_offset_z = alignedCellOffset(
      local_bounds.origin_z, world_bounds.origin_z, world_bounds.resolution_m);
  storage->source_minimum_x = alignedCellOffset(
      source_bounds.origin_x, world_bounds.origin_x, world_bounds.resolution_m);
  storage->source_minimum_y = alignedCellOffset(
      source_bounds.origin_y, world_bounds.origin_y, world_bounds.resolution_m);
  storage->source_minimum_z = alignedCellOffset(
      source_bounds.origin_z, world_bounds.origin_z, world_bounds.resolution_m);
  storage->source_maximum_x_exclusive =
      storage->source_minimum_x + source_bounds.width_cells;
  storage->source_maximum_y_exclusive =
      storage->source_minimum_y + source_bounds.height_cells;
  storage->source_maximum_z_exclusive =
      storage->source_minimum_z + source_bounds.depth_cells;
  storage->radius_cells =
      static_cast<int>(std::ceil(maximum_distance_m / world_bounds.resolution_m));
  storage->maximum_distance_m = maximum_distance_m;
  storage->maximum_squared_cells =
      std::pow(maximum_distance_m / world_bounds.resolution_m, 2.0);
  storage->suppressed_source_cells.assign(suppressed_cells.begin(),
                                          suppressed_cells.end());
  std::ranges::sort(storage->suppressed_source_cells,
                    [](const GridIndex3D first, const GridIndex3D second) {
                      return std::tuple{first.z, first.y, first.x} <
                             std::tuple{second.z, second.y, second.x};
                    });
  storage->suppressed_source_cells.erase(
      std::unique(storage->suppressed_source_cells.begin(),
                  storage->suppressed_source_cells.end()),
      storage->suppressed_source_cells.end());
  return storage;
}

} // namespace detail

KnownObstacleDistance3DBuildResult
buildKnownObstacleDistance3D(const ObservedOccupancyGrid3D& occupancy,
                             const GridBounds3D& local_bounds,
                             const double maximum_distance_m,
                             const std::span<const GridIndex3D> suppressed_source_cells,
                             BoundedWorkerPool* const worker_pool) {
  const auto started = std::chrono::steady_clock::now();
  auto storage = detail::makeKnownObstacleDistanceStorage3D(
      occupancy, local_bounds, maximum_distance_m, suppressed_source_cells);
  const auto source_started = std::chrono::steady_clock::now();
  const std::unordered_set<std::uint64_t> suppressed_keys =
      detail::knownObstacleSuppressedKeys3D(occupancy.bounds(),
                                            suppressed_source_cells);
  storage->source_chunks =
      detail::collectKnownObstacleSourceChunks3D(occupancy, *storage, suppressed_keys);
  const std::vector<detail::KnownObstacleSourcePoint3D> sources =
      detail::flattenKnownObstacleSources3D(storage->source_chunks);
  const detail::KnownObstacleSourceIndex3D source_index{std::span{sources}};
  storage->source_voxels = source_index.size();
  storage->source_fingerprint =
      detail::knownObstacleSourceFingerprint3D(*storage, storage->source_chunks);
  const double source_index_ms = std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - source_started)
                                     .count();

  const auto query_started = std::chrono::steady_clock::now();
  const std::vector<OccupancyChunkIndex3D> candidate_chunks =
      detail::knownObstacleOutputChunksAffectedBySources3D(*storage, sources);
  const auto computed = detail::computeKnownObstacleDistanceChunks3D(
      *storage, source_index, candidate_chunks, worker_pool);
  std::size_t queried_voxels{0U};
  for (std::size_t index = 0U; index < candidate_chunks.size(); ++index) {
    queried_voxels += detail::knownObstacleRegionVoxelCount3D(
        detail::knownObstacleChunkRegion3D(local_bounds, candidate_chunks[index]));
    if (computed[index] != nullptr) {
      storage->finite_distance_voxels += computed[index]->finite_voxels;
      storage->distance_chunks.emplace(candidate_chunks[index], computed[index]);
    }
  }
  const double query_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - query_started)
                              .count();
  KnownObstacleDistance3DBuildResult result{
      .field = KnownObstacleDistance3D::create(storage),
      .dirty_regions = {},
      .stats =
          KnownObstacleDistance3DBuildStats{
              .source_voxels = storage->source_voxels,
              .source_chunks = storage->source_chunks.size(),
              .stored_distance_chunks = storage->distance_chunks.size(),
              .finite_distance_voxels = storage->finite_distance_voxels,
              .recomputed_chunks = candidate_chunks.size(),
              .queried_voxels = queried_voxels,
              .source_index_ms = source_index_ms,
              .distance_query_ms = query_ms,
          },
      .mode = KnownObstacleDistance3DBuildMode::kFull,
      .incremental_fallback = false,
  };
  result.stats.duration_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
  return result;
}

} // namespace drone_city_nav
