#include "drone_city_nav/known_obstacle_distance_3d.hpp"

#include "drone_city_nav/bounded_worker_pool.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kGeometryTolerance{1.0e-9};
constexpr std::uint64_t kFnvOffsetBasis{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};
// Squared cell distance assigned to cells without a source. It must stay
// finite for the parabola intersections and far above any reachable value.
constexpr double kUnreachedSquaredCells{1.0e12};

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

[[nodiscard]] bool sameBounds(const GridBounds3D& first,
                              const GridBounds3D& second) noexcept {
  return std::abs(first.origin_x - second.origin_x) <= kGeometryTolerance &&
         std::abs(first.origin_y - second.origin_y) <= kGeometryTolerance &&
         std::abs(first.origin_z - second.origin_z) <= kGeometryTolerance &&
         std::abs(first.resolution_m - second.resolution_m) <= kGeometryTolerance &&
         first.width_cells == second.width_cells &&
         first.height_cells == second.height_cells &&
         first.depth_cells == second.depth_cells;
}

[[nodiscard]] std::size_t checkedVoxelCount(const GridBounds3D& bounds) {
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

// Cell geometry of one transform: the output window and the source halo, both
// expressed as offsets into the world grid.
struct TransformGeometry3D {
  GridBounds3D world_bounds{};
  GridBounds3D output_bounds{};
  GridBounds3D source_bounds{};
  int output_offset_x{0};
  int output_offset_y{0};
  int output_offset_z{0};
  int source_minimum_x{0};
  int source_minimum_y{0};
  int source_minimum_z{0};
  double maximum_distance_m{0.0};
  double maximum_squared_cells{0.0};

  [[nodiscard]] int sourceMaximumXExclusive() const noexcept {
    return source_minimum_x + source_bounds.width_cells;
  }

  [[nodiscard]] int sourceMaximumYExclusive() const noexcept {
    return source_minimum_y + source_bounds.height_cells;
  }

  [[nodiscard]] int sourceMaximumZExclusive() const noexcept {
    return source_minimum_z + source_bounds.depth_cells;
  }

  [[nodiscard]] std::size_t sourceIndex(const int world_x, const int world_y,
                                        const int world_z) const noexcept {
    return (static_cast<std::size_t>(world_z - source_minimum_z) *
                static_cast<std::size_t>(source_bounds.height_cells) +
            static_cast<std::size_t>(world_y - source_minimum_y)) *
               static_cast<std::size_t>(source_bounds.width_cells) +
           static_cast<std::size_t>(world_x - source_minimum_x);
  }

  [[nodiscard]] bool insideSource(const GridIndex3D world_cell) const noexcept {
    return world_cell.x >= source_minimum_x &&
           world_cell.x < sourceMaximumXExclusive() &&
           world_cell.y >= source_minimum_y &&
           world_cell.y < sourceMaximumYExclusive() &&
           world_cell.z >= source_minimum_z && world_cell.z < sourceMaximumZExclusive();
  }

  // Only sources within the exact capped radius of some output cell can change
  // an output distance; those are the field's identity.
  [[nodiscard]] bool canInfluenceOutput(const GridIndex3D world_cell) const noexcept {
    const double minimum_squared_distance =
        squaredDistanceBetweenIntervals(world_cell.x, world_cell.x, output_offset_x,
                                        output_offset_x + output_bounds.width_cells -
                                            1) +
        squaredDistanceBetweenIntervals(world_cell.y, world_cell.y, output_offset_y,
                                        output_offset_y + output_bounds.height_cells -
                                            1) +
        squaredDistanceBetweenIntervals(world_cell.z, world_cell.z, output_offset_z,
                                        output_offset_z + output_bounds.depth_cells -
                                            1);
    return minimum_squared_distance <= maximum_squared_cells;
  }
};

[[nodiscard]] TransformGeometry3D makeGeometry(const GridBounds3D& world_bounds,
                                               const GridBounds3D& local_bounds,
                                               const double maximum_distance_m) {
  TransformGeometry3D geometry;
  geometry.world_bounds = world_bounds;
  geometry.output_bounds = local_bounds;
  geometry.source_bounds = knownObstacleDistanceSourceBounds3D(
      world_bounds, local_bounds, maximum_distance_m);
  geometry.output_offset_x = alignedCellOffset(
      local_bounds.origin_x, world_bounds.origin_x, world_bounds.resolution_m);
  geometry.output_offset_y = alignedCellOffset(
      local_bounds.origin_y, world_bounds.origin_y, world_bounds.resolution_m);
  geometry.output_offset_z = alignedCellOffset(
      local_bounds.origin_z, world_bounds.origin_z, world_bounds.resolution_m);
  geometry.source_minimum_x =
      alignedCellOffset(geometry.source_bounds.origin_x, world_bounds.origin_x,
                        world_bounds.resolution_m);
  geometry.source_minimum_y =
      alignedCellOffset(geometry.source_bounds.origin_y, world_bounds.origin_y,
                        world_bounds.resolution_m);
  geometry.source_minimum_z =
      alignedCellOffset(geometry.source_bounds.origin_z, world_bounds.origin_z,
                        world_bounds.resolution_m);
  geometry.maximum_distance_m = maximum_distance_m;
  const double maximum_cells = maximum_distance_m / world_bounds.resolution_m;
  geometry.maximum_squared_cells = maximum_cells * maximum_cells;
  return geometry;
}

[[nodiscard]] std::uint64_t sourceKey(const GridBounds3D& world_bounds,
                                      const GridIndex3D world_cell) noexcept {
  return (static_cast<std::uint64_t>(world_cell.z) *
              static_cast<std::uint64_t>(world_bounds.height_cells) +
          static_cast<std::uint64_t>(world_cell.y)) *
             static_cast<std::uint64_t>(world_bounds.width_cells) +
         static_cast<std::uint64_t>(world_cell.x);
}

struct CollectedSources3D {
  // Squared cell distances over the source halo: zero at a source, otherwise
  // unreached. This is the transform input.
  std::vector<float> squared_cells;
  std::vector<std::uint64_t> influencing_keys;
  std::uint64_t fingerprint{0U};
  double collection_ms{0.0};
};

[[nodiscard]] std::uint64_t fingerprintOf(const TransformGeometry3D& geometry,
                                          std::vector<std::uint64_t>& keys) {
  std::ranges::sort(keys);
  std::uint64_t hash = kFnvOffsetBasis;
  const auto combine = [&hash](const std::uint64_t value) {
    hash ^= value;
    hash *= kFnvPrime;
  };
  const auto combine_bounds = [&](const GridBounds3D& bounds) {
    combine(std::bit_cast<std::uint64_t>(bounds.origin_x));
    combine(std::bit_cast<std::uint64_t>(bounds.origin_y));
    combine(std::bit_cast<std::uint64_t>(bounds.origin_z));
    combine(std::bit_cast<std::uint64_t>(bounds.resolution_m));
    combine(static_cast<std::uint64_t>(bounds.width_cells));
    combine(static_cast<std::uint64_t>(bounds.height_cells));
    combine(static_cast<std::uint64_t>(bounds.depth_cells));
  };
  combine_bounds(geometry.world_bounds);
  combine_bounds(geometry.output_bounds);
  combine_bounds(geometry.source_bounds);
  combine(std::bit_cast<std::uint64_t>(geometry.maximum_distance_m));
  for (const std::uint64_t key : keys) {
    combine(key);
  }
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] CollectedSources3D
collectSources(const ObservedOccupancyGrid3D& occupancy,
               const TransformGeometry3D& geometry,
               const std::span<const GridIndex3D> suppressed_source_cells) {
  const auto started = std::chrono::steady_clock::now();
  CollectedSources3D result;
  result.squared_cells.assign(checkedVoxelCount(geometry.source_bounds),
                              static_cast<float>(kUnreachedSquaredCells));
  std::unordered_set<std::uint64_t> suppressed;
  suppressed.reserve(suppressed_source_cells.size());
  for (const GridIndex3D cell : suppressed_source_cells) {
    if (occupancy.contains(cell)) {
      suppressed.insert(sourceKey(geometry.world_bounds, cell));
    }
  }
  constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
  const int first_chunk_x = geometry.source_minimum_x / kChunkSize;
  const int first_chunk_y = geometry.source_minimum_y / kChunkSize;
  const int first_chunk_z = geometry.source_minimum_z / kChunkSize;
  const int last_chunk_x = (geometry.sourceMaximumXExclusive() - 1) / kChunkSize;
  const int last_chunk_y = (geometry.sourceMaximumYExclusive() - 1) / kChunkSize;
  const int last_chunk_z = (geometry.sourceMaximumZExclusive() - 1) / kChunkSize;
  for (int chunk_z = first_chunk_z; chunk_z <= last_chunk_z; ++chunk_z) {
    for (int chunk_y = first_chunk_y; chunk_y <= last_chunk_y; ++chunk_y) {
      for (int chunk_x = first_chunk_x; chunk_x <= last_chunk_x; ++chunk_x) {
        const ObservedOccupancyGrid3D::Chunk* const chunk =
            occupancy.findChunk(OccupancyChunkIndex3D{chunk_x, chunk_y, chunk_z});
        if (chunk == nullptr) {
          continue;
        }
        std::size_t word_index{0U};
        for (const std::uint64_t word : chunk->occupied) {
          std::uint64_t occupied_bits = word;
          const std::size_t word_offset = word_index++ * 64U;
          while (occupied_bits != 0U) {
            const int bit_offset = std::countr_zero(occupied_bits);
            occupied_bits &= occupied_bits - 1U;
            const std::size_t bit_index =
                word_offset + static_cast<std::size_t>(bit_offset);
            const GridIndex3D cell{
                chunk_x * kChunkSize + static_cast<int>(bit_index % kChunkSize),
                chunk_y * kChunkSize +
                    static_cast<int>((bit_index / kChunkSize) % kChunkSize),
                chunk_z * kChunkSize +
                    static_cast<int>(bit_index /
                                     static_cast<std::size_t>(kChunkSize * kChunkSize)),
            };
            if (!geometry.insideSource(cell)) {
              continue;
            }
            const std::uint64_t key = sourceKey(geometry.world_bounds, cell);
            if (!suppressed.empty() && suppressed.contains(key)) {
              continue;
            }
            result.squared_cells[geometry.sourceIndex(cell.x, cell.y, cell.z)] = 0.0F;
            if (geometry.canInfluenceOutput(cell)) {
              result.influencing_keys.push_back(key);
            }
          }
        }
      }
    }
  }
  result.fingerprint = fingerprintOf(geometry, result.influencing_keys);
  result.collection_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - started)
                             .count();
  return result;
}

// One-dimensional squared Euclidean distance transform of sampled function
// values (Felzenszwalb and Huttenlocher). Scratch buffers are reused per line.
struct TransformScratch1D {
  std::vector<double> values;
  std::vector<double> output;
  std::vector<int> parabola_locations;
  std::vector<double> boundaries;

  void reserve(const std::size_t size) {
    values.resize(size);
    output.resize(size);
    parabola_locations.resize(size);
    boundaries.resize(size + 1U);
  }
};

void transformLine(TransformScratch1D& scratch, const std::size_t size) {
  std::vector<double>& input = scratch.values;
  std::vector<int>& locations = scratch.parabola_locations;
  std::vector<double>& boundaries = scratch.boundaries;
  int envelope_size = 0;
  locations[0] = 0;
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();
  for (int candidate = 1; candidate < static_cast<int>(size); ++candidate) {
    double intersection = 0.0;
    while (envelope_size >= 0) {
      const int location = locations[static_cast<std::size_t>(envelope_size)];
      const double candidate_value = static_cast<double>(candidate);
      const double location_value = static_cast<double>(location);
      intersection = ((input[static_cast<std::size_t>(candidate)] +
                       candidate_value * candidate_value) -
                      (input[static_cast<std::size_t>(location)] +
                       location_value * location_value)) /
                     (2.0 * static_cast<double>(candidate - location));
      if (intersection > boundaries[static_cast<std::size_t>(envelope_size)]) {
        break;
      }
      --envelope_size;
    }
    if (envelope_size < 0) {
      envelope_size = 0;
      locations[0] = candidate;
      boundaries[0] = -std::numeric_limits<double>::infinity();
      boundaries[1] = std::numeric_limits<double>::infinity();
    } else {
      ++envelope_size;
      locations[static_cast<std::size_t>(envelope_size)] = candidate;
      boundaries[static_cast<std::size_t>(envelope_size)] = intersection;
      boundaries[static_cast<std::size_t>(envelope_size) + 1U] =
          std::numeric_limits<double>::infinity();
    }
  }
  envelope_size = 0;
  for (int position = 0; position < static_cast<int>(size); ++position) {
    while (boundaries[static_cast<std::size_t>(envelope_size) + 1U] <
           static_cast<double>(position)) {
      ++envelope_size;
    }
    const int location = locations[static_cast<std::size_t>(envelope_size)];
    const double delta = static_cast<double>(position - location);
    scratch.output[static_cast<std::size_t>(position)] =
        std::min(kUnreachedSquaredCells,
                 delta * delta + input[static_cast<std::size_t>(location)]);
  }
}

template<typename LineFunction>
void forEachLine(BoundedWorkerPool* const worker_pool, const std::size_t line_count,
                 const LineFunction& function) {
  if (worker_pool != nullptr) {
    worker_pool->parallelFor(line_count, WorkerTaskLane::kWorldUpdate, function);
    return;
  }
  for (std::size_t line = 0U; line < line_count; ++line) {
    function(line);
  }
}

[[nodiscard]] TransformScratch1D& lineScratch(const std::size_t size) {
  thread_local TransformScratch1D scratch;
  if (scratch.values.size() < size) {
    scratch.reserve(size);
  }
  return scratch;
}

// Three separable passes over the source halo. Later passes only visit lines
// that intersect the output window, so the halo costs one extra x pass.
void transformSeparable(std::vector<float>& squared_cells,
                        const TransformGeometry3D& geometry,
                        BoundedWorkerPool* const worker_pool) {
  const int source_width = geometry.source_bounds.width_cells;
  const int source_height = geometry.source_bounds.height_cells;
  const int source_depth = geometry.source_bounds.depth_cells;
  const int output_x_begin = geometry.output_offset_x - geometry.source_minimum_x;
  const int output_y_begin = geometry.output_offset_y - geometry.source_minimum_y;
  const int output_width = geometry.output_bounds.width_cells;
  const int output_height = geometry.output_bounds.height_cells;
  const auto index = [source_width, source_height](const int x, const int y,
                                                   const int z) {
    return (static_cast<std::size_t>(z) * static_cast<std::size_t>(source_height) +
            static_cast<std::size_t>(y)) *
               static_cast<std::size_t>(source_width) +
           static_cast<std::size_t>(x);
  };
  const std::size_t x_line_count =
      static_cast<std::size_t>(source_height) * static_cast<std::size_t>(source_depth);
  forEachLine(worker_pool, x_line_count, [&](const std::size_t line) {
    const int z = static_cast<int>(line / static_cast<std::size_t>(source_height));
    const int y = static_cast<int>(line % static_cast<std::size_t>(source_height));
    TransformScratch1D& scratch = lineScratch(static_cast<std::size_t>(source_width));
    bool any_source{false};
    for (int x = 0; x < source_width; ++x) {
      const float value = squared_cells[index(x, y, z)];
      any_source = any_source || value == 0.0F;
      scratch.values[static_cast<std::size_t>(x)] = static_cast<double>(value);
    }
    if (!any_source) {
      return;
    }
    transformLine(scratch, static_cast<std::size_t>(source_width));
    for (int x = 0; x < source_width; ++x) {
      squared_cells[index(x, y, z)] =
          static_cast<float>(scratch.output[static_cast<std::size_t>(x)]);
    }
  });
  const std::size_t y_line_count =
      static_cast<std::size_t>(output_width) * static_cast<std::size_t>(source_depth);
  forEachLine(worker_pool, y_line_count, [&](const std::size_t line) {
    const int z = static_cast<int>(line / static_cast<std::size_t>(output_width));
    const int x = output_x_begin +
                  static_cast<int>(line % static_cast<std::size_t>(output_width));
    TransformScratch1D& scratch = lineScratch(static_cast<std::size_t>(source_height));
    bool any_reached{false};
    for (int y = 0; y < source_height; ++y) {
      const float value = squared_cells[index(x, y, z)];
      any_reached = any_reached || static_cast<double>(value) < kUnreachedSquaredCells;
      scratch.values[static_cast<std::size_t>(y)] = static_cast<double>(value);
    }
    if (!any_reached) {
      return;
    }
    transformLine(scratch, static_cast<std::size_t>(source_height));
    for (int y = 0; y < source_height; ++y) {
      squared_cells[index(x, y, z)] =
          static_cast<float>(scratch.output[static_cast<std::size_t>(y)]);
    }
  });
  const std::size_t z_line_count =
      static_cast<std::size_t>(output_width) * static_cast<std::size_t>(output_height);
  forEachLine(worker_pool, z_line_count, [&](const std::size_t line) {
    const int y = output_y_begin +
                  static_cast<int>(line / static_cast<std::size_t>(output_width));
    const int x = output_x_begin +
                  static_cast<int>(line % static_cast<std::size_t>(output_width));
    TransformScratch1D& scratch = lineScratch(static_cast<std::size_t>(source_depth));
    bool any_reached{false};
    for (int z = 0; z < source_depth; ++z) {
      const float value = squared_cells[index(x, y, z)];
      any_reached = any_reached || static_cast<double>(value) < kUnreachedSquaredCells;
      scratch.values[static_cast<std::size_t>(z)] = static_cast<double>(value);
    }
    if (!any_reached) {
      return;
    }
    transformLine(scratch, static_cast<std::size_t>(source_depth));
    for (int z = 0; z < source_depth; ++z) {
      squared_cells[index(x, y, z)] =
          static_cast<float>(scratch.output[static_cast<std::size_t>(z)]);
    }
  });
}

[[nodiscard]] KnownObstacleDistance3DBuildResult
transformCollectedSources(const TransformGeometry3D& geometry,
                          CollectedSources3D sources,
                          BoundedWorkerPool* const worker_pool,
                          const std::chrono::steady_clock::time_point started) {
  const auto transform_started = std::chrono::steady_clock::now();
  const std::size_t transform_voxels = sources.squared_cells.size();
  if (!sources.influencing_keys.empty()) {
    transformSeparable(sources.squared_cells, geometry, worker_pool);
  }
  const GridBounds3D& output = geometry.output_bounds;
  auto distances = std::make_shared<std::vector<float>>(
      checkedVoxelCount(output), std::numeric_limits<float>::infinity());
  std::size_t finite_voxels{0U};
  if (!sources.influencing_keys.empty()) {
    const double resolution_m = geometry.world_bounds.resolution_m;
    std::size_t output_index{0U};
    for (int z = 0; z < output.depth_cells; ++z) {
      for (int y = 0; y < output.height_cells; ++y) {
        const std::size_t row =
            geometry.sourceIndex(geometry.output_offset_x, geometry.output_offset_y + y,
                                 geometry.output_offset_z + z);
        for (int x = 0; x < output.width_cells; ++x, ++output_index) {
          const double squared = static_cast<double>(
              sources.squared_cells[row + static_cast<std::size_t>(x)]);
          if (squared > geometry.maximum_squared_cells) {
            continue;
          }
          (*distances)[output_index] =
              static_cast<float>(std::sqrt(squared) * resolution_m);
          ++finite_voxels;
        }
      }
    }
  }
  const double transform_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - transform_started)
                                  .count();
  KnownObstacleDistance3DBuildResult result{
      .field = KnownObstacleDistance3D::create(KnownObstacleDistance3D::Storage{
          .output_bounds = output,
          .source_bounds = geometry.source_bounds,
          .maximum_distance_m = geometry.maximum_distance_m,
          .source_fingerprint = sources.fingerprint,
          .source_voxels = sources.influencing_keys.size(),
          .finite_distance_voxels = finite_voxels,
          .distances_m = std::move(distances),
      }),
      .stats =
          KnownObstacleDistance3DBuildStats{
              .source_voxels = sources.influencing_keys.size(),
              .transform_voxels = transform_voxels,
              .finite_distance_voxels = finite_voxels,
              .source_collection_ms = sources.collection_ms,
              .transform_ms = transform_ms,
          },
      .mode = KnownObstacleDistance3DBuildMode::kFull,
  };
  result.stats.duration_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
  return result;
}

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

KnownObstacleDistance3D::KnownObstacleDistance3D(const ConstructionKey construction_key,
                                                 Storage storage)
    : storage_{std::move(storage)} {
  static_cast<void>(construction_key);
}

std::shared_ptr<const KnownObstacleDistance3D>
KnownObstacleDistance3D::create(Storage storage) {
  return std::make_shared<const KnownObstacleDistance3D>(ConstructionKey{},
                                                         std::move(storage));
}

bool KnownObstacleDistance3D::valid() const noexcept {
  return storage_.source_fingerprint != 0U && storage_.maximum_distance_m > 0.0 &&
         storage_.distances_m != nullptr &&
         storage_.distances_m->size() ==
             static_cast<std::size_t>(storage_.output_bounds.width_cells) *
                 static_cast<std::size_t>(storage_.output_bounds.height_cells) *
                 static_cast<std::size_t>(storage_.output_bounds.depth_cells);
}

const GridBounds3D& KnownObstacleDistance3D::bounds() const noexcept {
  return storage_.output_bounds;
}

const GridBounds3D& KnownObstacleDistance3D::sourceBounds() const noexcept {
  return storage_.source_bounds;
}

double KnownObstacleDistance3D::maximumDistanceM() const noexcept {
  return storage_.maximum_distance_m;
}

std::uint64_t KnownObstacleDistance3D::sourceFingerprint() const noexcept {
  return storage_.source_fingerprint;
}

std::size_t KnownObstacleDistance3D::sourceVoxelCount() const noexcept {
  return storage_.source_voxels;
}

std::size_t KnownObstacleDistance3D::finiteDistanceVoxelCount() const noexcept {
  return storage_.finite_distance_voxels;
}

float KnownObstacleDistance3D::distanceAt(const GridIndex3D local_cell) const noexcept {
  const GridBounds3D& bounds = storage_.output_bounds;
  if (!valid() || local_cell.x < 0 || local_cell.y < 0 || local_cell.z < 0 ||
      local_cell.x >= bounds.width_cells || local_cell.y >= bounds.height_cells ||
      local_cell.z >= bounds.depth_cells) {
    return std::numeric_limits<float>::infinity();
  }
  return (*storage_.distances_m)[(static_cast<std::size_t>(local_cell.z) *
                                      static_cast<std::size_t>(bounds.height_cells) +
                                  static_cast<std::size_t>(local_cell.y)) *
                                     static_cast<std::size_t>(bounds.width_cells) +
                                 static_cast<std::size_t>(local_cell.x)];
}

const std::shared_ptr<const std::vector<float>>&
KnownObstacleDistance3D::denseDistances() const noexcept {
  return storage_.distances_m;
}

const char* knownObstacleDistance3DBuildModeName(
    const KnownObstacleDistance3DBuildMode mode) noexcept {
  switch (mode) {
    case KnownObstacleDistance3DBuildMode::kFull:
      return "full";
    case KnownObstacleDistance3DBuildMode::kReused:
      return "reused";
  }
  return "invalid";
}

KnownObstacleDistance3DBuildResult
buildKnownObstacleDistance3D(const ObservedOccupancyGrid3D& occupancy,
                             const GridBounds3D& local_bounds,
                             const double maximum_distance_m,
                             const std::span<const GridIndex3D> suppressed_source_cells,
                             BoundedWorkerPool* const worker_pool) {
  const auto started = std::chrono::steady_clock::now();
  const TransformGeometry3D geometry =
      makeGeometry(occupancy.bounds(), local_bounds, maximum_distance_m);
  return transformCollectedSources(
      geometry, collectSources(occupancy, geometry, suppressed_source_cells),
      worker_pool, started);
}

KnownObstacleDistance3DBuildResult updateKnownObstacleDistance3D(
    const ObservedOccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
    const double maximum_distance_m,
    const std::shared_ptr<const KnownObstacleDistance3D>& previous,
    const std::span<const GridIndex3D> suppressed_source_cells,
    BoundedWorkerPool* const worker_pool) {
  const auto started = std::chrono::steady_clock::now();
  const TransformGeometry3D geometry =
      makeGeometry(occupancy.bounds(), local_bounds, maximum_distance_m);
  CollectedSources3D sources =
      collectSources(occupancy, geometry, suppressed_source_cells);
  const bool previous_compatible =
      previous != nullptr && previous->valid() &&
      sameBounds(previous->bounds(), geometry.output_bounds) &&
      sameBounds(previous->sourceBounds(), geometry.source_bounds) &&
      std::abs(previous->maximumDistanceM() - maximum_distance_m) <=
          kGeometryTolerance &&
      previous->sourceFingerprint() == sources.fingerprint;
  if (previous_compatible) {
    KnownObstacleDistance3DBuildResult result{
        .field = previous,
        .stats =
            KnownObstacleDistance3DBuildStats{
                .source_voxels = previous->sourceVoxelCount(),
                .transform_voxels = 0U,
                .finite_distance_voxels = previous->finiteDistanceVoxelCount(),
                .source_collection_ms = sources.collection_ms,
                .transform_ms = 0.0,
            },
        .mode = KnownObstacleDistance3DBuildMode::kReused,
    };
    result.stats.duration_ms = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - started)
                                   .count();
    return result;
  }
  return transformCollectedSources(geometry, std::move(sources), worker_pool, started);
}

} // namespace drone_city_nav
