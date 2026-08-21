#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffsetBasis{14695981039346656037ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

void hashWord(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
    hash ^= (value >> shift) & 0xFFU;
    hash *= kFnvPrime;
  }
}

void hashInteger(std::uint64_t& hash, const int value) noexcept {
  hashWord(hash, static_cast<std::uint64_t>(static_cast<std::int64_t>(value)));
}

void hashBounds(std::uint64_t& hash, const GridBounds3D& bounds) noexcept {
  hashWord(hash, std::bit_cast<std::uint64_t>(bounds.origin_x));
  hashWord(hash, std::bit_cast<std::uint64_t>(bounds.origin_y));
  hashWord(hash, std::bit_cast<std::uint64_t>(bounds.origin_z));
  hashWord(hash, std::bit_cast<std::uint64_t>(bounds.resolution_m));
  hashInteger(hash, bounds.width_cells);
  hashInteger(hash, bounds.height_cells);
  hashInteger(hash, bounds.depth_cells);
}

[[nodiscard]] bool sameResolution(const GridBounds3D& first,
                                  const GridBounds3D& second) noexcept {
  return std::abs(first.resolution_m - second.resolution_m) <= 1.0e-9;
}

[[nodiscard]] int alignedCellOffset(const double local_origin,
                                    const double world_origin,
                                    const double resolution_m) {
  const double offset = (local_origin - world_origin) / resolution_m;
  const double rounded = std::round(offset);
  if (std::abs(offset - rounded) > 1.0e-6) {
    throw std::invalid_argument{"observed ESDF bounds are not cell aligned"};
  }
  return static_cast<int>(rounded);
}

struct SourceCellRegion {
  int minimum_x{0};
  int minimum_y{0};
  int minimum_z{0};
  int maximum_x_exclusive{0};
  int maximum_y_exclusive{0};
  int maximum_z_exclusive{0};
};

[[nodiscard]] SourceCellRegion sourceCellRegion(const GridBounds3D& world,
                                                const GridBounds3D& local) {
  if (!sameResolution(world, local) || local.width_cells <= 0 ||
      local.height_cells <= 0 || local.depth_cells <= 0) {
    throw std::invalid_argument{"invalid observed ESDF local bounds"};
  }
  const int minimum_x =
      alignedCellOffset(local.origin_x, world.origin_x, world.resolution_m);
  const int minimum_y =
      alignedCellOffset(local.origin_y, world.origin_y, world.resolution_m);
  const int minimum_z =
      alignedCellOffset(local.origin_z, world.origin_z, world.resolution_m);
  const SourceCellRegion region{
      .minimum_x = minimum_x,
      .minimum_y = minimum_y,
      .minimum_z = minimum_z,
      .maximum_x_exclusive = minimum_x + local.width_cells,
      .maximum_y_exclusive = minimum_y + local.height_cells,
      .maximum_z_exclusive = minimum_z + local.depth_cells,
  };
  if (region.minimum_x < 0 || region.minimum_y < 0 || region.minimum_z < 0 ||
      region.maximum_x_exclusive > world.width_cells ||
      region.maximum_y_exclusive > world.height_cells ||
      region.maximum_z_exclusive > world.depth_cells) {
    throw std::invalid_argument{"observed ESDF local bounds exceed world bounds"};
  }
  return region;
}

[[nodiscard]] bool overlaps(const OccupancyChunkIndex3D& chunk,
                            const SourceCellRegion& region) noexcept {
  const int chunk_minimum_x = chunk.x * ObservedOccupancyGrid3D::kChunkSize;
  const int chunk_minimum_y = chunk.y * ObservedOccupancyGrid3D::kChunkSize;
  const int chunk_minimum_z = chunk.z * ObservedOccupancyGrid3D::kChunkSize;
  const int chunk_maximum_x = chunk_minimum_x + ObservedOccupancyGrid3D::kChunkSize;
  const int chunk_maximum_y = chunk_minimum_y + ObservedOccupancyGrid3D::kChunkSize;
  const int chunk_maximum_z = chunk_minimum_z + ObservedOccupancyGrid3D::kChunkSize;
  return chunk_maximum_x > region.minimum_x &&
         chunk_minimum_x < region.maximum_x_exclusive &&
         chunk_maximum_y > region.minimum_y &&
         chunk_minimum_y < region.maximum_y_exclusive &&
         chunk_maximum_z > region.minimum_z &&
         chunk_minimum_z < region.maximum_z_exclusive;
}

[[nodiscard]] std::size_t localLinearIndex(const GridBounds3D& bounds,
                                           const GridIndex3D cell) noexcept {
  return (static_cast<std::size_t>(cell.z) *
              static_cast<std::size_t>(bounds.height_cells) +
          static_cast<std::size_t>(cell.y)) *
             static_cast<std::size_t>(bounds.width_cells) +
         static_cast<std::size_t>(cell.x);
}

template<typename Callback>
void forEachObservedVoxel(const ObservedOccupancyGrid3D& occupancy,
                          const SourceCellRegion& region, Callback&& callback) {
  for (const auto& [chunk_index, chunk] : occupancy.chunks()) {
    if (!overlaps(chunk_index, region)) {
      continue;
    }
    for (std::size_t word_index = 0U; word_index < chunk.observed.size();
         ++word_index) {
      std::uint64_t observed_bits = chunk.observed[word_index];
      while (observed_bits != 0U) {
        const int bit_offset = std::countr_zero(observed_bits);
        const std::size_t bit_index =
            word_index * 64U + static_cast<std::size_t>(bit_offset);
        const int local_x = static_cast<int>(
            bit_index % static_cast<std::size_t>(ObservedOccupancyGrid3D::kChunkSize));
        const int local_y =
            static_cast<int>((bit_index / ObservedOccupancyGrid3D::kChunkSize) %
                             ObservedOccupancyGrid3D::kChunkSize);
        const int local_z = static_cast<int>(
            bit_index / static_cast<std::size_t>(ObservedOccupancyGrid3D::kChunkSize *
                                                 ObservedOccupancyGrid3D::kChunkSize));
        const GridIndex3D source{
            chunk_index.x * ObservedOccupancyGrid3D::kChunkSize + local_x,
            chunk_index.y * ObservedOccupancyGrid3D::kChunkSize + local_y,
            chunk_index.z * ObservedOccupancyGrid3D::kChunkSize + local_z,
        };
        if (source.x >= region.minimum_x && source.x < region.maximum_x_exclusive &&
            source.y >= region.minimum_y && source.y < region.maximum_y_exclusive &&
            source.z >= region.minimum_z && source.z < region.maximum_z_exclusive) {
          const bool occupied =
              (chunk.occupied[word_index] &
               (std::uint64_t{1U} << static_cast<unsigned int>(bit_offset))) != 0U;
          callback(source, occupied ? ObservedVoxelState::kOccupied
                                    : ObservedVoxelState::kFree);
        }
        observed_bits &= observed_bits - 1U;
      }
    }
  }
}

[[nodiscard]] int clampedCell(const double coordinate, const double origin,
                              const double resolution_m,
                              const int cell_count) noexcept {
  return std::clamp(static_cast<int>(std::floor((coordinate - origin) / resolution_m)),
                    0, cell_count - 1);
}

[[nodiscard]] std::uint64_t cellKey(const GridBounds3D& bounds,
                                    const GridIndex3D cell) noexcept {
  return (static_cast<std::uint64_t>(cell.z) *
              static_cast<std::uint64_t>(bounds.height_cells) +
          static_cast<std::uint64_t>(cell.y)) *
             static_cast<std::uint64_t>(bounds.width_cells) +
         static_cast<std::uint64_t>(cell.x);
}

[[nodiscard]] std::vector<GridIndex3D>
launchSupportCells(const ObservedOccupancyGrid3D& occupancy,
                   const LaunchSupportContact3D* const launch_support_contact) {
  std::vector<GridIndex3D> result;
  if (launch_support_contact == nullptr) {
    return result;
  }
  result.reserve(launch_support_contact->contact_cells.size());
  for (const AxisAlignedBox3D& box : launch_support_contact->contact_cells) {
    const Point3 center{0.5 * (box.minimum.x + box.maximum.x),
                        0.5 * (box.minimum.y + box.maximum.y),
                        0.5 * (box.minimum.z + box.maximum.z)};
    const std::optional<GridIndex3D> cell = occupancy.worldToCell(center);
    if (cell) {
      result.push_back(*cell);
    }
  }
  return result;
}

struct LaunchSupportCellCandidate {
  GridIndex3D index{};
  AxisAlignedBox3D bounds{};
};

[[nodiscard]] bool launchSupportEnvelopeIntersectsCell(
    const ProprioceptiveFreeSpaceSeed3D& seed, const double resolution_m,
    const Point3& cell_minimum, const Point3& cell_maximum) noexcept {
  const double axis_norm =
      std::hypot(std::hypot(seed.body_axis.x, seed.body_axis.y), seed.body_axis.z);
  if (!(axis_norm > 1.0e-9) || !(resolution_m > 0.0)) {
    return false;
  }
  const FootprintBodyAxis axis{seed.body_axis.x / axis_norm,
                               seed.body_axis.y / axis_norm,
                               seed.body_axis.z / axis_norm};
  const double lower_extent_m = std::max(0.0, seed.footprint.lower_extent_m);
  const double maximum_axial_extent_m =
      std::max(lower_extent_m, std::max(0.0, seed.footprint.upper_extent_m));
  const Point3 contact_center{seed.position.x - lower_extent_m * axis.x,
                              seed.position.y - lower_extent_m * axis.y,
                              seed.position.z - lower_extent_m * axis.z};
  const SweptFootprintConfig contact_envelope{
      .radius_m =
          std::hypot(std::max(0.0, seed.footprint.radius_m), maximum_axial_extent_m) +
          resolution_m,
      .lower_extent_m = resolution_m,
      .upper_extent_m = resolution_m,
  };
  return footprintIntersectsAxisAlignedBox(contact_center, axis, contact_envelope,
                                           cell_minimum, cell_maximum);
}

[[nodiscard]] std::vector<LaunchSupportCellCandidate>
launchSupportCellCandidates(const GridBounds3D& bounds,
                            const ProprioceptiveFreeSpaceSeed3D& seed) {
  const double broad_extent_m = std::max(0.0, seed.footprint.radius_m) +
                                std::max(std::max(0.0, seed.footprint.lower_extent_m),
                                         std::max(0.0, seed.footprint.upper_extent_m)) +
                                bounds.resolution_m;
  const int minimum_x = clampedCell(seed.position.x - broad_extent_m, bounds.origin_x,
                                    bounds.resolution_m, bounds.width_cells);
  const int maximum_x = clampedCell(seed.position.x + broad_extent_m, bounds.origin_x,
                                    bounds.resolution_m, bounds.width_cells);
  const int minimum_y = clampedCell(seed.position.y - broad_extent_m, bounds.origin_y,
                                    bounds.resolution_m, bounds.height_cells);
  const int maximum_y = clampedCell(seed.position.y + broad_extent_m, bounds.origin_y,
                                    bounds.resolution_m, bounds.height_cells);
  const int minimum_z = clampedCell(seed.position.z - broad_extent_m, bounds.origin_z,
                                    bounds.resolution_m, bounds.depth_cells);
  const int maximum_z = clampedCell(seed.position.z + broad_extent_m, bounds.origin_z,
                                    bounds.resolution_m, bounds.depth_cells);

  std::vector<LaunchSupportCellCandidate> result;
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        const Point3 minimum{
            bounds.origin_x + static_cast<double>(x) * bounds.resolution_m,
            bounds.origin_y + static_cast<double>(y) * bounds.resolution_m,
            bounds.origin_z + static_cast<double>(z) * bounds.resolution_m};
        const Point3 maximum{minimum.x + bounds.resolution_m,
                             minimum.y + bounds.resolution_m,
                             minimum.z + bounds.resolution_m};
        if (launchSupportEnvelopeIntersectsCell(seed, bounds.resolution_m, minimum,
                                                maximum)) {
          result.push_back(LaunchSupportCellCandidate{
              .index = GridIndex3D{x, y, z},
              .bounds = AxisAlignedBox3D{.minimum = minimum, .maximum = maximum},
          });
        }
      }
    }
  }
  return result;
}

[[nodiscard]] LaunchSupportContact3D
makeLaunchSupportContact3D(const GridBounds3D& bounds,
                           const ProprioceptiveFreeSpaceSeed3D& seed,
                           const LaunchSupportEvidenceSource evidence_source,
                           const std::size_t occupied_evidence_cells,
                           const std::vector<LaunchSupportCellCandidate>& candidates) {
  LaunchSupportContact3D contact{
      .seed = seed,
      .contact_cells = {},
      .occupied_evidence_cells = occupied_evidence_cells,
      .evidence_source = evidence_source,
      .maximum_lateral_departure_m = bounds.resolution_m,
      .minimum_axial_departure_m = 0.0,
      .maximum_axial_settling_m = bounds.resolution_m,
  };
  contact.contact_cells.reserve(candidates.size());
  std::ranges::transform(
      candidates, std::back_inserter(contact.contact_cells),
      [](const LaunchSupportCellCandidate& candidate) { return candidate.bounds; });
  return contact;
}

} // namespace

GridBounds3D selectLocalObservedEsdfBounds(const GridBounds3D& world_bounds,
                                           const Point3& position,
                                           const double half_extent_m) {
  if (!(world_bounds.resolution_m > 0.0) || world_bounds.width_cells <= 0 ||
      world_bounds.height_cells <= 0 || world_bounds.depth_cells <= 0 ||
      !(half_extent_m > 0.0) || !std::isfinite(position.x) ||
      !std::isfinite(position.y)) {
    throw std::invalid_argument{"invalid local observed ESDF bounds request"};
  }
  const int minimum_x =
      clampedCell(position.x - half_extent_m, world_bounds.origin_x,
                  world_bounds.resolution_m, world_bounds.width_cells);
  const int maximum_x =
      clampedCell(position.x + half_extent_m, world_bounds.origin_x,
                  world_bounds.resolution_m, world_bounds.width_cells);
  const int minimum_y =
      clampedCell(position.y - half_extent_m, world_bounds.origin_y,
                  world_bounds.resolution_m, world_bounds.height_cells);
  const int maximum_y =
      clampedCell(position.y + half_extent_m, world_bounds.origin_y,
                  world_bounds.resolution_m, world_bounds.height_cells);
  return GridBounds3D{
      .origin_x = world_bounds.origin_x +
                  static_cast<double>(minimum_x) * world_bounds.resolution_m,
      .origin_y = world_bounds.origin_y +
                  static_cast<double>(minimum_y) * world_bounds.resolution_m,
      .origin_z = world_bounds.origin_z,
      .resolution_m = world_bounds.resolution_m,
      .width_cells = maximum_x - minimum_x + 1,
      .height_cells = maximum_y - minimum_y + 1,
      .depth_cells = world_bounds.depth_cells,
  };
}

bool localObservedEsdfNeedsRecenter(const GridBounds3D& local_bounds,
                                    const GridBounds3D& world_bounds,
                                    const Point3& position,
                                    const double recenter_margin_m) noexcept {
  if (!sameResolution(local_bounds, world_bounds) || !(recenter_margin_m >= 0.0) ||
      !std::isfinite(position.x) || !std::isfinite(position.y)) {
    return true;
  }
  const double local_maximum_x =
      local_bounds.origin_x + local_bounds.width_cells * local_bounds.resolution_m;
  const double local_maximum_y =
      local_bounds.origin_y + local_bounds.height_cells * local_bounds.resolution_m;
  const double world_maximum_x =
      world_bounds.origin_x + world_bounds.width_cells * world_bounds.resolution_m;
  const double world_maximum_y =
      world_bounds.origin_y + world_bounds.height_cells * world_bounds.resolution_m;
  const bool room_left = local_bounds.origin_x > world_bounds.origin_x + 1.0e-9;
  const bool room_right = local_maximum_x < world_maximum_x - 1.0e-9;
  const bool room_down = local_bounds.origin_y > world_bounds.origin_y + 1.0e-9;
  const bool room_up = local_maximum_y < world_maximum_y - 1.0e-9;
  return (room_left && position.x - local_bounds.origin_x < recenter_margin_m) ||
         (room_right && local_maximum_x - position.x < recenter_margin_m) ||
         (room_down && position.y - local_bounds.origin_y < recenter_margin_m) ||
         (room_up && local_maximum_y - position.y < recenter_margin_m);
}

std::uint64_t observedOccupancyFingerprint(const ObservedOccupancyGrid3D& occupancy,
                                           const GridBounds3D& local_bounds) {
  const SourceCellRegion region = sourceCellRegion(occupancy.bounds(), local_bounds);
  using ChunkEntry = std::pair<OccupancyChunkIndex3D, const ObservedOccupancyChunk3D*>;
  std::vector<ChunkEntry> chunks;
  chunks.reserve(occupancy.chunks().size());
  for (const auto& [index, chunk] : occupancy.chunks()) {
    if (overlaps(index, region)) {
      chunks.emplace_back(index, &chunk);
    }
  }
  std::ranges::sort(chunks, {}, [](const ChunkEntry& entry) {
    return std::tuple{entry.first.z, entry.first.y, entry.first.x};
  });

  std::uint64_t hash = kFnvOffsetBasis;
  hashBounds(hash, local_bounds);
  for (const auto& [index, chunk] : chunks) {
    hashInteger(hash, index.x);
    hashInteger(hash, index.y);
    hashInteger(hash, index.z);
    for (const std::uint64_t word : chunk->observed) {
      hashWord(hash, word);
    }
    for (const std::uint64_t word : chunk->occupied) {
      hashWord(hash, word);
    }
  }
  return hash;
}

std::optional<LaunchSupportContact3D>
detectLaunchSupportContact3D(const ObservedOccupancyGrid3D& occupancy,
                             const ProprioceptiveFreeSpaceSeed3D& seed) {
  const GridBounds3D& bounds = occupancy.bounds();
  const std::vector<LaunchSupportCellCandidate> candidates =
      launchSupportCellCandidates(bounds, seed);
  const std::size_t occupied_evidence_cells = static_cast<std::size_t>(
      std::ranges::count_if(candidates, [&](const LaunchSupportCellCandidate& cell) {
        return occupancy.state(cell.index) == ObservedVoxelState::kOccupied;
      }));
  if (occupied_evidence_cells == 0U) {
    return std::nullopt;
  }
  return makeLaunchSupportContact3D(bounds, seed,
                                    LaunchSupportEvidenceSource::kObservedOccupancy,
                                    occupied_evidence_cells, candidates);
}

LaunchSupportContact3D
makeVehicleLandedSupportContact3D(const GridBounds3D& bounds,
                                  const ProprioceptiveFreeSpaceSeed3D& seed) {
  const std::vector<LaunchSupportCellCandidate> candidates =
      launchSupportCellCandidates(bounds, seed);
  return makeLaunchSupportContact3D(
      bounds, seed, LaunchSupportEvidenceSource::kVehicleLandDetector, 0U, candidates);
}

LaunchSupportDeparture3D planLaunchSupportDeparture3D(
    const ObservedOccupancyGrid3D& occupancy, const Point3& current_position,
    const LaunchSupportContact3D& contact, const double minimum_departure_m) {
  LaunchSupportDeparture3D result;
  const GridBounds3D& bounds = occupancy.bounds();
  const FootprintBodyAxis& requested_axis = contact.seed.body_axis;
  const double axis_norm =
      std::hypot(std::hypot(requested_axis.x, requested_axis.y), requested_axis.z);
  if (!(bounds.resolution_m > 0.0) || !(axis_norm > 1.0e-9) ||
      !std::isfinite(minimum_departure_m) || minimum_departure_m < 0.0 ||
      contact.contact_cells.empty()) {
    return result;
  }
  const FootprintBodyAxis axis{requested_axis.x / axis_norm,
                               requested_axis.y / axis_norm,
                               requested_axis.z / axis_norm};
  double contact_maximum_axial_m{-std::numeric_limits<double>::infinity()};
  for (const AxisAlignedBox3D& cell : contact.contact_cells) {
    for (const double x : {cell.minimum.x, cell.maximum.x}) {
      for (const double y : {cell.minimum.y, cell.maximum.y}) {
        for (const double z : {cell.minimum.z, cell.maximum.z}) {
          const Point3 offset{x - contact.seed.position.x, y - contact.seed.position.y,
                              z - contact.seed.position.z};
          contact_maximum_axial_m =
              std::max(contact_maximum_axial_m,
                       offset.x * axis.x + offset.y * axis.y + offset.z * axis.z);
        }
      }
    }
  }
  const double required_axial_departure_m = std::max(
      minimum_departure_m, contact_maximum_axial_m +
                               std::max(0.0, contact.seed.footprint.lower_extent_m) +
                               bounds.resolution_m);
  const double maximum_search_departure_m =
      required_axial_departure_m +
      std::max({bounds.resolution_m * 4.0,
                std::max(0.0, contact.seed.footprint.lower_extent_m) +
                    std::max(0.0, contact.seed.footprint.upper_extent_m),
                std::max(0.0, contact.seed.footprint.radius_m)});
  const Point3 seed_offset{current_position.x - contact.seed.position.x,
                           current_position.y - contact.seed.position.y,
                           current_position.z - contact.seed.position.z};
  const double current_axial_departure_m =
      seed_offset.x * axis.x + seed_offset.y * axis.y + seed_offset.z * axis.z;
  const int first_step =
      std::max(1, static_cast<int>(std::ceil(
                      (required_axial_departure_m - current_axial_departure_m) /
                      bounds.resolution_m)));
  const int maximum_step =
      std::max(first_step, static_cast<int>(std::ceil((maximum_search_departure_m -
                                                       current_axial_departure_m) /
                                                      bounds.resolution_m)));
  for (int step = first_step; step <= maximum_step; ++step) {
    const double displacement_m = static_cast<double>(step) * bounds.resolution_m;
    const Point3 target{current_position.x + axis.x * displacement_m,
                        current_position.y + axis.y * displacement_m,
                        current_position.z + axis.z * displacement_m};
    const SweptFootprintResult target_validation = validateRawFootprintAt(
        occupancy, target, axis, contact.seed.footprint, nullptr, nullptr);
    if (!target_validation.accepted()) {
      result.validation = target_validation;
      continue;
    }
    const SweptFootprintResult path_validation =
        validateRawSweptFootprint(occupancy, current_position, axis, target, axis,
                                  contact.seed.footprint, &contact.seed, &contact);
    if (!path_validation.accepted()) {
      result.validation = path_validation;
      continue;
    }
    result.target = target;
    result.validation = path_validation;
    result.axial_departure_m = current_axial_departure_m + displacement_m;
    result.executable = true;
    return result;
  }
  return result;
}

ObservedEsdf3D
buildObservedEsdf3D(const ObservedOccupancyGrid3D& occupancy,
                    const GridBounds3D& local_bounds, const double maximum_distance_m,
                    BoundedWorkerPool* const worker_pool,
                    const ProprioceptiveFreeSpaceSeed3D* const free_space_seed,
                    const LaunchSupportContact3D* const launch_support_contact) {
  const SourceCellRegion source_region =
      sourceCellRegion(occupancy.bounds(), local_bounds);
  OccupancyGrid3D occupied = occupancy.occupiedSnapshot();
  const std::vector<GridIndex3D> support_cells =
      launchSupportCells(occupancy, launch_support_contact);
  std::unordered_set<std::uint64_t> support_cell_keys;
  support_cell_keys.reserve(support_cells.size());
  for (const GridIndex3D cell : support_cells) {
    occupied.clearOccupied(cell);
    support_cell_keys.insert(cellKey(occupancy.bounds(), cell));
  }
  const DistanceField3D field = DistanceField3D::buildLocal(
      occupied, local_bounds, maximum_distance_m, worker_pool);
  auto local_occupancy = std::make_shared<ObservedOccupancyGrid3D>(local_bounds);
  ObservedEsdf3D result{
      .grid =
          mppi::EsdfGrid{
              .width = local_bounds.width_cells,
              .height = local_bounds.height_cells,
              .resolution_m = static_cast<float>(local_bounds.resolution_m),
              .origin_x_m = static_cast<float>(local_bounds.origin_x),
              .origin_y_m = static_cast<float>(local_bounds.origin_y),
              .depth = local_bounds.depth_cells,
              .origin_z_m = static_cast<float>(local_bounds.origin_z),
              .outside_is_unknown = true,
          },
      .distances_m =
          std::vector<float>(field.distancesM().size(), mppi::kUnknownEsdfDistanceM),
      .local_occupancy = local_occupancy,
      .occupancy_fingerprint = observedOccupancyFingerprint(occupancy, local_bounds),
  };
  result.stats.distance_field = field.stats();

  const auto classification_started = std::chrono::steady_clock::now();
  const auto sourceToLocal = [&source_region](const GridIndex3D source) {
    return GridIndex3D{source.x - source_region.minimum_x,
                       source.y - source_region.minimum_y,
                       source.z - source_region.minimum_z};
  };
  const auto publishKnownDistance = [&](const GridIndex3D local_cell) {
    result.distances_m.at(localLinearIndex(local_bounds, local_cell)) =
        field.distanceAt(local_cell);
  };
  forEachObservedVoxel(
      occupancy, source_region,
      [&](const GridIndex3D source, const ObservedVoxelState state) {
        const GridIndex3D local_cell = sourceToLocal(source);
        const bool launch_support_cell =
            support_cell_keys.contains(cellKey(occupancy.bounds(), source));
        static_cast<void>(local_occupancy->setState(
            local_cell, launch_support_cell ? ObservedVoxelState::kFree : state));
        publishKnownDistance(local_cell);
        result.stats.launch_support_voxels +=
            launch_support_cell && state != ObservedVoxelState::kFree ? 1U : 0U;
      });

  for (const GridIndex3D source : support_cells) {
    if (source.x < source_region.minimum_x ||
        source.x >= source_region.maximum_x_exclusive ||
        source.y < source_region.minimum_y ||
        source.y >= source_region.maximum_y_exclusive ||
        source.z < source_region.minimum_z ||
        source.z >= source_region.maximum_z_exclusive) {
      continue;
    }
    const GridIndex3D local_cell = sourceToLocal(source);
    if (local_occupancy->state(local_cell) != ObservedVoxelState::kFree) {
      static_cast<void>(
          local_occupancy->setState(local_cell, ObservedVoxelState::kFree));
      publishKnownDistance(local_cell);
      ++result.stats.launch_support_voxels;
    }
  }

  if (free_space_seed != nullptr) {
    const double seed_extent_m =
        std::max(0.0, free_space_seed->footprint.radius_m) +
        std::max(std::max(0.0, free_space_seed->footprint.lower_extent_m),
                 std::max(0.0, free_space_seed->footprint.upper_extent_m)) +
        local_bounds.resolution_m;
    const int minimum_x =
        clampedCell(free_space_seed->position.x - seed_extent_m, local_bounds.origin_x,
                    local_bounds.resolution_m, local_bounds.width_cells);
    const int maximum_x =
        clampedCell(free_space_seed->position.x + seed_extent_m, local_bounds.origin_x,
                    local_bounds.resolution_m, local_bounds.width_cells);
    const int minimum_y =
        clampedCell(free_space_seed->position.y - seed_extent_m, local_bounds.origin_y,
                    local_bounds.resolution_m, local_bounds.height_cells);
    const int maximum_y =
        clampedCell(free_space_seed->position.y + seed_extent_m, local_bounds.origin_y,
                    local_bounds.resolution_m, local_bounds.height_cells);
    const int minimum_z =
        clampedCell(free_space_seed->position.z - seed_extent_m, local_bounds.origin_z,
                    local_bounds.resolution_m, local_bounds.depth_cells);
    const int maximum_z =
        clampedCell(free_space_seed->position.z + seed_extent_m, local_bounds.origin_z,
                    local_bounds.resolution_m, local_bounds.depth_cells);
    for (int z = minimum_z; z <= maximum_z; ++z) {
      for (int y = minimum_y; y <= maximum_y; ++y) {
        for (int x = minimum_x; x <= maximum_x; ++x) {
          const GridIndex3D local_cell{x, y, z};
          if (local_occupancy->state(local_cell) != ObservedVoxelState::kUnknown) {
            continue;
          }
          const Point3 cell_minimum{
              local_bounds.origin_x +
                  static_cast<double>(x) * local_bounds.resolution_m,
              local_bounds.origin_y +
                  static_cast<double>(y) * local_bounds.resolution_m,
              local_bounds.origin_z +
                  static_cast<double>(z) * local_bounds.resolution_m,
          };
          const Point3 cell_maximum{
              cell_minimum.x + local_bounds.resolution_m,
              cell_minimum.y + local_bounds.resolution_m,
              cell_minimum.z + local_bounds.resolution_m,
          };
          if (!footprintIntersectsAxisAlignedBox(
                  free_space_seed->position, free_space_seed->body_axis,
                  free_space_seed->footprint, cell_minimum, cell_maximum)) {
            continue;
          }
          static_cast<void>(
              local_occupancy->setState(local_cell, ObservedVoxelState::kFree));
          publishKnownDistance(local_cell);
          ++result.stats.proprioceptive_free_voxels;
        }
      }
    }
  }

  result.stats.known_voxels = local_occupancy->knownVoxelCount();
  result.stats.free_voxels = local_occupancy->freeVoxelCount();
  result.stats.occupied_voxels = local_occupancy->occupiedVoxelCount();
  result.stats.unknown_voxels = result.distances_m.size() - result.stats.known_voxels;
  result.stats.classification_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                classification_started)
          .count();
  return result;
}

} // namespace drone_city_nav
