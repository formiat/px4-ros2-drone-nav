#include "survey_mission.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <unordered_map>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
constexpr std::size_t kVoxelsPerChunk{
    static_cast<std::size_t>(kChunkSize * kChunkSize * kChunkSize)};
constexpr std::uint32_t kUnreached{std::numeric_limits<std::uint32_t>::max()};
constexpr std::array<std::array<int, 3>, 6> kFaces{
    {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
// The vehicle's own cell may hug a wall: the flood starts from the nearest
// cell the body fits in within this reach (the proof's rule).
constexpr double kVehicleReachM{1.0};
// How far from the standoff's point a vantage may stand: the open metre round
// it keeps it off the walls, so in a corridor the point lies in the wall.
constexpr double kSnapReachM{2.5};
// The flood's margin round the volume, and how far from a cluster's centroid
// its vantage may stand when none is at the standoff: a box of the lidar's
// reach twice over held 36 million cells and no reading was ever complete
// (r2802), and a vantage 35 m away looks at nothing.
constexpr double kVantageReachM{6.0};
// The radius of observed free space round a vantage.
constexpr double kVantageOpenM{1.0};
// The way through what was seen starts at the nearest observed free cell the
// body fits in: the vehicle's own cell is often unobserved (the lidar's
// minimum range; r2804 read one known cell outdoors).
constexpr double kKnownSeedReachM{3.0};
// A planner's route this many times the reading's plus the margin defers the frontier.
constexpr double kDeferralFactor{3.0};
constexpr double kDeferralMarginM{10.0};

// The flood's box: the volume with a margin of twice the reach round it,
// within the envelope's band of heights, as dense arrays (the body's fit
// looked up in the grid's chunks cost 45 hash lookups a cell; the box of a
// 50 x 45 m volume holds millions).
struct Lattice {
  const ObservedOccupancyGrid3D& grid;
  const SurveyConfig& config;
  GridIndex3D volume_low{};
  GridIndex3D volume_high{};
  GridIndex3D flood_low{};
  GridIndex3D flood_high{};
  int nx{0};
  int ny{0};
  int nz{0};
  // The cells the body does not fit in (an occupied cell within its radius, K16).
  std::vector<std::uint8_t> blocked;
  // The route's length in lattice steps of every reached cell, through the
  // cells the body fits in, unknown included (K16: the reachable component).
  std::vector<std::uint32_t> distance;
  // The cells observed free, and the route's length through those alone:
  // what ranks the frontiers (r2802, r2803: a frontier under the pad's floor
  // ranked first at 3 m through the floor's unobserved cells, and the
  // planner, which flies unknown as free too, hunted it round the building
  // for 600 s; the way through what was seen is the survey's measure of
  // nearness, the way through unknown stays open to the planner).
  std::vector<std::uint8_t> known;
  std::vector<std::uint32_t> known_distance;
  std::vector<std::array<int, 3>> open_ball;

  Lattice(const ObservedOccupancyGrid3D& grid_in, const SurveyConfig& config_in)
      : grid{grid_in},
        config{config_in} {
    const GridBounds3D& bounds = grid.bounds();
    const auto cell = [&bounds](const double x, const double y, const double z) {
      return GridIndex3D{
          static_cast<int>(std::floor((x - bounds.origin_x) / bounds.resolution_m)),
          static_cast<int>(std::floor((y - bounds.origin_y) / bounds.resolution_m)),
          static_cast<int>(std::floor((z - bounds.origin_z) / bounds.resolution_m))};
    };
    const auto clamp = [&bounds](const GridIndex3D index) {
      return GridIndex3D{std::clamp(index.x, 0, bounds.width_cells - 1),
                         std::clamp(index.y, 0, bounds.height_cells - 1),
                         std::clamp(index.z, 0, bounds.depth_cells - 1)};
    };
    const double epsilon = 1.0e-6;
    volume_low =
        clamp(cell(config.volume_minimum.x + epsilon, config.volume_minimum.y + epsilon,
                   config.volume_minimum.z + epsilon));
    volume_high =
        clamp(cell(config.volume_maximum.x - epsilon, config.volume_maximum.y - epsilon,
                   config.volume_maximum.z - epsilon));
    const double margin = kFloodMarginM;
    flood_low =
        clamp(cell(config.volume_minimum.x - margin, config.volume_minimum.y - margin,
                   std::max(config.volume_minimum.z - margin,
                            config.envelope.minimum_target_z_m)));
    flood_high =
        clamp(cell(config.volume_maximum.x + margin, config.volume_maximum.y + margin,
                   std::min(config.volume_maximum.z + margin,
                            config.envelope.maximum_target_z_m)));
    nx = flood_high.x - flood_low.x + 1;
    ny = flood_high.y - flood_low.y + 1;
    nz = flood_high.z - flood_low.z + 1;
    const std::size_t count = static_cast<std::size_t>(nx) *
                              static_cast<std::size_t>(ny) *
                              static_cast<std::size_t>(nz);
    blocked.assign(count, 0U);
    distance.assign(count, kUnreached);
    known.assign(count, 0U);
    known_distance.assign(count, kUnreached);
    // The body's ball round every occupied cell of the box or within the
    // radius of it: the box's cells it covers are blocked.
    const double fit_radius_voxels = config.body_radius_m / bounds.resolution_m;
    const int reach = static_cast<int>(std::ceil(fit_radius_voxels - 1.0e-9));
    const double limit = fit_radius_voxels * fit_radius_voxels + 1.0e-9;
    const double open_cells = kVantageOpenM / bounds.resolution_m;
    const int open_reach = static_cast<int>(std::ceil(open_cells - 1.0e-9));
    for (int z = -open_reach; z <= open_reach; ++z) {
      for (int y = -open_reach; y <= open_reach; ++y) {
        for (int x = -open_reach; x <= open_reach; ++x) {
          if (static_cast<double>(x * x + y * y + z * z) <=
              open_cells * open_cells + 1.0e-9) {
            open_ball.push_back({x, y, z});
          }
        }
      }
    }
    std::vector<std::array<int, 3>> ball;
    for (int z = -reach; z <= reach; ++z) {
      for (int y = -reach; y <= reach; ++y) {
        for (int x = -reach; x <= reach; ++x) {
          if (static_cast<double>(x * x + y * y + z * z) <= limit) {
            ball.push_back({x, y, z});
          }
        }
      }
    }
    for (const auto& [index, storage] : grid.chunks()) {
      const GridIndex3D low{index.x * kChunkSize, index.y * kChunkSize,
                            index.z * kChunkSize};
      if (low.x + kChunkSize - 1 < flood_low.x - reach ||
          low.x > flood_high.x + reach ||
          low.y + kChunkSize - 1 < flood_low.y - reach ||
          low.y > flood_high.y + reach ||
          low.z + kChunkSize - 1 < flood_low.z - reach ||
          low.z > flood_high.z + reach) {
        continue;
      }
      const ObservedOccupancyGrid3D::Chunk& chunk = storage.get();
      for (std::size_t bit = 0U; bit < kVoxelsPerChunk; ++bit) {
        const ObservedVoxelState state =
            ObservedOccupancyGrid3D::chunkState(chunk, bit);
        if (state == ObservedVoxelState::kUnknown) {
          continue;
        }
        const GridIndex3D occupied{
            low.x + static_cast<int>(bit % kChunkSize),
            low.y + static_cast<int>((bit / kChunkSize) % kChunkSize),
            low.z + static_cast<int>(bit / (kChunkSize * kChunkSize))};
        if (state == ObservedVoxelState::kFree) {
          if (inFlood(occupied)) {
            known[at(occupied)] = 1U;
          }
          continue;
        }
        for (const std::array<int, 3>& offset : ball) {
          const GridIndex3D near{occupied.x + offset[0], occupied.y + offset[1],
                                 occupied.z + offset[2]};
          if (inFlood(near)) {
            blocked[at(near)] = 1U;
          }
        }
      }
    }
  }

  [[nodiscard]] static bool within(const GridIndex3D cell, const GridIndex3D low,
                                   const GridIndex3D high) noexcept {
    return cell.x >= low.x && cell.x <= high.x && cell.y >= low.y && cell.y <= high.y &&
           cell.z >= low.z && cell.z <= high.z;
  }

  [[nodiscard]] bool inVolume(const GridIndex3D cell) const noexcept {
    return within(cell, volume_low, volume_high);
  }

  [[nodiscard]] bool inFlood(const GridIndex3D cell) const noexcept {
    return within(cell, flood_low, flood_high);
  }

  [[nodiscard]] std::size_t at(const GridIndex3D cell) const noexcept {
    return (static_cast<std::size_t>(cell.z - flood_low.z) *
                static_cast<std::size_t>(ny) +
            static_cast<std::size_t>(cell.y - flood_low.y)) *
               static_cast<std::size_t>(nx) +
           static_cast<std::size_t>(cell.x - flood_low.x);
  }

  [[nodiscard]] bool fitsBody(const GridIndex3D cell) const noexcept {
    return inFlood(cell) && blocked[at(cell)] == 0U;
  }

  [[nodiscard]] std::uint32_t reached(const GridIndex3D cell) const noexcept {
    return inFlood(cell) ? distance[at(cell)] : kUnreached;
  }

  [[nodiscard]] bool fitsKnown(const GridIndex3D cell) const noexcept {
    return fitsBody(cell) && known[at(cell)] != 0U;
  }

  // Every cell within the open radius observed free: where the vehicle holds
  // to look stands in what was seen, a metre from anything unknown or
  // occupied (r2807: a vantage 0.6 m over the floor, the descent to it ended
  // on the floor).
  [[nodiscard]] bool openAround(const GridIndex3D cell) const noexcept {
    for (const std::array<int, 3>& offset : open_ball) {
      const GridIndex3D near{cell.x + offset[0], cell.y + offset[1],
                             cell.z + offset[2]};
      if (!inFlood(near) || known[at(near)] == 0U) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] std::uint32_t reachedKnown(const GridIndex3D cell) const noexcept {
    return inFlood(cell) ? known_distance[at(cell)] : kUnreached;
  }
};

// The flood of K16 from the vehicle: the route's length of every reached
// cell; and the same through the observed free cells alone. A vehicle outside
// the box (it may route outside the volume) floods from the box's cell
// nearest it.
void flood(Lattice& lattice, const Point3& vehicle, SurveyReading& reading) {
  const std::optional<GridIndex3D> centre_cell = lattice.grid.worldToCell(vehicle);
  if (!centre_cell.has_value()) {
    return;
  }
  // A vehicle outside the box (it routes outside the volume) places no
  // reading: a flood from the box's face read as a pinch (r2806, five rises
  // at the east face). Its height is clamped into the band.
  if (centre_cell->x < lattice.flood_low.x || centre_cell->x > lattice.flood_high.x ||
      centre_cell->y < lattice.flood_low.y || centre_cell->y > lattice.flood_high.y) {
    return;
  }
  const GridIndex3D centre{
      centre_cell->x, centre_cell->y,
      std::clamp(centre_cell->z, lattice.flood_low.z, lattice.flood_high.z)};
  const double resolution = lattice.grid.bounds().resolution_m;
  const int reach = static_cast<int>(std::ceil(kVehicleReachM / resolution));
  std::optional<GridIndex3D> seed;
  double best = std::numeric_limits<double>::infinity();
  for (int z = -reach; z <= reach; ++z) {
    for (int y = -reach; y <= reach; ++y) {
      for (int x = -reach; x <= reach; ++x) {
        const GridIndex3D cell{centre.x + x, centre.y + y, centre.z + z};
        const double d = static_cast<double>(x * x + y * y + z * z);
        if (d < best && lattice.fitsBody(cell)) {
          best = d;
          seed = cell;
        }
      }
    }
  }
  if (!seed.has_value()) {
    return;
  }
  reading.vehicle_placed = true;
  reading.seed = seed;
  const auto run = [&lattice](const GridIndex3D origin,
                              std::vector<std::uint32_t>& distances,
                              const bool known_only) {
    std::vector<GridIndex3D> level{origin};
    std::vector<GridIndex3D> next;
    distances[lattice.at(origin)] = 0U;
    std::size_t visited = 1U;
    for (std::uint32_t depth = 1U; !level.empty(); ++depth) {
      next.clear();
      for (const GridIndex3D cell : level) {
        for (const std::array<int, 3>& face : kFaces) {
          const GridIndex3D neighbour{cell.x + face[0], cell.y + face[1],
                                      cell.z + face[2]};
          if (!(known_only ? lattice.fitsKnown(neighbour)
                           : lattice.fitsBody(neighbour)) ||
              distances[lattice.at(neighbour)] != kUnreached) {
            continue;
          }
          if (++visited > lattice.config.flood_voxel_budget) {
            return visited;
          }
          distances[lattice.at(neighbour)] = depth;
          next.push_back(neighbour);
        }
      }
      std::swap(level, next);
    }
    return visited;
  };
  reading.reachable_voxels = run(*seed, lattice.distance, false);
  reading.budget_exhausted =
      reading.reachable_voxels > lattice.config.flood_voxel_budget;
  if (reading.budget_exhausted) {
    return;
  }
  const int known_reach = static_cast<int>(std::ceil(kKnownSeedReachM / resolution));
  std::optional<GridIndex3D> known_seed;
  best = std::numeric_limits<double>::infinity();
  for (int z = -known_reach; z <= known_reach; ++z) {
    for (int y = -known_reach; y <= known_reach; ++y) {
      for (int x = -known_reach; x <= known_reach; ++x) {
        const GridIndex3D cell{centre.x + x, centre.y + y, centre.z + z};
        const double d = static_cast<double>(x * x + y * y + z * z);
        if (d < best && lattice.fitsKnown(cell) &&
            lattice.reached(cell) != kUnreached) {
          best = d;
          known_seed = cell;
        }
      }
    }
  }
  if (known_seed.has_value()) {
    reading.known_voxels = run(*known_seed, lattice.known_distance, true);
  }
}

// The frontier cells: observed free inside the volume, a face neighbour
// unknown inside the volume, not looked at, the neighbour not passed by the body.
[[nodiscard]] std::vector<GridIndex3D> frontierCells(const Lattice& lattice,
                                                     const SurveyCellSet& looked,
                                                     const SurveyCellSet& passed) {
  std::vector<GridIndex3D> cells;
  for (const auto& [index, storage] : lattice.grid.chunks()) {
    const GridIndex3D low{index.x * kChunkSize, index.y * kChunkSize,
                          index.z * kChunkSize};
    const GridIndex3D high{low.x + kChunkSize - 1, low.y + kChunkSize - 1,
                           low.z + kChunkSize - 1};
    if (high.x < lattice.volume_low.x || low.x > lattice.volume_high.x ||
        high.y < lattice.volume_low.y || low.y > lattice.volume_high.y ||
        high.z < lattice.volume_low.z || low.z > lattice.volume_high.z) {
      continue;
    }
    const ObservedOccupancyGrid3D::Chunk& chunk = storage.get();
    for (std::size_t bit = 0U; bit < kVoxelsPerChunk; ++bit) {
      if (ObservedOccupancyGrid3D::chunkState(chunk, bit) !=
          ObservedVoxelState::kFree) {
        continue;
      }
      const GridIndex3D cell{low.x + static_cast<int>(bit % kChunkSize),
                             low.y + static_cast<int>((bit / kChunkSize) % kChunkSize),
                             low.z + static_cast<int>(bit / (kChunkSize * kChunkSize))};
      if (!lattice.inVolume(cell) || looked.contains(cell)) {
        continue;
      }
      for (const std::array<int, 3>& face : kFaces) {
        const GridIndex3D neighbour{cell.x + face[0], cell.y + face[1],
                                    cell.z + face[2]};
        // The unknown beyond must hold the body (K16's rule): a hole a ray
        // left in a wall or a floor, or the unknown under the envelope's
        // floor, is no frontier (r2758: 20 of 22 clusters on wall holes;
        // r2807: clusters under the floor, a vantage 0.6 m over it, a
        // contact, 489 frontiers chosen and no end in 1500 s).
        if (lattice.inVolume(neighbour) &&
            lattice.grid.state(neighbour) == ObservedVoxelState::kUnknown &&
            lattice.fitsBody(neighbour) && !passed.contains(neighbour)) {
          cells.push_back(cell);
          break;
        }
      }
    }
  }
  return cells;
}

// The 26-connected clusters of the frontier cells.
[[nodiscard]] std::vector<std::vector<GridIndex3D>>
clustersOf(const std::vector<GridIndex3D>& cells) {
  std::unordered_map<GridIndex3D, std::size_t, GridIndex3DHash, GridIndex3DEqual> index;
  index.reserve(cells.size());
  for (std::size_t i = 0U; i < cells.size(); ++i) {
    index.emplace(cells[i], i);
  }
  std::vector<std::size_t> parent(cells.size());
  for (std::size_t i = 0U; i < parent.size(); ++i) {
    parent[i] = i;
  }
  const auto find = [&parent](std::size_t i) {
    while (parent[i] != i) {
      parent[i] = parent[parent[i]];
      i = parent[i];
    }
    return i;
  };
  for (std::size_t i = 0U; i < cells.size(); ++i) {
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const auto found = index.find(
              GridIndex3D{cells[i].x + dx, cells[i].y + dy, cells[i].z + dz});
          if (found != index.end()) {
            const std::size_t a = find(i);
            const std::size_t b = find(found->second);
            if (a != b) {
              parent[a] = b;
            }
          }
        }
      }
    }
  }
  std::unordered_map<std::size_t, std::vector<GridIndex3D>> groups;
  for (std::size_t i = 0U; i < cells.size(); ++i) {
    groups[find(i)].push_back(cells[i]);
  }
  std::vector<std::vector<GridIndex3D>> result;
  result.reserve(groups.size());
  for (auto& [root, members] : groups) {
    result.push_back(std::move(members));
  }
  return result;
}

// The cell nearest the point within the reach that is observed free, open
// round it and reached through what was seen.
[[nodiscard]] std::optional<GridIndex3D>
snap(const Lattice& lattice, const Point3& point, const double reach_m) {
  const std::optional<GridIndex3D> centre = lattice.grid.worldToCell(point);
  if (!centre.has_value()) {
    return std::nullopt;
  }
  const int reach =
      static_cast<int>(std::ceil(reach_m / lattice.grid.bounds().resolution_m));
  std::optional<GridIndex3D> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int z = -reach; z <= reach; ++z) {
    for (int y = -reach; y <= reach; ++y) {
      for (int x = -reach; x <= reach; ++x) {
        const GridIndex3D cell{centre->x + x, centre->y + y, centre->z + z};
        if (lattice.reachedKnown(cell) == kUnreached || !lattice.openAround(cell)) {
          continue;
        }
        const double d = squaredDistance(lattice.grid.cellCenter(cell), point);
        if (d < best_distance && d <= reach_m * reach_m) {
          best_distance = d;
          best = cell;
        }
      }
    }
  }
  return best;
}

} // namespace

SurveyReading readSurvey(const ObservedOccupancyGrid3D& grid, const Point3& vehicle,
                         const SurveyConfig& config, const SurveyCellSet& looked,
                         const SurveyCellSet& passed) {
  const auto started = std::chrono::steady_clock::now();
  SurveyReading reading;
  Lattice lattice{grid, config};
  const std::vector<GridIndex3D> cells = frontierCells(lattice, looked, passed);
  reading.frontier_cells = cells.size();
  reading.frontier_set.insert(cells.begin(), cells.end());
  flood(lattice, vehicle, reading);
  const double resolution = grid.bounds().resolution_m;
  for (std::vector<GridIndex3D>& members : clustersOf(cells)) {
    if (members.size() < config.cluster_minimum_cells) {
      continue;
    }
    ++reading.clusters;
    SurveyFrontier frontier;
    frontier.cells = members.size();
    Point3 sum{};
    for (const GridIndex3D cell : members) {
      const Point3 centre = grid.cellCenter(cell);
      sum.x += centre.x;
      sum.y += centre.y;
      sum.z += centre.z;
    }
    const double count = static_cast<double>(members.size());
    frontier.centroid = Point3{sum.x / count, sum.y / count, sum.z / count};
    // The vantage on the vehicle's side of the centroid, the far vantage beyond it.
    double dx = frontier.centroid.x - vehicle.x;
    double dy = frontier.centroid.y - vehicle.y;
    const double horizontal = std::hypot(dx, dy);
    if (horizontal < 0.5) {
      dx = 1.0;
      dy = 0.0;
    } else {
      dx /= horizontal;
      dy /= horizontal;
    }
    const double z = std::clamp(frontier.centroid.z, config.envelope.minimum_target_z_m,
                                config.envelope.maximum_target_z_m);
    const Point3 near{frontier.centroid.x - config.vantage_standoff_m * dx,
                      frontier.centroid.y - config.vantage_standoff_m * dy, z};
    const Point3 far{frontier.centroid.x + config.vantage_standoff_m * dx,
                     frontier.centroid.y + config.vantage_standoff_m * dy, z};
    // The vantage stands in observed space, open round it and reached through
    // what was seen: the vehicle looks from where the sensor has been, never
    // from a cell unknown lays open (r2802: a vantage under the pad's floor;
    // r2807: one 0.6 m over the floor). A cluster with none waits.
    std::optional<GridIndex3D> vantage = snap(lattice, near, kSnapReachM);
    if (!vantage.has_value()) {
      vantage = snap(lattice, frontier.centroid,
                     std::min(config.sensor_reach_m, kVantageReachM));
    }
    if (!vantage.has_value()) {
      ++reading.clusters_without_vantage;
      continue;
    }
    frontier.vantage = grid.cellCenter(*vantage);
    frontier.route_m = static_cast<double>(lattice.reachedKnown(*vantage)) * resolution;
    // The pass goes through observed space only (r2806: a far vantage in
    // unknown space behind the centroid sent the planner round the world).
    const std::optional<GridIndex3D> far_vantage = snap(lattice, far, kSnapReachM);
    frontier.far_vantage_known = far_vantage.has_value();
    frontier.far_vantage =
        frontier.far_vantage_known ? grid.cellCenter(*far_vantage) : frontier.vantage;
    reading.reachable_members.insert(members.begin(), members.end());
    frontier.members = std::move(members);
    reading.frontiers.push_back(std::move(frontier));
  }
  // Nearest by the way through what was seen.
  std::sort(reading.frontiers.begin(), reading.frontiers.end(),
            [](const SurveyFrontier& lhs, const SurveyFrontier& rhs) {
              return lhs.route_m != rhs.route_m ? lhs.route_m < rhs.route_m
                                                : lhs.cells > rhs.cells;
            });
  reading.reading_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - started)
                           .count();
  return reading;
}

std::string summary(const SurveyReading& reading) {
  char line[320];
  std::snprintf(line, sizeof line,
                "SURVEY_READING frontier_cells=%zu clusters=%zu with_vantage=%zu "
                "without_vantage=%zu reachable_voxels=%zu known_voxels=%zu "
                "vehicle_placed=%s budget_exhausted=%s nearest_route_m=%.1f "
                "reading_ms=%.0f",
                reading.frontier_cells, reading.clusters, reading.frontiers.size(),
                reading.clusters_without_vantage, reading.reachable_voxels,
                reading.known_voxels, reading.vehicle_placed ? "true" : "false",
                reading.budget_exhausted ? "true" : "false",
                reading.frontiers.empty() ? 0.0 : reading.frontiers.front().route_m,
                reading.reading_ms);
  return line;
}

SurveyMission::SurveyMission(const SurveyConfig& config)
    : config_{config} {
}

void SurveyMission::setGridBounds(const GridBounds3D& bounds) {
  bounds_ = bounds;
}

void SurveyMission::recordPose(const Point3& position, const double yaw_rad,
                               const bool heading_valid, const std::int64_t now_ns) {
  position_ = position;
  position_valid_ = true;
  if (target_.has_value() && !insideDomain(position)) {
    target_left_domain_ = true;
  }
  // The body's passage: the cells within its radius of a position a lattice
  // step from the last recorded one.
  if (bounds_.has_value() &&
      (!passed_any_ ||
       distance3D(position, last_passed_position_) >= bounds_->resolution_m)) {
    passed_any_ = true;
    last_passed_position_ = position;
    const double resolution = bounds_->resolution_m;
    const GridIndex3D centre{
        static_cast<int>(std::floor((position.x - bounds_->origin_x) / resolution)),
        static_cast<int>(std::floor((position.y - bounds_->origin_y) / resolution)),
        static_cast<int>(std::floor((position.z - bounds_->origin_z) / resolution))};
    const int reach = static_cast<int>(std::ceil(config_.body_radius_m / resolution));
    for (int z = -reach; z <= reach; ++z) {
      for (int y = -reach; y <= reach; ++y) {
        for (int x = -reach; x <= reach; ++x) {
          const GridIndex3D cell{centre.x + x, centre.y + y, centre.z + z};
          const Point3 centre_m{bounds_->origin_x + (cell.x + 0.5) * resolution,
                                bounds_->origin_y + (cell.y + 0.5) * resolution,
                                bounds_->origin_z + (cell.z + 0.5) * resolution};
          if (distance3D(centre_m, position) <= config_.body_radius_m) {
            passed_.insert(cell);
          }
        }
      }
    }
  }
  // The look: the sensor's field holds the target's centroid from within
  // the standoff plus a metre, for the dwell.
  if (!target_.has_value() || target_looked_ || !heading_valid) {
    look_since_ns_ = 0;
    return;
  }
  const double dx = target_->centroid.x - position.x;
  const double dy = target_->centroid.y - position.y;
  const double dz = target_->centroid.z - position.z;
  const double horizontal = std::hypot(dx, dy);
  const double range = std::hypot(horizontal, dz);
  double bearing = std::atan2(dy, dx) - yaw_rad;
  bearing = std::remainder(bearing, 2.0 * std::numbers::pi);
  const double elevation = std::atan2(dz, horizontal);
  const bool in_field =
      range <= config_.vantage_standoff_m + config_.capture_radius_m &&
      std::abs(bearing) <= 0.5 * config_.sensor_horizontal_fov_rad &&
      std::abs(elevation) <= 0.5 * config_.sensor_vertical_fov_rad;
  if (!in_field) {
    look_since_ns_ = 0;
    return;
  }
  if (look_since_ns_ == 0) {
    look_since_ns_ = now_ns;
  } else if (static_cast<double>(now_ns - look_since_ns_) * 1.0e-9 >=
             config_.look_dwell_s) {
    target_looked_ = true;
  }
}

void SurveyMission::recordRoute(const bool available, const double remaining_m,
                                const std::int64_t now_ns) {
  if (!target_.has_value()) {
    route_unavailable_since_ns_ = 0;
    return;
  }
  if (available) {
    route_unavailable_since_ns_ = 0;
    // The planner's own route (specification F20, the value by the
    // planner's cost): a way far longer than the reading's means the
    // planner's clearance closes what the body's fit leaves open (r2805:
    // 153 m against 27.8 m); the frontier is deferred behind a nearer one,
    // and stays a frontier.
    target_planner_route_m_ = remaining_m;
    return;
  }
  if (route_unavailable_since_ns_ == 0) {
    route_unavailable_since_ns_ = now_ns;
  } else if (static_cast<double>(now_ns - route_unavailable_since_ns_) * 1.0e-9 >=
             config_.unroutable_s) {
    target_unroutable_ = true;
  }
}

void SurveyMission::applyReading(SurveyReading reading) {
  if (reading.vehicle_placed && !reading.budget_exhausted) {
    if (reading.reachable_voxels * kPinchedShare < largest_component_) {
      reading.pinched = true;
    } else {
      largest_component_ = std::max(largest_component_, reading.reachable_voxels);
    }
  }
  reading_ = std::move(reading);
  ++readings_;
}

bool SurveyMission::insideDomain(const Point3& point) const noexcept {
  return point.x >= config_.volume_minimum.x - kFloodMarginM &&
         point.x <= config_.volume_maximum.x + kFloodMarginM &&
         point.y >= config_.volume_minimum.y - kFloodMarginM &&
         point.y <= config_.volume_maximum.y + kFloodMarginM;
}

bool SurveyMission::reached(const Point3& point) const noexcept {
  return position_valid_ && distance3D(position_, point) <= config_.capture_radius_m;
}

void SurveyMission::finishTarget(const char* outcome, SurveyStep& out) {
  char line[256];
  std::snprintf(
      line, sizeof line,
      "SURVEY_FRONTIER %s centroid=(%.2f,%.2f,%.2f) cells=%zu chosen=%zu "
      "observed=%zu looked_at=%zu unreachable=%zu unroutable=%zu left_domain=%zu "
      "passes=%zu",
      outcome, target_->centroid.x, target_->centroid.y, target_->centroid.z,
      target_->cells, chosen_, observed_, looked_count_, unreachable_, unroutable_,
      left_domain_, passes_);
  out.events.emplace_back(line);
  target_.reset();
  target_looked_ = false;
  target_unroutable_ = false;
  target_left_domain_ = false;
  look_since_ns_ = 0;
  route_unavailable_since_ns_ = 0;
  target_planner_route_m_ = 0.0;
  phase_ = Phase::kApproach;
}

SurveyStep SurveyMission::step(const std::int64_t now_ns) {
  SurveyStep out;
  if (done_) {
    if (!home_sent_) {
      home_sent_ = true;
      out.command = SurveyCommand::kHome;
    }
    return out;
  }
  if (!reading_.has_value()) {
    return out;
  }
  if (reading_->pinched && position_valid_ &&
      (rise_sent_ns_ == 0 ||
       static_cast<double>(now_ns - rise_sent_ns_) * 1.0e-9 >= kRiseRepeatS)) {
    // Out of the pinch before anything is decided: a goal straight up.
    rise_sent_ns_ = now_ns;
    ++rises_;
    out.command = SurveyCommand::kGoal;
    out.goal =
        Point3{position_.x, position_.y,
               std::min(position_.z + kRiseM, config_.envelope.maximum_target_z_m)};
    char line[192];
    std::snprintf(line, sizeof line,
                  "SURVEY_RISE reachable_voxels=%zu largest_component=%zu "
                  "goal=(%.2f,%.2f,%.2f) rises=%zu",
                  reading_->reachable_voxels, largest_component_, out.goal.x,
                  out.goal.y, out.goal.z, rises_);
    out.events.emplace_back(line);
    return out;
  }
  if (!reading_->pinched) {
    rise_sent_ns_ = 0;
  }
  if (target_.has_value()) {
    std::size_t remaining = 0U;
    bool reachable = false;
    for (const GridIndex3D cell : target_->members) {
      if (reading_->frontier_set.contains(cell)) {
        ++remaining;
        reachable = reachable || reading_->reachable_members.contains(cell);
      }
    }
    if (remaining * 10U < target_->cells) {
      ++observed_;
      finishTarget("observed", out);
    } else if (target_looked_) {
      looked_.insert(target_->members.begin(), target_->members.end());
      ++looked_count_;
      finishTarget(phase_ == Phase::kPass ? "looked_at_on_pass" : "looked_at", out);
    } else if (target_planner_route_m_ >
                   kDeferralFactor * target_->route_m + kDeferralMarginM &&
               phase_ == Phase::kApproach) {
      const double planner_route_m = target_planner_route_m_;
      const auto nearer =
          std::find_if(reading_->frontiers.begin(), reading_->frontiers.end(),
                       [this, planner_route_m](const SurveyFrontier& candidate) {
                         return candidate.route_m < planner_route_m &&
                                distance3D(candidate.centroid, target_->centroid) > 1.0;
                       });
      if (nearer != reading_->frontiers.end()) {
        char line[192];
        std::snprintf(
            line, sizeof line,
            "SURVEY_FRONTIER deferred centroid=(%.2f,%.2f,%.2f) reading_route_m=%.1f "
            "planner_route_m=%.1f nearer_route_m=%.1f",
            target_->centroid.x, target_->centroid.y, target_->centroid.z,
            target_->route_m, planner_route_m, nearer->route_m);
        out.events.emplace_back(line);
        ++deferred_;
        target_ = *nearer;
        phase_ = Phase::kApproach;
        target_looked_ = false;
        target_unroutable_ = false;
        target_left_domain_ = false;
        look_since_ns_ = 0;
        route_unavailable_since_ns_ = 0;
        target_planner_route_m_ = 0.0;
        out.command = SurveyCommand::kGoal;
        out.goal = target_->vantage;
      } else {
        target_planner_route_m_ = 0.0;
      }
    } else if (target_left_domain_) {
      // The planner's way to the vantage led out of the flood's box: by the
      // planner's measure the cluster lies outside the reachable component
      // the reading drew (r2806: 60 m east of the volume, then 500 s).
      looked_.insert(target_->members.begin(), target_->members.end());
      ++left_domain_;
      finishTarget("left_domain", out);
    } else if (target_unroutable_) {
      // By the planner's map the vantage is enclosed: the cluster lies
      // outside the reachable component, which is a measurement, not a try.
      looked_.insert(target_->members.begin(), target_->members.end());
      ++unroutable_;
      finishTarget("unroutable", out);
    } else if (phase_ == Phase::kApproach && reached(target_->vantage)) {
      phase_ = Phase::kLook;
      reached_ns_ = now_ns;
    } else if (phase_ == Phase::kLook &&
               static_cast<double>(now_ns - reached_ns_) * 1.0e-9 >=
                   config_.look_wait_s &&
               !target_->far_vantage_known) {
      // No observed cell beyond the centroid to pass through: the sensor
      // stood at the vantage for the wait, which is the look it can give.
      looked_.insert(target_->members.begin(), target_->members.end());
      ++looked_count_;
      finishTarget("looked_at_from_vantage", out);
    } else if (phase_ == Phase::kLook &&
               static_cast<double>(now_ns - reached_ns_) * 1.0e-9 >=
                   config_.look_wait_s) {
      phase_ = Phase::kPass;
      ++passes_;
      out.command = SurveyCommand::kGoal;
      out.goal = target_->far_vantage;
      char line[192];
      std::snprintf(
          line, sizeof line,
          "SURVEY_FRONTIER pass centroid=(%.2f,%.2f,%.2f) far_vantage=(%.2f,%.2f,%.2f)",
          target_->centroid.x, target_->centroid.y, target_->centroid.z,
          target_->far_vantage.x, target_->far_vantage.y, target_->far_vantage.z);
      out.events.emplace_back(line);
    } else if (phase_ == Phase::kPass && reached(target_->far_vantage)) {
      // The pass carried the sensor past the centroid: looked at.
      looked_.insert(target_->members.begin(), target_->members.end());
      ++looked_count_;
      finishTarget("looked_at_on_pass", out);
    } else if (!reachable && reading_->complete()) {
      ++unreachable_;
      finishTarget("unreachable", out);
    }
  }
  if (!target_.has_value()) {
    if (!reading_->frontiers.empty()) {
      target_ = reading_->frontiers.front();
      phase_ = Phase::kApproach;
      target_looked_ = false;
      target_unroutable_ = false;
      target_left_domain_ = false;
      look_since_ns_ = 0;
      route_unavailable_since_ns_ = 0;
      target_planner_route_m_ = 0.0;
      ++chosen_;
      out.command = SurveyCommand::kGoal;
      out.goal = target_->vantage;
      char line[256];
      std::snprintf(
          line, sizeof line,
          "SURVEY_FRONTIER chosen centroid=(%.2f,%.2f,%.2f) vantage=(%.2f,%.2f,%.2f) "
          "cells=%zu route_m=%.1f candidates=%zu",
          target_->centroid.x, target_->centroid.y, target_->centroid.z,
          target_->vantage.x, target_->vantage.y, target_->vantage.z, target_->cells,
          target_->route_m, reading_->frontiers.size());
      out.events.emplace_back(line);
    } else if (reading_->complete()) {
      done_ = true;
      home_sent_ = true;
      out.command = SurveyCommand::kHome;
      char line[256];
      std::snprintf(
          line, sizeof line,
          "SURVEY_DONE frontier_cells=%zu clusters_without_vantage=%zu chosen=%zu "
          "observed=%zu looked_at=%zu unreachable=%zu unroutable=%zu passes=%zu "
          "readings=%zu",
          reading_->frontier_cells, reading_->clusters_without_vantage, chosen_,
          observed_, looked_count_, unreachable_, unroutable_, passes_, readings_);
      out.events.emplace_back(line);
    }
  }
  return out;
}

} // namespace drone_city_nav
