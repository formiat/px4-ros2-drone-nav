// The survey's reading of the obstacle memory (roadmap item 34): the frontiers
// inside the volume, the flood of K16 and the way through what was seen, the
// clusters and their vantages. The mission that acts on a reading is
// survey_mission.cpp.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "survey_mission.hpp"

namespace drone_city_nav {
namespace {

constexpr int kChunkSize{ObservedOccupancyGrid3D::kChunkSize};
constexpr std::size_t kVoxelsPerChunk{
    static_cast<std::size_t>(kChunkSize * kChunkSize * kChunkSize)};
// The cells of a chunk's layer.
constexpr std::size_t kChunkArea{static_cast<std::size_t>(kChunkSize) *
                                 static_cast<std::size_t>(kChunkSize)};
constexpr std::uint32_t kUnreached{std::numeric_limits<std::uint32_t>::max()};
constexpr std::array<std::array<int, 3>, 6> kFaces{
    {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
// The vehicle's own cell may hug a wall: the flood starts from the nearest
// cell the body fits in within this reach (the proof's rule).
constexpr double kVehicleReachM{1.0};
// How far from the standoff's point a vantage may stand: the open cylinder
// keeps a vantage off the walls and 1.5 m over the floor, and the point of a
// frontier in a wall lies at the wall.
constexpr double kSnapReachM{3.5};
// The flood's margin round the volume, and how far from a cluster's centroid
// its vantage may stand when none is at the standoff: a box of the lidar's
// reach twice over held 36 million cells and no reading was ever complete
// (r2802), and a vantage 35 m away looks at nothing.
constexpr double kVantageReachM{6.0};
// The observed free space round a vantage, a cylinder: the body and a cell
// to the sides, so a corridor two metres wide holds one (a ball of 1.5 m did
// not, and its frontiers waited unseen: 0.76 of the reachable volume covered,
// r2810); the body and the lidar-inertial estimate's height error below, a
// vehicle that hovers near a surface inside the scan's minimum range sinks
// onto it (r2808: the estimate 1.0 m over the truth a metre above the
// start's pad; r2807: a vantage 0.6 m over the floor).
constexpr double kVantageOpenSideM{0.8};
constexpr double kVantageOpenBelowM{1.5};
constexpr double kVantageOpenAboveM{0.8};
// The footprint of a look: the field that looked at a cell looked at the
// space round it; the boundary of what was seen creeps a cell a scan where
// rays find no return, and the new cells within the footprint are the same
// place looked at (r2809: places revisited up to six times, clusters of 19
// cells at the median, the volume covered in the first minute).
constexpr double kLookFootprintM{1.0};
// The way through what was seen starts at the nearest observed free cell the
// body fits in: the vehicle's own cell is often unobserved (the lidar's
// minimum range; r2804 read one known cell outdoors).
constexpr double kKnownSeedReachM{3.0};

// The flood's box: the volume with a margin of twice the reach round it,
// within the envelope's band of heights, as dense arrays (the body's fit
// looked up in the grid's chunks cost 45 hash lookups a cell; the box of a
// 50 x 45 m volume holds millions).
struct Lattice {
  const ObservedOccupancyGrid3D* grid;
  const SurveyConfig* config;
  GridIndex3D volume_low{};
  GridIndex3D volume_high{};
  GridIndex3D flood_low{};
  GridIndex3D flood_high{};
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
  // The cells of the body's ball: what a piece of unknown holds at least when
  // the body is in it.
  std::size_t body_cells{0U};
  // Within the look's footprint of a cell looked at.
  std::vector<std::uint8_t> looked_near;

  void markLooked(const SurveyCellSet& looked) {
    looked_near.assign(blocked.size(), 0U);
    const double cells = kLookFootprintM / grid->bounds().resolution_m;
    const int reach = static_cast<int>(std::ceil(cells - 1.0e-9));
    std::vector<std::array<int, 3>> footprint;
    for (int z = -reach; z <= reach; ++z) {
      for (int y = -reach; y <= reach; ++y) {
        for (int x = -reach; x <= reach; ++x) {
          if (static_cast<double>(x * x + y * y + z * z) <= cells * cells + 1.0e-9) {
            footprint.push_back({x, y, z});
          }
        }
      }
    }
    for (const GridIndex3D cell : looked) {
      for (const std::array<int, 3>& offset : footprint) {
        const GridIndex3D near{cell.x + offset[0], cell.y + offset[1],
                               cell.z + offset[2]};
        if (inFlood(near)) {
          looked_near[at(near)] = 1U;
        }
      }
    }
  }

  [[nodiscard]] bool lookedAt(const GridIndex3D cell,
                              const SurveyCellSet& looked) const {
    return inFlood(cell) ? looked_near[at(cell)] != 0U : looked.contains(cell);
  }

  Lattice(const ObservedOccupancyGrid3D& grid_in, const SurveyConfig& config_in)
      : grid{&grid_in},
        config{&config_in} {
    const GridBounds3D& bounds = grid->bounds();
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
    volume_low = clamp(cell(config->volume_minimum.x + epsilon,
                            config->volume_minimum.y + epsilon,
                            config->volume_minimum.z + epsilon));
    volume_high = clamp(cell(config->volume_maximum.x - epsilon,
                             config->volume_maximum.y - epsilon,
                             config->volume_maximum.z - epsilon));
    const double margin = kFloodMarginM;
    flood_low =
        clamp(cell(config->volume_minimum.x - margin, config->volume_minimum.y - margin,
                   std::max(config->volume_minimum.z - margin,
                            config->envelope.minimum_target_z_m)));
    flood_high =
        clamp(cell(config->volume_maximum.x + margin, config->volume_maximum.y + margin,
                   std::min(config->volume_maximum.z + margin,
                            config->envelope.maximum_target_z_m)));
    const std::size_t count = extent(0) * extent(1) * extent(2);
    blocked.assign(count, 0U);
    distance.assign(count, kUnreached);
    known.assign(count, 0U);
    known_distance.assign(count, kUnreached);
    // The body's ball round every occupied cell of the box or within the
    // radius of it: the box's cells it covers are blocked.
    const double fit_radius_voxels = config->body_radius_m / bounds.resolution_m;
    const int reach = static_cast<int>(std::ceil(fit_radius_voxels - 1.0e-9));
    const double limit = fit_radius_voxels * fit_radius_voxels + 1.0e-9;
    const double side_cells = kVantageOpenSideM / bounds.resolution_m;
    const int side = static_cast<int>(std::ceil(side_cells - 1.0e-9));
    const int below =
        static_cast<int>(std::ceil(kVantageOpenBelowM / bounds.resolution_m - 1.0e-9));
    const int above =
        static_cast<int>(std::ceil(kVantageOpenAboveM / bounds.resolution_m - 1.0e-9));
    for (int z = -below; z <= above; ++z) {
      for (int y = -side; y <= side; ++y) {
        for (int x = -side; x <= side; ++x) {
          if (static_cast<double>(x * x + y * y) <= side_cells * side_cells + 1.0e-9) {
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
    body_cells = ball.size();
    for (const auto& [index, storage] : grid->chunks()) {
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
            low.z + static_cast<int>(bit / kChunkArea)};
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

  // The squared distance, in cells, from a cell to the volume's box: zero inside.
  [[nodiscard]] int volumeDistance2(const GridIndex3D cell) const noexcept {
    const auto axis = [](const int value, const int low, const int high) {
      if (value < low) {
        return low - value;
      }
      return value > high ? value - high : 0;
    };
    const int dx = axis(cell.x, volume_low.x, volume_high.x);
    const int dy = axis(cell.y, volume_low.y, volume_high.y);
    const int dz = axis(cell.z, volume_low.z, volume_high.z);
    return dx * dx + dy * dy + dz * dz;
  }

  // Whether the unknown beyond a free cell leads into the volume: inside it,
  // from one cell of the volume to another (the volume's face is not a
  // frontier: the volume is the mission's order); outside it, to a cell no
  // farther from the volume than the free cell and within the sensor's reach
  // of the volume, so that a way round the outside, which runs along the
  // volume before it turns in, is followed where the sensor can look into the
  // volume from. The register's row of 2026-11-04: the parts of the volume
  // reached only round the outside got no frontier while frontiers were read
  // inside it alone, 0.763 of the reachable truth covered. Measured: with
  // this rule the lidar set covered the volume whole (r3151, r3152); unknown
  // strictly nearer the volume alone (an entrance seen from outside) found
  // nothing new (r3157: 0.763, the street by the door is never observed from
  // inside); the box 40 m wider than the volume without the reach sent the
  // stereo set round the city (r3149). The stereo set now covers the volume
  // in 800 to 1800 s of simulation (r3155: 0.83 at 793 s, the clusters left
  // inside the volume).
  [[nodiscard]] bool leadsIntoVolume(const GridIndex3D cell,
                                     const GridIndex3D beyond) const noexcept {
    const int from = volumeDistance2(cell);
    const int to = volumeDistance2(beyond);
    if (from == 0) {
      return to == 0;
    }
    const double reach_cells = config->sensor_reach_m / grid->bounds().resolution_m;
    return to <= from && static_cast<double>(to) <= reach_cells * reach_cells + 1.0e-9;
  }

  [[nodiscard]] bool inFlood(const GridIndex3D cell) const noexcept {
    return within(cell, flood_low, flood_high);
  }

  // Cells along an axis of the flood's box.
  [[nodiscard]] std::size_t extent(const int axis) const noexcept {
    const std::array<int, 3> cells{flood_high.x - flood_low.x,
                                   flood_high.y - flood_low.y,
                                   flood_high.z - flood_low.z};
    return static_cast<std::size_t>(cells.at(static_cast<std::size_t>(axis))) + 1U;
  }

  [[nodiscard]] std::size_t at(const GridIndex3D cell) const noexcept {
    return (static_cast<std::size_t>(cell.z - flood_low.z) * extent(1) +
            static_cast<std::size_t>(cell.y - flood_low.y)) *
               extent(0) +
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

  // Every cell of the open cylinder observed free: where the vehicle holds
  // to look stands in what was seen, clear of anything unknown or occupied
  // (r2807: a vantage 0.6 m over the floor, the descent to it ended on the
  // floor).
  [[nodiscard]] bool openAround(const GridIndex3D cell) const noexcept {
    return std::ranges::all_of(
        open_ball, [this, cell](const std::array<int, 3>& offset) {
          const GridIndex3D near{cell.x + offset[0], cell.y + offset[1],
                                 cell.z + offset[2]};
          return inFlood(near) && known[at(near)] != 0U;
        });
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
  const std::optional<GridIndex3D> centre_cell = lattice.grid->worldToCell(vehicle);
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
  const double resolution = lattice.grid->bounds().resolution_m;
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
          if (++visited > lattice.config->flood_voxel_budget) {
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
      reading.reachable_voxels > lattice.config->flood_voxel_budget;
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

// Whether an unknown cell lies in a pocket: a face-connected piece of unknown
// in the flood's box with fewer cells than the body's ball. The moving layer
// returns a voxel to unknown when its hit is not seen again (K65), and the
// pair's stray returns in open air left unknown voxels by ones and twos all
// through a hall already seen: each a cluster of ten frontier cells, 616 of
// them chosen in 2100 s at a median of eleven cells with the volume covered
// since the twelve hundredth, and no end (r3225). A pocket holds nothing the
// body could be in unseen. The verdict of every cell visited is kept.
[[nodiscard]] bool inPocket(const Lattice& lattice, const GridIndex3D unknown,
                            std::unordered_map<GridIndex3D, bool, GridIndex3DHash,
                                               GridIndex3DEqual>& verdicts) {
  if (const auto known = verdicts.find(unknown); known != verdicts.end()) {
    return known->second;
  }
  std::vector<GridIndex3D> piece{unknown};
  std::unordered_set<GridIndex3D, GridIndex3DHash, GridIndex3DEqual> seen{unknown};
  bool pocket{true};
  for (std::size_t next = 0U; next < piece.size() && pocket; ++next) {
    for (const std::array<int, 3>& face : kFaces) {
      const GridIndex3D neighbour{piece[next].x + face[0], piece[next].y + face[1],
                                  piece[next].z + face[2]};
      if (!lattice.inFlood(neighbour) ||
          lattice.grid->state(neighbour) != ObservedVoxelState::kUnknown ||
          !seen.insert(neighbour).second) {
        continue;
      }
      // A piece that joins one already judged is that piece.
      const auto known = verdicts.find(neighbour);
      piece.push_back(neighbour);
      if ((known != verdicts.end() && !known->second) ||
          piece.size() >= lattice.body_cells) {
        pocket = false;
        break;
      }
    }
  }
  for (const GridIndex3D cell : piece) {
    verdicts[cell] = pocket;
  }
  return pocket;
}

// The frontier cells: observed free in the flood's box, a face neighbour
// unknown in the box that leads into the volume and is no pocket, not looked
// at, the neighbour not passed by the body.
[[nodiscard]] std::vector<GridIndex3D> frontierCells(const Lattice& lattice,
                                                     const SurveyCellSet& looked,
                                                     const SurveyCellSet& passed) {
  std::vector<GridIndex3D> cells;
  std::unordered_map<GridIndex3D, bool, GridIndex3DHash, GridIndex3DEqual> pockets;
  for (const auto& [index, storage] : lattice.grid->chunks()) {
    const GridIndex3D low{index.x * kChunkSize, index.y * kChunkSize,
                          index.z * kChunkSize};
    const GridIndex3D high{low.x + kChunkSize - 1, low.y + kChunkSize - 1,
                           low.z + kChunkSize - 1};
    if (high.x < lattice.flood_low.x || low.x > lattice.flood_high.x ||
        high.y < lattice.flood_low.y || low.y > lattice.flood_high.y ||
        high.z < lattice.flood_low.z || low.z > lattice.flood_high.z) {
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
                             low.z + static_cast<int>(bit / kChunkArea)};
      if (!lattice.inFlood(cell) || lattice.lookedAt(cell, looked)) {
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
        if (lattice.inFlood(neighbour) && lattice.leadsIntoVolume(cell, neighbour) &&
            lattice.grid->state(neighbour) == ObservedVoxelState::kUnknown &&
            lattice.fitsBody(neighbour) && !passed.contains(neighbour) &&
            !inPocket(lattice, neighbour, pockets)) {
          cells.push_back(cell);
          break;
        }
      }
    }
  }
  return cells;
}

// The 26-connected clusters of the frontier cells. A cluster wider than the
// sensor's reach is not one place to look at: it is divided by the reach's
// cube on the grid's own lattice, each part with its own vantage (r3169: what
// the stereo set had left was one cluster of 32 thousand cells through the
// whole hall, the free cells over a floor it had not seen joined to the
// stairwell's; the one vantage by its centroid saw nothing of it and the
// survey ended at 0.78 of the volume, the lower level never looked into).
[[nodiscard]] std::vector<std::vector<GridIndex3D>>
clustersOf(const std::vector<GridIndex3D>& cells, const int reach_cells) {
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
    GridIndex3D low = members.front();
    GridIndex3D high = low;
    for (const GridIndex3D cell : members) {
      low = GridIndex3D{std::min(low.x, cell.x), std::min(low.y, cell.y),
                        std::min(low.z, cell.z)};
      high = GridIndex3D{std::max(high.x, cell.x), std::max(high.y, cell.y),
                         std::max(high.z, cell.z)};
    }
    if (std::max({high.x - low.x, high.y - low.y, high.z - low.z}) < reach_cells) {
      result.push_back(std::move(members));
      continue;
    }
    std::unordered_map<GridIndex3D, std::vector<GridIndex3D>, GridIndex3DHash,
                       GridIndex3DEqual>
        parts;
    for (const GridIndex3D cell : members) {
      parts[GridIndex3D{cell.x / reach_cells, cell.y / reach_cells,
                        cell.z / reach_cells}]
          .push_back(cell);
    }
    for (auto& [cube, part] : parts) {
      result.push_back(std::move(part));
    }
  }
  return result;
}

// The cell nearest the point within the reach that is observed free, open
// round it, reached through what was seen, and with a line of sight to the
// cluster's frontier cell nearest its centroid: a vantage looks at its frontier (r2818:
// a vantage in the hall over a frontier under its floor, the vehicle held over the
// stairwell until the lidar-inertial height ran 1.9 m off and it sat on the
// floor).
[[nodiscard]] std::optional<GridIndex3D> snap(const Lattice& lattice,
                                              const Point3& point, const double reach_m,
                                              const Point3& sight) {
  // The search starts at the grid's cell nearest the point: a far vantage
  // beyond a frontier at the grid's edge lies outside it.
  const GridBounds3D& bounds = lattice.grid->bounds();
  const auto index = [&bounds](const double value, const double origin,
                               const int cells) {
    return std::clamp(
        static_cast<int>(std::floor((value - origin) / bounds.resolution_m)), 0,
        cells - 1);
  };
  const GridIndex3D centre{index(point.x, bounds.origin_x, bounds.width_cells),
                           index(point.y, bounds.origin_y, bounds.height_cells),
                           index(point.z, bounds.origin_z, bounds.depth_cells)};
  const int reach =
      static_cast<int>(std::ceil(reach_m / lattice.grid->bounds().resolution_m));
  std::vector<std::pair<double, GridIndex3D>> candidates;
  for (int z = -reach; z <= reach; ++z) {
    for (int y = -reach; y <= reach; ++y) {
      for (int x = -reach; x <= reach; ++x) {
        const GridIndex3D cell{centre.x + x, centre.y + y, centre.z + z};
        if (lattice.reachedKnown(cell) == kUnreached || !lattice.openAround(cell)) {
          continue;
        }
        const double d = squaredDistance(lattice.grid->cellCenter(cell), point);
        if (d <= reach_m * reach_m) {
          candidates.emplace_back(d, cell);
        }
      }
    }
  }
  // The line of sight costs a ray: checked nearest first, until one sees.
  std::sort(candidates.begin(), candidates.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
  for (const auto& [distance, cell] : candidates) {
    if (lineOfSight(*lattice.grid, lattice.grid->cellCenter(cell), sight)) {
      return cell;
    }
  }
  return std::nullopt;
}

} // namespace

SurveyReading readSurvey(const ObservedOccupancyGrid3D& grid, const Point3& vehicle,
                         const SurveyConfig& config, const SurveyCellSet& looked,
                         const SurveyCellSet& passed) {
  const auto started = std::chrono::steady_clock::now();
  SurveyReading reading;
  Lattice lattice{grid, config};
  lattice.markLooked(looked);
  const std::vector<GridIndex3D> cells = frontierCells(lattice, looked, passed);
  reading.frontier_cells = cells.size();
  reading.frontier_set.insert(cells.begin(), cells.end());
  flood(lattice, vehicle, reading);
  const double resolution = grid.bounds().resolution_m;
  const int reach_cells =
      std::max(1, static_cast<int>(std::ceil(config.sensor_reach_m / resolution)));
  for (std::vector<GridIndex3D>& members : clustersOf(cells, reach_cells)) {
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
    frontier.sight = grid.cellCenter(*std::min_element(
        members.begin(), members.end(),
        [&grid, &frontier](const auto& lhs, const auto& rhs) {
          return squaredDistance(grid.cellCenter(lhs), frontier.centroid) <
                 squaredDistance(grid.cellCenter(rhs), frontier.centroid);
        }));
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
    std::optional<GridIndex3D> vantage =
        snap(lattice, near, kSnapReachM, frontier.sight);
    if (!vantage.has_value()) {
      vantage = snap(lattice, frontier.centroid,
                     std::min(config.sensor_reach_m, kVantageReachM), frontier.sight);
    }
    if (!vantage.has_value()) {
      ++reading.clusters_without_vantage;
      continue;
    }
    frontier.vantage = grid.cellCenter(*vantage);
    frontier.route_m = static_cast<double>(lattice.reachedKnown(*vantage)) * resolution;
    // The pass goes through observed space only (r2806: a far vantage in
    // unknown space behind the centroid sent the planner round the world).
    const std::optional<GridIndex3D> far_vantage =
        snap(lattice, far, kSnapReachM, frontier.sight);
    frontier.far_vantage_known = far_vantage.has_value();
    frontier.far_vantage =
        far_vantage.has_value() ? grid.cellCenter(*far_vantage) : frontier.vantage;
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
  std::array<char, 320U> line{};
  std::snprintf(line.data(), line.size(),
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
  return line.data();
}

bool lineOfSight(const ObservedOccupancyGrid3D& grid, const Point3& from,
                 const Point3& to) {
  // The first metre is the lidar's minimum range: unknown round the vehicle
  // is not what hides the point.
  constexpr double kSkipM{1.0};
  const double length = distance3D(from, to);
  const double step = 0.5 * grid.bounds().resolution_m;
  const std::optional<GridIndex3D> target = grid.worldToCell(to);
  if (!target.has_value()) {
    return false;
  }
  const int steps = static_cast<int>(std::ceil((length - kSkipM) / step));
  for (int index = 0; index < steps; ++index) {
    const double share = (kSkipM + static_cast<double>(index) * step) / length;
    const std::optional<GridIndex3D> cell = grid.worldToCell(
        Point3{from.x + share * (to.x - from.x), from.y + share * (to.y - from.y),
               from.z + share * (to.z - from.z)});
    if (!cell.has_value()) {
      return false;
    }
    if (GridIndex3DEqual{}(*cell, *target)) {
      return true;
    }
    if (grid.state(*cell) != ObservedVoxelState::kFree) {
      return false;
    }
  }
  return true;
}

} // namespace drone_city_nav
