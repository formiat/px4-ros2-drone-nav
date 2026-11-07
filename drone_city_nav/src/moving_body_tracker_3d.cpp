#include "drone_city_nav/moving_body_tracker_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>

namespace drone_city_nav {
namespace {

struct CellKeyHash {
  [[nodiscard]] std::size_t operator()(const GridIndex3D& cell) const noexcept {
    const auto x = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell.x));
    const auto y = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell.y));
    const auto z = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell.z));
    return static_cast<std::size_t>((x * 73856093ULL) ^ (y * 19349663ULL) ^
                                    (z * 83492791ULL));
  }
};

[[nodiscard]] std::size_t findRoot(std::vector<std::size_t>& parent,
                                   std::size_t index) {
  while (parent[index] != index) {
    parent[index] = parent[parent[index]];
    index = parent[index];
  }
  return index;
}

void unite(std::vector<std::size_t>& parent, const std::size_t first,
           const std::size_t second) {
  const std::size_t first_root = findRoot(parent, first);
  const std::size_t second_root = findRoot(parent, second);
  if (first_root != second_root) {
    parent[second_root] = first_root;
  }
}

[[nodiscard]] Point3 cellCentre(const GridIndex3D cell, const GridBounds3D& bounds) {
  return Point3{bounds.origin_x + (cell.x + 0.5) * bounds.resolution_m,
                bounds.origin_y + (cell.y + 0.5) * bounds.resolution_m,
                bounds.origin_z + (cell.z + 0.5) * bounds.resolution_m};
}

struct Cluster {
  MovingBody3D body;
  std::vector<GridIndex3D> cells;
};

// The voxels of one scan clustered by 26-connectivity, each cluster read as a
// body: the mean of its centres, its horizontal reach and its height.
[[nodiscard]] std::vector<Cluster>
clusterBodies(const std::span<const GridIndex3D> cells, const GridBounds3D& bounds,
              const MovingBodyTrackerConfig3D& config) {
  std::unordered_map<GridIndex3D, std::size_t, CellKeyHash> index_of;
  index_of.reserve(cells.size());
  for (std::size_t index = 0U; index < cells.size(); ++index) {
    index_of.emplace(cells[index], index);
  }
  std::vector<std::size_t> parent(cells.size());
  std::iota(parent.begin(), parent.end(), std::size_t{0U});
  for (std::size_t index = 0U; index < cells.size(); ++index) {
    const GridIndex3D cell = cells[index];
    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const auto near = index_of.find({cell.x + dx, cell.y + dy, cell.z + dz});
          if (near != index_of.end()) {
            unite(parent, index, near->second);
          }
        }
      }
    }
  }
  std::unordered_map<std::size_t, std::vector<GridIndex3D>> members;
  for (std::size_t index = 0U; index < cells.size(); ++index) {
    members[findRoot(parent, index)].push_back(cells[index]);
  }
  std::vector<Cluster> clusters;
  const double half = 0.5 * bounds.resolution_m;
  for (auto& [root, group] : members) {
    static_cast<void>(root);
    if (group.size() < config.least_voxels) {
      continue;
    }
    MovingBody3D body{};
    body.bottom_m = std::numeric_limits<double>::infinity();
    body.top_m = -std::numeric_limits<double>::infinity();
    for (const GridIndex3D cell : group) {
      const Point3 centre = cellCentre(cell, bounds);
      body.centre_map.x += centre.x;
      body.centre_map.y += centre.y;
      body.centre_map.z += centre.z;
      body.bottom_m = std::min(body.bottom_m, centre.z - half);
      body.top_m = std::max(body.top_m, centre.z + half);
    }
    const auto count = static_cast<double>(group.size());
    body.centre_map.x /= count;
    body.centre_map.y /= count;
    body.centre_map.z /= count;
    for (const GridIndex3D cell : group) {
      const Point3 centre = cellCentre(cell, bounds);
      body.radius_m = std::max(body.radius_m, std::hypot(centre.x - body.centre_map.x,
                                                         centre.y - body.centre_map.y) +
                                                  half);
    }
    if (body.radius_m > config.widest_radius_m) {
      continue;
    }
    body.voxels = group.size();
    clusters.push_back(Cluster{.body = body, .cells = std::move(group)});
  }
  // The largest first, so that a body split by an occlusion keeps its track
  // with its larger part.
  std::ranges::sort(clusters, [](const Cluster& first, const Cluster& second) {
    return first.body.voxels > second.body.voxels;
  });
  return clusters;
}

// The share of a set of cells found in another, carried by a shift.
[[nodiscard]] double coverage(const std::span<const GridIndex3D> cells,
                              const std::span<const GridIndex3D> among,
                              const GridIndex3D shift) {
  if (cells.empty()) {
    return 0.0;
  }
  std::size_t found = 0U;
  for (const GridIndex3D cell : cells) {
    const GridIndex3D carried{cell.x + shift.x, cell.y + shift.y, cell.z + shift.z};
    found += std::ranges::find(among, carried) != among.end() ? 1U : 0U;
  }
  return static_cast<double>(found) / static_cast<double>(cells.size());
}

constexpr std::size_t kHistoryCells{512U};

} // namespace

bool MovingBodyTracker3D::movedSinceFirst(const Track& track,
                                          const std::span<const GridIndex3D> cells,
                                          const double resolution_m) {
  const double dx = track.body.centre_map.x - track.first_centre_map.x;
  const double dy = track.body.centre_map.y - track.first_centre_map.y;
  if (std::hypot(dx, dy) < resolution_m) {
    return false;
  }
  const GridIndex3D shift{static_cast<int>(std::lround(dx / resolution_m)),
                          static_cast<int>(std::lround(dy / resolution_m)), 0};
  const double carried = coverage(track.first_cells, cells, shift);
  const double uncarried = coverage(track.first_cells, cells, GridIndex3D{});
  return carried >= 0.5 && carried - uncarried >= 0.25;
}

MovingBodyTracker3D::MovingBodyTracker3D(const MovingBodyTrackerConfig3D& config)
    : config_{config} {
}

void MovingBodyTracker3D::update(const std::span<const GridIndex3D> cells,
                                 const GridBounds3D& bounds,
                                 const std::int64_t now_ms) {
  std::vector<Cluster> seen = clusterBodies(cells, bounds, config_);
  std::vector<Track> next;
  next.reserve(seen.size());
  std::vector<bool> taken(tracks_.size(), false);
  for (Cluster& cluster : seen) {
    MovingBody3D& body = cluster.body;
    std::size_t nearest = tracks_.size();
    double nearest_m = config_.match_m;
    for (std::size_t index = 0U; index < tracks_.size(); ++index) {
      if (taken[index]) {
        continue;
      }
      const double apart =
          std::hypot(body.centre_map.x - tracks_[index].body.centre_map.x,
                     body.centre_map.y - tracks_[index].body.centre_map.y);
      if (apart <= nearest_m) {
        nearest_m = apart;
        nearest = index;
      }
    }
    Track track{};
    if (nearest < tracks_.size()) {
      taken[nearest] = true;
      Track& before = tracks_[nearest];
      track.first_ms = before.first_ms;
      track.first_centre_map = before.first_centre_map;
      track.first_cells = std::move(before.first_cells);
      track.cells_since_first = std::move(before.cells_since_first);
      track.previous_cells = std::move(before.cells);
      track.moved = before.moved;
      track.velocity_map_mps = before.velocity_map_mps;
      track.body = body;
      if (!track.moved) {
        track.moved = movedSinceFirst(track, cluster.cells, bounds.resolution_m);
        for (const GridIndex3D cell : cluster.cells) {
          if (track.cells_since_first.size() >= kHistoryCells) {
            break;
          }
          if (std::ranges::find(track.cells_since_first, cell) ==
              track.cells_since_first.end()) {
            track.cells_since_first.push_back(cell);
          }
        }
      }
      const double elapsed_s = static_cast<double>(now_ms - before.last_ms) * 1.0e-3;
      // A step leaves a voxel behind; a body that keeps every voxel this scan
      // has not stepped, and its velocity decays toward rest.
      const bool stepped =
          track.moved &&
          std::ranges::any_of(track.previous_cells, [&](const GridIndex3D cell) {
            return std::ranges::find(cluster.cells, cell) == cluster.cells.end();
          });
      if (!stepped) {
        track.velocity_map_mps.x *= 1.0 - config_.velocity_gain;
        track.velocity_map_mps.y *= 1.0 - config_.velocity_gain;
      } else if (elapsed_s > 0.0) {
        const Vec3 step{(body.centre_map.x - before.body.centre_map.x) / elapsed_s,
                        (body.centre_map.y - before.body.centre_map.y) / elapsed_s,
                        0.0};
        track.velocity_map_mps.x +=
            config_.velocity_gain * (step.x - track.velocity_map_mps.x);
        track.velocity_map_mps.y +=
            config_.velocity_gain * (step.y - track.velocity_map_mps.y);
      }
    } else {
      track.first_ms = now_ms;
      track.first_centre_map = body.centre_map;
      track.first_cells = cluster.cells;
      track.cells_since_first = cluster.cells;
    }
    track.last_ms = now_ms;
    body.age_s = static_cast<double>(now_ms - track.first_ms) * 1.0e-3;
    body.velocity_map_mps =
        std::hypot(track.velocity_map_mps.x, track.velocity_map_mps.y) >=
                config_.standing_speed_mps
            ? track.velocity_map_mps
            : Vec3{};
    track.body = body;
    track.cells = std::move(cluster.cells);
    next.push_back(std::move(track));
  }
  tracks_ = std::move(next);
  bodies_.clear();
  bodies_.reserve(tracks_.size());
  for (const Track& track : tracks_) {
    bodies_.push_back(track.body);
  }
}

void MovingBodyTracker3D::clear() {
  tracks_.clear();
  bodies_.clear();
}

const std::vector<MovingBody3D>& MovingBodyTracker3D::bodies() const noexcept {
  return bodies_;
}

std::span<const GridIndex3D>
MovingBodyTracker3D::cells(const std::size_t body) const noexcept {
  return body < tracks_.size() ? std::span<const GridIndex3D>{tracks_[body].cells}
                               : std::span<const GridIndex3D>{};
}

std::span<const GridIndex3D>
MovingBodyTracker3D::cellsSinceFirst(const std::size_t body) const noexcept {
  return body < tracks_.size()
             ? std::span<const GridIndex3D>{tracks_[body].cells_since_first}
             : std::span<const GridIndex3D>{};
}

std::span<const GridIndex3D>
MovingBodyTracker3D::previousCells(const std::size_t body) const noexcept {
  return body < tracks_.size()
             ? std::span<const GridIndex3D>{tracks_[body].previous_cells}
             : std::span<const GridIndex3D>{};
}

std::vector<GridIndex3D> sweptCells3D(const MovingBody3D& body, const double sweep_s,
                                      const GridBounds3D& bounds) {
  std::vector<GridIndex3D> cells;
  if (!body.moving() || !(sweep_s > 0.0) || !(bounds.resolution_m > 0.0)) {
    return cells;
  }
  const Point3& from = body.centre_map;
  const Point3 to{from.x + body.velocity_map_mps.x * sweep_s,
                  from.y + body.velocity_map_mps.y * sweep_s, from.z};
  const auto cell_of = [&](const double value, const double origin) {
    return static_cast<int>(std::floor((value - origin) / bounds.resolution_m));
  };
  const int x_low =
      std::max(0, cell_of(std::min(from.x, to.x) - body.radius_m, bounds.origin_x));
  const int x_high =
      std::min(bounds.width_cells - 1,
               cell_of(std::max(from.x, to.x) + body.radius_m, bounds.origin_x));
  const int y_low =
      std::max(0, cell_of(std::min(from.y, to.y) - body.radius_m, bounds.origin_y));
  const int y_high =
      std::min(bounds.height_cells - 1,
               cell_of(std::max(from.y, to.y) + body.radius_m, bounds.origin_y));
  const int z_low = std::max(0, cell_of(body.bottom_m, bounds.origin_z));
  const int z_high =
      std::min(bounds.depth_cells - 1, cell_of(body.top_m, bounds.origin_z));
  const double along_x = to.x - from.x;
  const double along_y = to.y - from.y;
  const double along_squared = along_x * along_x + along_y * along_y;
  for (int z = z_low; z <= z_high; ++z) {
    const double centre_z = bounds.origin_z + (z + 0.5) * bounds.resolution_m;
    if (centre_z < body.bottom_m || centre_z > body.top_m) {
      continue;
    }
    for (int y = y_low; y <= y_high; ++y) {
      const double centre_y = bounds.origin_y + (y + 0.5) * bounds.resolution_m;
      for (int x = x_low; x <= x_high; ++x) {
        const double centre_x = bounds.origin_x + (x + 0.5) * bounds.resolution_m;
        // The nearest point of the segment the centre travels.
        const double share =
            std::clamp(((centre_x - from.x) * along_x + (centre_y - from.y) * along_y) /
                           along_squared,
                       0.0, 1.0);
        const double apart = std::hypot(centre_x - (from.x + share * along_x),
                                        centre_y - (from.y + share * along_y));
        if (apart <= body.radius_m) {
          cells.push_back({x, y, z});
        }
      }
    }
  }
  return cells;
}

} // namespace drone_city_nav
