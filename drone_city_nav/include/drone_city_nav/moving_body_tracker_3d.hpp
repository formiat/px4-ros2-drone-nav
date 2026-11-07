#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace drone_city_nav {

// Roadmap item 21, specification K66: a body the obstacle memory follows, as
// the planner reads it: where it stands, how fast its centre was measured
// moving, and its envelope.
struct MovingBody3D {
  Point3 centre_map{};
  // The horizontal velocity the body's centre was measured moving at over the
  // scans it was followed; zero while it stands (under the standing speed).
  Vec3 velocity_map_mps{};
  // The horizontal reach from the centre to the farthest of its voxels, and
  // the voxels' bottom and top.
  double radius_m{0.0};
  double bottom_m{0.0};
  double top_m{0.0};
  // How long the body has been followed.
  double age_s{0.0};
  std::size_t voxels{0U};

  [[nodiscard]] bool moving() const noexcept {
    return velocity_map_mps.x != 0.0 || velocity_map_mps.y != 0.0;
  }
};

// The bodies the memory follows, at the stamp of the scan they were read on.
struct MovingBodyEvidence3D {
  std::int64_t stamp_ns{0};
  std::vector<MovingBody3D> bodies;
};

struct MovingBodyTrackerConfig3D {
  // A body is the body last seen within this distance of it: the fastest
  // walk the scenarios know (3 m/s) over a scan period, and the jitter of a
  // centre whose voxels come and go.
  double match_m{1.0};
  // Under this measured speed a body stands, and its velocity reads zero.
  double standing_speed_mps{0.3};
  // The share of a measured step the velocity takes in each scan: the centre
  // jitters by a voxel between scans, and a walk reads steady over a few.
  double velocity_gain{0.3};
  // A cluster smaller than this is a noise return, not a body; one reaching
  // farther than this from its centre is a surface, not a body.
  std::size_t least_voxels{4U};
  double widest_radius_m{1.0};
};

// Follows clusters of voxels as bodies between scans: the voxels are
// clustered by 26-connectivity, each cluster is matched to the body nearest
// where it was last seen, and the body's velocity is read off its centre's
// steps. A step leaves a voxel behind: a cluster that only gained voxels
// since the last scan came into view rather than moved, and its centre's
// shift is no velocity. A body that is not seen again is gone.
class MovingBodyTracker3D {
public:
  explicit MovingBodyTracker3D(const MovingBodyTrackerConfig3D& config = {});

  // The voxels this scan, by their cell indices on the grid, at the scan's
  // time on the memory's clock.
  void update(std::span<const GridIndex3D> cells, const GridBounds3D& bounds,
              std::int64_t now_ms);
  void clear();
  [[nodiscard]] const std::vector<MovingBody3D>& bodies() const noexcept;
  // The voxels of a body this scan, and the ones it had the scan before.
  [[nodiscard]] std::span<const GridIndex3D> cells(std::size_t body) const noexcept;
  [[nodiscard]] std::span<const GridIndex3D>
  previousCells(std::size_t body) const noexcept;

private:
  struct Track {
    MovingBody3D body;
    Vec3 velocity_map_mps{};
    std::int64_t first_ms{0};
    std::int64_t last_ms{0};
    std::vector<GridIndex3D> cells;
    std::vector<GridIndex3D> previous_cells;
  };

  MovingBodyTrackerConfig3D config_;
  std::vector<Track> tracks_;
  std::vector<MovingBody3D> bodies_;
};

// The cells of the grid a moving body sweeps in `sweep_s` at its measured
// velocity: its cylinder carried along the velocity, the cells whose centres
// lie inside. A standing body sweeps nothing.
[[nodiscard]] std::vector<GridIndex3D>
sweptCells3D(const MovingBody3D& body, double sweep_s, const GridBounds3D& bounds);

} // namespace drone_city_nav
