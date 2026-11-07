#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
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
  // farther than this from its centre is a surface, not a body; one lower
  // than this is a sheet of the range's noise over a far floor or ceiling, a
  // cell thick (r3027: 191 such bodies at once), not a body of a person's
  // height.
  std::size_t least_voxels{4U};
  double widest_radius_m{1.0};
  double least_height_m{1.0};
  // One taller than this is a wall's skin (r3061: skins 3 to 5.5 m tall read
  // as bodies walking along the walls), not a body of a person's height.
  double tallest_m{2.5};
};

// Follows clusters of voxels as bodies between scans: the voxels are
// clustered by 26-connectivity, each cluster is matched to the body nearest
// where it was last seen, and the body's velocity is read off its centre's
// steps once the cluster is seen to have moved: its voxels of three scans
// before, carried by the centre's displacement since (a cell at least), cover
// the voxels of this scan better than they do uncarried, by a quarter, and
// the cluster has not doubled meanwhile (three scans, not the first: a body
// the vehicle flies toward gains voxels as the range closes, r3037). A
// surface coming into view only gains voxels and covers itself uncarried; a
// surface's noise shifts no centre by a cell. A body that is not seen again
// is gone.
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
  // Every voxel the body has had since it was first seen, until it was seen
  // to move (512 at most): what the layer takes when it does.
  [[nodiscard]] std::span<const GridIndex3D>
  cellsSinceFirst(std::size_t body) const noexcept;

private:
  struct Reference {
    Point3 centre_map{};
    std::vector<GridIndex3D> cells;
  };

  struct Track {
    MovingBody3D body;
    Vec3 velocity_map_mps{};
    std::int64_t first_ms{0};
    std::int64_t last_ms{0};
    std::vector<GridIndex3D> cells;
    std::vector<GridIndex3D> previous_cells;
    std::vector<GridIndex3D> cells_since_first;
    // The centre and the voxels of the last scans, the oldest first.
    std::deque<Reference> recent;
    bool moved{false};
  };

  [[nodiscard]] static bool movedLately(const Track& track,
                                        std::span<const GridIndex3D> cells,
                                        double resolution_m);

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
