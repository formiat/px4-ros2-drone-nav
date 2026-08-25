#pragma once

#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace drone_city_nav {

struct ProductionMppiRawWorld2D {
  RawMapVersion version{};
  std::int64_t ready_stamp_ns{0};
  double reconstruction_ms{0.0};
  std::shared_ptr<const OccupancyGrid2D> occupancy;
};

struct ProductionMppiRawWorld3D {
  RawMapVersion version{};
  std::int64_t ready_stamp_ns{0};
  double reconstruction_ms{0.0};
  std::shared_ptr<const ObservedOccupancyGrid3D> occupancy;
  std::shared_ptr<const VersionedObservedRawWorld3D> execution_owner;
  std::vector<OccupancyChunkIndex3D> dirty_chunks;
  bool full_reset{false};
};

} // namespace drone_city_nav
