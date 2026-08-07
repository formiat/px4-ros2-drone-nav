#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

class BoundedWorkerPool;
class StaticEsdfCache;

struct DistanceField3DBuildStats {
  std::size_t source_voxels{0U};
  std::size_t voxel_count{0U};
  double duration_ms{0.0};
  double x_pass_ms{0.0};
  double y_pass_ms{0.0};
  double z_pass_ms{0.0};
  double finalize_ms{0.0};
};

class DistanceField3D {
public:
  [[nodiscard]] static DistanceField3D build(const OccupancyGrid3D& occupancy,
                                             double maximum_distance_m,
                                             BoundedWorkerPool* worker_pool = nullptr);
  [[nodiscard]] static DistanceField3D
  buildLocal(const OccupancyGrid3D& occupancy, const GridBounds3D& local_bounds,
             double maximum_distance_m, BoundedWorkerPool* worker_pool = nullptr);

  [[nodiscard]] const GridBounds3D& bounds() const noexcept;
  [[nodiscard]] double maximumDistanceM() const noexcept;
  [[nodiscard]] bool contains(GridIndex3D index) const noexcept;
  [[nodiscard]] std::size_t linearIndex(GridIndex3D index) const;
  [[nodiscard]] float distanceAt(GridIndex3D index) const;
  [[nodiscard]] std::span<const float> distancesM() const noexcept;
  [[nodiscard]] const DistanceField3DBuildStats& stats() const noexcept;

private:
  friend class StaticEsdfCache;

  GridBounds3D bounds_{};
  double maximum_distance_m_{0.0};
  std::vector<float> distances_m_;
  DistanceField3DBuildStats stats_{};
};

} // namespace drone_city_nav
