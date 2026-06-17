#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <span>
#include <vector>

namespace drone_city_nav {

enum class ClearanceSource {
  kOccupied,
  kBlocked,
};

class ClearanceField2D {
public:
  [[nodiscard]] static ClearanceField2D
  build(const OccupancyGrid2D& grid, double max_distance_m, ClearanceSource source);

  [[nodiscard]] const GridBounds& bounds() const noexcept;
  [[nodiscard]] double maxDistanceM() const noexcept;
  [[nodiscard]] bool contains(GridIndex cell) const noexcept;
  [[nodiscard]] double distanceAt(GridIndex cell) const;
  [[nodiscard]] std::span<const double> distancesM() const noexcept;

private:
  GridBounds bounds_{};
  double max_distance_m_{0.0};
  std::vector<double> distance_m_;
};

} // namespace drone_city_nav
