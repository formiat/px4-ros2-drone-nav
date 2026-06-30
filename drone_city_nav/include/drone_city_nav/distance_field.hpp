#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace drone_city_nav {

enum class DistanceFieldSource {
  kOccupied,
  kProhibited,
};

struct DistanceFieldBuildStats {
  std::size_t source_cells{0U};
  std::size_t width_cells{0U};
  std::size_t height_cells{0U};
  double duration_ms{0.0};
  std::string_view algorithm{"edt"};
};

class DistanceField2D {
public:
  [[nodiscard]] static DistanceField2D
  build(const OccupancyGrid2D& grid, double max_distance_m, DistanceFieldSource source);

  [[nodiscard]] const GridBounds& bounds() const noexcept;
  [[nodiscard]] double maxDistanceM() const noexcept;
  [[nodiscard]] DistanceFieldSource source() const noexcept;
  [[nodiscard]] bool contains(GridIndex cell) const noexcept;
  [[nodiscard]] double distanceAt(GridIndex cell) const;
  [[nodiscard]] std::span<const double> distancesM() const noexcept;
  [[nodiscard]] const DistanceFieldBuildStats& stats() const noexcept;

private:
  GridBounds bounds_{};
  double max_distance_m_{0.0};
  DistanceFieldSource source_{DistanceFieldSource::kOccupied};
  std::vector<double> distance_m_;
  DistanceFieldBuildStats stats_{};
};

} // namespace drone_city_nav
