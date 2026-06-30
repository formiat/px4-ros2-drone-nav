#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

enum class ClearanceSource {
  kOccupied,
  kProhibited,
};

class ClearanceField2D {
public:
  [[nodiscard]] static ClearanceField2D
  build(const OccupancyGrid2D& grid, double max_distance_m, ClearanceSource source);

  [[nodiscard]] const GridBounds& bounds() const noexcept;
  [[nodiscard]] double maxDistanceM() const noexcept;
  [[nodiscard]] ClearanceSource source() const noexcept;
  [[nodiscard]] bool contains(GridIndex cell) const noexcept;
  [[nodiscard]] double distanceAt(GridIndex cell) const;
  [[nodiscard]] std::span<const double> distancesM() const noexcept;

private:
  GridBounds bounds_{};
  double max_distance_m_{0.0};
  ClearanceSource source_{ClearanceSource::kOccupied};
  std::vector<double> distance_m_;
};

struct ClearanceFieldCacheLookup {
  const ClearanceField2D* field{nullptr};
  bool cache_hit{false};
};

class ClearanceFieldCache {
public:
  [[nodiscard]] ClearanceFieldCacheLookup getOrBuild(const OccupancyGrid2D& grid,
                                                     double max_distance_m,
                                                     ClearanceSource source);

  void clear() noexcept;

private:
  [[nodiscard]] bool matches(const OccupancyGrid2D& grid, double max_distance_m,
                             ClearanceSource source) const noexcept;

  GridBounds bounds_{};
  double max_distance_m_{0.0};
  ClearanceSource source_{ClearanceSource::kOccupied};
  std::uint64_t cells_hash_{0U};
  std::uint64_t inflated_hash_{0U};
  std::vector<CellState> cells_snapshot_;
  std::vector<std::uint8_t> inflated_snapshot_;
  std::optional<ClearanceField2D> field_;
};

} // namespace drone_city_nav
