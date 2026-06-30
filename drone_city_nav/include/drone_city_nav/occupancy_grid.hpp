#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav {

class DistanceField2D;

enum class CellState : std::int8_t {
  kUnknown = -1,
  kFree = 0,
  kOccupied = 100,
};

struct OccupancyGridFingerprint {
  GridBounds bounds{};
  std::uint64_t cells_hash{0U};
  std::uint64_t inflated_hash{0U};
};

class OccupancyGrid2D {
public:
  explicit OccupancyGrid2D(const GridBounds& bounds);

  [[nodiscard]] const GridBounds& bounds() const noexcept;
  [[nodiscard]] int width() const noexcept;
  [[nodiscard]] int height() const noexcept;
  [[nodiscard]] double resolution() const noexcept;
  [[nodiscard]] double originX() const noexcept;
  [[nodiscard]] double originY() const noexcept;
  [[nodiscard]] std::size_t cellCount() const noexcept;

  [[nodiscard]] bool contains(GridIndex cell) const noexcept;
  [[nodiscard]] std::optional<GridIndex> worldToCell(Point2 point) const noexcept;
  [[nodiscard]] Point2 cellCenter(GridIndex cell) const noexcept;
  [[nodiscard]] std::size_t linearIndex(GridIndex cell) const;

  [[nodiscard]] CellState state(GridIndex cell) const;
  [[nodiscard]] bool isOccupied(GridIndex cell) const;
  [[nodiscard]] bool isInflated(GridIndex cell) const;
  [[nodiscard]] bool isProhibited(GridIndex cell) const;
  [[nodiscard]] std::span<const CellState> cells() const noexcept;
  [[nodiscard]] std::span<const std::uint8_t> inflatedCells() const noexcept;
  [[nodiscard]] OccupancyGridFingerprint prohibitedFingerprint() const noexcept;

  void reset(CellState value = CellState::kUnknown);
  void setUnknown(GridIndex cell);
  void setFree(GridIndex cell);
  void setOccupied(GridIndex cell);
  void markRay(Point2 start, Point2 end, bool endpoint_occupied);
  void rebuildInflation(double radius_m);
  void applyInflationFromDistanceField(const DistanceField2D& field, double radius_m);
  void mergeInflationFrom(const OccupancyGrid2D& source);

  [[nodiscard]] std::vector<GridIndex> cellsOnLine(GridIndex start,
                                                   GridIndex end) const;

private:
  GridBounds bounds_{};
  std::vector<CellState> cells_;
  std::vector<std::uint8_t> inflated_;
};

} // namespace drone_city_nav
