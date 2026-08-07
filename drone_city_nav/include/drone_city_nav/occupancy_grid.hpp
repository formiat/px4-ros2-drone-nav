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
  [[nodiscard]] std::span<const CellState> cells() const noexcept;
  [[nodiscard]] OccupancyGridFingerprint rawFingerprint() const noexcept;
  [[nodiscard]] std::uint64_t occupiedFingerprint() const noexcept;

  void reset(CellState value = CellState::kUnknown);
  void setUnknown(GridIndex cell);
  void setFree(GridIndex cell);
  void setOccupied(GridIndex cell);
  void markRay(Point2 start, Point2 end, bool endpoint_occupied);

  [[nodiscard]] std::vector<GridIndex> cellsOnLine(GridIndex start,
                                                   GridIndex end) const;

private:
  GridBounds bounds_{};
  std::vector<CellState> cells_;
};

// Non-owning view used when safety must inspect the newest middleware snapshot
// without waiting for a full grid conversion.
class RawOccupancyGridView2D {
public:
  RawOccupancyGridView2D(const GridBounds& bounds, std::span<const std::int8_t> cells,
                         std::int8_t minimum_occupied_value);

  [[nodiscard]] const GridBounds& bounds() const noexcept;
  [[nodiscard]] bool contains(GridIndex cell) const noexcept;
  [[nodiscard]] std::optional<GridIndex> worldToCell(Point2 point) const noexcept;
  [[nodiscard]] bool isOccupied(GridIndex cell) const noexcept;

private:
  GridBounds bounds_{};
  std::span<const std::int8_t> cells_;
  std::int8_t minimum_occupied_value_{100};
};

} // namespace drone_city_nav
