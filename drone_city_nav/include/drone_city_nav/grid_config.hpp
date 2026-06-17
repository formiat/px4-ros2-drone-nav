#pragma once

#include "drone_city_nav/types.hpp"

#include <cstddef>

namespace drone_city_nav {

constexpr int kMaxGridAxisCells = 10'000;
constexpr std::size_t kMaxGridCellCount = 4'000'000U;

[[nodiscard]] int boundedPositiveCellCount(double length_m,
                                           double resolution_m) noexcept;

[[nodiscard]] GridBounds boundedGridBounds(double origin_x, double origin_y,
                                           double resolution_m, double width_m,
                                           double height_m) noexcept;

[[nodiscard]] bool gridBoundsUsable(const GridBounds& bounds) noexcept;

[[nodiscard]] std::size_t gridBoundsCellCount(const GridBounds& bounds) noexcept;

} // namespace drone_city_nav
