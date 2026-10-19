#include "drone_city_nav/stereo_silent_patches.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <numeric>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr std::uint8_t kSilent{0U};
constexpr std::uint8_t kFree{1U};
constexpr std::uint8_t kHit{2U};

struct StrideGrid {
  std::size_t columns{0U};
  std::size_t rows{0U};
  std::vector<std::uint8_t> kind;
  std::vector<float> depth_m;

  [[nodiscard]] std::size_t at(const std::size_t row,
                               const std::size_t column) const noexcept {
    return row * columns + column;
  }
};

// The returns laid back on the stride grid: a hit with its depth, a free ray,
// or silence where no return was.
StrideGrid strideGrid(const std::span<const StereoDepthReturn> returns,
                      const StereoPairGeometry& geometry, const std::size_t stride,
                      const std::size_t width, const std::size_t height) {
  StrideGrid grid;
  grid.columns = width / stride;
  grid.rows = height / stride;
  grid.kind.assign(grid.columns * grid.rows, kSilent);
  grid.depth_m.assign(grid.columns * grid.rows, 0.0F);
  for (const StereoDepthReturn& stereo_return : returns) {
    // The ray's pixel from its direction (forward-left-up): column from the
    // left component, row from the up one.
    if (!(stereo_return.point.x > 0.0)) {
      continue;
    }
    const double column_px = geometry.principal_x_px - geometry.focal_px *
                                                           stereo_return.point.y /
                                                           stereo_return.point.x;
    const double row_px = geometry.principal_y_px - geometry.focal_px *
                                                        stereo_return.point.z /
                                                        stereo_return.point.x;
    const auto column =
        static_cast<std::size_t>(std::lround(column_px / static_cast<double>(stride)));
    const auto row =
        static_cast<std::size_t>(std::lround(row_px / static_cast<double>(stride)));
    if (column_px < 0.0 || row_px < 0.0 || column >= grid.columns || row >= grid.rows) {
      continue;
    }
    const std::size_t index = grid.at(row, column);
    grid.kind[index] = stereo_return.hit ? kHit : kFree;
    grid.depth_m[index] = static_cast<float>(stereo_return.point.x);
  }
  return grid;
}

double quantile(std::vector<double>& values, const double share) {
  std::ranges::sort(values);
  return values[static_cast<std::size_t>(share *
                                         static_cast<double>(values.size() - 1U))];
}

} // namespace

std::vector<SilentPatch> silentPatches(
    const std::span<const StereoDepthReturn> returns,
    const StereoPairGeometry& geometry, const StereoDepthReturnsConfig& returns_config,
    const std::size_t width, const std::size_t height, const SilentPatchConfig& config,
    const std::span<const Point3> free_through, const double free_through_range_m) {
  std::vector<SilentPatch> patches;
  const std::size_t stride = returns_config.pixel_stride;
  if (stride == 0U || width < stride || height < stride) {
    return patches;
  }
  const StrideGrid grid = strideGrid(returns, geometry, stride, width, height);
  // Free readings of another sensor on the stride grid: the cell each one
  // looks through, kept where it reaches past the pair's silence.
  std::vector<std::uint8_t> free_through_cells(grid.columns * grid.rows, 0U);
  for (const Point3& direction : free_through) {
    if (!(direction.x > 0.0)) {
      continue;
    }
    const double column_px =
        geometry.principal_x_px - geometry.focal_px * direction.y / direction.x;
    const double row_px =
        geometry.principal_y_px - geometry.focal_px * direction.z / direction.x;
    const auto column =
        static_cast<std::size_t>(std::lround(column_px / static_cast<double>(stride)));
    const auto row =
        static_cast<std::size_t>(std::lround(row_px / static_cast<double>(stride)));
    if (column_px >= 0.0 && row_px >= 0.0 && column < grid.columns && row < grid.rows) {
      free_through_cells[grid.at(row, column)] = 1U;
    }
  }
  std::vector<std::uint32_t> label(grid.columns * grid.rows, 0U);
  std::vector<std::size_t> stack;
  std::uint32_t next_label{1U};
  for (std::size_t seed = 0U; seed < label.size(); ++seed) {
    if (grid.kind[seed] != kSilent || label[seed] != 0U) {
      continue;
    }
    // One patch: its silent cells by four-connectivity, its rim the hits
    // beside them.
    std::vector<std::size_t> cells;
    std::vector<double> rim_depths;
    std::vector<std::size_t> rim_cells;
    std::vector<std::uint8_t> rim_seen(label.size(), 0U);
    std::size_t free_neighbours{0U};
    bool free_through_it{false};
    bool at_border{false};
    stack.assign(1U, seed);
    label[seed] = next_label;
    while (!stack.empty()) {
      const std::size_t index = stack.back();
      stack.pop_back();
      cells.push_back(index);
      free_through_it = free_through_it || free_through_cells[index] != 0U;
      const std::size_t row = index / grid.columns;
      const std::size_t column = index % grid.columns;
      at_border = at_border || row == 0U || column == 0U || row + 1U == grid.rows ||
                  column + 1U == grid.columns;
      const std::array<std::pair<long, long>, 8> around{
          {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}}};
      for (const auto& [d_row, d_column] : around) {
        const long n_row = static_cast<long>(row) + d_row;
        const long n_column = static_cast<long>(column) + d_column;
        if (n_row < 0 || n_column < 0 || n_row >= static_cast<long>(grid.rows) ||
            n_column >= static_cast<long>(grid.columns)) {
          continue;
        }
        const std::size_t neighbour = grid.at(static_cast<std::size_t>(n_row),
                                              static_cast<std::size_t>(n_column));
        if (grid.kind[neighbour] == kSilent) {
          if ((d_row == 0 || d_column == 0) && label[neighbour] == 0U) {
            label[neighbour] = next_label;
            stack.push_back(neighbour);
          }
        } else if (grid.kind[neighbour] == kHit && rim_seen[neighbour] == 0U) {
          rim_seen[neighbour] = 1U;
          rim_depths.push_back(grid.depth_m[neighbour]);
          rim_cells.push_back(neighbour);
        } else if (grid.kind[neighbour] == kFree && rim_seen[neighbour] == 0U) {
          rim_seen[neighbour] = 1U;
          ++free_neighbours;
        }
      }
    }
    ++next_label;
    if (cells.size() < config.least_cells) {
      continue;
    }
    SilentPatch patch{.cells = cells.size(),
                      .rim_hits = rim_depths.size(),
                      .free_neighbours = free_neighbours};
    // The sectors round the patch's centre the rim reaches into.
    double centre_row{0.0};
    double centre_column{0.0};
    for (const std::size_t index : cells) {
      centre_row += static_cast<double>(index / grid.columns);
      centre_column += static_cast<double>(index % grid.columns);
    }
    centre_row /= static_cast<double>(cells.size());
    centre_column /= static_cast<double>(cells.size());
    std::array<bool, 8> sectors{};
    for (const std::size_t index : rim_cells) {
      const double angle =
          std::atan2(static_cast<double>(index / grid.columns) - centre_row,
                     static_cast<double>(index % grid.columns) - centre_column);
      sectors[static_cast<std::size_t>(
                  std::floor((angle + std::numbers::pi) / (std::numbers::pi / 4.0))) %
              8U] = true;
    }
    const auto rim_sectors =
        static_cast<std::size_t>(std::ranges::count(sectors, true));
    if (at_border) {
      patch.reason = "at_border";
    } else if (rim_depths.size() < config.least_rim_hits) {
      patch.reason = "rim_unmatched";
    } else if (rim_sectors < config.least_rim_sectors) {
      patch.reason = "rim_not_round";
    } else {
      patch.rim_depth_m = quantile(rim_depths, 0.5);
      patch.rim_spread_m = quantile(rim_depths, 0.75) - quantile(rim_depths, 0.25);
      if (patch.rim_depth_m > config.confident_depth_m) {
        patch.reason = "rim_beyond_reach";
      } else if (patch.rim_spread_m > config.rim_flat_m) {
        patch.reason = "rim_recedes";
      } else if (free_through_it && free_through_range_m > patch.rim_depth_m) {
        patch.reason = "free_through";
      } else {
        patch.closed = true;
        patch.reason = "closed";
      }
    }
    patch.cell_indices = std::move(cells);
    patches.push_back(patch);
  }
  return patches;
}

double silentRangeAheadM(const std::span<const SilentPatch> patches,
                         const StereoPairGeometry& geometry, const std::size_t stride,
                         const std::size_t stride_columns,
                         const std::size_t stride_rows,
                         const double cone_half_angle_rad) {
  double range_m = std::numeric_limits<double>::quiet_NaN();
  const double cone_px = geometry.focal_px * std::tan(cone_half_angle_rad);
  for (const SilentPatch& patch : patches) {
    if (!patch.closed) {
      continue;
    }
    for (const std::size_t index : patch.cell_indices) {
      const double column_px = static_cast<double>((index % stride_columns) * stride) +
                               0.5 * static_cast<double>(stride);
      const double row_px = static_cast<double>((index / stride_columns) * stride) +
                            0.5 * static_cast<double>(stride);
      if (std::hypot(column_px - geometry.principal_x_px,
                     row_px - geometry.principal_y_px) <= cone_px &&
          index / stride_columns < stride_rows) {
        range_m = std::isfinite(range_m) ? std::min(range_m, patch.rim_depth_m)
                                         : patch.rim_depth_m;
        break;
      }
    }
  }
  return range_m;
}

bool smokeFace(const SilentPatch& patch, const double raw_grey,
               const SmokeFaceConfig& config) {
  if (!patch.closed || patch.cells == 0U) {
    return false;
  }
  const double see_through =
      static_cast<double>(patch.free_neighbours) / static_cast<double>(patch.cells);
  return raw_grey >= config.lit_raw_grey && see_through < config.see_through_share;
}

} // namespace drone_city_nav
