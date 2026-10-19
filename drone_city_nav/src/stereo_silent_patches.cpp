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

std::vector<std::uint8_t> litCells(const std::span<const std::uint8_t> grey,
                                   const std::size_t row_step, const std::size_t width,
                                   const std::size_t height, const std::size_t stride,
                                   const std::uint8_t lit_grey) {
  std::vector<std::uint8_t> lit;
  if (stride == 0U || width < stride || height < stride) {
    return lit;
  }
  const std::size_t columns = width / stride;
  const std::size_t rows = height / stride;
  lit.assign(columns * rows, 0U);
  for (std::size_t row = 0U; row < rows; ++row) {
    for (std::size_t column = 0U; column < columns; ++column) {
      const std::size_t at = row * stride * row_step + column * stride;
      lit[row * columns + column] = at < grey.size() && grey[at] >= lit_grey ? 1U : 0U;
    }
  }
  return lit;
}

std::vector<std::uint8_t> seenCells(const std::span<const StereoDepthReturn> returns,
                                    const StereoPairGeometry& geometry,
                                    const std::size_t stride, const std::size_t width,
                                    const std::size_t height) {
  std::vector<std::uint8_t> seen;
  if (stride == 0U || width < stride || height < stride) {
    return seen;
  }
  const StrideGrid grid = strideGrid(returns, geometry, stride, width, height);
  seen.assign(grid.kind.size(), 0U);
  for (std::size_t index = 0U; index < grid.kind.size(); ++index) {
    seen[index] = grid.kind[index] == kSilent ? 0U : 1U;
  }
  return seen;
}

std::optional<std::size_t> strideCellOf(const Point3& direction,
                                        const StereoPairGeometry& geometry,
                                        const std::size_t stride,
                                        const std::size_t width,
                                        const std::size_t height) {
  if (stride == 0U || !(direction.x > 0.0)) {
    return std::nullopt;
  }
  const double column_px =
      geometry.principal_x_px - geometry.focal_px * direction.y / direction.x;
  const double row_px =
      geometry.principal_y_px - geometry.focal_px * direction.z / direction.x;
  if (column_px < 0.0 || row_px < 0.0) {
    return std::nullopt;
  }
  const auto column =
      static_cast<std::size_t>(std::lround(column_px / static_cast<double>(stride)));
  const auto row =
      static_cast<std::size_t>(std::lround(row_px / static_cast<double>(stride)));
  if (column >= width / stride || row >= height / stride) {
    return std::nullopt;
  }
  return row * (width / stride) + column;
}

std::vector<Point3> smokeFaceRays(const std::span<const SilentPatch> regions,
                                  const StereoPairGeometry& geometry,
                                  const std::size_t stride, const std::size_t width,
                                  const double nearest_m, const double slab_m,
                                  const double step_m, const std::size_t pitch_cells) {
  std::vector<Point3> points;
  if (stride == 0U || width < stride || pitch_cells == 0U || !(step_m > 0.0)) {
    return points;
  }
  const std::size_t columns = width / stride;
  for (const SilentPatch& face : regions) {
    if (!face.closed) {
      continue;
    }
    const double from_m = std::max(nearest_m, face.rim_depth_m);
    for (const std::size_t index : face.cell_indices) {
      const std::size_t row_cell = index / columns;
      const std::size_t column_cell = index % columns;
      if (row_cell % pitch_cells != 0U || column_cell % pitch_cells != 0U) {
        continue;
      }
      const double column_px =
          static_cast<double>(column_cell * stride) + 0.5 * static_cast<double>(stride);
      const double row_px =
          static_cast<double>(row_cell * stride) + 0.5 * static_cast<double>(stride);
      const double left = -(column_px - geometry.principal_x_px) / geometry.focal_px;
      const double up = -(row_px - geometry.principal_y_px) / geometry.focal_px;
      const double norm = std::sqrt(1.0 + left * left + up * up);
      for (double range_m = from_m; range_m <= from_m + slab_m; range_m += step_m) {
        const double along = range_m / norm;
        points.push_back(Point3{along, along * left, along * up});
      }
    }
  }
  return points;
}

std::vector<SilentPatch> smokeFaces(const std::span<const StereoDepthReturn> returns,
                                    const StereoPairGeometry& geometry,
                                    const StereoDepthReturnsConfig& returns_config,
                                    const std::size_t width, const std::size_t height,
                                    const std::span<const std::uint8_t> lit_cells,
                                    const SilentPatchConfig& config,
                                    const SmokeFaceConfig& face_config) {
  std::vector<SilentPatch> faces;
  const std::size_t stride = returns_config.pixel_stride;
  if (stride == 0U || width < stride || height < stride) {
    return faces;
  }
  const StrideGrid grid = strideGrid(returns, geometry, stride, width, height);
  if (lit_cells.size() != grid.kind.size()) {
    return faces;
  }
  std::vector<std::uint32_t> label(grid.kind.size(), 0U);
  std::vector<std::size_t> stack;
  std::uint32_t next_label{1U};
  for (std::size_t seed = 0U; seed < label.size(); ++seed) {
    if (grid.kind[seed] != kSilent || lit_cells[seed] == 0U || label[seed] != 0U) {
      continue;
    }
    // One region of lit silence by four-connectivity; its rim the hits
    // beside it, its see-through the free rays beside it; the dark silence
    // beside it is neither.
    std::vector<std::size_t> cells;
    std::vector<double> rim_depths;
    std::vector<std::size_t> rim_cells;
    std::vector<std::uint8_t> seen(label.size(), 0U);
    std::size_t free_neighbours{0U};
    stack.assign(1U, seed);
    label[seed] = next_label;
    while (!stack.empty()) {
      const std::size_t index = stack.back();
      stack.pop_back();
      cells.push_back(index);
      const std::size_t row = index / grid.columns;
      const std::size_t column = index % grid.columns;
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
          if (lit_cells[neighbour] != 0U && (d_row == 0 || d_column == 0) &&
              label[neighbour] == 0U) {
            label[neighbour] = next_label;
            stack.push_back(neighbour);
          }
        } else if (seen[neighbour] == 0U) {
          seen[neighbour] = 1U;
          if (grid.kind[neighbour] == kHit) {
            rim_depths.push_back(grid.depth_m[neighbour]);
            rim_cells.push_back(neighbour);
          } else {
            ++free_neighbours;
          }
        }
      }
    }
    ++next_label;
    if (cells.size() < config.least_cells) {
      continue;
    }
    SilentPatch face{.cells = cells.size(),
                     .rim_hits = rim_depths.size(),
                     .free_neighbours = free_neighbours};
    const double see_through =
        static_cast<double>(free_neighbours) / static_cast<double>(cells.size());
    // The sectors round the region's centre the rim reaches into, as a
    // patch's.
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
    if (rim_depths.size() < config.least_rim_hits) {
      face.reason = "rim_unmatched";
    } else if (rim_sectors < face_config.least_rim_sectors) {
      face.reason = "rim_not_round";
    } else {
      face.rim_depth_m = quantile(rim_depths, 0.5);
      face.rim_spread_m = quantile(rim_depths, 0.75) - quantile(rim_depths, 0.25);
      if (face.rim_depth_m > config.confident_depth_m) {
        face.reason = "rim_beyond_reach";
      } else if (face.rim_spread_m > config.rim_flat_m) {
        face.reason = "rim_recedes";
      } else if (see_through >= face_config.see_through_share) {
        face.reason = "seen_through";
      } else {
        face.closed = true;
        face.reason = "face";
      }
    }
    face.cell_indices = std::move(cells);
    faces.push_back(face);
  }
  return faces;
}

} // namespace drone_city_nav
