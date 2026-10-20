#include "drone_city_nav/stereo_silent_patches.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <numeric>
#include <string>
#include <string_view>
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

// The share of the rim depths' variance a plane over the cells' rows and
// columns explains: nought for a scatter, one for a rim that recedes along
// itself. Least squares on the three normal equations.
double rimTrendShare(const std::vector<std::size_t>& rim_cells,
                     const std::vector<double>& rim_depths, const std::size_t columns) {
  const std::size_t n = rim_cells.size();
  if (n < 3U || rim_depths.size() != n) {
    return 0.0;
  }
  double sr{0.0}, sc{0.0}, sd{0.0};
  for (std::size_t i = 0U; i < n; ++i) {
    sr += static_cast<double>(rim_cells[i] / columns);
    sc += static_cast<double>(rim_cells[i] % columns);
    sd += rim_depths[i];
  }
  const double mr = sr / static_cast<double>(n);
  const double mc = sc / static_cast<double>(n);
  const double md = sd / static_cast<double>(n);
  double srr{0.0}, scc{0.0}, src{0.0}, srd{0.0}, scd{0.0}, sdd{0.0};
  for (std::size_t i = 0U; i < n; ++i) {
    const double r = static_cast<double>(rim_cells[i] / columns) - mr;
    const double col = static_cast<double>(rim_cells[i] % columns) - mc;
    const double d = rim_depths[i] - md;
    srr += r * r;
    scc += col * col;
    src += r * col;
    srd += r * d;
    scd += col * d;
    sdd += d * d;
  }
  const double det = srr * scc - src * src;
  if (!(sdd > 1.0e-9) || std::abs(det) < 1.0e-9) {
    return 0.0;
  }
  const double a = (srd * scc - scd * src) / det;
  const double b = (scd * srr - srd * src) / det;
  const double explained = a * srd + b * scd;
  return std::clamp(explained / sdd, 0.0, 1.0);
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

std::vector<std::uint8_t> rawCells(const std::span<const std::uint8_t> grey,
                                   const std::size_t row_step, const std::size_t width,
                                   const std::size_t height, const std::size_t stride,
                                   const double gain) {
  std::vector<std::uint8_t> raw;
  if (stride == 0U || width < stride || height < stride) {
    return raw;
  }
  const std::size_t columns = width / stride;
  const std::size_t rows = height / stride;
  const double over = std::max(gain, 1.0);
  raw.assign(columns * rows, 0U);
  for (std::size_t row = 0U; row < rows; ++row) {
    for (std::size_t column = 0U; column < columns; ++column) {
      const std::size_t at = row * stride * row_step + column * stride;
      raw[row * columns + column] =
          at < grey.size()
              ? static_cast<std::uint8_t>(std::min(255.0, std::round(grey[at] / over)))
              : 0U;
    }
  }
  return raw;
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

bool FaceApproach::observe(const SilentPatch* candidate, const double along_m,
                           const SmokeFaceConfig& config) {
  halo_m_ = config.halo_m;
  const bool seen = candidate != nullptr && std::isfinite(candidate->rim_depth_m);
  const bool plume_like =
      seen && candidate->cells >= config.lay_cells &&
      static_cast<double>(candidate->free_neighbours) <=
          config.lay_see_through_share * static_cast<double>(candidate->cells);
  const bool front = plume_like &&
                     candidate->rim_spread_m <= config.front_rim_spread_m &&
                     candidate->rim_trend < config.front_rim_trend_share;
  front_frames_ = front ? front_frames_ + 1U : 0U;
  bool approach{false};
  if (!seen ||
      (!window_.empty() &&
       std::hypot(candidate->centre_row - centre_row_,
                  candidate->centre_column - centre_column_) > config.drift_cells)) {
    if (++lost_ > config.lost_frames) {
      window_.clear();
    }
  } else {
    lost_ = 0U;
    centre_row_ = candidate->centre_row;
    centre_column_ = candidate->centre_column;
    window_.emplace_back(along_m, candidate->rim_depth_m);
    while (window_.size() > config.window_frames ||
           window_.front().first < along_m - 2.0 * config.least_travel_m) {
      window_.pop_front();
    }
    double count{0.0};
    double sum_a{0.0};
    double sum_d{0.0};
    double sum_aa{0.0};
    double sum_ad{0.0};
    for (const auto& [travel_m, depth_m] : window_) {
      count += 1.0;
      sum_a += travel_m;
      sum_d += depth_m;
      sum_aa += travel_m * travel_m;
      sum_ad += travel_m * depth_m;
    }
    const double spread = count * sum_aa - sum_a * sum_a;
    const double span_m = along_m - window_.front().first;
    approach = span_m >= config.least_travel_m && spread > 1.0e-9 &&
               (count * sum_ad - sum_a * sum_d) / spread <= -config.shrink_slope;
  }
  const bool lay = plume_like && (approach || front_frames_ >= config.front_frames);
  if (lay) {
    laid_ = true;
    laid_lost_ = 0U;
    last_depth_m_ = candidate->rim_depth_m;
    last_along_m_ = along_m;
  } else if (laid_ && ++laid_lost_ > config.lost_frames) {
    laid_ = false;
  }
  return lay;
}

double FaceApproach::boundM(const double along_m) const noexcept {
  return laid_ ? std::max(0.0, last_depth_m_ - (along_m - last_along_m_) - halo_m_)
               : std::numeric_limits<double>::quiet_NaN();
}

std::vector<Point3> smokeFaceRays(const std::span<const SilentPatch> regions,
                                  const char* reason,
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
    if (!face.closed || face.rim_depth_m < nearest_m ||
        std::string_view{face.reason} != std::string_view{reason}) {
      continue;
    }
    const double from_m = face.rim_depth_m;
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

std::size_t dropEnclosedFreeRays(std::vector<StereoDepthReturn>& returns,
                                 const std::span<const SilentPatch> regions,
                                 const StereoPairGeometry& geometry,
                                 const std::size_t stride, const std::size_t width,
                                 const std::size_t height, const double carve_share) {
  if (stride == 0U || width < stride || height < stride) {
    return 0U;
  }
  std::vector<std::uint8_t> drop((width / stride) * (height / stride), 0U);
  bool any{false};
  for (const SilentPatch& region : regions) {
    if (region.cells == 0U || static_cast<double>(region.free_neighbours) >=
                                  carve_share * static_cast<double>(region.cells)) {
      continue;
    }
    for (const std::size_t cell : region.enclosed_free_cells) {
      if (cell < drop.size()) {
        drop[cell] = 1U;
        any = true;
      }
    }
  }
  if (!any) {
    return 0U;
  }
  const std::size_t before = returns.size();
  std::erase_if(returns, [&](const StereoDepthReturn& ray) {
    if (ray.hit || ray.unobservable) {
      return false;
    }
    const std::optional<std::size_t> cell =
        strideCellOf(ray.point, geometry, stride, width, height);
    return cell.has_value() && drop[*cell] != 0U;
  });
  return before - returns.size();
}

std::string silentPatchSummary(const SilentPatch& patch) {
  // The centroid on the stride grid, as column,row over 320 columns of a
  // 1280 px frame at the stride of 4: where in the frame the region stands.
  double row{0.0};
  double column{0.0};
  for (const std::size_t index : patch.cell_indices) {
    row += static_cast<double>(index / 320U);
    column += static_cast<double>(index % 320U);
  }
  const double n =
      std::max<double>(1.0, static_cast<double>(patch.cell_indices.size()));
  return std::to_string(patch.cells) + "@" + std::to_string(std::lround(column / n)) +
         "," + std::to_string(std::lround(row / n)) + ":" + patch.reason + ":rim" +
         std::to_string(patch.rim_hits) + "@" +
         std::to_string(patch.rim_depth_m).substr(0, 4) + "+" +
         std::to_string(patch.rim_spread_m).substr(0, 4) + ":trend" +
         std::to_string(patch.rim_trend).substr(0, 4) + ":free" +
         std::to_string(patch.free_neighbours);
}

SilentPatch* plumeCandidate(const std::span<SilentPatch> regions,
                            const SilentPatchConfig& config,
                            const SmokeFaceConfig& face_config,
                            const std::size_t columns, const std::size_t rows) {
  SilentPatch* candidate{nullptr};
  const auto width = static_cast<double>(columns);
  const auto height = static_cast<double>(rows);
  for (SilentPatch& region : regions) {
    const std::string_view reason{region.reason};
    const bool central = region.centre_column >= width / 8.0 &&
                         region.centre_column < 7.0 * width / 8.0 &&
                         region.centre_row >= height / 6.0 &&
                         region.centre_row < 2.0 * height / 3.0;
    const bool opaque =
        static_cast<double>(region.free_neighbours) <
        face_config.plume_see_through_share * static_cast<double>(region.cells);
    if (central && opaque && region.cells >= face_config.plume_cells &&
        region.rim_hits >= config.least_rim_hits && region.rim_depth_m > 0.0 &&
        region.rim_depth_m <= config.confident_depth_m && reason != "surface" &&
        reason != "body" && (candidate == nullptr || region.cells > candidate->cells)) {
      candidate = &region;
    }
  }
  return candidate;
}

double smokeFacesAheadM(const std::span<const SilentPatch> regions,
                        const bool rimmed_due, const bool body_due,
                        const StereoPairGeometry& geometry, const std::size_t stride,
                        const std::size_t stride_columns, const std::size_t stride_rows,
                        const double cone_half_angle_rad) {
  std::vector<SilentPatch> chosen;
  for (const SilentPatch& region : regions) {
    const std::string_view reason{region.reason};
    if (region.closed &&
        ((rimmed_due && reason == "plume") || (body_due && reason == "body"))) {
      chosen.push_back(region);
    }
  }
  return silentRangeAheadM(chosen, geometry, stride, stride_columns, stride_rows,
                           cone_half_angle_rad);
}

std::string smokeFacesLaid(const std::span<const SilentPatch> regions,
                           const char* reason, const double nearest_m) {
  std::string laid;
  for (const SilentPatch& region : regions) {
    if (region.closed && region.rim_depth_m >= nearest_m &&
        std::string_view{region.reason} == std::string_view{reason}) {
      laid += " " + silentPatchSummary(region);
    }
  }
  return laid;
}

std::vector<SilentPatch> smokeFaces(const std::span<const StereoDepthReturn> returns,
                                    const StereoPairGeometry& geometry,
                                    const StereoDepthReturnsConfig& returns_config,
                                    const std::size_t width, const std::size_t height,
                                    const std::span<const std::uint8_t> raw_cells,
                                    const SilentPatchConfig& config,
                                    const SmokeFaceConfig& face_config) {
  std::vector<SilentPatch> faces;
  const std::size_t stride = returns_config.pixel_stride;
  if (stride == 0U || width < stride || height < stride) {
    return faces;
  }
  const StrideGrid grid = strideGrid(returns, geometry, stride, width, height);
  if (raw_cells.size() != grid.kind.size()) {
    return faces;
  }
  const auto lit = [&](const std::size_t index) {
    return static_cast<double>(raw_cells[index]) >= face_config.lit_raw_grey;
  };
  // A free ray with no hit beside it is one the silence encloses (a match
  // through thin smoke), not the rim's own edge seen past the face.
  const auto enclosed = [&](const std::size_t index) {
    const long row = static_cast<long>(index / grid.columns);
    const long column = static_cast<long>(index % grid.columns);
    for (long d_row = -1; d_row <= 1; ++d_row) {
      for (long d_column = -1; d_column <= 1; ++d_column) {
        const long n_row = row + d_row;
        const long n_column = column + d_column;
        if (n_row < 0 || n_column < 0 || n_row >= static_cast<long>(grid.rows) ||
            n_column >= static_cast<long>(grid.columns)) {
          continue;
        }
        if (grid.kind[grid.at(static_cast<std::size_t>(n_row),
                              static_cast<std::size_t>(n_column))] == kHit) {
          return false;
        }
      }
    }
    return true;
  };
  std::vector<std::uint32_t> label(grid.kind.size(), 0U);
  std::vector<std::size_t> stack;
  std::uint32_t next_label{1U};
  for (std::size_t seed = 0U; seed < label.size(); ++seed) {
    if (grid.kind[seed] != kSilent || !lit(seed) || label[seed] != 0U) {
      continue;
    }
    // One region of lit silence by four-connectivity; its neighbours: the
    // hits are its rim, the enclosed free rays its see-through, the dark
    // silence its darkness.
    std::vector<std::size_t> cells;
    std::vector<double> rim_depths;
    std::vector<std::size_t> rim_cells;
    std::vector<std::uint8_t> seen(label.size(), 0U);
    std::size_t free_neighbours{0U};
    std::vector<std::size_t> enclosed_free;
    std::size_t dark_neighbours{0U};
    std::size_t neighbours{0U};
    bool at_border{false};
    stack.assign(1U, seed);
    label[seed] = next_label;
    while (!stack.empty()) {
      const std::size_t index = stack.back();
      stack.pop_back();
      cells.push_back(index);
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
        if (grid.kind[neighbour] == kSilent && lit(neighbour)) {
          if ((d_row == 0 || d_column == 0) && label[neighbour] == 0U) {
            label[neighbour] = next_label;
            stack.push_back(neighbour);
          }
        } else if (seen[neighbour] == 0U) {
          seen[neighbour] = 1U;
          ++neighbours;
          if (grid.kind[neighbour] == kHit) {
            rim_depths.push_back(grid.depth_m[neighbour]);
            rim_cells.push_back(neighbour);
          } else if (grid.kind[neighbour] == kFree) {
            if (enclosed(neighbour)) {
              ++free_neighbours;
              enclosed_free.push_back(neighbour);
            }
          } else {
            ++dark_neighbours;
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
    for (const std::size_t index : cells) {
      face.centre_row += static_cast<double>(index / grid.columns);
      face.centre_column += static_cast<double>(index % grid.columns);
    }
    face.centre_row /= static_cast<double>(cells.size());
    face.centre_column /= static_cast<double>(cells.size());
    const double see_through =
        static_cast<double>(free_neighbours) / static_cast<double>(cells.size());
    std::vector<double> own;
    own.reserve(cells.size());
    for (const std::size_t index : cells) {
      own.push_back(static_cast<double>(raw_cells[index]));
    }
    const double brightness = quantile(own, 0.5);
    if (rim_depths.size() >= config.least_rim_hits) {
      face.rim_trend = rimTrendShare(rim_cells, rim_depths, grid.columns);
      face.rim_depth_m = quantile(rim_depths, 0.25);
      face.rim_spread_m = quantile(rim_depths, 0.75) - quantile(rim_depths, 0.25);
      std::vector<double> centre_depths;
      for (std::size_t i = 0U; i < rim_cells.size(); ++i) {
        const auto row = static_cast<double>(rim_cells[i] / grid.columns);
        if (std::abs(row - static_cast<double>(grid.rows) / 2.0) <=
            static_cast<double>(grid.rows) / 12.0) {
          centre_depths.push_back(rim_depths[i]);
        }
      }
      if (centre_depths.size() >= face_config.centre_rim_hits) {
        face.centre_rim_depth_m = quantile(centre_depths, 0.25);
      }
    }
    if (brightness >= face_config.surface_raw_grey) {
      face.reason = "surface";
    } else if (see_through >= face_config.see_through_share) {
      face.reason = "seen_through";
    } else if (rim_depths.size() >= config.least_rim_hits) {
      // The sectors round the region's centre the rim reaches into, as a
      // patch's.
      std::array<bool, 8> sectors{};
      for (const std::size_t index : rim_cells) {
        const double angle =
            std::atan2(static_cast<double>(index / grid.columns) - face.centre_row,
                       static_cast<double>(index % grid.columns) - face.centre_column);
        sectors[static_cast<std::size_t>(
                    std::floor((angle + std::numbers::pi) / (std::numbers::pi / 4.0))) %
                8U] = true;
      }
      const auto rim_sectors =
          static_cast<std::size_t>(std::ranges::count(sectors, true));
      const double trend = face.rim_trend;
      if (rim_sectors < face_config.least_rim_sectors) {
        face.reason = "rim_not_round";
      } else if (quantile(rim_depths, 0.5) > config.confident_depth_m) {
        face.reason = "rim_beyond_reach";
      } else if (face.rim_spread_m > face_config.rim_scatter_m ||
                 trend >= face_config.rim_trend_share) {
        face.reason = "rim_recedes";
      } else {
        face.closed = true;
        face.reason = "face";
      }
    } else if (at_border) {
      face.reason = "rim_unmatched";
    } else if (static_cast<double>(dark_neighbours) <
               face_config.dark_ring_share *
                   static_cast<double>(std::max<std::size_t>(1U, neighbours))) {
      face.reason = "rim_unmatched";
    } else {
      // A lit body in the dark: its edge, the silence two to three cells out
      // against the region's own brightness.
      std::vector<std::uint8_t> distance(label.size(), 0U);
      for (const std::size_t index : cells) {
        const long row = static_cast<long>(index / grid.columns);
        const long column = static_cast<long>(index % grid.columns);
        for (long d_row = -3; d_row <= 3; ++d_row) {
          for (long d_column = -3; d_column <= 3; ++d_column) {
            const long n_row = row + d_row;
            const long n_column = column + d_column;
            if (n_row < 0 || n_column < 0 || n_row >= static_cast<long>(grid.rows) ||
                n_column >= static_cast<long>(grid.columns)) {
              continue;
            }
            const std::size_t neighbour = grid.at(static_cast<std::size_t>(n_row),
                                                  static_cast<std::size_t>(n_column));
            const auto reach = static_cast<std::uint8_t>(
                std::max(std::labs(d_row), std::labs(d_column)));
            if (label[neighbour] == next_label - 1U) {
              continue;
            }
            if (distance[neighbour] == 0U || reach < distance[neighbour]) {
              distance[neighbour] = reach;
            }
          }
        }
      }
      std::vector<double> shell;
      for (std::size_t index = 0U; index < distance.size(); ++index) {
        if (distance[index] == 3U && grid.kind[index] == kSilent) {
          shell.push_back(static_cast<double>(raw_cells[index]));
        }
      }
      if (shell.empty() ||
          quantile(shell, 0.5) > face_config.shell_ratio * brightness) {
        face.reason = "soft_edge";
      } else {
        face.closed = true;
        face.reason = "body";
        face.rim_depth_m = brightness >= face_config.near_grey  ? face_config.near_m
                           : brightness >= face_config.mid_grey ? face_config.mid_m
                                                                : face_config.far_m;
      }
    }
    face.cell_indices = std::move(cells);
    face.enclosed_free_cells = std::move(enclosed_free);
    faces.push_back(face);
  }
  return faces;
}

} // namespace drone_city_nav
