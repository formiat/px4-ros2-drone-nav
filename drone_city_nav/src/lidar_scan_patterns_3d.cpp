#include "drone_city_nav/lidar_scan_patterns_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

double quantile(std::vector<double>& values, const double share) {
  std::ranges::sort(values);
  return values[static_cast<std::size_t>(share *
                                         static_cast<double>(values.size() - 1U))];
}

constexpr std::array<std::pair<long, long>, 8> kAround{
    {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}}};

} // namespace

LidarScanPatternsResult
classifyLidarScanPatterns3D(OrganizedLidarScan3DResult& scan,
                            const std::span<const float> intensities,
                            const OrganizedLidarScan3DConfig& scan_config,
                            const LidarScanPatternsConfig& config) {
  LidarScanPatternsResult result;
  const std::size_t columns = scan_config.horizontal_samples;
  const std::size_t rows = scan_config.vertical_samples;
  if (columns == 0U || rows == 0U || scan.beams.size() != columns * rows) {
    return result;
  }
  const auto at = [columns](const std::size_t row, const std::size_t column) {
    return row * columns + column;
  };
  // The horizontal scan wraps round; the vertical does not.
  const auto neighbour =
      [&](const std::size_t index,
          const std::pair<long, long>& step) -> std::optional<std::size_t> {
    const long row = static_cast<long>(index / columns) + step.first;
    if (row < 0 || row >= static_cast<long>(rows)) {
      return std::nullopt;
    }
    const long column = (static_cast<long>(index % columns) + step.second +
                         static_cast<long>(columns)) %
                        static_cast<long>(columns);
    return at(static_cast<std::size_t>(row), static_cast<std::size_t>(column));
  };

  // Aerosol: weak hits among weak hits near the same range.
  if (intensities.size() == scan.beams.size()) {
    std::vector<double> surface;
    for (std::size_t index = 0U; index < scan.beams.size(); ++index) {
      if (scan.beams[index].hit && std::isfinite(intensities[index])) {
        surface.push_back(intensities[index]);
      }
    }
    if (!surface.empty()) {
      const double weak_under =
          config.weak_intensity_share *
          std::max(quantile(surface, 0.5), config.matte_surface_intensity);
      std::vector<std::uint8_t> weak(scan.beams.size(), 0U);
      for (std::size_t index = 0U; index < scan.beams.size(); ++index) {
        weak[index] = scan.beams[index].hit && std::isfinite(intensities[index]) &&
                              intensities[index] < weak_under
                          ? 1U
                          : 0U;
      }
      std::vector<std::uint8_t> aerosol(scan.beams.size(), 0U);
      for (std::size_t index = 0U; index < scan.beams.size(); ++index) {
        if (weak[index] == 0U) {
          continue;
        }
        std::size_t alike{0U};
        for (const auto& step : kAround) {
          const std::optional<std::size_t> other = neighbour(index, step);
          if (other.has_value() && weak[*other] != 0U &&
              std::abs(scan.beams[*other].range_m - scan.beams[index].range_m) <=
                  config.aerosol_range_band_m) {
            ++alike;
          }
        }
        aerosol[index] = alike >= config.aerosol_least_neighbours ? 1U : 0U;
      }
      for (std::size_t index = 0U; index < scan.beams.size(); ++index) {
        if (aerosol[index] == 0U) {
          continue;
        }
        LidarBeamSample3D& beam = scan.beams[index];
        beam.range_m = std::max(beam.range_m, config.aerosol_least_range_m);
        scan.unobservable.push_back(
            LidarBeamSample3D{.direction_lidar_flu = beam.direction_lidar_flu,
                              .range_m = beam.range_m,
                              .hit = true,
                              .valid = true,
                              .interpolated = true});
        beam.hit = false;
        --scan.hit_beams;
        ++scan.miss_beams;
        ++result.aerosol_beams;
      }
    }
  }

  // Dropout: bundles of no-return beams with a near, flat border of hits.
  std::vector<std::uint32_t> label(scan.beams.size(), 0U);
  std::vector<std::size_t> stack;
  std::uint32_t next_label{1U};
  const auto no_return = [&](const std::size_t index) {
    const LidarBeamSample3D& beam = scan.beams[index];
    return beam.valid && !beam.hit && beam.range_m >= scan_config.maximum_range_m;
  };
  for (std::size_t seed = 0U; seed < scan.beams.size(); ++seed) {
    if (!no_return(seed) || label[seed] != 0U) {
      continue;
    }
    std::vector<std::size_t> bundle;
    std::vector<std::size_t> border;
    std::vector<std::uint8_t> border_seen(scan.beams.size(), 0U);
    stack.assign(1U, seed);
    label[seed] = next_label;
    bool too_large{false};
    while (!stack.empty()) {
      const std::size_t index = stack.back();
      stack.pop_back();
      bundle.push_back(index);
      if (bundle.size() > config.dropout_most_beams) {
        too_large = true;
      }
      for (const auto& step : kAround) {
        const std::optional<std::size_t> other = neighbour(index, step);
        if (!other.has_value()) {
          continue;
        }
        if (no_return(*other)) {
          if ((step.first == 0 || step.second == 0) && label[*other] == 0U) {
            label[*other] = next_label;
            stack.push_back(*other);
          }
        } else if (border_seen[*other] == 0U) {
          border_seen[*other] = 1U;
          border.push_back(*other);
        }
      }
    }
    ++next_label;
    if (too_large || border.empty()) {
      continue;
    }
    std::vector<double> border_ranges;
    for (const std::size_t index : border) {
      if (scan.beams[index].valid && scan.beams[index].hit) {
        border_ranges.push_back(scan.beams[index].range_m);
      }
    }
    if (static_cast<double>(border_ranges.size()) <
        config.dropout_border_hit_share * static_cast<double>(border.size())) {
      continue;
    }
    const double median = quantile(border_ranges, 0.5);
    if (median > config.dropout_border_range_m ||
        quantile(border_ranges, 0.75) - quantile(border_ranges, 0.25) >
            config.dropout_border_spread_m) {
      continue;
    }
    for (const std::size_t index : bundle) {
      LidarBeamSample3D& beam = scan.beams[index];
      beam.range_m = median;
      scan.unobservable.push_back(
          LidarBeamSample3D{.direction_lidar_flu = beam.direction_lidar_flu,
                            .range_m = median,
                            .hit = true,
                            .valid = true,
                            .interpolated = true});
    }
    ++result.dropout_bundles;
    result.dropout_beams += bundle.size();
  }
  return result;
}

} // namespace drone_city_nav
