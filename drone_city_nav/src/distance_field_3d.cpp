#include "drone_city_nav/distance_field_3d.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kLargeSquaredDistance = 1.0e20;

[[nodiscard]] std::size_t checkedVoxelCount(const GridBounds3D& bounds) {
  const auto width = static_cast<std::size_t>(bounds.width_cells);
  const auto height = static_cast<std::size_t>(bounds.height_cells);
  const auto depth = static_cast<std::size_t>(bounds.depth_cells);
  if (height != 0U && width > std::numeric_limits<std::size_t>::max() / height) {
    throw std::overflow_error{"Occupancy3D dimensions overflow"};
  }
  const std::size_t plane = width * height;
  if (depth != 0U && plane > std::numeric_limits<std::size_t>::max() / depth) {
    throw std::overflow_error{"Occupancy3D dimensions overflow"};
  }
  return plane * depth;
}

void transform1D(const std::vector<double>& input, std::vector<double>& output) {
  const int size = static_cast<int>(input.size());
  std::vector<int> locations(static_cast<std::size_t>(size), 0);
  std::vector<double> boundaries(static_cast<std::size_t>(size + 1), 0.0);
  int envelope_size = 0;
  locations[0] = 0;
  boundaries[0] = -kInfinity;
  boundaries[1] = kInfinity;
  for (int candidate = 1; candidate < size; ++candidate) {
    double intersection = 0.0;
    while (envelope_size >= 0) {
      const int location = locations[static_cast<std::size_t>(envelope_size)];
      const double candidate_value = static_cast<double>(candidate);
      const double location_value = static_cast<double>(location);
      intersection = ((input[static_cast<std::size_t>(candidate)] +
                       candidate_value * candidate_value) -
                      (input[static_cast<std::size_t>(location)] +
                       location_value * location_value)) /
                     (2.0 * static_cast<double>(candidate - location));
      if (intersection > boundaries[static_cast<std::size_t>(envelope_size)]) {
        break;
      }
      --envelope_size;
    }
    if (envelope_size < 0) {
      envelope_size = 0;
      locations[0] = candidate;
      boundaries[0] = -kInfinity;
      boundaries[1] = kInfinity;
    } else {
      ++envelope_size;
      locations[static_cast<std::size_t>(envelope_size)] = candidate;
      boundaries[static_cast<std::size_t>(envelope_size)] = intersection;
      boundaries[static_cast<std::size_t>(envelope_size) + 1U] = kInfinity;
    }
  }
  output.resize(input.size());
  envelope_size = 0;
  for (int position = 0; position < size; ++position) {
    while (boundaries[static_cast<std::size_t>(envelope_size) + 1U] <
           static_cast<double>(position)) {
      ++envelope_size;
    }
    const int location = locations[static_cast<std::size_t>(envelope_size)];
    const double delta = static_cast<double>(position - location);
    output[static_cast<std::size_t>(position)] =
        delta * delta + input[static_cast<std::size_t>(location)];
  }
}

} // namespace

DistanceField3D DistanceField3D::build(const OccupancyGrid3D& occupancy,
                                       const double maximum_distance_m) {
  return buildLocal(occupancy, occupancy.bounds(), maximum_distance_m);
}

DistanceField3D DistanceField3D::buildLocal(const OccupancyGrid3D& occupancy,
                                            const GridBounds3D& local_bounds,
                                            const double maximum_distance_m) {
  const auto started = std::chrono::steady_clock::now();
  DistanceField3D field;
  if (!(local_bounds.resolution_m > 0.0) || local_bounds.width_cells <= 0 ||
      local_bounds.height_cells <= 0 || local_bounds.depth_cells <= 0 ||
      std::abs(local_bounds.resolution_m - occupancy.bounds().resolution_m) > 1.0e-9) {
    throw std::invalid_argument{"invalid local DistanceField3D bounds"};
  }
  field.bounds_ = local_bounds;
  field.maximum_distance_m_ = maximum_distance_m;
  const std::size_t voxel_count = checkedVoxelCount(field.bounds_);
  field.stats_.voxel_count = voxel_count;
  std::vector<float> squared(voxel_count, static_cast<float>(kLargeSquaredDistance));
  const int width = field.bounds_.width_cells;
  const int height = field.bounds_.height_cells;
  const int depth = field.bounds_.depth_cells;
  const auto index = [width, height](const int x, const int y, const int z) {
    return (static_cast<std::size_t>(z) * static_cast<std::size_t>(height) +
            static_cast<std::size_t>(y)) *
               static_cast<std::size_t>(width) +
           static_cast<std::size_t>(x);
  };
  const int maximum_axis = std::max({width, height, depth});
  std::vector<double> input(static_cast<std::size_t>(maximum_axis),
                            kLargeSquaredDistance);
  std::vector<double> output;

  const auto x_pass_started = std::chrono::steady_clock::now();
  input.resize(static_cast<std::size_t>(width));
  for (int z = 0; z < depth; ++z) {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const std::optional<GridIndex3D> source_cell = occupancy.worldToCell(
            Point3{field.bounds_.origin_x +
                       (static_cast<double>(x) + 0.5) * field.bounds_.resolution_m,
                   field.bounds_.origin_y +
                       (static_cast<double>(y) + 0.5) * field.bounds_.resolution_m,
                   field.bounds_.origin_z +
                       (static_cast<double>(z) + 0.5) * field.bounds_.resolution_m});
        const bool occupied =
            source_cell.has_value() && occupancy.isOccupied(*source_cell);
        input[static_cast<std::size_t>(x)] = occupied ? 0.0 : kLargeSquaredDistance;
        field.stats_.source_voxels += occupied ? 1U : 0U;
      }
      transform1D(input, output);
      for (int x = 0; x < width; ++x) {
        squared[index(x, y, z)] =
            static_cast<float>(output[static_cast<std::size_t>(x)]);
      }
    }
  }
  field.stats_.x_pass_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - x_pass_started)
                               .count();

  const auto y_pass_started = std::chrono::steady_clock::now();
  input.resize(static_cast<std::size_t>(height));
  for (int z = 0; z < depth; ++z) {
    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
        input[static_cast<std::size_t>(y)] =
            static_cast<double>(squared[index(x, y, z)]);
      }
      transform1D(input, output);
      for (int y = 0; y < height; ++y) {
        squared[index(x, y, z)] =
            static_cast<float>(output[static_cast<std::size_t>(y)]);
      }
    }
  }
  field.stats_.y_pass_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - y_pass_started)
                               .count();

  const auto z_pass_started = std::chrono::steady_clock::now();
  input.resize(static_cast<std::size_t>(depth));
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      for (int z = 0; z < depth; ++z) {
        input[static_cast<std::size_t>(z)] =
            static_cast<double>(squared[index(x, y, z)]);
      }
      transform1D(input, output);
      for (int z = 0; z < depth; ++z) {
        squared[index(x, y, z)] =
            static_cast<float>(output[static_cast<std::size_t>(z)]);
      }
    }
  }
  field.stats_.z_pass_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - z_pass_started)
                               .count();

  const auto finalize_started = std::chrono::steady_clock::now();
  field.distances_m_.resize(voxel_count);
  const bool capped = std::isfinite(maximum_distance_m) && maximum_distance_m > 0.0;
  for (std::size_t voxel = 0U; voxel < voxel_count; ++voxel) {
    const double squared_cells = static_cast<double>(squared[voxel]);
    if (field.stats_.source_voxels == 0U ||
        !(squared_cells < kLargeSquaredDistance * 0.5)) {
      field.distances_m_[voxel] = std::numeric_limits<float>::infinity();
      continue;
    }
    const double distance_m = std::sqrt(squared_cells) * field.bounds_.resolution_m;
    field.distances_m_[voxel] = capped && distance_m > maximum_distance_m
                                    ? std::numeric_limits<float>::infinity()
                                    : static_cast<float>(distance_m);
  }
  field.stats_.finalize_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - finalize_started)
                                 .count();
  field.stats_.duration_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
  return field;
}

const GridBounds3D& DistanceField3D::bounds() const noexcept {
  return bounds_;
}

double DistanceField3D::maximumDistanceM() const noexcept {
  return maximum_distance_m_;
}

bool DistanceField3D::contains(const GridIndex3D index) const noexcept {
  return index.x >= 0 && index.y >= 0 && index.z >= 0 &&
         index.x < bounds_.width_cells && index.y < bounds_.height_cells &&
         index.z < bounds_.depth_cells;
}

std::size_t DistanceField3D::linearIndex(const GridIndex3D index) const {
  if (!contains(index)) {
    throw std::out_of_range{"DistanceField3D index outside grid"};
  }
  return (static_cast<std::size_t>(index.z) *
              static_cast<std::size_t>(bounds_.height_cells) +
          static_cast<std::size_t>(index.y)) *
             static_cast<std::size_t>(bounds_.width_cells) +
         static_cast<std::size_t>(index.x);
}

float DistanceField3D::distanceAt(const GridIndex3D index) const {
  return distances_m_.at(linearIndex(index));
}

std::span<const float> DistanceField3D::distancesM() const noexcept {
  return distances_m_;
}

const DistanceField3DBuildStats& DistanceField3D::stats() const noexcept {
  return stats_;
}

} // namespace drone_city_nav
