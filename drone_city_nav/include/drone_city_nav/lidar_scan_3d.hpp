#pragma once

#include "drone_city_nav/obstacle_memory_3d.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace drone_city_nav {

struct LidarBeamSample3D {
  Vec3 direction_lidar_flu{};
  double range_m{0.0};
  bool hit{false};
  bool valid{false};
  // A surface sample reconstructed between two adjacent beam returns rather
  // than measured by a beam of its own: it carries occupied evidence at its
  // endpoint only, never free-space evidence along a ray.
  bool interpolated{false};
};

struct OrganizedLidarScan3DConfig {
  std::size_t horizontal_samples{240U};
  std::size_t vertical_samples{19U};
  double horizontal_min_angle_rad{-3.14159265358979323846};
  double horizontal_max_angle_rad{3.14159265358979323846};
  double vertical_min_angle_rad{-1.5707963267948966192};
  double vertical_max_angle_rad{1.5707963267948966192};
  double minimum_range_m{0.2};
  double maximum_range_m{35.0};
  double hit_epsilon_m{0.05};
};

struct OrganizedLidarScan3DResult {
  std::vector<LidarBeamSample3D> beams;
  std::size_t hit_beams{0U};
  std::size_t miss_beams{0U};
  std::size_t invalid_beams{0U};
  bool organized_dimensions_match{false};
};

[[nodiscard]] bool
organizedLidarScan3DConfigIsValid(const OrganizedLidarScan3DConfig& config) noexcept;

[[nodiscard]] OrganizedLidarScan3DResult
decodeOrganizedLidarScan3D(std::span<const Point3> returns_lidar_flu,
                           const OrganizedLidarScan3DConfig& config);

// Surface continuity between adjacent beams of an organized scan. A sparse
// beam layout samples a wall as rows and columns of returns whose spacing
// grows with range; the band between two rows, or the strip between two
// columns of a wall seen at a grazing angle, stays unknown until the sensor
// is close, although the wall is continuous there, and a vehicle crossing
// that gap meets the wall the map does not show. Two adjacent returns are
// joined by surface samples spaced `sample_spacing_m` apart when their range
// step fits one surface viewed within an incidence limit of its normal.
//
// Vertical neighbours use `maximum_incidence_rad`, kept tight, and their
// joining segment must rise at least `minimum_vertical_fraction` of its
// length: a wall answers a column of beams with a vertical segment and a range
// step of a few percent, while a ceiling or floor in front of a wall answers
// with a much larger step, and joining those two returns would draw a ramp
// through free volume. Horizontal neighbours use `maximum_row_incidence_rad`,
// kept wide: a row of beams meets a wall at grazing incidence at the very
// place where the columns are sparse, and a depth discontinuity between two
// columns steps by the whole gap behind the edge, far beyond any incidence.
struct LidarSurfaceInterpolation3DConfig {
  bool enabled{true};
  double maximum_incidence_rad{0.5235987755982988};
  double maximum_row_incidence_rad{1.3962634015954636};
  double minimum_vertical_fraction{0.5};
  double sample_spacing_m{0.175};
};

[[nodiscard]] bool lidarSurfaceInterpolation3DConfigIsValid(
    const LidarSurfaceInterpolation3DConfig& config) noexcept;

[[nodiscard]] std::vector<LidarBeamSample3D>
interpolateOrganizedLidarSurfaces3D(std::span<const LidarBeamSample3D> beams,
                                    const OrganizedLidarScan3DConfig& scan_config,
                                    const LidarSurfaceInterpolation3DConfig& config);

} // namespace drone_city_nav
