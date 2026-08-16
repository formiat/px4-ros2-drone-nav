#pragma once

#include "drone_city_nav/types.hpp"

#include <cmath>
#include <stdexcept>
#include <string_view>

namespace drone_city_nav {

enum class MapHorizontalAxis {
  kX,
  kY,
};

[[nodiscard]] inline MapHorizontalAxis
mapHorizontalAxisFromName(const std::string_view name) {
  if (name == "map_x") {
    return MapHorizontalAxis::kX;
  }
  if (name == "map_y") {
    return MapHorizontalAxis::kY;
  }
  throw std::invalid_argument{"map axis must be map_x or map_y"};
}

struct MapToSdfTransform {
  MapHorizontalAxis sdf_x_from{MapHorizontalAxis::kY};
  MapHorizontalAxis sdf_y_from{MapHorizontalAxis::kX};
  double sdf_x_scale{1.0};
  double sdf_y_scale{1.0};
  double sdf_z_scale{1.0};
  double sdf_x_offset_m{-225.0};
  double sdf_y_offset_m{-135.0};
  double sdf_z_offset_m{0.0};

  void validate() const {
    if (sdf_x_from == sdf_y_from) {
      throw std::invalid_argument{"SDF X and Y must use distinct map axes"};
    }
    if (!validUnitScale(sdf_x_scale) || !validUnitScale(sdf_y_scale) ||
        !validUnitScale(sdf_z_scale)) {
      throw std::invalid_argument{"map-to-SDF scales must be +1 or -1"};
    }
    if (!std::isfinite(sdf_x_offset_m) || !std::isfinite(sdf_y_offset_m) ||
        !std::isfinite(sdf_z_offset_m)) {
      throw std::invalid_argument{"map-to-SDF offsets must be finite"};
    }
  }

  [[nodiscard]] Point3 mapToSdf(const Point3& map) const noexcept {
    const Point3 vector = mapVectorToSdf(map);
    return Point3{vector.x + sdf_x_offset_m, vector.y + sdf_y_offset_m,
                  vector.z + sdf_z_offset_m};
  }

  [[nodiscard]] Point3 sdfToMap(const Point3& sdf) const noexcept {
    return sdfVectorToMap(
        Point3{sdf.x - sdf_x_offset_m, sdf.y - sdf_y_offset_m, sdf.z - sdf_z_offset_m});
  }

  [[nodiscard]] Point3 mapVectorToSdf(const Point3& map) const noexcept {
    return Point3{sdf_x_scale * horizontal(map, sdf_x_from),
                  sdf_y_scale * horizontal(map, sdf_y_from), sdf_z_scale * map.z};
  }

  [[nodiscard]] Point3 sdfVectorToMap(const Point3& sdf) const noexcept {
    Point3 map{};
    horizontal(map, sdf_x_from) = sdf.x / sdf_x_scale;
    horizontal(map, sdf_y_from) = sdf.y / sdf_y_scale;
    map.z = sdf.z / sdf_z_scale;
    return map;
  }

private:
  [[nodiscard]] static bool validUnitScale(const double scale) noexcept {
    return std::isfinite(scale) && std::abs(std::abs(scale) - 1.0) <= 1.0e-9;
  }

  [[nodiscard]] static double horizontal(const Point3& point,
                                         const MapHorizontalAxis axis) noexcept {
    return axis == MapHorizontalAxis::kX ? point.x : point.y;
  }

  [[nodiscard]] static double& horizontal(Point3& point,
                                          const MapHorizontalAxis axis) noexcept {
    return axis == MapHorizontalAxis::kX ? point.x : point.y;
  }
};

} // namespace drone_city_nav
