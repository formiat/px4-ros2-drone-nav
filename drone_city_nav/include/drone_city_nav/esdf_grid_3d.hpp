#pragma once

namespace drone_city_nav {

// Controller-neutral geometry for a dense derived distance field. Depth one
// represents a planar field; larger depths represent a full 3D field.
inline constexpr float kUnknownEsdfDistanceM{-1.0F};

struct EsdfGrid3D {
  int width{0};
  int height{0};
  float resolution_m{0.0F};
  float origin_x_m{0.0F};
  float origin_y_m{0.0F};
  int depth{1};
  float origin_z_m{0.0F};
  bool outside_is_unknown{false};
};

struct EsdfDirtyRegion3D {
  int minimum_x{0};
  int minimum_y{0};
  int minimum_z{0};
  int maximum_x_exclusive{0};
  int maximum_y_exclusive{0};
  int maximum_z_exclusive{0};
};

} // namespace drone_city_nav
