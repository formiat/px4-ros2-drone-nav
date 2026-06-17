#pragma once

#include "drone_city_nav/debug_image.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace drone_city_nav {

struct ImagePixel {
  int x{0};
  int y{0};
};

struct GridImageView {
  int width{0};
  int height{0};
  double resolution_m{0.0};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  std::span<const std::int8_t> cells{};
};

struct LidarDebugRenderConfig {
  int image_size_px{900};
  double fallback_view_radius_m{45.0};
  Pixel background{12U, 16U, 20U};
  Pixel grid_occupied_color{0U, 220U, 95U};
  Pixel grid_inflated_color{120U, 92U, 28U};
  Pixel remembered_hit_color{255U, 218U, 75U};
  Pixel current_hit_color{255U, 60U, 60U};
  Pixel path_line_color{85U, 220U, 255U};
  Pixel path_waypoint_color{75U, 255U, 190U};
  Pixel drone_color{90U, 145U, 255U};
  Pixel drone_heading_color{235U, 245U, 255U};
};

struct LidarDebugFrame {
  Point2 drone_position{};
  Point2 heading_direction{1.0, 0.0};
  std::optional<GridImageView> grid;
  std::span<const Point2> path{};
  std::span<const Point2> current_hits{};
  std::span<const Point2> remembered_hits{};
};

[[nodiscard]] std::optional<ImagePixel>
worldToDebugImagePixel(Point2 point, const LidarDebugRenderConfig& config,
                       const LidarDebugFrame& frame) noexcept;

[[nodiscard]] DebugImage renderLidarDebugImage(const LidarDebugRenderConfig& config,
                                               const LidarDebugFrame& frame);

} // namespace drone_city_nav
