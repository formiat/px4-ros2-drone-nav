#include "drone_city_nav/lidar_debug_renderer.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav {
namespace {

constexpr double kImagePaddingPx = 8.0;

[[nodiscard]] bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] std::optional<ImagePixel>
gridWorldToPixel(const Point2 point, const int image_size_px,
                 const GridImageView& grid) noexcept {
  if (!finite2D(point)) {
    return std::nullopt;
  }

  const double width_m = static_cast<double>(grid.width) * grid.resolution_m;
  const double height_m = static_cast<double>(grid.height) * grid.resolution_m;
  if (!(grid.resolution_m > 0.0) || !(width_m > 0.0) || !(height_m > 0.0)) {
    return std::nullopt;
  }

  const double usable_px =
      std::max(1.0, static_cast<double>(image_size_px - 1) - 2.0 * kImagePaddingPx);
  const double scale = std::min(usable_px / width_m, usable_px / height_m);
  const double drawn_width_px = width_m * scale;
  const double drawn_height_px = height_m * scale;
  const double offset_x =
      (static_cast<double>(image_size_px - 1) - drawn_width_px) * 0.5;
  const double offset_y =
      (static_cast<double>(image_size_px - 1) - drawn_height_px) * 0.5;
  const double local_x = point.x - grid.origin_x_m;
  const double local_y = point.y - grid.origin_y_m;
  const int x = static_cast<int>(std::lround(offset_x + local_x * scale));
  const int y = static_cast<int>(std::lround(static_cast<double>(image_size_px - 1) -
                                             (offset_y + local_y * scale)));
  if (x < 0 || y < 0 || x >= image_size_px || y >= image_size_px) {
    return std::nullopt;
  }
  return ImagePixel{x, y};
}

[[nodiscard]] std::optional<ImagePixel>
droneCenteredWorldToPixel(const Point2 point, const LidarDebugRenderConfig& config,
                          const Point2 drone_position) noexcept {
  if (!finite2D(point) || !finite2D(drone_position) ||
      !(config.fallback_view_radius_m > 0.0)) {
    return std::nullopt;
  }

  const double scale = static_cast<double>(std::max(1, config.image_size_px)) /
                       (2.0 * config.fallback_view_radius_m);
  const double center = static_cast<double>(std::max(1, config.image_size_px)) * 0.5;
  const int x =
      static_cast<int>(std::lround(center + (point.x - drone_position.x) * scale));
  const int y =
      static_cast<int>(std::lround(center - (point.y - drone_position.y) * scale));
  if (x < 0 || y < 0 || x >= config.image_size_px || y >= config.image_size_px) {
    return std::nullopt;
  }
  return ImagePixel{x, y};
}

void drawPoints(DebugImage& image, const std::span<const Point2> points,
                const LidarDebugRenderConfig& config, const LidarDebugFrame& frame,
                const int radius, const Pixel color) {
  for (const Point2 point : points) {
    const auto pixel = worldToDebugImagePixel(point, config, frame);
    if (pixel.has_value()) {
      drawDisc(image, pixel->x, pixel->y, radius, color);
    }
  }
}

void drawPath(DebugImage& image, const LidarDebugRenderConfig& config,
              const LidarDebugFrame& frame) {
  if (frame.path.empty()) {
    return;
  }

  for (std::size_t i = 1U; i < frame.path.size(); ++i) {
    const auto from_pixel = worldToDebugImagePixel(frame.path[i - 1U], config, frame);
    const auto to_pixel = worldToDebugImagePixel(frame.path[i], config, frame);
    if (from_pixel.has_value() && to_pixel.has_value()) {
      drawLine(image, from_pixel->x, from_pixel->y, to_pixel->x, to_pixel->y,
               config.path_line_color);
    }
  }

  drawPoints(image, frame.path, config, frame, 3, config.path_waypoint_color);
}

void drawDrone(DebugImage& image, const LidarDebugRenderConfig& config,
               const LidarDebugFrame& frame) {
  const auto pixel = worldToDebugImagePixel(frame.drone_position, config, frame);
  if (!pixel.has_value()) {
    return;
  }

  drawDisc(image, pixel->x, pixel->y, 5, config.drone_color);

  const int nose_x =
      pixel->x + static_cast<int>(std::lround(14.0 * frame.heading_direction.x));
  const int nose_y =
      pixel->y - static_cast<int>(std::lround(14.0 * frame.heading_direction.y));
  drawLine(image, pixel->x, pixel->y, nose_x, nose_y, config.drone_heading_color);
}

} // namespace

std::optional<ImagePixel>
worldToDebugImagePixel(const Point2 point, const LidarDebugRenderConfig& config,
                       const LidarDebugFrame& frame) noexcept {
  const int image_size_px = std::max(1, config.image_size_px);
  if (frame.grid.has_value()) {
    return gridWorldToPixel(point, image_size_px, *frame.grid);
  }
  return droneCenteredWorldToPixel(point, config, frame.drone_position);
}

DebugImage renderLidarDebugImage(const LidarDebugRenderConfig& config,
                                 const LidarDebugFrame& frame) {
  const int image_size_px = std::max(1, config.image_size_px);
  DebugImage image{image_size_px, image_size_px, config.background};
  drawPoints(image, frame.remembered_hits, config, frame, 1,
             config.remembered_hit_color);
  drawPath(image, config, frame);
  drawPoints(image, frame.current_hits, config, frame, 2, config.current_hit_color);
  drawDrone(image, config, frame);
  return image;
}

} // namespace drone_city_nav
