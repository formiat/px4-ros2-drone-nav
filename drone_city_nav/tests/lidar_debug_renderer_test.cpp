#include "drone_city_nav/lidar_debug_renderer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] const Pixel& pixelAt(const DebugImage& image, const int x, const int y) {
  return image
      .pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width) +
              static_cast<std::size_t>(x)];
}

[[nodiscard]] bool samePixel(const Pixel lhs, const Pixel rhs) noexcept {
  return lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b;
}

} // namespace

TEST(LidarDebugRenderer, GridWorldToPixelUsesFullMapCoordinates) {
  std::vector<std::int8_t> cells(100, 0);
  LidarDebugRenderConfig config;
  config.image_size_px = 100;
  LidarDebugFrame frame;
  frame.grid = GridImageView{10, 10, 1.0, -5.0, -5.0, cells};

  const auto pixel = worldToDebugImagePixel(Point2{0.0, 0.0}, config, frame);

  ASSERT_TRUE(pixel.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const ImagePixel pixel_value = pixel.value();
  EXPECT_NEAR(pixel_value.x, 50, 1);
  EXPECT_NEAR(pixel_value.y, 49, 1);
}

TEST(LidarDebugRenderer, FallsBackToDroneCenteredViewWithoutGrid) {
  LidarDebugRenderConfig config;
  config.image_size_px = 100;
  config.fallback_view_radius_m = 10.0;
  LidarDebugFrame frame;
  frame.drone_position = Point2{10.0, 20.0};

  const auto pixel = worldToDebugImagePixel(Point2{10.0, 20.0}, config, frame);

  ASSERT_TRUE(pixel.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const ImagePixel pixel_value = pixel.value();
  EXPECT_EQ(pixel_value.x, 50);
  EXPECT_EQ(pixel_value.y, 50);
}

TEST(LidarDebugRenderer, DrawsRememberedHitsAmberAndCurrentHitsRed) {
  LidarDebugRenderConfig config;
  config.image_size_px = 101;
  config.fallback_view_radius_m = 10.0;
  const std::vector<Point2> remembered_hits{Point2{-2.0, 0.0}};
  const std::vector<Point2> current_hits{Point2{2.0, 2.0}};
  LidarDebugFrame frame;
  frame.drone_position = Point2{0.0, 0.0};
  frame.remembered_hits = remembered_hits;
  frame.current_hits = current_hits;

  const DebugImage image = renderLidarDebugImage(config, frame);

  const auto remembered_pixel =
      worldToDebugImagePixel(remembered_hits.front(), config, frame);
  const auto current_pixel =
      worldToDebugImagePixel(current_hits.front(), config, frame);
  ASSERT_TRUE(remembered_pixel.has_value());
  ASSERT_TRUE(current_pixel.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const ImagePixel remembered_pixel_value = remembered_pixel.value();
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const ImagePixel current_pixel_value = current_pixel.value();
  EXPECT_TRUE(
      samePixel(pixelAt(image, remembered_pixel_value.x, remembered_pixel_value.y),
                config.remembered_hit_color));
  EXPECT_TRUE(samePixel(pixelAt(image, current_pixel_value.x, current_pixel_value.y),
                        config.current_hit_color));
}

TEST(LidarDebugRenderer, DrawsPathCyanWithoutThrowingOnOutOfBoundsPoints) {
  LidarDebugRenderConfig config;
  config.image_size_px = 101;
  config.fallback_view_radius_m = 10.0;
  const std::vector<Point2> path{Point2{-4.0, 0.0}, Point2{0.0, 0.0},
                                 Point2{50.0, 50.0}};
  LidarDebugFrame frame;
  frame.drone_position = Point2{0.0, 0.0};
  frame.path = path;

  const DebugImage image = renderLidarDebugImage(config, frame);
  const auto waypoint_pixel = worldToDebugImagePixel(path.front(), config, frame);

  ASSERT_TRUE(waypoint_pixel.has_value());
  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  const ImagePixel waypoint_pixel_value = waypoint_pixel.value();
  EXPECT_TRUE(samePixel(pixelAt(image, waypoint_pixel_value.x, waypoint_pixel_value.y),
                        config.path_waypoint_color));
}

} // namespace drone_city_nav
