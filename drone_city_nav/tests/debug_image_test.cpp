#include "drone_city_nav/debug_image.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

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

TEST(DebugImage, SetIgnoresOutOfBounds) {
  const Pixel background{1U, 2U, 3U};
  DebugImage image{3, 3, background};

  image.set(-1, 1, Pixel{255U, 0U, 0U});
  image.set(3, 1, Pixel{255U, 0U, 0U});
  image.set(1, 3, Pixel{255U, 0U, 0U});

  for (const Pixel pixel : image.pixels) {
    EXPECT_TRUE(samePixel(pixel, background));
  }
}

TEST(DebugImage, DrawLineDrawsEndpointsAndMiddle) {
  const Pixel background{0U, 0U, 0U};
  const Pixel line{255U, 255U, 255U};
  DebugImage image{5, 5, background};

  drawLine(image, 0, 0, 4, 4, line);

  EXPECT_TRUE(samePixel(pixelAt(image, 0, 0), line));
  EXPECT_TRUE(samePixel(pixelAt(image, 2, 2), line));
  EXPECT_TRUE(samePixel(pixelAt(image, 4, 4), line));
  EXPECT_TRUE(samePixel(pixelAt(image, 0, 4), background));
}

TEST(DebugImage, DrawDiscDrawsExpectedRadius) {
  const Pixel background{0U, 0U, 0U};
  const Pixel disc{10U, 20U, 30U};
  DebugImage image{7, 7, background};

  drawDisc(image, 3, 3, 2, disc);

  EXPECT_TRUE(samePixel(pixelAt(image, 3, 3), disc));
  EXPECT_TRUE(samePixel(pixelAt(image, 5, 3), disc));
  EXPECT_TRUE(samePixel(pixelAt(image, 3, 5), disc));
  EXPECT_TRUE(samePixel(pixelAt(image, 6, 3), background));
  EXPECT_TRUE(samePixel(pixelAt(image, 6, 6), background));
}

TEST(DebugImage, WritePpmWritesValidHeaderAndPayload) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "drone_city_nav_debug_image.ppm";
  DebugImage image{2, 1, Pixel{1U, 2U, 3U}};
  image.set(1, 0, Pixel{4U, 5U, 6U});

  ASSERT_TRUE(writePpm(path, image));

  std::ifstream input{path, std::ios::binary};
  ASSERT_TRUE(input.is_open());
  const std::string content{std::istreambuf_iterator<char>{input},
                            std::istreambuf_iterator<char>{}};
  ASSERT_EQ(content.size(), std::string{"P6\n2 1\n255\n"}.size() + 6U);
  EXPECT_EQ(content.substr(0, 11), "P6\n2 1\n255\n");
  EXPECT_EQ(static_cast<unsigned char>(content[11]), 1U);
  EXPECT_EQ(static_cast<unsigned char>(content[12]), 2U);
  EXPECT_EQ(static_cast<unsigned char>(content[13]), 3U);
  EXPECT_EQ(static_cast<unsigned char>(content[14]), 4U);
  EXPECT_EQ(static_cast<unsigned char>(content[15]), 5U);
  EXPECT_EQ(static_cast<unsigned char>(content[16]), 6U);

  std::error_code ignored;
  std::filesystem::remove(path, ignored);
}

} // namespace drone_city_nav
