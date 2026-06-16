#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

namespace drone_city_nav {

struct Pixel {
  std::uint8_t r{0U};
  std::uint8_t g{0U};
  std::uint8_t b{0U};
};

struct DebugImage {
  int width{1};
  int height{1};
  std::vector<Pixel> pixels;

  DebugImage(int image_width, int image_height, Pixel background);

  void set(int x, int y, Pixel color);
};

void drawLine(DebugImage& image, int x0, int y0, int x1, int y1, Pixel color);

void drawDisc(DebugImage& image, int center_x, int center_y, int radius, Pixel color);

[[nodiscard]] bool writePpm(const std::filesystem::path& path, const DebugImage& image);

} // namespace drone_city_nav
