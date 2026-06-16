#include "drone_city_nav/debug_image.hpp"

#include <algorithm>
#include <fstream>

namespace drone_city_nav {

DebugImage::DebugImage(const int image_width, const int image_height,
                       const Pixel background)
    : width{std::max(1, image_width)},
      height{std::max(1, image_height)},
      pixels(static_cast<std::size_t>(width * height), background) {
}

void DebugImage::set(const int x, const int y, const Pixel color) {
  if (x < 0 || y < 0 || x >= width || y >= height) {
    return;
  }
  const auto pixel_index =
      static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
      static_cast<std::size_t>(x);
  pixels[pixel_index] = color;
}

void drawLine(DebugImage& image, int x0, int y0, const int x1, const int y1,
              const Pixel color) {
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;

  while (true) {
    image.set(x0, y0, color);
    if (x0 == x1 && y0 == y1) {
      break;
    }

    const int doubled_error = 2 * error;
    if (doubled_error >= dy) {
      error += dy;
      x0 += sx;
    }
    if (doubled_error <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

void drawDisc(DebugImage& image, const int center_x, const int center_y,
              const int radius, const Pixel color) {
  for (int y = center_y - radius; y <= center_y + radius; ++y) {
    for (int x = center_x - radius; x <= center_x + radius; ++x) {
      const int dx = x - center_x;
      const int dy = y - center_y;
      if (dx * dx + dy * dy <= radius * radius) {
        image.set(x, y, color);
      }
    }
  }
}

bool writePpm(const std::filesystem::path& path, const DebugImage& image) {
  std::ofstream output{path, std::ios::binary};
  if (!output.is_open()) {
    return false;
  }

  output << "P6\n" << image.width << ' ' << image.height << "\n255\n";
  for (const Pixel pixel : image.pixels) {
    output.put(static_cast<char>(pixel.r));
    output.put(static_cast<char>(pixel.g));
    output.put(static_cast<char>(pixel.b));
  }
  return output.good();
}

} // namespace drone_city_nav
