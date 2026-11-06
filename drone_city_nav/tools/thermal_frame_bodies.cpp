// thermal_frame_bodies FRAME WIDTH HEIGHT [RANGE_M]: the thermal core's readers over
// one frame (roadmap items 21 and 32): the frame is WIDTH x HEIGHT pixels of 16 bits,
// hundredths of a kelvin, little endian, as the core publishes it. One line for the
// source's reading (specification K49), one for each warm region (K64) with whether it
// is of a body's size at RANGE_M, where given, and at the range its own size gives.
// tools/thermal_body_rig.py feeds it the rig's frames.
#include "drone_city_nav/thermal_reading.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <span>
#include <vector>

int main(int argc, char** argv) {
  const std::span<char*> arguments(argv, static_cast<std::size_t>(argc));
  if (arguments.size() < 4U) {
    std::fprintf(stderr, "usage: thermal_frame_bodies FRAME WIDTH HEIGHT [RANGE_M]\n");
    return 2;
  }
  const auto width = static_cast<std::size_t>(std::strtoul(arguments[2], nullptr, 10));
  const auto height = static_cast<std::size_t>(std::strtoul(arguments[3], nullptr, 10));
  const double range_m =
      arguments.size() > 4U ? std::strtod(arguments[4], nullptr) : 0.0;
  std::vector<std::uint16_t> frame(width * height);
  std::ifstream stream{arguments[1], std::ios::binary};
  stream.read(
      reinterpret_cast<char*>(frame.data()), // NOLINT: the frame's bytes as they are
      static_cast<std::streamsize>(frame.size() * sizeof(std::uint16_t)));
  if (!stream) {
    std::fprintf(stderr, "thermal_frame_bodies: %s holds no %zu x %zu frame\n",
                 arguments[1], width, height);
    return 1;
  }
  using drone_city_nav::ThermalBody;
  const drone_city_nav::ThermalReadingConfig source_config;
  const drone_city_nav::ThermalBodyConfig body_config;
  const drone_city_nav::ThermalReading source =
      drone_city_nav::readThermalFrame(frame, width, height, source_config);
  std::printf("SOURCE hot=%d hot_pixels=%zu\n", source.hot() ? 1 : 0,
              source.hot_pixels);
  for (const ThermalBody& region : drone_city_nav::readThermalBodies(
           frame, width, height, source_config, body_config)) {
    std::printf(
        "REGION warm_pixels=%zu width_deg=%.2f height_deg=%.2f range_by_size_m=%.2f "
        "body_at_range=%d body_by_size=%d\n",
        region.warm_pixels, region.width_rad * 57.29577951308232,
        region.height_rad * 57.29577951308232, region.range_by_size_m,
        range_m > 0.0 && drone_city_nav::bodySized(region, range_m, body_config) ? 1
                                                                                 : 0,
        drone_city_nav::bodySized(region, region.range_by_size_m, body_config) ? 1 : 0);
  }
  return 0;
}
