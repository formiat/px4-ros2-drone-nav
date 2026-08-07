#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/static_esdf_cache.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

struct Options {
  std::filesystem::path occupancy;
  std::filesystem::path output;
  double maximum_distance_m{26.0};
  std::size_t workers{4U};
};

[[nodiscard]] Options parseOptions(const std::vector<std::string_view>& arguments) {
  Options options;
  for (std::size_t index = 0U; index < arguments.size(); ++index) {
    const std::string_view argument = arguments.at(index);
    const auto value = [&]() -> std::string_view {
      if (index + 1U >= arguments.size()) {
        throw std::invalid_argument{"missing value after " + std::string{argument}};
      }
      return arguments.at(++index);
    };
    if (argument == "--occupancy") {
      options.occupancy = value();
    } else if (argument == "--output") {
      options.output = value();
    } else if (argument == "--maximum-distance-m") {
      options.maximum_distance_m = std::stod(std::string{value()});
    } else if (argument == "--workers") {
      const std::string text{value()};
      std::size_t consumed{0U};
      const unsigned long parsed = std::stoul(text, &consumed);
      if (consumed != text.size()) {
        throw std::invalid_argument{"invalid worker count"};
      }
      options.workers = static_cast<std::size_t>(parsed);
    } else {
      throw std::invalid_argument{"unknown argument: " + std::string{argument}};
    }
  }
  if (options.occupancy.empty() || options.output.empty() ||
      !(options.maximum_distance_m > 0.0) || options.workers == 0U) {
    throw std::invalid_argument{
        "usage: generate_static_esdf_cache --occupancy PATH --output PATH "
        "[--maximum-distance-m 26] [--workers 4]"};
  }
  return options;
}

} // namespace

int main(const int argc, char** argv) {
  try {
    std::vector<std::string_view> arguments;
    if (argc > 1) {
      arguments.reserve(static_cast<std::size_t>(argc - 1));
    }
    for (int index = 1; index < argc; ++index) {
      // The C main entry point exposes arguments as a pointer array.
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      arguments.emplace_back(argv[index]);
    }
    const Options options = parseOptions(arguments);
    const drone_city_nav::OccupancyGrid3D occupancy =
        drone_city_nav::OccupancyGrid3D::load(options.occupancy);
    drone_city_nav::BoundedWorkerPool worker_pool{options.workers};
    const drone_city_nav::DistanceField3D field =
        drone_city_nav::DistanceField3D::build(occupancy, options.maximum_distance_m,
                                               &worker_pool);
    drone_city_nav::StaticEsdfCache::write(options.output, occupancy, field);
    const drone_city_nav::StaticEsdfCache cache =
        drone_city_nav::StaticEsdfCache::load(options.output);
    std::cout << "STATIC_ESDF_CACHE_GENERATED output=" << options.output
              << " fingerprint=" << cache.occupancyFingerprint()
              << " voxels=" << field.stats().voxel_count
              << " build_ms=" << field.stats().duration_ms
              << " chunks=" << cache.storedChunkCount()
              << " bytes=" << cache.compressedBytes() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "generate_static_esdf_cache: " << error.what() << '\n';
    return 1;
  }
}
