#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/free_space_topology_extractor_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <exception>
#include <filesystem>
#include <iostream>
#include <utility>

int main(const int argc, const char* const argv[]) {
  if (argc != 3) {
    std::cerr << "usage: free_space_topology_compiler OCCUPANCY3D TOPOLOGY3D\n";
    return 2;
  }
  try {
    const std::filesystem::path occupancy_path{argv[1]};
    const std::filesystem::path topology_path{argv[2]};
    const drone_city_nav::OccupancyGrid3D occupancy =
        drone_city_nav::OccupancyGrid3D::load(occupancy_path);
    drone_city_nav::ExtractedFreeSpaceTopology3D extracted =
        drone_city_nav::extractFreeSpaceTopology3D(occupancy);
    const std::size_t region_count = extracted.regions.size();
    const std::size_t portal_count = extracted.portals.size();
    const std::size_t segment_count = extracted.segments.size();
    const drone_city_nav::FreeSpaceTopology3D topology{
        occupancy.fingerprint(), occupancy.bounds(), std::move(extracted.regions),
        std::move(extracted.portals), std::move(extracted.segments)};
    topology.write(topology_path);
    std::cout << "FREE_SPACE_TOPOLOGY_COMPILED occupancy=" << occupancy_path
              << " output=" << topology_path
              << " fingerprint=" << occupancy.fingerprint()
              << " chunks=" << extracted.stats.processed_chunks
              << " regions=" << region_count << " portals=" << portal_count
              << " segments=" << segment_count << '\n';
  } catch (const std::exception& error) {
    std::cerr << "FREE_SPACE_TOPOLOGY_COMPILE_FAILED error=" << error.what() << '\n';
    return 1;
  }
  return 0;
}
