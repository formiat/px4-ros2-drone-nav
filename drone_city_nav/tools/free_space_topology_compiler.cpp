#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/free_space_topology_extractor_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/static_esdf_cache.hpp"

#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

struct Options {
  std::filesystem::path occupancy;
  std::filesystem::path esdf;
  std::filesystem::path output;
  drone_city_nav::FreeSpaceTopologyExtractorConfig extractor;
  std::size_t minimum_segments{0U};
  bool verbose{false};
};

[[nodiscard]] double parseDouble(const std::string_view text, const char* description) {
  const std::string value{text};
  std::size_t consumed{0U};
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size()) {
    throw std::invalid_argument{std::string{"invalid "} + description};
  }
  return parsed;
}

[[nodiscard]] std::size_t parseSize(const std::string_view text,
                                    const char* description) {
  const std::string value{text};
  std::size_t consumed{0U};
  const unsigned long long parsed = std::stoull(value, &consumed);
  if (consumed != value.size()) {
    throw std::invalid_argument{std::string{"invalid "} + description};
  }
  return static_cast<std::size_t>(parsed);
}

[[nodiscard]] Options parseOptions(const std::vector<std::string_view>& arguments) {
  Options options;
  for (std::size_t index = 0U; index < arguments.size(); ++index) {
    const std::string_view argument = arguments[index];
    const auto value = [&]() -> std::string_view {
      if (index + 1U >= arguments.size()) {
        throw std::invalid_argument{"missing value after " + std::string{argument}};
      }
      return arguments[++index];
    };
    if (argument == "--occupancy") {
      options.occupancy = std::string{value()};
    } else if (argument == "--esdf") {
      options.esdf = std::string{value()};
    } else if (argument == "--output") {
      options.output = std::string{value()};
    } else if (argument == "--maximum-clearance-m") {
      options.extractor.maximum_clearance_m = parseDouble(value(), "maximum clearance");
    } else if (argument == "--open-space-clearance-m") {
      options.extractor.open_space_clearance_m =
          parseDouble(value(), "open-space clearance");
    } else if (argument == "--speed-limit-mps") {
      options.extractor.speed_limit_mps = parseDouble(value(), "speed limit");
    } else if (argument == "--medial-clearance-weight") {
      options.extractor.medial_clearance_weight =
          parseDouble(value(), "medial clearance weight");
    } else if (argument == "--medial-ridge-prominence-m") {
      options.extractor.medial_ridge_prominence_m =
          parseDouble(value(), "medial ridge prominence");
    } else if (argument == "--medial-band-radius-cells") {
      options.extractor.medial_band_radius_cells =
          parseSize(value(), "medial band radius");
    } else if (argument == "--chunk-size-cells") {
      options.extractor.chunk_size_cells = parseSize(value(), "chunk size");
    } else if (argument == "--minimum-open-region-voxels") {
      options.extractor.minimum_open_region_voxels =
          parseSize(value(), "minimum open-region voxel count");
    } else if (argument == "--minimum-constrained-component-voxels") {
      options.extractor.minimum_constrained_component_voxels =
          parseSize(value(), "minimum constrained-component voxel count");
    } else if (argument == "--minimum-portal-voxels") {
      options.extractor.minimum_portal_voxels =
          parseSize(value(), "minimum portal voxel count");
    } else if (argument == "--minimum-center-z-m") {
      options.extractor.minimum_center_z_m = parseDouble(value(), "minimum center z");
    } else if (argument == "--maximum-center-z-m") {
      options.extractor.maximum_center_z_m = parseDouble(value(), "maximum center z");
    } else if (argument == "--footprint-radius-m") {
      options.extractor.footprint.radius_m = parseDouble(value(), "footprint radius");
    } else if (argument == "--footprint-lower-extent-m") {
      options.extractor.footprint.lower_extent_m =
          parseDouble(value(), "footprint lower extent");
    } else if (argument == "--footprint-upper-extent-m") {
      options.extractor.footprint.upper_extent_m =
          parseDouble(value(), "footprint upper extent");
    } else if (argument == "--footprint-sweep-step-m") {
      options.extractor.footprint.sweep_step_m =
          parseDouble(value(), "footprint sweep step");
    } else if (argument == "--minimum-segments") {
      options.minimum_segments = parseSize(value(), "minimum segment count");
    } else if (argument == "--verbose") {
      options.verbose = true;
    } else {
      throw std::invalid_argument{"unknown argument: " + std::string{argument}};
    }
  }
  if (options.occupancy.empty() || options.esdf.empty() || options.output.empty() ||
      !drone_city_nav::freeSpaceTopologyExtractorConfigIsValid(options.extractor)) {
    throw std::invalid_argument{
        "usage: free_space_topology_compiler --occupancy PATH --esdf PATH "
        "--output PATH [extractor options]"};
  }
  return options;
}

} // namespace

int main(const int argc, const char* const argv[]) {
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
    const drone_city_nav::StaticEsdfCache cache =
        drone_city_nav::StaticEsdfCache::load(options.esdf);
    if (!cache.compatibleWith(occupancy, options.extractor.maximum_clearance_m)) {
      throw std::invalid_argument{"ESDF3D is incompatible with Occupancy3D"};
    }
    const drone_city_nav::StaticEsdfCacheExtraction clearance =
        cache.extract(occupancy.bounds(), options.extractor.maximum_clearance_m);
    drone_city_nav::ExtractedFreeSpaceTopology3D extracted =
        drone_city_nav::extractFreeSpaceTopology3D(occupancy, clearance.field,
                                                   options.extractor);
    const std::size_t region_count = extracted.regions.size();
    const std::size_t portal_count = extracted.portals.size();
    const std::size_t segment_count = extracted.segments.size();
    if (segment_count < options.minimum_segments) {
      throw std::runtime_error{"extracted topology has fewer segments than required"};
    }
    if (options.verbose) {
      for (const drone_city_nav::PassagePortal& portal : extracted.portals) {
        std::cout << "FREE_SPACE_TOPOLOGY_PORTAL id=" << portal.id
                  << " region=" << portal.region_id << " center=" << portal.center.x
                  << ',' << portal.center.y << ',' << portal.center.z
                  << " surface_voxels=" << portal.surface_voxels.size()
                  << " clearance_min_m=" << portal.minimum_clearance_m
                  << " clearance_mean_m=" << portal.mean_clearance_m
                  << " clearance_max_m=" << portal.maximum_clearance_m << '\n';
      }
      for (const drone_city_nav::PassageSegment& segment : extracted.segments) {
        const drone_city_nav::Point3& first = segment.centerline.front().position;
        const drone_city_nav::Point3& last = segment.centerline.back().position;
        std::cout << "FREE_SPACE_TOPOLOGY_SEGMENT id=" << segment.id
                  << " first=" << first.x << ',' << first.y << ',' << first.z
                  << " last=" << last.x << ',' << last.y << ',' << last.z
                  << " samples=" << segment.centerline.size()
                  << " portals=" << segment.endpoint_portal_ids.size()
                  << " minimum_clearance_m=" << segment.minimum_clearance_m << '\n';
      }
    }
    const drone_city_nav::FreeSpaceTopology3D topology{
        occupancy.fingerprint(), occupancy.bounds(), std::move(extracted.regions),
        std::move(extracted.portals), std::move(extracted.segments)};
    topology.write(options.output);
    std::cout << "FREE_SPACE_TOPOLOGY_COMPILED occupancy=" << options.occupancy
              << " esdf=" << options.esdf << " output=" << options.output
              << " fingerprint=" << occupancy.fingerprint()
              << " analysis_chunks=" << extracted.stats.processed_chunks
              << " esdf_chunks=" << clearance.stats.requested_chunks
              << " esdf_decode_ms=" << clearance.stats.decode_ms
              << " esdf_copy_ms=" << clearance.stats.copy_ms
              << " feasible_voxels=" << extracted.stats.footprint_feasible_voxels
              << " open_voxels=" << extracted.stats.open_space_voxels
              << " constrained_voxels=" << extracted.stats.constrained_voxels
              << " medial_ridge_voxels=" << extracted.stats.medial_ridge_voxels
              << " medial_band_voxels=" << extracted.stats.medial_band_voxels
              << " open_components=" << extracted.stats.open_space_components
              << " constrained_components=" << extracted.stats.constrained_components
              << " rejected_components="
              << extracted.stats.rejected_constrained_components
              << " portal_patches=" << extracted.stats.portal_patches
              << " rejected_insufficient_portals="
              << extracted.stats.rejected_for_insufficient_portals
              << " rejected_disconnected_medial="
              << extracted.stats.rejected_for_disconnected_medial_graph
              << " raw_unsafe_segments=" << extracted.stats.raw_unsafe_segments
              << " rejected_no_safe_segments="
              << extracted.stats.rejected_for_no_safe_segments
              << " regions=" << region_count << " portals=" << portal_count
              << " segments=" << segment_count
              << " classification_ms=" << extracted.stats.classification_ms
              << " labeling_ms=" << extracted.stats.component_labeling_ms
              << " topology_ms=" << extracted.stats.topology_build_ms
              << " total_ms=" << extracted.stats.duration_ms << '\n';
  } catch (const std::exception& error) {
    std::cerr << "FREE_SPACE_TOPOLOGY_COMPILE_FAILED error=" << error.what() << '\n';
    return 1;
  }
  return 0;
}
