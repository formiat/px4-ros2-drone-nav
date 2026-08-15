#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/free_space_topology_extractor_3d.hpp"
#include "drone_city_nav/static_esdf_cache.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <optional>
#include <ranges>
#include <utility>

#include "advanced_passage_fixture.hpp"

namespace drone_city_nav::test {
namespace {

[[nodiscard]] ExtractedFreeSpaceTopology3D
extract(const AdvancedPassageFixture& fixture) {
  return extractFreeSpaceTopology3D(fixture.occupancy,
                                    FreeSpaceTopologyExtractorConfig{
                                        .maximum_clearance_m = 6.0,
                                        .open_space_clearance_m = 3.0,
                                        .speed_limit_mps = 10.0,
                                        .medial_clearance_weight = 2.0,
                                        .chunk_size_cells = 32U,
                                        .minimum_open_region_voxels = 16U,
                                        .minimum_constrained_component_voxels = 16U,
                                        .minimum_portal_voxels = 4U,
                                        .minimum_center_z_m = std::nullopt,
                                        .maximum_center_z_m = std::nullopt,
                                    });
}

TEST(FreeSpaceTopologyExtractor3D, ExtractsEveryGeneralizedPositiveFixture) {
  for (const AdvancedPassageFixtureKind kind : advancedPassageFixtureKinds()) {
    const AdvancedPassageFixture fixture = buildAdvancedPassageFixture(kind);
    const ExtractedFreeSpaceTopology3D topology = extract(fixture);
    EXPECT_EQ(topology.stats.processed_chunks, 8U) << fixture.name;
    EXPECT_GT(topology.stats.footprint_feasible_voxels, 0U) << fixture.name;
    if (!fixture.expectation.should_extract_passage) {
      EXPECT_TRUE(topology.portals.empty()) << fixture.name;
      EXPECT_TRUE(topology.segments.empty()) << fixture.name;
      continue;
    }
    EXPECT_GE(topology.portals.size(), fixture.expectation.minimum_portal_count)
        << fixture.name << " open=" << topology.stats.open_space_voxels
        << " constrained=" << topology.stats.constrained_voxels
        << " open_components=" << topology.stats.open_space_components
        << " constrained_components=" << topology.stats.constrained_components
        << " rejected=" << topology.stats.rejected_constrained_components;
    EXPECT_GE(topology.segments.size(), fixture.expectation.minimum_segment_count)
        << fixture.name;
    EXPECT_FALSE(topology.regions.empty()) << fixture.name;
  }
}

TEST(FreeSpaceTopologyExtractor3D, PortalVoxelPatchIsAuthoritativeGeometry) {
  const AdvancedPassageFixture fixture =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kArchTunnel);
  const ExtractedFreeSpaceTopology3D topology = extract(fixture);
  ASSERT_GE(topology.portals.size(), 2U);
  for (const PassagePortal& portal : topology.portals) {
    EXPECT_GE(portal.surface_voxels.size(), 4U);
    EXPECT_FALSE(portal.traversable_anchors.empty());
    EXPECT_GE(portal.opening_polygon.size(), 3U);
    EXPECT_NEAR(std::hypot(std::hypot(portal.outward_normal.x, portal.outward_normal.y),
                           portal.outward_normal.z),
                1.0, 1.0e-6);
    EXPECT_NEAR(portal.outward_normal.x * portal.local_u_axis.x +
                    portal.outward_normal.y * portal.local_u_axis.y +
                    portal.outward_normal.z * portal.local_u_axis.z,
                0.0, 1.0e-6);
    EXPECT_LE(portal.minimum_clearance_m, portal.mean_clearance_m);
    EXPECT_LE(portal.mean_clearance_m, portal.maximum_clearance_m);
  }
}

TEST(FreeSpaceTopologyExtractor3D, MedialSegmentsPreserveSlopeAndShaftGeometry) {
  const auto maximum_axis_delta = [](const ExtractedFreeSpaceTopology3D& topology,
                                     const char axis) {
    double maximum = 0.0;
    for (const PassageSegment& segment : topology.segments) {
      const Point3& first = segment.centerline.front().position;
      const Point3& last = segment.centerline.back().position;
      const double delta = axis == 'z' ? std::abs(last.z - first.z)
                                       : std::hypot(last.x - first.x, last.y - first.y);
      maximum = std::max(maximum, delta);
    }
    return maximum;
  };

  const ExtractedFreeSpaceTopology3D slope =
      extract(buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kSlopedTunnel));
  EXPECT_GT(maximum_axis_delta(slope, 'z'), 8.0);
  EXPECT_GT(maximum_axis_delta(slope, 'h'), 8.0);

  const ExtractedFreeSpaceTopology3D shaft =
      extract(buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kVerticalShaft));
  EXPECT_GT(maximum_axis_delta(shaft, 'z'), 12.0);
}

TEST(FreeSpaceTopologyExtractor3D, JunctionRoadmapIsSparse) {
  for (const AdvancedPassageFixtureKind kind :
       {AdvancedPassageFixtureKind::kTJunction,
        AdvancedPassageFixtureKind::kXJunction}) {
    const AdvancedPassageFixture fixture = buildAdvancedPassageFixture(kind);
    const ExtractedFreeSpaceTopology3D topology = extract(fixture);
    ASSERT_GE(topology.portals.size(), fixture.expectation.minimum_portal_count);
    ASSERT_GE(topology.segments.size(), fixture.expectation.minimum_segment_count);
    const std::size_t pairwise_edges =
        topology.portals.size() * (topology.portals.size() - 1U) / 2U;
    EXPECT_LE(topology.segments.size(), 2U * topology.portals.size() - 3U)
        << fixture.name;
    if (topology.portals.size() >= 4U) {
      EXPECT_LT(topology.segments.size(), pairwise_edges) << fixture.name;
    }
    EXPECT_TRUE(std::ranges::any_of(topology.segments, [](const PassageSegment&
                                                              segment) {
      return segment.first_endpoint_neighbors.size() > 1U ||
             segment.second_endpoint_neighbors.size() > 1U;
    })) << fixture.name;
  }
}

TEST(FreeSpaceTopologyExtractor3D, RejectsWideRoofedHangar) {
  const AdvancedPassageFixture fixture =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kWideHangar);
  const ExtractedFreeSpaceTopology3D topology = extract(fixture);
  EXPECT_TRUE(topology.regions.empty());
  EXPECT_TRUE(topology.portals.empty());
  EXPECT_TRUE(topology.segments.empty());
  EXPECT_GT(topology.stats.rejected_constrained_components, 0U);
}

TEST(FreeSpaceTopologyExtractor3D, SparseArtifactRoundTripsAuthoritativeGeometry) {
  const AdvancedPassageFixture fixture =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kXJunction);
  ExtractedFreeSpaceTopology3D extracted = extract(fixture);
  const std::size_t expected_regions = extracted.regions.size();
  const std::size_t expected_portals = extracted.portals.size();
  const std::size_t expected_segments = extracted.segments.size();
  const FreeSpaceTopology3D topology{
      fixture.occupancy.fingerprint(), fixture.occupancy.bounds(),
      std::move(extracted.regions), std::move(extracted.portals),
      std::move(extracted.segments)};
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      "drone_city_nav_sparse_topology_round_trip.topology3d";
  topology.write(path);
  const FreeSpaceTopology3D loaded = FreeSpaceTopology3D::load(path);
  std::filesystem::remove(path);

  EXPECT_TRUE(loaded.compatibleWith(fixture.occupancy));
  EXPECT_EQ(loaded.regions().size(), expected_regions);
  EXPECT_EQ(loaded.portals().size(), expected_portals);
  EXPECT_EQ(loaded.segments().size(), expected_segments);
  EXPECT_TRUE(loaded.traversalEdges().empty());
  ASSERT_FALSE(loaded.portals().empty());
  EXPECT_FALSE(loaded.portals().front().surface_voxels.empty());
  ASSERT_FALSE(loaded.segments().empty());
  EXPECT_FALSE(loaded.segments().front().centerline.empty());
}

TEST(FreeSpaceTopologyExtractor3D, ReusesCompatibleStaticEsdfArtifact) {
  const AdvancedPassageFixture fixture =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kCurvedTunnel);
  const DistanceField3D field = DistanceField3D::build(fixture.occupancy, 6.0);
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      "drone_city_nav_topology_extractor_static_esdf.esdf3d";
  StaticEsdfCache::write(path, fixture.occupancy, field);
  const StaticEsdfCache cache = StaticEsdfCache::load(path);
  const StaticEsdfCacheExtraction cached =
      cache.extract(fixture.occupancy.bounds(), 6.0);
  std::filesystem::remove(path);

  const ExtractedFreeSpaceTopology3D direct = extract(fixture);
  const ExtractedFreeSpaceTopology3D reused = extractFreeSpaceTopology3D(
      fixture.occupancy, cached.field,
      FreeSpaceTopologyExtractorConfig{.maximum_clearance_m = 6.0,
                                       .open_space_clearance_m = 3.0,
                                       .speed_limit_mps = 10.0,
                                       .medial_clearance_weight = 2.0,
                                       .medial_ridge_prominence_m = 0.1,
                                       .medial_band_radius_cells = 1U,
                                       .chunk_size_cells = 32U,
                                       .minimum_open_region_voxels = 16U,
                                       .minimum_constrained_component_voxels = 16U,
                                       .minimum_portal_voxels = 4U,
                                       .minimum_center_z_m = std::nullopt,
                                       .maximum_center_z_m = std::nullopt});
  EXPECT_EQ(reused.regions.size(), direct.regions.size());
  EXPECT_EQ(reused.portals.size(), direct.portals.size());
  EXPECT_EQ(reused.segments.size(), direct.segments.size());
  EXPECT_EQ(reused.stats.medial_ridge_voxels, direct.stats.medial_ridge_voxels);
}

} // namespace
} // namespace drone_city_nav::test
