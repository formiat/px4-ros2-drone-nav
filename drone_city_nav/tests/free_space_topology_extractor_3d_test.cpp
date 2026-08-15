#include "drone_city_nav/free_space_topology_extractor_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <ranges>

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

} // namespace
} // namespace drone_city_nav::test
