#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/free_space_topology_extractor_3d.hpp"
#include "drone_city_nav/free_space_topology_router.hpp"
#include "drone_city_nav/passage_volume.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <ranges>
#include <utility>

#include "advanced_passage_fixture.hpp"

namespace drone_city_nav::test {
namespace {

[[nodiscard]] FreeSpaceTopology3D
extractTopology(const AdvancedPassageFixture& fixture) {
  ExtractedFreeSpaceTopology3D extracted = extractFreeSpaceTopology3D(
      fixture.occupancy, FreeSpaceTopologyExtractorConfig{
                             .maximum_clearance_m = 6.0,
                             .open_space_clearance_m = 3.0,
                             .speed_limit_mps = 10.0,
                             .medial_clearance_weight = 2.0,
                             .chunk_size_cells = 32U,
                             .minimum_open_region_voxels = 16U,
                             .minimum_constrained_component_voxels = 16U,
                             .minimum_portal_voxels = 4U,
                         });
  return FreeSpaceTopology3D{fixture.occupancy.fingerprint(),
                             fixture.occupancy.bounds(), std::move(extracted.regions),
                             std::move(extracted.portals),
                             std::move(extracted.segments)};
}

TEST(FreeSpaceTopologyRouter, ResolvesAndCachesRouteSpecificSparseTraversal) {
  const AdvancedPassageFixture fixture =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kSlopedTunnel);
  const FreeSpaceTopology3D topology = extractTopology(fixture);
  const FreeSpaceTopologyRouter router{topology};

  const FreeSpaceTopologyRoute first =
      router.resolve(fixture.expectation.open_space_seeds.front(),
                     fixture.expectation.open_space_seeds.back());
  ASSERT_NE(first.traversals, nullptr);
  ASSERT_EQ(first.traversals->size(), 1U);
  EXPECT_EQ(first.stats.entry_search_count, 2U);
  EXPECT_EQ(first.stats.traversal_cache_hits, 0U);
  EXPECT_EQ(first.stats.traversal_cache_misses, 1U);
  const PassageTraversalEdge& traversal = first.traversals->front();
  EXPECT_FALSE(traversal.segment_spans.empty());
  EXPECT_NEAR(traversal.segment_spans.front().begin_station_m, 0.0, 1.0e-9);
  EXPECT_NEAR(traversal.segment_spans.back().end_station_m,
              traversal.centerline.back().station_m, 1.0e-6);
  for (std::size_t index = 1U; index < traversal.segment_spans.size(); ++index) {
    EXPECT_NEAR(traversal.segment_spans[index - 1U].end_station_m,
                traversal.segment_spans[index].begin_station_m, 1.0e-6);
  }
  EXPECT_GT(traversal.centerline.back().position.z -
                traversal.centerline.front().position.z,
            8.0);
  for (std::size_t index = 1U; index < traversal.centerline.size(); ++index) {
    EXPECT_TRUE(validateRawSweptFootprint(
                    fixture.occupancy, traversal.centerline[index - 1U].position,
                    FootprintBodyAxis{}, traversal.centerline[index].position,
                    FootprintBodyAxis{}, SweptFootprintConfig{})
                    .accepted());
  }

  const FreeSpaceTopologyRoute second =
      router.resolve(fixture.expectation.open_space_seeds.front(),
                     fixture.expectation.open_space_seeds.back());
  ASSERT_NE(second.traversals, nullptr);
  ASSERT_EQ(second.traversals->size(), first.traversals->size());
  EXPECT_EQ(second.stats.traversal_cache_hits, second.traversals->size());
  EXPECT_EQ(second.stats.traversal_cache_misses, 0U);
  EXPECT_EQ(second.traversals->front().id, traversal.id);
  EXPECT_EQ(second.traversals->front().segment_spans, traversal.segment_spans);
}

TEST(FreeSpaceTopologyRouter, BoundsJunctionCandidatesWithoutPairwiseMaterialization) {
  const AdvancedPassageFixture fixture =
      buildAdvancedPassageFixture(AdvancedPassageFixtureKind::kXJunction);
  const FreeSpaceTopology3D topology = extractTopology(fixture);
  const FreeSpaceTopologyRouter router{
      topology, FreeSpaceTopologyRouterConfig{.maximum_entry_portals_per_component = 2U,
                                              .maximum_traversals_per_component = 2U}};

  const FreeSpaceTopologyRoute route = router.resolve(
      fixture.expectation.open_space_seeds[0], fixture.expectation.open_space_seeds[2]);
  ASSERT_NE(route.traversals, nullptr);
  ASSERT_EQ(route.traversals->size(), 2U);
  EXPECT_EQ(route.stats.entry_search_count, 2U);
  EXPECT_LE(route.stats.evaluated_portal_pairs, 2U * (topology.portals().size() - 1U));
  EXPECT_LT(route.traversals->size(),
            topology.portals().size() * (topology.portals().size() - 1U) / 2U);
  EXPECT_TRUE(
      std::ranges::all_of(*route.traversals, [](const PassageTraversalEdge& traversal) {
        return !traversal.segment_spans.empty() && traversal.centerline.size() >= 2U;
      }));
}

TEST(FreeSpaceTopologyRouter, BuildsVaryingFull3DEnvelopeForGeneralizedTraversals) {
  for (const AdvancedPassageFixtureKind kind :
       {AdvancedPassageFixtureKind::kSlopedTunnel,
        AdvancedPassageFixtureKind::kVerticalShaft,
        AdvancedPassageFixtureKind::kCurvedTunnel}) {
    const AdvancedPassageFixture fixture = buildAdvancedPassageFixture(kind);
    const FreeSpaceTopology3D topology = extractTopology(fixture);
    const FreeSpaceTopologyRouter router{topology};
    const FreeSpaceTopologyRoute resolved =
        router.resolve(fixture.expectation.open_space_seeds.front(),
                       fixture.expectation.open_space_seeds.back());
    ASSERT_NE(resolved.traversals, nullptr) << fixture.name;
    ASSERT_FALSE(resolved.traversals->empty()) << fixture.name;
    const PassageTraversalEdge& traversal = resolved.traversals->front();
    const std::vector<SelectedPassageTraversal> selected{SelectedPassageTraversal{
        .passage_traversal_id = traversal.id,
        .direction_sign = 1,
        .begin_station_m = traversal.centerline.front().station_m,
        .end_station_m = traversal.centerline.back().station_m,
        .min_z_m = traversal.min_z_m,
        .max_z_m = traversal.max_z_m,
        .width_m = traversal.width_m,
        .height_m = traversal.height_m,
        .minimum_clearance_m = traversal.minimum_clearance_m,
        .speed_limit_mps = traversal.speed_limit_mps,
        .segment_spans = traversal.segment_spans,
    }};
    std::vector<ConstrainedRouteSpan> spans =
        makeConstrainedRouteSpans(traversal.centerline, selected, 1U,
                                  RouteEnvelopeConfig{.minimum_span_length_m = 1.0});
    ASSERT_EQ(spans.size(), 1U) << fixture.name;
    const PassageVolumeConfig volume_config{
        .cross_section_spacing_m = 1.0,
        .lateral_probe_step_m = 0.5,
        .secondary_probe_step_m = 0.5,
        .maximum_cross_section_probe_m = 8.0,
        .minimum_wall_clearance_m = 0.0,
    };
    const std::vector<PassageVolume> volumes = derivePassageVolumes(
        traversal.centerline, spans, fixture.occupancy, volume_config);
    ASSERT_EQ(volumes.size(), 1U) << fixture.name;
    ASSERT_TRUE(volumes.front().raw_validated) << fixture.name;
    ASSERT_GT(volumes.front().cross_sections.size(), 2U) << fixture.name;
    EXPECT_EQ(projectPassageVolumeEnvelopes(spans, volumes, volume_config.footprint),
              1U)
        << fixture.name;
    EXPECT_EQ(spans.front().envelope.size(), volumes.front().cross_sections.size())
        << fixture.name;
    for (const PassageCrossSection& section : volumes.front().cross_sections) {
      const auto dot = [](const Vec3& first, const Vec3& second) {
        return first.x * second.x + first.y * second.y + first.z * second.z;
      };
      EXPECT_NEAR(dot(section.tangent, section.lateral_axis), 0.0, 1.0e-6)
          << fixture.name;
      EXPECT_NEAR(dot(section.tangent, section.secondary_axis), 0.0, 1.0e-6)
          << fixture.name;
      EXPECT_TRUE(section.raw_validated) << fixture.name;
    }
    if (kind == AdvancedPassageFixtureKind::kVerticalShaft) {
      EXPECT_TRUE(std::ranges::any_of(volumes.front().cross_sections,
                                      [](const PassageCrossSection& section) {
                                        return std::abs(section.tangent.z) > 0.9;
                                      }));
    }
  }
}

TEST(FreeSpaceTopologyRouter, PreservesLegacyEagerTraversals) {
  const OccupancyGrid3D occupancy = OccupancyGrid3D::load(TEST_OCCUPANCY3D_PATH);
  const FreeSpaceTopology3D topology =
      FreeSpaceTopology3D::load(TEST_FREE_SPACE_TOPOLOGY3D_PATH);
  ASSERT_TRUE(topology.segments().empty());
  const FreeSpaceTopologyRouter router{topology};

  const FreeSpaceTopologyRoute route =
      router.resolve(Point3{54.0, 54.0, 18.0}, Point3{54.0, 378.0, 18.0});
  ASSERT_NE(route.traversals, nullptr);
  EXPECT_EQ(route.traversals->size(), topology.traversalEdges().size());
  EXPECT_TRUE(topology.compatibleWith(occupancy));
}

} // namespace
} // namespace drone_city_nav::test
