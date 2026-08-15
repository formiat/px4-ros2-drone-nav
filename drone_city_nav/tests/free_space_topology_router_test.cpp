#include "drone_city_nav/free_space_topology_3d.hpp"
#include "drone_city_nav/free_space_topology_extractor_3d.hpp"
#include "drone_city_nav/free_space_topology_router.hpp"
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
  EXPECT_FALSE(traversal.segment_ids.empty());
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
        return !traversal.segment_ids.empty() && traversal.centerline.size() >= 2U;
      }));
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
