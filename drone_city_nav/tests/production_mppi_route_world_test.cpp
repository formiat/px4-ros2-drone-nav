#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

TEST(ProductionMppiRouteWorldTest,
     CompletedWorldBuildPreservesNewerActivatedRouteOwnership) {
  auto generation_33_route = std::make_shared<const std::vector<RouteSample3D>>(2U);
  auto generation_34_route = std::make_shared<const std::vector<RouteSample3D>>(3U);
  auto generation_33_identity = std::make_shared<ProductionActivatedRoute3D>();
  generation_33_identity->identity.generation = 33U;
  generation_33_identity->geometry.route = generation_33_route;
  auto generation_34_identity = std::make_shared<ProductionActivatedRoute3D>();
  generation_34_identity->identity.generation = 34U;
  generation_34_identity->geometry.route = generation_34_route;

  ProductionMppiPreparedEsdf completed_world_build;
  completed_world_build.local_world_generation.generation = 118U;
  completed_world_build.producer_instance_id = 7U;
  completed_world_build.revision = 9002U;
  completed_world_build.source_raw_revision = 451U;
  completed_world_build.grid.width = 17;
  completed_world_build.distances_m =
      std::make_shared<const std::vector<float>>(4U, 2.0F);
  completed_world_build.global_guide_generation = 33U;
  completed_world_build.route_3d = generation_33_route;
  completed_world_build.activated_route_3d = generation_33_identity;
  completed_world_build.route_intent.id = 33U;
  completed_world_build.static_route_extension_request = true;

  ProductionMppiPreparedEsdf resident = completed_world_build;
  resident.local_world_generation.generation = 117U;
  resident.revision = 9001U;
  resident.source_raw_revision = 447U;
  resident.global_guide_generation = 34U;
  resident.route_3d = generation_34_route;
  resident.activated_route_3d = generation_34_identity;
  resident.route_intent.id = 1234U;
  resident.static_route_extension_request = false;

  adoptWorldResources(resident, completed_world_build);

  EXPECT_EQ(resident.local_world_generation.generation, 118U);
  EXPECT_EQ(resident.revision, 9002U);
  EXPECT_EQ(resident.source_raw_revision, 451U);
  EXPECT_EQ(resident.grid.width, 17);
  EXPECT_EQ(resident.distances_m, completed_world_build.distances_m);
  EXPECT_EQ(resident.global_guide_generation, 34U);
  EXPECT_EQ(resident.route_3d, generation_34_route);
  ASSERT_EQ(resident.activated_route_3d, generation_34_identity);
  EXPECT_EQ(resident.activated_route_3d->identity.generation, 34U);
  EXPECT_EQ(resident.route_intent.id, 1234U);
  EXPECT_FALSE(resident.static_route_extension_request);
}

} // namespace
} // namespace drone_city_nav
