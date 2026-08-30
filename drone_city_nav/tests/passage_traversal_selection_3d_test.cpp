#include "drone_city_nav/passage_traversal_selection_3d.hpp"

#include <gtest/gtest.h>

#include <span>
#include <string>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PassageTraversalEdge makeTraversal(std::string id,
                                                 std::vector<Point3> points,
                                                 const double lateral_offset_m = 0.0) {
  for (Point3& point : points) {
    point.y += lateral_offset_m;
  }
  std::vector<RouteSample3D> centerline = sampleRoute3D(points, 0.25, 3.0);
  return PassageTraversalEdge{
      .id = PassageTraversalId{std::move(id)},
      .region_id = "region",
      .entry_portal_id = "entry",
      .exit_portal_id = "exit",
      .centerline = centerline,
      .entry = centerline.front().position,
      .exit = centerline.back().position,
      .min_z_m = 2.0,
      .max_z_m = 8.0,
      .width_m = 6.0,
      .height_m = 6.0,
      .minimum_clearance_m = 2.0,
      .speed_limit_mps = 3.0,
      .segment_spans = {PassageTraversalSegmentSpan{
          .passage_segment_id = "segment",
          .begin_station_m = 1.0,
          .end_station_m = 3.0,
      }},
  };
}

TEST(PassageTraversalSelection3DTest,
     AssociatesForwardTopologyAndMapsSegmentStationsToRoute) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}}, 0.25, 4.0);
  const std::vector<PassageTraversalEdge> topology{
      makeTraversal("opening", {{2.0, 0.0, 5.0}, {8.0, 0.0, 5.0}}),
  };

  const std::vector<SelectedPassageTraversal> selected =
      selectRoutePassageTraversals3D(route, topology);

  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected.front().passage_traversal_id, "opening");
  EXPECT_EQ(selected.front().direction_sign, 1);
  EXPECT_NEAR(selected.front().begin_station_m, 2.0, 1.0e-6);
  EXPECT_NEAR(selected.front().end_station_m, 8.0, 1.0e-6);
  ASSERT_EQ(selected.front().segment_spans.size(), 1U);
  EXPECT_NEAR(selected.front().segment_spans.front().begin_station_m, 3.0, 1.0e-6);
  EXPECT_NEAR(selected.front().segment_spans.front().end_station_m, 5.0, 1.0e-6);
}

TEST(PassageTraversalSelection3DTest,
     ReversedRouteRetainsTopologyIdentityAndReversesDirection) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{10.0, 0.0, 5.0}, {0.0, 0.0, 5.0}}, 0.25, 4.0);
  const std::vector<PassageTraversalEdge> topology{
      makeTraversal("opening", {{2.0, 0.0, 5.0}, {8.0, 0.0, 5.0}}),
  };

  const std::vector<SelectedPassageTraversal> selected =
      selectRoutePassageTraversals3D(route, topology);

  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected.front().direction_sign, -1);
  EXPECT_NEAR(selected.front().begin_station_m, 2.0, 1.0e-6);
  EXPECT_NEAR(selected.front().end_station_m, 8.0, 1.0e-6);
  ASSERT_EQ(selected.front().segment_spans.size(), 1U);
  EXPECT_NEAR(selected.front().segment_spans.front().begin_station_m, 5.0, 1.0e-6);
  EXPECT_NEAR(selected.front().segment_spans.front().end_station_m, 7.0, 1.0e-6);
}

TEST(PassageTraversalSelection3DTest,
     MatchingEndpointsCannotHideADeviatingTopologyCenterline) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}}, 0.25, 4.0);
  const std::vector<PassageTraversalEdge> topology{
      makeTraversal("detour", {{2.0, 0.0, 5.0}, {5.0, 3.0, 5.0}, {8.0, 0.0, 5.0}}),
  };

  EXPECT_TRUE(selectRoutePassageTraversals3D(route, topology).empty());
}

TEST(PassageTraversalSelection3DTest,
     MatchingTopologyCannotHideADeviatingRouteBetweenItsEndpoints) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0},
                                        {3.0, 0.0, 5.0},
                                        {3.5, 3.0, 5.0},
                                        {4.0, 0.0, 5.0},
                                        {6.0, 0.0, 5.0},
                                        {6.5, 3.0, 5.0},
                                        {7.0, 0.0, 5.0},
                                        {10.0, 0.0, 5.0}},
                    0.25, 4.0);
  const std::vector<PassageTraversalEdge> topology{
      makeTraversal("opening", {{2.0, 0.0, 5.0}, {8.0, 0.0, 5.0}}),
  };

  EXPECT_TRUE(selectRoutePassageTraversals3D(route, topology).empty());
}

TEST(PassageTraversalSelection3DTest,
     InvalidOrOverlappingTopologySegmentSpansAreNotPublished) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}}, 0.25, 4.0);
  PassageTraversalEdge traversal =
      makeTraversal("opening", {{2.0, 0.0, 5.0}, {8.0, 0.0, 5.0}});
  traversal.segment_spans.push_back(PassageTraversalSegmentSpan{
      .passage_segment_id = "overlap",
      .begin_station_m = 2.0,
      .end_station_m = 4.0,
  });

  EXPECT_TRUE(selectRoutePassageTraversals3D(
                  route, std::span<const PassageTraversalEdge>{&traversal, 1U})
                  .empty());

  traversal.segment_spans.back().begin_station_m = 7.0;
  traversal.segment_spans.back().end_station_m = 8.0;
  EXPECT_TRUE(selectRoutePassageTraversals3D(
                  route, std::span<const PassageTraversalEdge>{&traversal, 1U})
                  .empty());
}

TEST(PassageTraversalSelection3DTest,
     AmbiguousOverlappingDecoratorsChooseTheClosestGeometryDeterministically) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}}, 0.25, 4.0);
  const std::vector<PassageTraversalEdge> topology{
      makeTraversal("offset", {{2.0, 0.0, 5.0}, {8.0, 0.0, 5.0}}, 0.5),
      makeTraversal("exact", {{2.0, 0.0, 5.0}, {8.0, 0.0, 5.0}}),
  };

  const std::vector<SelectedPassageTraversal> selected =
      selectRoutePassageTraversals3D(route, topology);

  ASSERT_EQ(selected.size(), 1U);
  EXPECT_EQ(selected.front().passage_traversal_id, "exact");
}

TEST(PassageTraversalSelection3DTest,
     MissingOrInvalidOptionalTopologyNeverRejectsTheRoute) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{0.0, 0.0, 5.0}, {10.0, 0.0, 5.0}}, 0.25, 4.0);
  PassageTraversalSelectionConfig3D invalid_config;
  invalid_config.maximum_centerline_distance_m = -1.0;

  EXPECT_TRUE(selectRoutePassageTraversals3D(route, {}).empty());
  EXPECT_TRUE(selectRoutePassageTraversals3D(
                  route,
                  std::vector<PassageTraversalEdge>{
                      makeTraversal("opening", {{2.0, 0.0, 5.0}, {8.0, 0.0, 5.0}})},
                  invalid_config)
                  .empty());
}

} // namespace
} // namespace drone_city_nav
