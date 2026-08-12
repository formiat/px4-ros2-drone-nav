#include "drone_city_nav/cooperative_channel_route.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] CooperativeChannelRouteConfig routeConfig() {
  return CooperativeChannelRouteConfig{
      .preferred_transition_length_m = 5.0,
      .minimum_transition_length_m = 2.0,
      .footprint =
          SweptFootprintConfig{
              .radius_m = 0.5,
              .lower_extent_m = 0.25,
              .upper_extent_m = 0.25,
              .perimeter_samples = 12U,
              .radial_rings = 2U,
              .axial_samples = 3U,
              .sweep_step_m = 0.25,
          },
  };
}

[[nodiscard]] OccupancyGrid3D emptyOccupancy() {
  return OccupancyGrid3D{GridBounds3D{-15.0, -10.0, 0.0, 1.0, 50, 20, 10}};
}

[[nodiscard]] ConstrainedFreeSpaceEdge channel() {
  return ConstrainedFreeSpaceEdge{
      .id = "channel_test",
      .centerline = sampleRoute3D(
          std::vector<Point3>{{0.0, 0.0, 5.0}, {20.0, 0.0, 5.0}}, 1.0, 10.0),
      .entry = Point3{0.0, 0.0, 5.0},
      .exit = Point3{20.0, 0.0, 5.0},
      .min_z_m = 1.0,
      .max_z_m = 9.0,
      .width_m = 14.0,
      .height_m = 8.0,
      .minimum_clearance_m = 4.0,
      .speed_limit_mps = 10.0,
  };
}

[[nodiscard]] ChannelLaneSet lanes() {
  return makeGeometricChannelLanes(channel(), ChannelLaneConfig{
                                                  .minimum_center_separation_m = 5.0,
                                                  .minimum_wall_clearance_m = 0.5,
                                                  .maximum_lane_count = 5U,
                                                  .footprint = routeConfig().footprint,
                                              });
}

[[nodiscard]] ConstrainedRouteSpan span(const int direction_sign) {
  return ConstrainedRouteSpan{
      .channel_id = "channel_test",
      .route_generation = 7U,
      .direction_sign = direction_sign,
      .begin_station_m = 10.0,
      .end_station_m = 30.0,
      .envelope =
          {
              RouteEnvelopeSample{
                  .station_m = 10.0,
                  .lateral_free_left_m = 7.0,
                  .lateral_free_right_m = 7.0,
                  .min_z_m = 1.0,
                  .max_z_m = 9.0,
                  .minimum_clearance_m = 4.0,
                  .reference_z_m = 5.0,
                  .reference_speed_mps = 10.0,
              },
              RouteEnvelopeSample{
                  .station_m = 30.0,
                  .lateral_free_left_m = 7.0,
                  .lateral_free_right_m = 7.0,
                  .min_z_m = 1.0,
                  .max_z_m = 9.0,
                  .minimum_clearance_m = 4.0,
                  .reference_z_m = 5.0,
                  .reference_speed_mps = 10.0,
              },
          },
  };
}

TEST(CooperativeChannelRoute, AppliesOppositePhysicalLanesByDirection) {
  const std::vector<RouteSample3D> forward = sampleRoute3D(
      std::vector<Point3>{{-10.0, 0.0, 5.0}, {30.0, 0.0, 5.0}}, 1.0, 10.0);
  const ChannelLaneSet lane_set = lanes();
  const OccupancyGrid3D occupancy = emptyOccupancy();

  const CooperativeChannelRouteResult forward_result = applyCooperativeChannelLanes(
      forward, std::vector<ConstrainedRouteSpan>{span(1)},
      std::vector<ChannelLaneSet>{lane_set}, occupancy, routeConfig());

  ASSERT_TRUE(forward_result.valid);
  ASSERT_EQ(forward_result.assignments.size(), 1U);
  EXPECT_TRUE(forward_result.assignments.front().applied());
  EXPECT_EQ(forward_result.assignments.front().lane_index, 0U);
  EXPECT_EQ(forward_result.assignments.front().lane_count, 3U);
  const auto forward_middle = std::ranges::min_element(
      forward_result.route, {},
      [](const RouteSample3D& sample) { return std::abs(sample.position.x - 10.0); });
  ASSERT_NE(forward_middle, forward_result.route.end());
  EXPECT_NEAR(forward_middle->position.y, -5.0, 1.0e-6);
  EXPECT_NEAR(forward_result.route.front().position.y, 0.0, 1.0e-9);
  EXPECT_NEAR(forward_result.route.back().position.y, 0.0, 1.0e-9);

  const std::vector<RouteSample3D> reverse = sampleRoute3D(
      std::vector<Point3>{{30.0, 0.0, 5.0}, {-10.0, 0.0, 5.0}}, 1.0, 10.0);
  const CooperativeChannelRouteResult reverse_result = applyCooperativeChannelLanes(
      reverse, std::vector<ConstrainedRouteSpan>{span(-1)},
      std::vector<ChannelLaneSet>{lane_set}, occupancy, routeConfig());

  ASSERT_TRUE(reverse_result.valid);
  ASSERT_EQ(reverse_result.assignments.size(), 1U);
  EXPECT_EQ(reverse_result.assignments.front().lane_index, 2U);
  const auto reverse_middle = std::ranges::min_element(
      reverse_result.route, {},
      [](const RouteSample3D& sample) { return std::abs(sample.position.x - 10.0); });
  ASSERT_NE(reverse_middle, reverse_result.route.end());
  EXPECT_NEAR(reverse_middle->position.y, 5.0, 1.0e-6);
}

TEST(CooperativeChannelRoute, FallsBackToExclusiveCenterlineOnRawCollision) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{-10.0, 0.0, 5.0}, {30.0, 0.0, 5.0}}, 1.0, 10.0);
  OccupancyGrid3D occupancy = emptyOccupancy();
  const std::optional<GridIndex3D> obstacle =
      occupancy.worldToCell(Point3{10.0, -5.0, 5.0});
  ASSERT_TRUE(obstacle.has_value());
  occupancy.setOccupied(obstacle.value_or(GridIndex3D{}));

  const CooperativeChannelRouteResult result = applyCooperativeChannelLanes(
      route, std::vector<ConstrainedRouteSpan>{span(1)},
      std::vector<ChannelLaneSet>{lanes()}, occupancy, routeConfig());

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.assignments.size(), 1U);
  EXPECT_EQ(result.assignments.front().lane_count, 1U);
  EXPECT_EQ(result.assignments.front().status,
            CooperativeChannelLaneRouteStatus::kRawValidationRejected);
  EXPECT_TRUE(std::ranges::all_of(result.route, [](const RouteSample3D& sample) {
    return std::abs(sample.position.y) <= 1.0e-9;
  }));
}

TEST(CooperativeChannelRoute, PreservesMiteredLaneThroughRightAngleChannel) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{-10.0, 0.0, 5.0},
                                        {0.0, 0.0, 5.0},
                                        {20.0, 0.0, 5.0},
                                        {20.0, 20.0, 5.0},
                                        {20.0, 30.0, 5.0}},
                    1.0, 10.0);
  ConstrainedFreeSpaceEdge turning_channel = channel();
  turning_channel.centerline = sampleRoute3D(
      std::vector<Point3>{{0.0, 0.0, 5.0}, {20.0, 0.0, 5.0}, {20.0, 20.0, 5.0}}, 1.0,
      10.0);
  const ChannelLaneSet lane_set = makeGeometricChannelLanes(
      turning_channel, ChannelLaneConfig{
                           .minimum_center_separation_m = 5.0,
                           .minimum_wall_clearance_m = 0.5,
                           .maximum_lane_count = 5U,
                           .footprint = routeConfig().footprint,
                       });
  ConstrainedRouteSpan constrained = span(1);
  constrained.end_station_m = 50.0;
  constrained.envelope.back().station_m = 50.0;
  const OccupancyGrid3D occupancy{GridBounds3D{-15.0, -10.0, 0.0, 1.0, 50, 50, 10}};

  const CooperativeChannelRouteResult result = applyCooperativeChannelLanes(
      route, std::vector<ConstrainedRouteSpan>{constrained},
      std::vector<ChannelLaneSet>{lane_set}, occupancy, routeConfig());

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.assignments.size(), 1U);
  ASSERT_TRUE(result.assignments.front().applied());
  const auto corner =
      std::ranges::min_element(result.route, {}, [](const RouteSample3D& sample) {
        return std::hypot(sample.position.x - 25.0, sample.position.y + 5.0);
      });
  ASSERT_NE(corner, result.route.end());
  EXPECT_NEAR(corner->position.x, 25.0, 0.75);
  EXPECT_NEAR(corner->position.y, -5.0, 0.75);
}

TEST(CooperativeChannelRoute, UsesExclusiveModeWhenTransitionDoesNotFit) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{-1.0, 0.0, 5.0}, {21.0, 0.0, 5.0}}, 1.0, 10.0);
  ConstrainedRouteSpan constrained = span(1);
  constrained.begin_station_m = 1.0;
  constrained.end_station_m = 21.0;

  const CooperativeChannelRouteResult result = applyCooperativeChannelLanes(
      route, std::vector<ConstrainedRouteSpan>{constrained},
      std::vector<ChannelLaneSet>{lanes()}, emptyOccupancy(), routeConfig());

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.assignments.size(), 1U);
  EXPECT_EQ(result.assignments.front().lane_count, 1U);
  EXPECT_EQ(result.assignments.front().status,
            CooperativeChannelLaneRouteStatus::kInsufficientTransition);
}

} // namespace
} // namespace drone_city_nav
