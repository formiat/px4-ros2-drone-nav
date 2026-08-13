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
      .desired_center_separation_m = 5.0,
      .directional_offset_fraction = 0.5,
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
  return OccupancyGrid3D{GridBounds3D{-15.0, -10.0, 0.0, 1.0, 50, 50, 10}};
}

[[nodiscard]] PassageVolume volume(const std::span<const RouteSample3D> route,
                                   const ConstrainedRouteSpan& constrained) {
  std::vector<PassageCrossSection> sections;
  for (const RouteSample3D& sample : route) {
    if (sample.station_m + 1.0e-9 < constrained.begin_station_m ||
        sample.station_m - 1.0e-9 > constrained.end_station_m) {
      continue;
    }
    sections.push_back(PassageCrossSection{
        .station_m = sample.station_m,
        .center = sample.position,
        .tangent = sample.tangent,
        .lateral_axis = Vec3{-sample.tangent.y, sample.tangent.x, 0.0},
        .secondary_axis = Vec3{0.0, 0.0, 1.0},
        .minimum_lateral_offset_m = -6.0,
        .maximum_lateral_offset_m = 6.0,
        .minimum_secondary_offset_m = -4.0,
        .maximum_secondary_offset_m = 4.0,
        .raw_validated = true,
    });
  }
  return PassageVolume{
      .passage_id = constrained.channel_id,
      .span_index = 0U,
      .begin_station_m = constrained.begin_station_m,
      .end_station_m = constrained.end_station_m,
      .minimum_lateral_offset_m = -6.0,
      .maximum_lateral_offset_m = 6.0,
      .minimum_secondary_offset_m = -4.0,
      .maximum_secondary_offset_m = 4.0,
      .minimum_physical_width_m = 14.0,
      .minimum_physical_secondary_extent_m = 9.0,
      .cross_sections = std::move(sections),
      .raw_validated = true,
  };
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

TEST(CooperativeChannelRoute, AppliesContinuousOppositeOffsetsByDirection) {
  const std::vector<RouteSample3D> forward = sampleRoute3D(
      std::vector<Point3>{{-10.0, 0.0, 5.0}, {30.0, 0.0, 5.0}}, 1.0, 10.0);
  const OccupancyGrid3D occupancy = emptyOccupancy();
  const ConstrainedRouteSpan forward_span = span(1);

  const CooperativeChannelRouteResult forward_result = applyCooperativeChannelCorridors(
      forward, std::vector<ConstrainedRouteSpan>{forward_span},
      std::vector<PassageVolume>{volume(forward, forward_span)}, occupancy,
      routeConfig());

  ASSERT_TRUE(forward_result.valid);
  ASSERT_EQ(forward_result.assignments.size(), 1U);
  EXPECT_TRUE(forward_result.assignments.front().applied());
  EXPECT_DOUBLE_EQ(forward_result.assignments.front().applied_lateral_offset_m, -3.0);
  const auto forward_middle = std::ranges::min_element(
      forward_result.route, {},
      [](const RouteSample3D& sample) { return std::abs(sample.position.x - 10.0); });
  ASSERT_NE(forward_middle, forward_result.route.end());
  EXPECT_NEAR(forward_middle->position.y, -3.0, 1.0e-6);
  EXPECT_NEAR(forward_result.route.front().position.y, 0.0, 1.0e-9);
  EXPECT_NEAR(forward_result.route.back().position.y, 0.0, 1.0e-9);

  const std::vector<RouteSample3D> reverse = sampleRoute3D(
      std::vector<Point3>{{30.0, 0.0, 5.0}, {-10.0, 0.0, 5.0}}, 1.0, 10.0);
  const ConstrainedRouteSpan reverse_span = span(-1);
  const CooperativeChannelRouteResult reverse_result = applyCooperativeChannelCorridors(
      reverse, std::vector<ConstrainedRouteSpan>{reverse_span},
      std::vector<PassageVolume>{volume(reverse, reverse_span)}, occupancy,
      routeConfig());

  ASSERT_TRUE(reverse_result.valid);
  ASSERT_EQ(reverse_result.assignments.size(), 1U);
  EXPECT_DOUBLE_EQ(reverse_result.assignments.front().applied_lateral_offset_m, 3.0);
  const auto reverse_middle = std::ranges::min_element(
      reverse_result.route, {},
      [](const RouteSample3D& sample) { return std::abs(sample.position.x - 10.0); });
  ASSERT_NE(reverse_middle, reverse_result.route.end());
  EXPECT_NEAR(reverse_middle->position.y, 3.0, 1.0e-6);
}

TEST(CooperativeChannelRoute, FallsBackToCenterlineOnRawCollision) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{-10.0, 0.0, 5.0}, {30.0, 0.0, 5.0}}, 1.0, 10.0);
  OccupancyGrid3D occupancy = emptyOccupancy();
  const std::optional<GridIndex3D> obstacle =
      occupancy.worldToCell(Point3{10.0, -3.0, 5.0});
  ASSERT_TRUE(obstacle.has_value());
  occupancy.setOccupied(obstacle.value_or(GridIndex3D{}));
  const ConstrainedRouteSpan constrained = span(1);

  const CooperativeChannelRouteResult result = applyCooperativeChannelCorridors(
      route, std::vector<ConstrainedRouteSpan>{constrained},
      std::vector<PassageVolume>{volume(route, constrained)}, occupancy, routeConfig());

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.assignments.size(), 1U);
  EXPECT_EQ(result.assignments.front().status,
            CooperativeChannelRouteStatus::kRawValidationRejected);
  EXPECT_DOUBLE_EQ(result.assignments.front().applied_lateral_offset_m, 0.0);
  EXPECT_TRUE(std::ranges::all_of(result.route, [](const RouteSample3D& sample) {
    return std::abs(sample.position.y) <= 1.0e-9;
  }));
}

TEST(CooperativeChannelRoute, PreservesContinuousOffsetThroughRightAngle) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{-10.0, 0.0, 5.0},
                                        {0.0, 0.0, 5.0},
                                        {20.0, 0.0, 5.0},
                                        {20.0, 20.0, 5.0},
                                        {20.0, 30.0, 5.0}},
                    1.0, 10.0);
  ConstrainedRouteSpan constrained = span(1);
  constrained.end_station_m = 50.0;
  constrained.envelope.back().station_m = 50.0;

  const CooperativeChannelRouteResult result = applyCooperativeChannelCorridors(
      route, std::vector<ConstrainedRouteSpan>{constrained},
      std::vector<PassageVolume>{volume(route, constrained)}, emptyOccupancy(),
      routeConfig());

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.assignments.size(), 1U);
  ASSERT_TRUE(result.assignments.front().applied());
  const auto corner =
      std::ranges::min_element(result.route, {}, [](const RouteSample3D& sample) {
        return std::hypot(sample.position.x - 23.0, sample.position.y + 3.0);
      });
  ASSERT_NE(corner, result.route.end());
  EXPECT_NEAR(corner->position.x, 23.0, 0.75);
  EXPECT_NEAR(corner->position.y, -3.0, 0.75);
}

TEST(CooperativeChannelRoute, UsesCenteredModeWhenTransitionDoesNotFit) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{-1.0, 0.0, 5.0}, {21.0, 0.0, 5.0}}, 1.0, 10.0);
  ConstrainedRouteSpan constrained = span(1);
  constrained.begin_station_m = 1.0;
  constrained.end_station_m = 21.0;

  const CooperativeChannelRouteResult result = applyCooperativeChannelCorridors(
      route, std::vector<ConstrainedRouteSpan>{constrained},
      std::vector<PassageVolume>{volume(route, constrained)}, emptyOccupancy(),
      routeConfig());

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.assignments.size(), 1U);
  EXPECT_EQ(result.assignments.front().status,
            CooperativeChannelRouteStatus::kInsufficientTransition);
  EXPECT_DOUBLE_EQ(result.assignments.front().applied_lateral_offset_m, 0.0);
}

TEST(CooperativeChannelRoute, UsesPhysicalCrossSectionForExecutionEnvelope) {
  const std::vector<RouteSample3D> route = sampleRoute3D(
      std::vector<Point3>{{-10.0, 0.0, 5.0}, {30.0, 0.0, 5.0}}, 1.0, 10.0);
  const ConstrainedRouteSpan constrained = span(1);
  PassageVolume passage = volume(route, constrained);
  passage.minimum_secondary_offset_m = 0.0;
  passage.maximum_secondary_offset_m = 0.0;
  for (PassageCrossSection& section : passage.cross_sections) {
    section.minimum_secondary_offset_m = -1.0;
    section.maximum_secondary_offset_m = 1.0;
  }

  const CooperativeChannelRouteResult result = applyCooperativeChannelCorridors(
      route, std::vector<ConstrainedRouteSpan>{constrained},
      std::vector<PassageVolume>{passage}, emptyOccupancy(), routeConfig());

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.constrained_spans.size(), 1U);
  ASSERT_FALSE(result.constrained_spans.front().envelope.empty());
  for (const RouteEnvelopeSample& envelope :
       result.constrained_spans.front().envelope) {
    EXPECT_LT(envelope.min_z_m, envelope.reference_z_m);
    EXPECT_GT(envelope.max_z_m, envelope.reference_z_m);
    EXPECT_GE(envelope.max_z_m - envelope.min_z_m, 2.0 - 1.0e-9);
  }
}

} // namespace
} // namespace drone_city_nav
