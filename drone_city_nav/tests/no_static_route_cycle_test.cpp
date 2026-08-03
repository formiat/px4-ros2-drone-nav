#include "drone_city_nav/no_static_route_cycle.hpp"

#include <gtest/gtest.h>

#include <numbers>

namespace drone_city_nav {
namespace {

TEST(NoStaticRouteCycleTest, DetectsRepeatedEndpointWithoutMissionProgress) {
  NoStaticRouteCycleDetector detector{NoStaticRouteCycleConfig{
      .observation_window_s = 20.0,
      .minimum_generation_changes = 4U,
      .repeated_endpoint_radius_m = 2.0,
      .maximum_vehicle_displacement_m = 5.0,
      .maximum_mission_progress_m = 2.0,
  }};
  NoStaticRouteCycleResult result;
  for (std::uint64_t generation = 1U; generation <= 4U; ++generation) {
    result = detector.observe(NoStaticRouteCycleObservation{
        .guide_generation = generation,
        .stamp_ns = static_cast<std::int64_t>(generation) * 1'000'000'000,
        .vehicle_position = Point2{0.2 * static_cast<double>(generation), 0.0},
        .guide_endpoint = Point2{10.0 + static_cast<double>(generation % 2U), 5.0},
        .approach_heading_rad = 0.0,
        .mission_distance_m = 100.0 - 0.2 * static_cast<double>(generation),
    });
  }

  EXPECT_TRUE(result.cycle_detected);
  EXPECT_EQ(result.generation_changes, 4U);
}

TEST(NoStaticRouteCycleTest, DoesNotFlagRealMissionProgress) {
  NoStaticRouteCycleDetector detector{NoStaticRouteCycleConfig{
      .observation_window_s = 20.0,
      .minimum_generation_changes = 3U,
      .repeated_endpoint_radius_m = 2.0,
      .maximum_vehicle_displacement_m = 20.0,
      .maximum_mission_progress_m = 2.0,
  }};
  NoStaticRouteCycleResult result;
  for (std::uint64_t generation = 1U; generation <= 3U; ++generation) {
    result = detector.observe(NoStaticRouteCycleObservation{
        .guide_generation = generation,
        .stamp_ns = static_cast<std::int64_t>(generation) * 1'000'000'000,
        .vehicle_position = Point2{static_cast<double>(generation), 0.0},
        .guide_endpoint = Point2{10.0, 5.0},
        .mission_distance_m = 100.0 - 5.0 * static_cast<double>(generation),
    });
  }

  EXPECT_FALSE(result.cycle_detected);
}

TEST(NoStaticRouteCycleTest, SamplesDirectedEdgesAcrossFailedGuide) {
  const std::vector<Point2> guide{{0.0, 0.0}, {8.0, 0.0}, {8.0, 4.0}};

  const std::vector<NoStaticDirectedTabuSample> samples =
      sampleNoStaticDirectedTabu(guide, 4.0);

  ASSERT_EQ(samples.size(), 3U);
  EXPECT_DOUBLE_EQ(samples[0U].point.x, 4.0);
  EXPECT_DOUBLE_EQ(samples[1U].point.x, 8.0);
  EXPECT_NEAR(samples[0U].approach_heading_rad, 0.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(samples[2U].point.y, 4.0);
  EXPECT_NEAR(samples[2U].approach_heading_rad, std::numbers::pi / 2.0, 1.0e-9);
}

} // namespace
} // namespace drone_city_nav
