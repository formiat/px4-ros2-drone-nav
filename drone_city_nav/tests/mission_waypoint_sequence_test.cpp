#include "drone_city_nav/mission_waypoint_sequence.hpp"

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

namespace drone_city_nav {
namespace {

TEST(MissionWaypointSequenceTest, RejectsAnEmptyParameter) {
  EXPECT_THROW(missionWaypointsFromFlatParameters(std::array<double, 0U>{}),
               std::invalid_argument);
}

TEST(MissionWaypointSequenceTest, RejectsIncompleteWaypointTriples) {
  EXPECT_THROW(missionWaypointsFromFlatParameters(std::array<double, 2U>{1.0, 2.0}),
               std::invalid_argument);
}

TEST(MissionWaypointSequenceTest, RequiresTerminalStopBeforeAdvancing) {
  MissionWaypointSequence sequence{{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}},
                                   MissionWaypointSequenceConfig{.goal_radius_m = 2.0,
                                                                 .stop_speed_mps = 0.8,
                                                                 .stop_hold_s = 2.0}};

  EXPECT_FALSE(sequence
                   .update({.stamp_ns = 1'000'000'000,
                            .goal_captured = true,
                            .horizontal_speed_mps = 1.0})
                   .waypoint_completed);
  EXPECT_FALSE(sequence
                   .update({.stamp_ns = 2'000'000'000,
                            .goal_captured = true,
                            .horizontal_speed_mps = 0.5})
                   .waypoint_completed);
  const MissionWaypointUpdate advanced = sequence.update(
      {.stamp_ns = 4'000'000'000, .goal_captured = true, .horizontal_speed_mps = 0.5});
  EXPECT_TRUE(advanced.waypoint_completed);
  EXPECT_TRUE(advanced.advanced);
  EXPECT_FALSE(advanced.mission_completed);
  EXPECT_EQ(advanced.completed_index, 0U);
  EXPECT_EQ(sequence.activeIndex(), 1U);
  EXPECT_EQ(sequence.completedWaypointCount(), 1U);
}

TEST(MissionWaypointSequenceTest, CompletesOnlyAfterTheLastWaypoint) {
  MissionWaypointSequence sequence{{Point3{1.0, 2.0, 3.0}, Point3{4.0, 5.0, 6.0}},
                                   MissionWaypointSequenceConfig{.goal_radius_m = 2.0,
                                                                 .stop_speed_mps = 0.8,
                                                                 .stop_hold_s = 0.0}};

  const MissionWaypointUpdate first = sequence.update(
      {.stamp_ns = 1, .goal_captured = true, .horizontal_speed_mps = 0.0});
  EXPECT_FALSE(first.waypoint_completed);
  const MissionWaypointUpdate second = sequence.update(
      {.stamp_ns = 2, .goal_captured = true, .horizontal_speed_mps = 0.0});
  EXPECT_TRUE(second.advanced);
  const MissionWaypointUpdate final = sequence.update(
      {.stamp_ns = 3, .goal_captured = true, .horizontal_speed_mps = 0.0});
  EXPECT_FALSE(final.waypoint_completed);
  const MissionWaypointUpdate complete = sequence.update(
      {.stamp_ns = 4, .goal_captured = true, .horizontal_speed_mps = 0.0});
  EXPECT_TRUE(complete.waypoint_completed);
  EXPECT_TRUE(complete.mission_completed);
  EXPECT_TRUE(sequence.missionCompleted());
  EXPECT_EQ(sequence.completedWaypointCount(), 2U);
}

} // namespace
} // namespace drone_city_nav
