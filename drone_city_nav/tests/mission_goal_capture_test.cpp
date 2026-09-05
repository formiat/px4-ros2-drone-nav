#include "drone_city_nav/mission_goal_capture.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(MissionGoalCaptureLatchTest, RequiresExactTerminalRoute) {
  MissionGoalCaptureLatch latch;
  mppi::State state;
  state.x = 9.0F;
  state.y = 10.0F;
  state.z = 18.0F;

  const MissionGoalCaptureResult result = latch.update(MissionGoalCaptureObservation{
      .mission_goal = Point3{10.0, 10.0, 18.0},
      .state = state,
      .terminal_route_available = false,
  });

  EXPECT_FALSE(result.latched);
}

TEST(MissionGoalCaptureLatchTest, LatchesOnlyWhenRestingInsideTheCaptureRadius) {
  MissionGoalCaptureLatch latch;
  mppi::State state;
  state.x = 7.5F;
  state.y = 10.0F;
  state.z = 18.0F;
  // At rest but outside the capture radius: the mission has not accepted it.
  EXPECT_FALSE(latch
                   .update(MissionGoalCaptureObservation{
                       .mission_goal = Point3{10.0, 10.0, 18.0},
                       .state = state,
                       .terminal_route_available = true,
                   })
                   .latched);

  // Inside the capture radius but still moving: no stationary hold could be
  // certified there yet.
  state.x = 9.0F;
  state.vx = 1.0F;
  const MissionGoalCaptureResult moving = latch.update(MissionGoalCaptureObservation{
      .mission_goal = Point3{10.0, 10.0, 18.0},
      .state = state,
      .terminal_route_available = true,
  });
  EXPECT_FALSE(moving.latched);
  EXPECT_NEAR(moving.speed_mps, 1.0, 1.0e-6);

  // Resting one metre from the goal is a capture: the hold pins this rest
  // position, the vehicle never has to creep onto the exact coordinate.
  state.vx = 0.1F;
  const MissionGoalCaptureResult resting = latch.update(MissionGoalCaptureObservation{
      .mission_goal = Point3{10.0, 10.0, 18.0},
      .state = state,
      .terminal_route_available = true,
  });
  EXPECT_TRUE(resting.newly_latched);
  EXPECT_NEAR(resting.distance_m, 1.0, 1.0e-6);
}

TEST(MissionGoalCaptureLatchTest, ReleasesAfterLeavingTheCaptureRadius) {
  // A latched planner holds and flies no route. A vehicle that drifted out of
  // the capture radius cannot be acknowledged there, so the latch releases
  // and the route takes it back inside; resting inside latches it again.
  MissionGoalCaptureLatch latch;
  mppi::State state;
  state.x = 10.0F;
  state.y = 10.0F;
  state.z = 18.0F;
  ASSERT_TRUE(latch
                  .update(MissionGoalCaptureObservation{
                      .mission_goal = Point3{10.0, 10.0, 18.0},
                      .state = state,
                      .terminal_route_available = true,
                  })
                  .newly_latched);

  // Still inside the radius: the latch holds, moving or not.
  state.x = 11.5F;
  state.vx = 0.5F;
  const MissionGoalCaptureResult inside = latch.update(MissionGoalCaptureObservation{
      .mission_goal = Point3{10.0, 10.0, 18.0},
      .state = state,
      .terminal_route_available = false,
  });
  EXPECT_TRUE(inside.latched);
  EXPECT_FALSE(inside.newly_latched);

  state.x = 12.1F;
  state.vx = 0.0F;
  const MissionGoalCaptureResult outside = latch.update(MissionGoalCaptureObservation{
      .mission_goal = Point3{10.0, 10.0, 18.0},
      .state = state,
      .terminal_route_available = false,
  });
  EXPECT_FALSE(outside.latched);
  EXPECT_FALSE(outside.newly_latched);
  EXPECT_FALSE(latch.latchedFor(Point3{10.0, 10.0, 18.0}));

  // Back inside but moving: not yet.
  state.x = 11.0F;
  state.vx = 1.0F;
  EXPECT_FALSE(latch
                   .update(MissionGoalCaptureObservation{
                       .mission_goal = Point3{10.0, 10.0, 18.0},
                       .state = state,
                       .terminal_route_available = true,
                   })
                   .latched);

  state.vx = 0.0F;
  const MissionGoalCaptureResult relatched = latch.update(MissionGoalCaptureObservation{
      .mission_goal = Point3{10.0, 10.0, 18.0},
      .state = state,
      .terminal_route_available = true,
  });
  EXPECT_TRUE(relatched.latched);
  EXPECT_TRUE(relatched.newly_latched);
  EXPECT_TRUE(latch.latchedFor(Point3{10.0, 10.0, 18.0}));
}

TEST(MissionGoalCaptureLatchTest, NewMissionResetsLatch) {
  MissionGoalCaptureLatch latch;
  mppi::State state;
  state.x = 10.0F;
  state.y = 10.0F;
  state.z = 18.0F;
  ASSERT_TRUE(latch
                  .update(MissionGoalCaptureObservation{
                      .mission_goal = Point3{10.0, 10.0, 18.0},
                      .state = state,
                      .terminal_route_available = true,
                  })
                  .latched);

  const MissionGoalCaptureResult result = latch.update(MissionGoalCaptureObservation{
      .mission_goal = Point3{30.0, 30.0, 18.0},
      .state = state,
      .terminal_route_available = true,
  });

  EXPECT_FALSE(result.latched);
}

TEST(MissionGoalCaptureLatchTest, RequiresThreeDimensionalCapture) {
  MissionGoalCaptureLatch latch;
  mppi::State state;
  state.x = 10.0F;
  state.y = 10.0F;
  state.z = 21.0F;
  EXPECT_FALSE(latch
                   .update(MissionGoalCaptureObservation{
                       .mission_goal = Point3{10.0, 10.0, 18.0},
                       .state = state,
                       .terminal_route_available = true,
                   })
                   .latched);
  EXPECT_FALSE(latch.latchedFor(Point3{10.0, 10.0, 18.0}));
}

} // namespace
} // namespace drone_city_nav
