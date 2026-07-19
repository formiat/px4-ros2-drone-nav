#include "drone_city_nav/passage_traversal_monitor.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] PassageOpening testOpening() {
  return PassageOpening{
      .id = "opening",
      .structure_id = "structure",
      .center = Point3{0.0, 0.0, 5.0},
      .normal_xy = Point2{1.0, 0.0},
      .width_m = 10.0,
      .height_m = 6.0,
      .depth_m = 4.0,
      .min_z_m = 2.0,
      .max_z_m = 8.0,
  };
}

void observe(PassageTraversalMonitor& monitor,
             const KnownPassageOpeningWorldPoint& observation) {
  static_cast<void>(monitor.update(observation));
}

[[nodiscard]] KnownPassageOpeningFrame testFrame() {
  return KnownPassageOpeningFrame{
      .center = Point2{0.0, 0.0},
      .normal = Point2{1.0, 0.0},
      .lateral = Point2{0.0, 1.0},
      .half_width_m = 5.0,
      .half_depth_m = 2.0,
  };
}

[[nodiscard]] KnownPassageOpeningWorldPoint point(const double depth_m,
                                                  const double lateral_m = 0.0,
                                                  const double altitude_m = 5.0) {
  return KnownPassageOpeningWorldPoint{
      .point = Point2{depth_m, lateral_m},
      .z_m = altitude_m,
  };
}

TEST(PassageTraversalMonitorTest, WallClearanceExcludesDepthPlanes) {
  const PassageMargins margins = passageMargins(
      KnownPassageOpeningLocalPoint{.u_m = -1.99, .v_m = 0.0, .z_m = 5.0},
      testOpening(), testFrame());

  EXPECT_NEAR(margins.boundary_margin_m, 0.01, 1.0e-9);
  EXPECT_DOUBLE_EQ(margins.wall_clearance_m, 3.0);
  EXPECT_EQ(margins.nearest_boundary, PassageBoundary::kDepthEntry);
  EXPECT_EQ(margins.nearest_wall_boundary, PassageBoundary::kVerticalLower);
}

TEST(PassageTraversalMonitorTest, EnteringDoesNotImplyCompletedTraversal) {
  PassageTraversalMonitor monitor{testOpening(), testFrame()};

  observe(monitor, point(-3.0));
  const PassageTraversalUpdate inside = monitor.update(point(-1.5));

  EXPECT_TRUE(inside.entered_now);
  EXPECT_TRUE(monitor.metrics().entered);
  EXPECT_TRUE(monitor.metrics().entry_crossed);
  EXPECT_FALSE(monitor.metrics().exit_crossed);
  EXPECT_FALSE(monitor.metrics().completed);
}

TEST(PassageTraversalMonitorTest, CompletesEntryToExitTraversalWithHysteresis) {
  PassageTraversalMonitor monitor{testOpening(), testFrame(),
                                  PassageTraversalMonitorConfig{0.25}};

  observe(monitor, point(-2.5));
  observe(monitor, point(-1.7));
  observe(monitor, point(0.0));
  const PassageTraversalUpdate completed = monitor.update(point(2.3));

  EXPECT_TRUE(completed.completed_now);
  EXPECT_TRUE(monitor.metrics().entered);
  EXPECT_TRUE(monitor.metrics().entry_crossed);
  EXPECT_TRUE(monitor.metrics().exit_crossed);
  EXPECT_TRUE(monitor.metrics().completed);
  EXPECT_LE(monitor.metrics().min_local_depth_m, -1.7);
  EXPECT_GE(monitor.metrics().max_local_depth_m, 0.0);
}

TEST(PassageTraversalMonitorTest, DoesNotCompleteWithoutEntryApproach) {
  PassageTraversalMonitor monitor{testOpening(), testFrame()};

  observe(monitor, point(0.0));
  observe(monitor, point(2.5));

  EXPECT_TRUE(monitor.metrics().entered);
  EXPECT_FALSE(monitor.metrics().entry_crossed);
  EXPECT_FALSE(monitor.metrics().completed);
}

TEST(PassageTraversalMonitorTest, RejectsTraversalOutsideCrossSection) {
  PassageTraversalMonitor monitor{testOpening(), testFrame()};

  observe(monitor, point(-2.5));
  observe(monitor, point(-1.5, 5.5));
  observe(monitor, point(0.0, 5.5));
  observe(monitor, point(2.5, 5.5));

  EXPECT_TRUE(monitor.metrics().entry_crossed);
  EXPECT_FALSE(monitor.metrics().entered);
  EXPECT_FALSE(monitor.metrics().exit_crossed);
  EXPECT_FALSE(monitor.metrics().completed);
}

TEST(PassageTraversalMonitorTest, ReverseTraversalIsNotCompleted) {
  PassageTraversalMonitor monitor{testOpening(), testFrame()};

  observe(monitor, point(2.5));
  observe(monitor, point(0.0));
  observe(monitor, point(-2.5));

  EXPECT_TRUE(monitor.metrics().entered);
  EXPECT_FALSE(monitor.metrics().entry_crossed);
  EXPECT_FALSE(monitor.metrics().completed);
}

} // namespace
} // namespace drone_city_nav
