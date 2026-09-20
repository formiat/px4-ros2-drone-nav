#include "drone_city_nav/executed_horizon_clearance_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace drone_city_nav {
namespace {

// A 1 m lattice, three cells wide and deep, twenty cells long: the first ten
// cells along x are observed free, far from anything; the rest are unknown.
constexpr int kLength{20};
constexpr int kKnownLength{10};

[[nodiscard]] EsdfGrid3D grid() {
  return EsdfGrid3D{.width = kLength,
                    .height = 3,
                    .resolution_m = 1.0F,
                    .origin_x_m = 0.0F,
                    .origin_y_m = 0.0F,
                    .depth = 3,
                    .origin_z_m = 0.0F,
                    .outside_is_unknown = true};
}

[[nodiscard]] std::vector<float> esdf() {
  std::vector<float> field(static_cast<std::size_t>(kLength) * 3U * 3U, 5.0F);
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < 3; ++y) {
      for (int x = kKnownLength; x < kLength; ++x) {
        field[(static_cast<std::size_t>(z) * 3U + static_cast<std::size_t>(y)) *
                  static_cast<std::size_t>(kLength) +
              static_cast<std::size_t>(x)] = kUnknownEsdfDistanceM;
      }
    }
  }
  return field;
}

[[nodiscard]] SweptFootprintConfig pointFootprint() {
  SweptFootprintConfig footprint;
  footprint.radius_m = 0.0;
  footprint.lower_extent_m = 0.0;
  footprint.upper_extent_m = 0.0;
  footprint.perimeter_samples = 0U;
  footprint.radial_rings = 0U;
  footprint.axial_samples = 0U;
  footprint.sweep_step_m = 0.5;
  return footprint;
}

[[nodiscard]] FiniteMotionHorizon3D horizonAlongX(const float first_x,
                                                  const std::size_t states) {
  FiniteMotionHorizon3D horizon;
  for (std::size_t index = 0U; index < states; ++index) {
    MotionState3D state;
    state.x = first_x + static_cast<float>(index);
    state.y = 1.5F;
    state.z = 1.5F;
    horizon.states.push_back(state);
  }
  return horizon;
}

TEST(ExecutedHorizonClearance3DTest, TheFirstUnobservedSampleEndsTheObservedRange) {
  // From x = 0.5 the motion runs one metre per state. The segment leaving
  // x = 9.5 sweeps into the unknown cells from x = 10: the observed range
  // along the motion is the nine metres travelled before that segment.
  const ExecutedHorizonClearance3D clearance = measureExecutedHorizonClearance3D(
      horizonAlongX(0.5F, 15U), 0U, grid(), esdf(), pointFootprint(), 0.1);
  ASSERT_TRUE(clearance.available);
  EXPECT_TRUE(clearance.unobserved());
  EXPECT_NEAR(clearance.distanceToUnobservedM(), 9.0, 1.0e-6);
  // Unknown space is not a clearance constraint.
  EXPECT_FALSE(clearance.constrained());
  EXPECT_TRUE(std::isfinite(clearance.minimum_clearance_m));
}

TEST(ExecutedHorizonClearance3DTest, AMotionThroughObservedSpaceHasNoFrontier) {
  const ExecutedHorizonClearance3D clearance = measureExecutedHorizonClearance3D(
      horizonAlongX(0.5F, 8U), 0U, grid(), esdf(), pointFootprint(), 0.1);
  ASSERT_TRUE(clearance.available);
  EXPECT_FALSE(clearance.unobserved());
  EXPECT_TRUE(std::isinf(clearance.distanceToUnobservedM()));
}

TEST(ExecutedHorizonClearance3DTest, TheRangeIsMeasuredFromTheRemainingMotion) {
  // Execution has advanced four states: the observed range is what remains.
  const ExecutedHorizonClearance3D clearance = measureExecutedHorizonClearance3D(
      horizonAlongX(0.5F, 15U), 4U, grid(), esdf(), pointFootprint(), 0.1);
  ASSERT_TRUE(clearance.available);
  EXPECT_NEAR(clearance.distanceToUnobservedM(), 5.0, 1.0e-6);
}

[[nodiscard]] std::vector<RouteSample3D> routeAlongX(const double first_x,
                                                     const std::size_t samples) {
  std::vector<RouteSample3D> route;
  for (std::size_t index = 0U; index < samples; ++index) {
    route.push_back(RouteSample3D{
        .position = {first_x + static_cast<double>(index), 1.5, 1.5},
        .tangent = {1.0, 0.0, 0.0},
        .station_m = static_cast<double>(index),
    });
  }
  return route;
}

TEST(ExecutedHorizonClearance3DTest, TheRouteAheadReportsTheFrontierBeyondTheHorizon) {
  // The route runs from x = 0.5 one metre per sample; the vehicle projects at
  // station 2. The segment leaving x = 9.5 (station 9) sweeps into the unknown
  // cells from x = 10: the observed range along the route is seven metres.
  const std::optional<double> range_m = measureRouteObservedRange3D(
      routeAlongX(0.5, 20U), 2.0, 30.0, grid(), esdf(), pointFootprint());
  ASSERT_TRUE(range_m.has_value());
  EXPECT_NEAR(*range_m, 7.0, 1.0e-6);
}

TEST(ExecutedHorizonClearance3DTest, TheRouteIsProbedNoFartherThanTheLookahead) {
  EXPECT_FALSE(measureRouteObservedRange3D(routeAlongX(0.5, 20U), 2.0, 5.0, grid(),
                                           esdf(), pointFootprint())
                   .has_value());
  EXPECT_TRUE(measureRouteObservedRange3D(routeAlongX(0.5, 20U), 2.0, 8.0, grid(),
                                          esdf(), pointFootprint())
                  .has_value());
}

TEST(ExecutedHorizonClearance3DTest, ARouteThroughObservedSpaceHasNoFrontier) {
  EXPECT_FALSE(measureRouteObservedRange3D(routeAlongX(0.5, 8U), 0.0, 30.0, grid(),
                                           esdf(), pointFootprint())
                   .has_value());
}

[[nodiscard]] std::vector<float> esdfWithTightColumn(const int column,
                                                     const float clearance_m) {
  std::vector<float> field = esdf();
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < 3; ++y) {
      field[(static_cast<std::size_t>(z) * 3U + static_cast<std::size_t>(y)) *
                static_cast<std::size_t>(kLength) +
            static_cast<std::size_t>(column)] = clearance_m;
    }
  }
  return field;
}

TEST(ExecutedHorizonClearance3DTest, TheRouteAheadReportsWhereItRunsClose) {
  // The route runs from x = 0.5 one metre per sample and the vehicle projects
  // at station 2. Everything keeps 5 m but the column at x = 6, which keeps 3:
  // the segment that leaves station 5 sweeps through it, three metres ahead of
  // the projection, and the conservative query answers for less than the cell
  // distance because the sweep samples are not on its centre.
  const ExecutedHorizonClearance3D clearance =
      measureRouteClearance3D(routeAlongX(0.5, 20U), 2.0, 30.0, grid(),
                              esdfWithTightColumn(6, 3.0F), pointFootprint(), 2.0);
  ASSERT_TRUE(clearance.available);
  ASSERT_TRUE(clearance.constrained());
  EXPECT_NEAR(clearance.distanceToConstraintM(), 3.0, 1.0e-6);
  EXPECT_GT(clearance.constrainedClearanceM(), 0.0);
  EXPECT_LT(clearance.constrainedClearanceM(), 2.0);
  EXPECT_DOUBLE_EQ(clearance.minimum_clearance_m, clearance.constrainedClearanceM());
  // A point footprint is its own body: the body keeps the same clearance.
  EXPECT_DOUBLE_EQ(clearance.constrained_samples.front().body_clearance_m,
                   clearance.constrainedClearanceM());
}

// A true distance field to a wall plane at x = `wall_x_m` across the known
// part of the lattice, so that a wider footprint keeps less of it.
[[nodiscard]] std::vector<float> esdfToWallPlane(const float wall_x_m) {
  std::vector<float> field = esdf();
  for (int z = 0; z < 3; ++z) {
    for (int y = 0; y < 3; ++y) {
      for (int x = 0; x < kKnownLength; ++x) {
        field[(static_cast<std::size_t>(z) * 3U + static_cast<std::size_t>(y)) *
                  static_cast<std::size_t>(kLength) +
              static_cast<std::size_t>(x)] =
            std::abs(wall_x_m - (static_cast<float>(x) + 0.5F));
      }
    }
  }
  return field;
}

TEST(ExecutedHorizonClearance3DTest, TheBodyClearanceIsMeasuredWithTheBodyFootprint) {
  // The conservative query answers for the whole 3D offset of every sample,
  // so it is not monotone in the radius on a lattice; the property that
  // holds exactly is that a body of the envelope's own size keeps the
  // envelope's clearance, and a smaller body is measured on its own.
  SweptFootprintConfig footprint = pointFootprint();
  footprint.radius_m = 0.4;
  footprint.body_radius_m = 0.4;
  footprint.body_lower_extent_m = 0.0;
  footprint.body_upper_extent_m = 0.0;
  footprint.perimeter_samples = 8U;
  footprint.radial_rings = 1U;
  footprint.axial_samples = 2U;
  const ExecutedHorizonClearance3D same = measureRouteClearance3D(
      routeAlongX(0.5, 10U), 2.0, 7.0, grid(), esdfToWallPlane(9.5F), footprint, 2.0);
  ASSERT_TRUE(same.constrained());
  for (const ConstrainedHorizonSample3D& sample : same.constrained_samples) {
    EXPECT_DOUBLE_EQ(sample.body_clearance_m, sample.clearance_m);
  }

  footprint.body_radius_m = 0.2;
  const ExecutedHorizonClearance3D smaller = measureRouteClearance3D(
      routeAlongX(0.5, 10U), 2.0, 7.0, grid(), esdfToWallPlane(9.5F), footprint, 2.0);
  ASSERT_TRUE(smaller.constrained());
  ASSERT_EQ(smaller.constrained_samples.size(), same.constrained_samples.size());
  bool measured_on_its_own{false};
  for (std::size_t index = 0U; index < smaller.constrained_samples.size(); ++index) {
    const ConstrainedHorizonSample3D& sample = smaller.constrained_samples[index];
    EXPECT_TRUE(std::isfinite(sample.body_clearance_m));
    EXPECT_DOUBLE_EQ(sample.clearance_m, same.constrained_samples[index].clearance_m);
    measured_on_its_own =
        measured_on_its_own || sample.body_clearance_m != sample.clearance_m;
  }
  EXPECT_TRUE(measured_on_its_own);
}

TEST(ExecutedHorizonClearance3DTest, MemoryAnswersHowFarAMotionWasObserved) {
  // A 20 m cube of 0.25 m voxels, observed free for x below 6 m: the shaft of
  // r500 seen from inside, its far wall never looked at.
  ObservedOccupancyGrid3D occupancy{GridBounds3D{
      .resolution_m = 0.25, .width_cells = 80, .height_cells = 80, .depth_cells = 80}};
  for (int x = 0; x < 24; ++x) {
    for (int y = 0; y < 80; ++y) {
      for (int z = 0; z < 80; ++z) {
        occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree);
      }
    }
  }
  const Point3 vehicle{2.0, 10.0, 10.0};

  // Toward the unobserved space: 4 m of memory, to within a voxel.
  EXPECT_NEAR(
      measureObservedRangeAlong3D(occupancy, vehicle, Vec3{1.0, 0.0, 0.0}, 0.55, 30.0),
      4.0, 0.25);
  // Along it the memory reaches as far as it is probed.
  EXPECT_NEAR(
      measureObservedRangeAlong3D(occupancy, vehicle, Vec3{0.0, 1.0, 0.0}, 0.55, 6.0),
      6.0, 0.25);
  // The body is as wide as it is: beside the unobserved space, a motion along
  // it is not observed across its width.
  EXPECT_NEAR(measureObservedRangeAlong3D(occupancy, Point3{5.7, 10.0, 10.0},
                                          Vec3{0.0, 1.0, 0.0}, 0.55, 6.0),
              0.55, 1.0e-9);
  EXPECT_DOUBLE_EQ(measureObservedRangeAlong3D(occupancy, vehicle, Vec3{}, 0.55, 30.0),
                   0.0);

  // A known wall closes the lines that meet it: beside a wall along x = 4 m
  // the body's width brushes it, and what lies unobserved behind the wall is
  // not what a motion along it enters.
  for (int y = 0; y < 80; ++y) {
    for (int z = 0; z < 80; ++z) {
      occupancy.setState(GridIndex3D{16, y, z}, ObservedVoxelState::kOccupied);
      for (int x = 17; x < 24; ++x) {
        occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kUnknown);
      }
    }
  }
  EXPECT_NEAR(measureObservedRangeAlong3D(occupancy, Point3{3.4, 10.0, 10.0},
                                          Vec3{0.2, 1.0, 0.0}, 0.55, 6.0),
              6.0, 0.25);
}

TEST(ExecutedHorizonClearance3DTest, AHorizonMayNotCarrySpeedAlongAMotionNothingSees) {
  // The 20 m cube again, observed free for x below 6 m, and the stereo set:
  // 6.4 m inside 60 degrees of the heading, 2.8 m inside 22.5 of the vertical.
  ObservedOccupancyGrid3D occupancy{GridBounds3D{
      .resolution_m = 0.25, .width_cells = 80, .height_cells = 80, .depth_cells = 80}};
  for (int x = 0; x < 24; ++x) {
    for (int y = 0; y < 80; ++y) {
      for (int z = 0; z < 80; ++z) {
        occupancy.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree);
      }
    }
  }
  SensorBrakingContract3D contract;
  contract.guaranteed_detection_range_m = 6.4;
  contract.physical_margin_m = 2.0;
  contract.forward_vertical_half_angle_rad = 0.9145;
  contract.forward_horizontal_half_angle_rad = 1.0472;
  contract.vertical_detection_range_m = 2.8;
  contract.vertical_cone_half_angle_rad = 0.3927;
  contract.vertical_physical_margin_m = 1.0;
  const StoppingCapability capability;
  const auto first_unseen = [&](const std::vector<MotionState3D>& horizon) {
    return firstUnseenMotionState3D(horizon, occupancy, contract, capability, 10.0,
                                    0.55, 0.25, 0.27);
  };

  // r518: the horizon turns toward the unobserved space at 2 m/s while its
  // planned heading still faces away. The first such state is named.
  const std::vector<MotionState3D> turning{
      MotionState3D{.x = 3.0F, .y = 10.0F, .z = 10.0F, .vy = 2.0F, .yaw = 1.5708F},
      MotionState3D{
          .x = 3.2F, .y = 10.4F, .z = 10.0F, .vx = 1.4F, .vy = 1.4F, .yaw = 1.5708F},
      MotionState3D{.x = 3.6F, .y = 10.6F, .z = 10.0F, .vx = 2.0F, .yaw = 2.8F},
      MotionState3D{.x = 4.0F, .y = 10.6F, .z = 10.0F, .vx = 2.0F, .yaw = 2.8F},
  };
  EXPECT_EQ(first_unseen(turning), 2U);

  // Faced, the same motion is the forward sensor's to answer for.
  std::vector<MotionState3D> faced = turning;
  faced[2].yaw = 0.2F;
  faced[3].yaw = 0.2F;
  EXPECT_EQ(first_unseen(faced), faced.size());

  // Unfaced through space seen before, at a speed its depth admits, and at
  // rest anywhere.
  const std::vector<MotionState3D> backing{
      MotionState3D{.x = 5.0F, .y = 10.0F, .z = 10.0F, .vx = -1.0F, .yaw = 0.0F},
      MotionState3D{.x = 5.5F, .y = 10.0F, .z = 10.0F, .vx = 0.2F, .yaw = 3.0F},
  };
  EXPECT_EQ(first_unseen(backing), backing.size());

  // A hover correction drifts a few centimetres beside the body and is not
  // refused for it; the same drift kept up is, once it has travelled the
  // envelope's clearance.
  std::vector<MotionState3D> drifting;
  for (int step = 0; step < 30; ++step) {
    drifting.push_back(MotionState3D{.x = 5.6F + 0.015F * static_cast<float>(step),
                                     .y = 10.0F,
                                     .z = 10.0F,
                                     .vx = 0.3F,
                                     .yaw = 3.0F});
  }
  EXPECT_EQ(
      first_unseen(std::vector<MotionState3D>(drifting.begin(), drifting.begin() + 15)),
      15U);
  EXPECT_EQ(first_unseen(drifting), 19U);

  // A horizon that is slowing down is answering for a speed the vehicle
  // already has, however it got it.
  const std::vector<MotionState3D> braking{
      MotionState3D{.x = 5.0F, .y = 10.0F, .z = 10.0F, .vx = 2.5F, .yaw = 3.0F},
      MotionState3D{.x = 5.12F, .y = 10.0F, .z = 10.0F, .vx = 2.3F, .yaw = 3.0F},
      MotionState3D{.x = 5.23F, .y = 10.0F, .z = 10.0F, .vx = 2.1F, .yaw = 3.0F},
      MotionState3D{.x = 5.33F, .y = 10.0F, .z = 10.0F, .vx = 1.9F, .yaw = 3.0F},
  };
  EXPECT_EQ(first_unseen(braking), braking.size());

  // A lidar that sees all around refuses nothing.
  contract.forward_horizontal_half_angle_rad = 3.141592653589793;
  EXPECT_EQ(first_unseen(turning), turning.size());
}

TEST(ExecutedHorizonClearance3DTest, TheRouteClearanceStopsAtTheLookahead) {
  const ExecutedHorizonClearance3D near =
      measureRouteClearance3D(routeAlongX(0.5, 20U), 2.0, 2.0, grid(),
                              esdfWithTightColumn(6, 3.0F), pointFootprint(), 2.0);
  ASSERT_TRUE(near.available);
  EXPECT_FALSE(near.constrained());
}

TEST(ExecutedHorizonClearance3DTest, TheRouteClearanceIgnoresUnobservedSpace) {
  // Unknown space stays traversable and carries no clearance, so a route that
  // leaves the observed cells reports no constraint and no frontier here.
  const ExecutedHorizonClearance3D clearance = measureRouteClearance3D(
      routeAlongX(0.5, 20U), 2.0, 30.0, grid(), esdf(), pointFootprint(), 2.0);
  ASSERT_TRUE(clearance.available);
  EXPECT_FALSE(clearance.constrained());
  EXPECT_FALSE(clearance.unobserved());
}

} // namespace
} // namespace drone_city_nav
