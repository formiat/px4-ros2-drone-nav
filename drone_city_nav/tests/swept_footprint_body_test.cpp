#include "drone_city_nav/proprioceptive_contact_seed_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

// The thrust axis tilts by atan(horizontal / (gravity - vertical)) when the
// controller accelerates at both limits while descending.
TEST(SweptFootprintTest, MaximumBodyTiltFollowsTheThrustAxisAtTheDynamicsLimits) {
  EXPECT_NEAR(maximumBodyTiltRad(4.0, 4.0, 9.80665), std::atan2(4.0, 5.80665), 1.0e-12);
  EXPECT_NEAR(maximumBodyTiltRad(4.0, 0.0, 9.80665), std::atan2(4.0, 9.80665), 1.0e-12);
  EXPECT_DOUBLE_EQ(maximumBodyTiltRad(0.0, 4.0, 9.80665), 0.0);
  EXPECT_NEAR(maximumBodyTiltRad(4.0, 12.0, 9.80665), std::numbers::pi / 2.0, 1.0e-12);
  const FootprintBodyAxis axis = bodyAxisFromWorldAcceleration(Vec3{4.0, 0.0, -4.0});
  EXPECT_NEAR(std::acos(axis.z), maximumBodyTiltRad(4.0, 4.0), 1.0e-9);
}

// Every point of the physical body, tilted in any direction by any angle up
// to the limit, lies inside the upright body the enveloped footprint
// describes, and the envelope contains that body; at zero tilt the footprint
// is unchanged.
TEST(SweptFootprintTest, TiltEnvelopedFootprintContainsThePhysicalBodyAtEveryTilt) {
  const SweptFootprintConfig footprint{
      .radius_m = 0.82,
      .lower_extent_m = 0.23,
      .upper_extent_m = 0.35,
      .body_radius_m = 0.55,
      .perimeter_samples = 12U,
      .radial_rings = 2U,
      .axial_samples = 3U,
      .sweep_step_m = 0.25,
      .safe_clearance_threshold_m = 0.1,
  };
  const SweptFootprintConfig untilted = tiltEnvelopedFootprint(footprint, 0.0);
  EXPECT_NEAR(untilted.radius_m, footprint.radius_m, 1.0e-12);
  EXPECT_NEAR(untilted.body_radius_m, footprint.body_radius_m, 1.0e-12);
  EXPECT_NEAR(untilted.lower_extent_m, footprint.lower_extent_m, 1.0e-12);
  EXPECT_NEAR(untilted.upper_extent_m, footprint.upper_extent_m, 1.0e-12);

  const double tilt_rad = maximumBodyTiltRad(4.0, 4.0);
  const SweptFootprintConfig enveloped = tiltEnvelopedFootprint(footprint, tilt_rad);
  // The body becomes the hull at every tilt; the envelope contains it and
  // keeps the clearance it already carried.
  EXPECT_GT(enveloped.body_radius_m, footprint.body_radius_m);
  EXPECT_GT(enveloped.body_lower_extent_m, footprint.body_lower_extent_m);
  EXPECT_GT(enveloped.body_upper_extent_m, footprint.body_upper_extent_m);
  EXPECT_NEAR(enveloped.radius_m, footprint.radius_m, 1.0e-12);
  EXPECT_GE(enveloped.radius_m, enveloped.body_radius_m);
  EXPECT_NEAR(enveloped.lower_extent_m, enveloped.body_lower_extent_m, 1.0e-12);
  EXPECT_NEAR(enveloped.upper_extent_m, enveloped.body_upper_extent_m, 1.0e-12);
  const SweptFootprintConfig body_at_any_tilt = physicalBodyFootprint(enveloped);
  EXPECT_NEAR(body_at_any_tilt.radius_m, enveloped.body_radius_m, 1.0e-12);
  EXPECT_NEAR(body_at_any_tilt.lower_extent_m, enveloped.body_lower_extent_m, 1.0e-12);
  EXPECT_NEAR(body_at_any_tilt.upper_extent_m, enveloped.body_upper_extent_m, 1.0e-12);
  EXPECT_EQ(enveloped.perimeter_samples, footprint.perimeter_samples);
  EXPECT_EQ(enveloped.sweep_step_m, footprint.sweep_step_m);
  EXPECT_EQ(enveloped.safe_clearance_threshold_m, footprint.safe_clearance_threshold_m);
  constexpr double kTolerance{1.0e-9};
  const double body_radius_m = footprint.body_radius_m;
  for (int tilt_step = 0; tilt_step <= 8; ++tilt_step) {
    const double tilt = tilt_rad * tilt_step / 8.0;
    for (int heading_step = 0; heading_step < 12; ++heading_step) {
      const double heading = 2.0 * std::numbers::pi * heading_step / 12.0;
      // The tilted axis and two perpendicular directions spanning its rim.
      const double ax = std::sin(tilt) * std::cos(heading);
      const double ay = std::sin(tilt) * std::sin(heading);
      const double az = std::cos(tilt);
      const double ux = std::cos(tilt) * std::cos(heading);
      const double uy = std::cos(tilt) * std::sin(heading);
      const double uz = -std::sin(tilt);
      const double vx = -std::sin(heading);
      const double vy = std::cos(heading);
      for (const double axial :
           {-footprint.lower_extent_m, 0.0, footprint.upper_extent_m}) {
        for (int rim_step = 0; rim_step < 16; ++rim_step) {
          const double rim = 2.0 * std::numbers::pi * rim_step / 16.0;
          const double rx = std::cos(rim) * ux + std::sin(rim) * vx;
          const double ry = std::cos(rim) * uy + std::sin(rim) * vy;
          const double rz = std::cos(rim) * uz;
          const double x = axial * ax + body_radius_m * rx;
          const double y = axial * ay + body_radius_m * ry;
          const double z = axial * az + body_radius_m * rz;
          EXPECT_LE(std::hypot(x, y), enveloped.body_radius_m + kTolerance);
          EXPECT_GE(z, -enveloped.body_lower_extent_m - kTolerance);
          EXPECT_LE(z, enveloped.body_upper_extent_m + kTolerance);
        }
      }
    }
  }
}

TEST(SweptFootprintTest, LeanedReachGrowsTheRimAndKeepsTheExtents) {
  // The x500 body with the lean the urban dynamics command, 0.444 rad: the rim
  // reaches 0.648 m instead of 0.55 m while the body keeps its 0.23 m dip, so
  // a braking vehicle cannot put a rotor into a wall the upright body cleared.
  const SweptFootprintConfig footprint{.radius_m = 0.82,
                                       .lower_extent_m = 0.23,
                                       .upper_extent_m = 0.35,
                                       .body_radius_m = 0.55,
                                       .body_lower_extent_m = 0.23,
                                       .body_upper_extent_m = 0.35};
  const double tilt_rad = maximumBodyTiltRad(4.0, 1.4);

  const SweptFootprintConfig upright = leanedReachFootprint(footprint, 0.0);
  EXPECT_DOUBLE_EQ(upright.body_radius_m, footprint.body_radius_m);
  EXPECT_DOUBLE_EQ(upright.radius_m, footprint.radius_m);

  const SweptFootprintConfig leaned = leanedReachFootprint(footprint, tilt_rad);
  EXPECT_NEAR(leaned.body_radius_m,
              0.55 * std::cos(tilt_rad) + 0.35 * std::sin(tilt_rad), 1.0e-9);
  EXPECT_GT(leaned.body_radius_m, footprint.body_radius_m);
  EXPECT_DOUBLE_EQ(leaned.body_lower_extent_m, footprint.body_lower_extent_m);
  EXPECT_DOUBLE_EQ(leaned.body_upper_extent_m, footprint.body_upper_extent_m);
  EXPECT_DOUBLE_EQ(leaned.lower_extent_m, footprint.lower_extent_m);
  EXPECT_DOUBLE_EQ(leaned.upper_extent_m, footprint.upper_extent_m);
  EXPECT_DOUBLE_EQ(leaned.radius_m, footprint.radius_m);

  // A wall face 0.60 m from the flight path: the upright body clears it, the
  // leaning one does not. This is the r320 geometry, where a rotor passed
  // 0.114 m beyond a mapped face at 0.29 rad of roll.
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 0.25, 40, 40, 40}};
  for (int z = 18; z <= 26; ++z) {
    for (int x = 0; x < 40; ++x) {
      ASSERT_TRUE(
          occupancy.setState(GridIndex3D{x, 26, z}, ObservedVoxelState::kOccupied));
    }
  }
  const Point3 pose{2.0, 5.9, 5.5};
  EXPECT_TRUE(validateRawFootprintAt(occupancy, pose, FootprintBodyAxis{},
                                     physicalBodyFootprint(footprint))
                  .accepted());
  EXPECT_EQ(validateRawFootprintAt(occupancy, pose, FootprintBodyAxis{},
                                   physicalBodyFootprint(leaned))
                .status,
            SweptFootprintStatus::kRawCollision);
}

} // namespace
} // namespace drone_city_nav
