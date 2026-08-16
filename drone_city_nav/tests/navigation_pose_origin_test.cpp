#include "drone_city_nav/navigation_pose.hpp"

#include <gtest/gtest.h>

#include <numbers>

namespace drone_city_nav {
namespace {

TEST(NavigationPoseOriginTest, AppliesThreeDimensionalMapOrigin) {
  const Px4LocalPositionSample sample{
      .x_m = 3.0,
      .y_m = 4.0,
      .z_m = -2.0,
      .heading_rad = 0.25,
      .stamp_ns = 10,
      .xy_valid = true,
      .z_valid = true,
      .heading_good_for_control = true,
  };
  const Px4LocalPoseConfig config{
      .use_heading_for_yaw = true,
      .initial_heading_rad = 0.0,
      .map_origin_x_m = 18.0,
      .map_origin_y_m = 20.0,
      .map_origin_z_m = 1.8,
  };

  const auto pose = makeNavigationPoseFromPx4LocalPosition(sample, config);

  ASSERT_TRUE(pose.has_value());
  const NavigationPose2D actual = pose.value_or(NavigationPose2D{});
  EXPECT_DOUBLE_EQ(21.0, actual.pose.position.x);
  EXPECT_DOUBLE_EQ(24.0, actual.pose.position.y);
  EXPECT_DOUBLE_EQ(3.8, actual.altitude_m);
}

TEST(NavigationPoseOriginTest, ConvertsPx4NorthEastIntoUrbanEnuMap) {
  const Px4LocalPositionSample sample{
      .x_m = 3.0,
      .y_m = 4.0,
      .z_m = -2.0,
      .heading_rad = 0.0,
      .stamp_ns = 10,
      .xy_valid = true,
      .z_valid = true,
      .heading_good_for_control = true,
  };
  const Px4LocalPoseConfig config{
      .use_heading_for_yaw = true,
      .map_origin_x_m = 18.0,
      .map_origin_y_m = 20.0,
      .map_origin_z_m = 1.8,
      .px4_to_map_m00 = 0.0,
      .px4_to_map_m01 = 1.0,
      .px4_to_map_m10 = 1.0,
      .px4_to_map_m11 = 0.0,
  };

  const auto pose = makeNavigationPoseFromPx4LocalPosition(sample, config);

  ASSERT_TRUE(pose.has_value());
  const NavigationPose2D actual = pose.value_or(NavigationPose2D{});
  EXPECT_DOUBLE_EQ(22.0, actual.pose.position.x);
  EXPECT_DOUBLE_EQ(23.0, actual.pose.position.y);
  EXPECT_DOUBLE_EQ(3.8, actual.altitude_m);
  EXPECT_NEAR(std::numbers::pi / 2.0, actual.pose.yaw_rad, 1.0e-12);
}

} // namespace
} // namespace drone_city_nav
