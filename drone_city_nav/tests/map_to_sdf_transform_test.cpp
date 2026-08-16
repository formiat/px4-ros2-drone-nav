#include "drone_city_nav/map_to_sdf_transform.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"

#include <gtest/gtest.h>

#include <numbers>

namespace drone_city_nav {
namespace {

TEST(MapToSdfTransformTest, IdentityRoundTripPreservesAllCoordinates) {
  const MapToSdfTransform transform{
      .sdf_x_from = MapHorizontalAxis::kX,
      .sdf_y_from = MapHorizontalAxis::kY,
      .sdf_x_offset_m = 0.0,
      .sdf_y_offset_m = 0.0,
  };
  transform.validate();

  const Point3 map{12.5, -8.25, 4.0};
  const Point3 sdf = transform.mapToSdf(map);
  const Point3 recovered = transform.sdfToMap(sdf);

  EXPECT_DOUBLE_EQ(map.x, sdf.x);
  EXPECT_DOUBLE_EQ(map.y, sdf.y);
  EXPECT_DOUBLE_EQ(map.z, sdf.z);
  EXPECT_DOUBLE_EQ(map.x, recovered.x);
  EXPECT_DOUBLE_EQ(map.y, recovered.y);
  EXPECT_DOUBLE_EQ(map.z, recovered.z);
}

TEST(MapToSdfTransformTest, SignedPermutationRoundTripPreservesAllCoordinates) {
  const MapToSdfTransform transform{
      .sdf_x_from = MapHorizontalAxis::kY,
      .sdf_y_from = MapHorizontalAxis::kX,
      .sdf_x_scale = -1.0,
      .sdf_y_scale = 1.0,
      .sdf_z_scale = -1.0,
      .sdf_x_offset_m = 20.0,
      .sdf_y_offset_m = -4.0,
      .sdf_z_offset_m = 10.0,
  };
  transform.validate();

  const Point3 map{3.0, 7.0, 2.0};
  const Point3 sdf = transform.mapToSdf(map);
  const Point3 recovered = transform.sdfToMap(sdf);

  EXPECT_DOUBLE_EQ(13.0, sdf.x);
  EXPECT_DOUBLE_EQ(-1.0, sdf.y);
  EXPECT_DOUBLE_EQ(8.0, sdf.z);
  EXPECT_DOUBLE_EQ(map.x, recovered.x);
  EXPECT_DOUBLE_EQ(map.y, recovered.y);
  EXPECT_DOUBLE_EQ(map.z, recovered.z);
}

TEST(MapToSdfTransformTest, RejectsRepeatedHorizontalAxis) {
  const MapToSdfTransform transform{
      .sdf_x_from = MapHorizontalAxis::kX,
      .sdf_y_from = MapHorizontalAxis::kX,
  };

  EXPECT_THROW(transform.validate(), std::invalid_argument);
}

TEST(Px4MapFrameTransformTest, UrbanEnuMapSwapsNorthAndEast) {
  const Px4MapFrameTransform transform{
      .map_origin = Point3{-17.0, -36.0, 1.8},
      .m00 = 0.0,
      .m01 = 1.0,
      .m10 = 1.0,
      .m11 = 0.0,
  };
  transform.validate();

  const Point2 map_position = transform.localPositionToMap(Point2{3.0, 5.0});
  const Point2 local_position = transform.mapPositionToLocal(map_position);
  const Point2 map_velocity = transform.localVectorToMap(Point2{2.0, -4.0});

  EXPECT_DOUBLE_EQ(-12.0, map_position.x);
  EXPECT_DOUBLE_EQ(-33.0, map_position.y);
  EXPECT_DOUBLE_EQ(3.0, local_position.x);
  EXPECT_DOUBLE_EQ(5.0, local_position.y);
  EXPECT_DOUBLE_EQ(-4.0, map_velocity.x);
  EXPECT_DOUBLE_EQ(2.0, map_velocity.y);
  EXPECT_NEAR(std::numbers::pi / 2.0, transform.px4HeadingToMapYaw(0.0), 1.0e-12);
  EXPECT_NEAR(0.0, transform.mapYawToPx4Heading(std::numbers::pi / 2.0), 1.0e-12);
  EXPECT_DOUBLE_EQ(-0.5, transform.mapYawRateToPx4(0.5));
}

TEST(Px4MapFrameTransformTest, RejectsNonOrthonormalMatrix) {
  const Px4MapFrameTransform transform{
      .m00 = 1.0,
      .m01 = 1.0,
      .m10 = 0.0,
      .m11 = 1.0,
  };

  EXPECT_THROW(transform.validate(), std::invalid_argument);
}

} // namespace
} // namespace drone_city_nav
