#include "drone_city_nav/lidar_projection.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace drone_city_nav {
namespace {

[[nodiscard]] LidarBeamProjection project(const LidarProjectionPose& pose,
                                          const LidarProjectionConfig& config,
                                          const float range_m,
                                          const double angle_min_rad = 0.0) {
  return projectLidarBeam(pose, config, 0.1, 35.0, angle_min_rad, 0.1, 0U, range_m);
}

[[nodiscard]] double norm(const Point3 vector) {
  return std::hypot(vector.x, vector.y, vector.z);
}

} // namespace

TEST(LidarProjection, ExplicitFluToFrdProjectionKeepsLevelForwardBeam) {
  const LidarProjectionPose pose{
      Point2{5.0, 6.0}, 18.0, std::numbers::pi / 2.0, 0.0, 0.0, true, true};
  LidarProjectionConfig config{};
  config.compensate_attitude = true;

  const LidarBeamProjection projection = project(pose, config, 4.0F);

  EXPECT_EQ(projection.status, LidarBeamProjectionStatus::kAccepted);
  EXPECT_TRUE(projection.hit);
  EXPECT_NEAR(projection.endpoint.x, 5.0, 1.0e-6);
  EXPECT_NEAR(projection.endpoint.y, 10.0, 1.0e-6);
  EXPECT_NEAR(projection.endpoint_altitude_m, 18.0, 1.0e-6);
  EXPECT_NEAR(projection.lidar_direction.x, 1.0, 1.0e-9);
  EXPECT_NEAR(projection.lidar_direction.y, 0.0, 1.0e-9);
  EXPECT_NEAR(projection.body_frd_direction.x, 1.0, 1.0e-9);
  EXPECT_NEAR(projection.body_frd_direction.y, 0.0, 1.0e-9);
  EXPECT_NEAR(norm(projection.ned_direction), 1.0, 1.0e-9);
  EXPECT_NEAR(norm(projection.ray_direction_map), 1.0, 1.0e-9);
  EXPECT_NEAR(projection.ray_origin_map_m.x, 5.0, 1.0e-9);
  EXPECT_NEAR(projection.ray_origin_map_m.y, 6.0, 1.0e-9);
  EXPECT_NEAR(projection.ray_origin_map_m.z, 18.0, 1.0e-9);
  EXPECT_NEAR(projection.endpoint_map_m.x,
              projection.ray_origin_map_m.x +
                  projection.used_range_m * projection.ray_direction_map.x,
              1.0e-9);
  EXPECT_NEAR(projection.endpoint_map_m.y,
              projection.ray_origin_map_m.y +
                  projection.used_range_m * projection.ray_direction_map.y,
              1.0e-9);
  EXPECT_NEAR(projection.endpoint_map_m.z,
              projection.ray_origin_map_m.z +
                  projection.used_range_m * projection.ray_direction_map.z,
              1.0e-9);
}

TEST(LidarProjection, ConfiguredMountYawReorientsLevelBeam) {
  const LidarProjectionPose pose{Point2{0.0, 0.0}, 18.0, 0.0, 0.0, 0.0, true, true};
  LidarProjectionConfig config{};
  config.compensate_attitude = true;
  config.lidar_mount_yaw_rad = -std::numbers::pi / 2.0;

  const LidarBeamProjection east_projection = project(pose, config, 4.0F);
  const LidarBeamProjection north_projection =
      project(pose, config, 4.0F, std::numbers::pi / 2.0);

  EXPECT_EQ(east_projection.status, LidarBeamProjectionStatus::kAccepted);
  EXPECT_NEAR(east_projection.endpoint.x, 0.0, 1.0e-6);
  EXPECT_NEAR(east_projection.endpoint.y, 4.0, 1.0e-6);
  EXPECT_NEAR(east_projection.body_frd_direction.x, 0.0, 1.0e-9);
  EXPECT_NEAR(east_projection.body_frd_direction.y, 1.0, 1.0e-9);

  EXPECT_EQ(north_projection.status, LidarBeamProjectionStatus::kAccepted);
  EXPECT_NEAR(north_projection.endpoint.x, 4.0, 1.0e-6);
  EXPECT_NEAR(north_projection.endpoint.y, 0.0, 1.0e-6);
  EXPECT_NEAR(north_projection.body_frd_direction.x, 1.0, 1.0e-9);
  EXPECT_NEAR(north_projection.body_frd_direction.y, 0.0, 1.0e-9);
}

TEST(LidarProjection, PitchChangesProjectedAltitude) {
  LidarProjectionPose pose{Point2{0.0, 0.0}, 18.0, 0.0, 0.0, -0.4, true, true};
  LidarProjectionConfig config{};
  config.compensate_attitude = true;
  config.lidar_z_offset_m = 0.3;

  const LidarBeamProjection projection = project(pose, config, 10.0F);

  EXPECT_EQ(projection.status, LidarBeamProjectionStatus::kAccepted);
  EXPECT_LT(projection.endpoint.x, 10.0);
  EXPECT_LT(projection.endpoint_altitude_m, 18.3);
}

TEST(LidarProjection, TiltedProjectionUsesBodyFrdAxes) {
  LidarProjectionPose pose{Point2{0.0, 0.0}, 18.0, 0.0, 0.25, -0.35, true, true};
  LidarProjectionConfig config{};
  config.compensate_attitude = true;

  const LidarBeamProjection projection = project(pose, config, 10.0F);

  EXPECT_EQ(projection.status, LidarBeamProjectionStatus::kAccepted);
  EXPECT_NEAR(norm(projection.ned_direction), 1.0, 1.0e-9);
  EXPECT_NEAR(projection.endpoint.x, 10.0 * std::cos(0.35), 1.0e-6);
  EXPECT_NEAR(projection.endpoint.y, 0.0, 1.0e-6);
  EXPECT_NEAR(projection.endpoint_altitude_m, 18.0 - 10.0 * std::sin(0.35), 1.0e-6);
}

TEST(LidarProjection, InvalidAttitudeFallsBackToLevelProjection) {
  LidarProjectionPose pose{Point2{0.0, 0.0}, 18.0, 0.0, 0.5, -0.5, true, false};
  LidarProjectionConfig config{};
  config.compensate_attitude = true;

  const LidarBeamProjection projection = project(pose, config, 10.0F);

  EXPECT_EQ(projection.status, LidarBeamProjectionStatus::kAccepted);
  EXPECT_NEAR(projection.endpoint.x, 10.0, 1.0e-6);
  EXPECT_NEAR(projection.endpoint.y, 0.0, 1.0e-6);
  EXPECT_NEAR(projection.endpoint_altitude_m, 18.0, 1.0e-6);
}

TEST(LidarProjection, AltitudeFilterRejectsGroundProjection) {
  LidarProjectionPose pose{Point2{0.0, 0.0}, 5.0, 0.0, 0.0, -0.8, true, true};
  LidarProjectionConfig config{};
  config.compensate_attitude = true;
  config.min_projected_altitude_m = 1.0;

  const LidarBeamProjection projection = project(pose, config, 10.0F);

  EXPECT_EQ(projection.status, LidarBeamProjectionStatus::kAltitudeRejected);
}

} // namespace drone_city_nav
