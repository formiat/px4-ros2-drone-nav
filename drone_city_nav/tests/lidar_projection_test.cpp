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
  return projectLidarBeam(pose, config, 0.1, 35.0, angle_min_rad, 0.1, 0U, range_m,
                          0.0);
}

} // namespace

TEST(LidarProjection, ZeroTiltKeepsLegacyNoSwapEndpoint) {
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
}

TEST(LidarProjection, ZeroTiltKeepsLegacySwapEndpoint) {
  const LidarProjectionPose pose{
      Point2{5.0, 6.0}, 18.0, std::numbers::pi / 2.0, 0.0, 0.0, true, true};
  LidarProjectionConfig config{};
  config.compensate_attitude = true;
  config.swap_lidar_xy_to_local_frame = true;

  const LidarBeamProjection projection = project(pose, config, 4.0F);

  EXPECT_EQ(projection.status, LidarBeamProjectionStatus::kAccepted);
  EXPECT_NEAR(projection.endpoint.x, 9.0, 1.0e-6);
  EXPECT_NEAR(projection.endpoint.y, 6.0, 1.0e-6);
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

TEST(LidarProjection, AltitudeFilterRejectsGroundProjection) {
  LidarProjectionPose pose{Point2{0.0, 0.0}, 5.0, 0.0, 0.0, -0.8, true, true};
  LidarProjectionConfig config{};
  config.compensate_attitude = true;
  config.min_projected_altitude_m = 1.0;

  const LidarBeamProjection projection = project(pose, config, 10.0F);

  EXPECT_EQ(projection.status, LidarBeamProjectionStatus::kAltitudeRejected);
}

} // namespace drone_city_nav
