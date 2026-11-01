#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include "stereo_map_registration.hpp"

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecond{1'000'000'000};

// A room: two walls and a floor, in the map's frame.
[[nodiscard]] std::vector<Eigen::Vector3d> room() {
  std::vector<Eigen::Vector3d> points;
  for (double a = -4.0; a <= 4.0; a += 0.2) {
    for (double b = -2.0; b <= 2.0; b += 0.2) {
      points.emplace_back(5.0, a, b); // the wall ahead
      points.emplace_back(a, 5.0, b); // the wall to the right
    }
    for (double b = -4.0; b <= 4.0; b += 0.2) {
      points.emplace_back(a, b, 2.0); // the floor
    }
  }
  return points;
}

// The room as a frame of depth seen from a true pose the filter does not know
// about: the filter believes itself at the map's origin, level, facing x.
[[nodiscard]] std::vector<Eigen::Vector3d> frameFrom(const double yaw_rad,
                                                     const Eigen::Vector3d& position) {
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd{yaw_rad, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
  std::vector<Eigen::Vector3d> body;
  for (const Eigen::Vector3d& point : room()) {
    const Eigen::Vector3d relative = rotation.transpose() * (point - position);
    if (relative.norm() <= kMapRangeM) {
      body.push_back(relative);
    }
  }
  return body;
}

[[nodiscard]] VisualInertialClonePose identityPose(const std::int64_t /*stamp_ns*/) {
  VisualInertialClonePose pose;
  pose.position_ned_m = Eigen::Vector3d::Zero();
  pose.body_to_ned = Eigen::Quaterniond::Identity();
  return pose;
}

TEST(StereoMapRegistrationTest, ATurnedAndShiftedFrameIsReadAndTheTargetConvergesToIt) {
  StereoMapRegistration registration;
  registration.lay(0, room(), identityPose(0));
  EXPECT_FALSE(registration.due(kSecond / 4));
  EXPECT_TRUE(registration.due(25 * kSecond));
  const double yaw = 1.0 * std::numbers::pi / 180.0;
  const Eigen::Vector3d shift{0.3, 0.0, 0.0};
  const std::vector<Eigen::Vector3d> frame = frameFrom(yaw, shift);
  StereoMapRegistrationReport report;
  for (int i = 0; i < 30; ++i) {
    const std::int64_t stamp = 25 * kSecond + i * kSecond;
    report = registration.registerFrame(stamp, frame, identityPose(stamp), true);
    if (i == 0) {
      EXPECT_STREQ(report.outcome, "applied");
      EXPECT_NEAR(report.yaw_innovation_rad, yaw, 0.1 * yaw);
      EXPECT_NEAR(report.innovation_m, shift.norm(), 0.05);
      EXPECT_GE(report.matched_points, static_cast<double>(kMapMinimumMatchedPoints));
      EXPECT_LE(report.yaw_step_rad, kMapYawStepRad + 1.0e-12);
    }
  }
  const MapFrameOffset target = registration.target();
  EXPECT_NEAR(target.yaw_rad, yaw, 0.15 * yaw);
  EXPECT_NEAR(target.translation.x(), shift.x(), 0.05);
  EXPECT_NEAR(target.translation.y(), 0.0, 0.05);
  // The published frame follows the target at its rates.
  VisualInertialEstimate estimate;
  estimate.body_to_ned = Eigen::Quaterniond::Identity();
  registration.applyFrame(estimate, 60 * kSecond);
  registration.applyFrame(estimate, 70 * kSecond);
  EXPECT_NEAR(estimate.position_ned_m.x(), shift.x(), 0.05);
}

TEST(StereoMapRegistrationTest, TheMeasuringModeReadsWithoutMoving) {
  StereoMapRegistration registration;
  registration.lay(0, room(), identityPose(0));
  const std::vector<Eigen::Vector3d> frame =
      frameFrom(1.0 * std::numbers::pi / 180.0, Eigen::Vector3d{0.3, 0.0, 0.0});
  for (int i = 0; i < 5; ++i) {
    const std::int64_t stamp = 25 * kSecond + i * kSecond;
    const StereoMapRegistrationReport report =
        registration.registerFrame(stamp, frame, identityPose(stamp), false);
    EXPECT_STREQ(report.outcome, "applied");
    EXPECT_NEAR(report.innovation_m, 0.3, 0.05) << i;
  }
  EXPECT_EQ(registration.target().yaw_rad, 0.0);
  EXPECT_EQ(registration.target().translation.norm(), 0.0);
}

TEST(StereoMapRegistrationTest, AnEmptyFrameAndTheFreshMapAreSilent) {
  StereoMapRegistration registration;
  EXPECT_FALSE(registration.due(25 * kSecond)); // nothing laid yet
  registration.lay(0, room(), identityPose(0));
  const StereoMapRegistrationReport empty =
      registration.registerFrame(25 * kSecond, {}, identityPose(25 * kSecond), true);
  EXPECT_STREQ(empty.outcome, "weak");
  EXPECT_EQ(registration.target().translation.norm(), 0.0);
  // What was laid just now carries the pose's drift: not registered against.
  const std::vector<Eigen::Vector3d> frame =
      frameFrom(0.0, Eigen::Vector3d{0.3, 0.0, 0.0});
  const StereoMapRegistrationReport fresh =
      registration.registerFrame(30 * kSecond, frame, identityPose(30 * kSecond), true);
  EXPECT_STREQ(fresh.outcome, "applied");
  registration.lay(30 * kSecond, frame, identityPose(30 * kSecond));
  const StereoMapRegistrationReport young =
      registration.registerFrame(31 * kSecond, frame, identityPose(31 * kSecond), true);
  EXPECT_LE(young.matched_points, fresh.matched_points);
}

} // namespace
} // namespace drone_city_nav
