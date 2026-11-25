#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "stereo_loop_closure.hpp"

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecond{1'000'000'000};
constexpr double kDegree{std::numbers::pi / 180.0};
constexpr int kWidth{640};
constexpr int kHeight{480};
constexpr double kFocalPx{320.0};
constexpr double kBaselineM{0.2};

// The camera looks along the body's x: its optical frame (x right, y down, z
// forward) in forward-right-down.
[[nodiscard]] Eigen::Matrix3d cameraToBody() {
  Eigen::Matrix3d rotation;
  rotation << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  return rotation;
}

const Eigen::Vector3d kMount{0.32, -0.10, -0.02};

[[nodiscard]] StereoLoopClosureConfig config() {
  return StereoLoopClosureConfig{
      .focal_px = kFocalPx,
      .principal_x_px = 0.5 * (kWidth - 1),
      .principal_y_px = 0.5 * (kHeight - 1),
      .baseline_m = kBaselineM,
      .left_camera = {.camera_to_body = Eigen::Quaterniond{cameraToBody()},
                      .position_body_m = kMount}};
}

[[nodiscard]] VisualInertialClonePose pose(const Eigen::Vector3d& position,
                                           const double yaw_rad) {
  VisualInertialClonePose result;
  result.position_ned_m = position;
  result.body_to_ned =
      Eigen::Quaterniond{Eigen::AngleAxisd{yaw_rad, Eigen::Vector3d::UnitZ()}};
  return result;
}

// A flat rectangle of the world with a texture of its own: its corners in the
// order of the texture's (left top, right top, right bottom, left bottom).
struct Surface {
  std::array<Eigen::Vector3d, 4> corners;
  cv::Mat texture;
};

// Blocks of two sizes: corners for the detector at every scale it reads.
[[nodiscard]] cv::Mat texture(const int seed) {
  cv::RNG generator{static_cast<std::uint64_t>(seed)};
  cv::Mat coarse(24, 24, CV_8U);
  cv::Mat fine(96, 96, CV_8U);
  generator.fill(coarse, cv::RNG::UNIFORM, 40, 216);
  generator.fill(fine, cv::RNG::UNIFORM, 0, 80);
  cv::Mat large;
  cv::Mat small;
  cv::resize(coarse, large, {768, 768}, 0.0, 0.0, cv::INTER_NEAREST);
  cv::resize(fine, small, {768, 768}, 0.0, 0.0, cv::INTER_NEAREST);
  cv::Mat result;
  cv::addWeighted(large, 1.0, small, 0.5, -20.0, result);
  return result;
}

[[nodiscard]] Surface surface(const Eigen::Vector3d& left_top,
                              const Eigen::Vector3d& right_top,
                              const Eigen::Vector3d& right_bottom,
                              const Eigen::Vector3d& left_bottom, const int seed) {
  return Surface{{left_top, right_top, right_bottom, left_bottom}, texture(seed)};
}

// A wall eight metres ahead, two boxes' faces before it and the floor: north
// is ahead, east to the right, down is down.
[[nodiscard]] std::vector<Surface> room() {
  return {
      surface({8.0, -8.0, -5.0}, {8.0, 8.0, -5.0}, {8.0, 8.0, 1.5}, {8.0, -8.0, 1.5},
              1),
      surface({3.0, -4.0, 1.5}, {8.0, -4.0, 1.5}, {8.0, 4.0, 1.5}, {3.0, 4.0, 1.5}, 2),
      surface({5.0, -3.0, -1.5}, {5.0, -1.0, -1.5}, {5.0, -1.0, 1.5}, {5.0, -3.0, 1.5},
              3),
      surface({6.0, 1.0, -2.0}, {6.0, 3.5, -2.0}, {6.0, 3.5, 1.5}, {6.0, 1.0, 1.5}, 4)};
}

// The wall alone: one plane, whatever its texture.
[[nodiscard]] std::vector<Surface> wall() {
  return {room().front()};
}

// The scene as a camera at the body's pose sees it, the far surfaces first.
[[nodiscard]] cv::Mat render(std::vector<Surface> scene,
                             const VisualInertialClonePose& body,
                             const double along_baseline_m) {
  const Eigen::Matrix3d camera_to_world =
      body.body_to_ned.toRotationMatrix() * cameraToBody();
  const Eigen::Vector3d centre =
      body.position_ned_m + body.body_to_ned.toRotationMatrix() * kMount +
      camera_to_world * Eigen::Vector3d{along_baseline_m, 0.0, 0.0};
  const auto depth = [&](const Surface& item) {
    double sum = 0.0;
    for (const Eigen::Vector3d& corner : item.corners) {
      sum += (camera_to_world.transpose() * (corner - centre)).z();
    }
    return sum;
  };
  std::sort(scene.begin(), scene.end(),
            [&](const Surface& a, const Surface& b) { return depth(a) > depth(b); });
  cv::Mat image(kHeight, kWidth, CV_8U, cv::Scalar{128});
  for (const Surface& item : scene) {
    std::array<cv::Point2f, 4> seen;
    const auto side = static_cast<float>(item.texture.cols);
    const std::array<cv::Point2f, 4> drawn{
        {{0.0F, 0.0F}, {side, 0.0F}, {side, side}, {0.0F, side}}};
    for (std::size_t i = 0U; i < 4U; ++i) {
      const Eigen::Vector3d point =
          camera_to_world.transpose() * (item.corners.at(i) - centre);
      EXPECT_GT(point.z(), 0.2) << "a surface behind the camera";
      seen.at(i) = {
          static_cast<float>(kFocalPx * point.x() / point.z() + 0.5 * (kWidth - 1)),
          static_cast<float>(kFocalPx * point.y() / point.z() + 0.5 * (kHeight - 1))};
    }
    const cv::Mat projection = cv::getPerspectiveTransform(drawn.data(), seen.data());
    cv::Mat layer;
    cv::Mat mask;
    cv::warpPerspective(item.texture, layer, projection, image.size(),
                        cv::INTER_LINEAR);
    cv::warpPerspective(cv::Mat(item.texture.size(), CV_8U, cv::Scalar{255}), mask,
                        projection, image.size(), cv::INTER_NEAREST);
    layer.copyTo(image, mask);
  }
  return image;
}

// A flight that leaves a place and comes back to it: the keyframe at the
// place, thirty-five more where nothing is seen (a blank pair keeps no
// feature), and the return, where the vehicle truly is at `truth` and the
// filter believes it at `believed`.
[[nodiscard]] StereoLoopClosureReport comeBack(const std::vector<Surface>& scene,
                                               const VisualInertialClonePose& truth,
                                               const VisualInertialClonePose& believed,
                                               const bool act) {
  StereoLoopClosure closure{config()};
  const MapFrameOffset frame;
  const VisualInertialClonePose home = pose(Eigen::Vector3d::Zero(), 0.0);
  EXPECT_TRUE(closure.due(home));
  const StereoLoopClosureReport first = closure.observe(
      0, render(scene, home, 0.0), render(scene, home, kBaselineM), home, frame, act);
  EXPECT_STREQ(first.outcome, "none");
  EXPECT_GT(first.features, 100U);
  const cv::Mat blank(kHeight, kWidth, CV_8U, cv::Scalar{128});
  for (int step = 1; step <= 35; ++step) {
    const VisualInertialClonePose away = pose({-1.0 * step, 0.0, 0.0}, 0.0);
    EXPECT_TRUE(closure.due(away));
    const StereoLoopClosureReport report =
        closure.observe(step * kSecond, blank, blank, away, frame, act);
    EXPECT_EQ(report.features, 0U);
    EXPECT_EQ(report.candidates, 0U);
  }
  return closure.observe(40 * kSecond, render(scene, truth, 0.0),
                         render(scene, truth, kBaselineM), believed, frame, act);
}

const VisualInertialClonePose kTruth = pose({0.6, 0.4, -0.1}, 8.0 * kDegree);

TEST(StereoLoopClosureTest, APlaceSeenAgainIsRecognisedAndTheFrameIsMovedToIt) {
  // The filter has drifted 0.37 m and half a degree on the way round.
  const VisualInertialClonePose believed = pose({0.9, 0.2, -0.05}, 8.5 * kDegree);
  const StereoLoopClosureReport report = comeBack(room(), kTruth, believed, true);
  ASSERT_STREQ(report.outcome, "accepted") << StereoLoopClosure::summary(report);
  EXPECT_TRUE(report.accepted);
  EXPECT_EQ(report.matched_stamp_ns, 0);
  EXPECT_GE(report.inliers, kLoopMinimumInliers);
  EXPECT_GE(report.off_plane, kLoopMinimumOffPlane);
  // The closure reads where the vehicle truly is from the place.
  EXPECT_LT((report.shift_body - kTruth.position_ned_m).norm(), 0.08)
      << StereoLoopClosure::summary(report);
  EXPECT_LT(report.turn_body.angularDistance(kTruth.body_to_ned), 1.0 * kDegree);
  EXPECT_NEAR(report.innovation_m, 0.37, 0.1);
  // The frame the graph asks for puts the filter's pose there.
  EXPECT_LT(
      (report.target.apply(believed.position_ned_m) - kTruth.position_ned_m).norm(),
      0.1)
      << StereoLoopClosure::summary(report);
  EXPECT_NEAR(report.target.yaw_rad, -0.5 * kDegree, 0.5 * kDegree);
  EXPECT_EQ(report.closures, 1U);
  EXPECT_EQ(report.keyframes, 37U);
}

TEST(StereoLoopClosureTest, TheMeasuringModeReadsTheClosureAndMovesNothing) {
  const VisualInertialClonePose believed = pose({0.9, 0.2, -0.05}, 8.5 * kDegree);
  const StereoLoopClosureReport report = comeBack(room(), kTruth, believed, false);
  ASSERT_STREQ(report.outcome, "accepted") << StereoLoopClosure::summary(report);
  EXPECT_LT((report.shift_body - kTruth.position_ned_m).norm(), 0.08);
  EXPECT_EQ(report.target.yaw_rad, 0.0);
  EXPECT_TRUE(report.target.translation.isZero());
}

TEST(StereoLoopClosureTest, AClosureFartherFromTheEstimateThanItsDriftAllowsIsDropped) {
  // 71 m of path between the two looks allow 1.2 m; the filter's belief lies
  // 3 m from where the closure puts the vehicle: one of the two is wrong, and
  // the closure is not taken on its own word.
  const VisualInertialClonePose believed = pose({0.6, 3.4, -0.1}, 8.0 * kDegree);
  const StereoLoopClosureReport report = comeBack(room(), kTruth, believed, true);
  EXPECT_STREQ(report.outcome, "innovation") << StereoLoopClosure::summary(report);
  EXPECT_FALSE(report.accepted);
  EXPECT_NEAR(report.innovation_m, 3.0, 0.2);
  EXPECT_TRUE(report.target.translation.isZero());
  EXPECT_EQ(report.closures, 0U);
}

TEST(StereoLoopClosureTest, AFlatSurfaceAloneClosesNoLoop) {
  // Every inlier in one plane: a repeated surface would answer the same with
  // a pose shifted along itself (r4233, r4234), and this one is not asked.
  const VisualInertialClonePose believed = pose({0.9, 0.2, -0.05}, 8.5 * kDegree);
  const StereoLoopClosureReport report = comeBack(wall(), kTruth, believed, true);
  EXPECT_STREQ(report.outcome, "planar") << StereoLoopClosure::summary(report);
  EXPECT_FALSE(report.accepted);
  EXPECT_GE(report.inliers, kLoopMinimumInliers);
  EXPECT_LT(report.off_plane, kLoopMinimumOffPlane);
}

TEST(StereoLoopClosureTest, AKeyframeIsDueEveryMetreOrFifteenDegrees) {
  StereoLoopClosure closure{config()};
  const cv::Mat blank(kHeight, kWidth, CV_8U, cv::Scalar{128});
  const VisualInertialClonePose home = pose(Eigen::Vector3d::Zero(), 0.0);
  EXPECT_TRUE(closure.due(home));
  EXPECT_EQ(closure.observe(0, blank, blank, home, MapFrameOffset{}, true).keyframes,
            1U);
  EXPECT_FALSE(closure.due(pose({0.9, 0.0, 0.0}, 14.0 * kDegree)));
  EXPECT_TRUE(closure.due(pose({1.0, 0.0, 0.0}, 0.0)));
  EXPECT_TRUE(closure.due(pose({0.0, 0.0, 0.0}, -15.5 * kDegree)));
  const std::string line = StereoLoopClosure::summary(closure.observe(
      kSecond, blank, blank, pose({1.0, 0.0, 0.0}, 0.0), MapFrameOffset{}, true));
  EXPECT_NE(line.find("VIO_LOOP_CLOSURE outcome=none"), std::string::npos) << line;
  EXPECT_NE(line.find("keyframes=2"), std::string::npos) << line;
}

} // namespace
} // namespace drone_city_nav
