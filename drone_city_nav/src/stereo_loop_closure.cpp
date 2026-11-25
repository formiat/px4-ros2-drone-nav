#include "stereo_loop_closure.hpp"

#include "drone_city_nav/pose_graph_4dof.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] double wrapped(const double angle_rad) {
  return std::remainder(angle_rad, 2.0 * std::numbers::pi);
}

[[nodiscard]] double yawOf(const Eigen::Matrix3d& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

[[nodiscard]] Eigen::Matrix3d aboutVertical(const double yaw_rad) {
  return Eigen::AngleAxisd{yaw_rad, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
}

[[nodiscard]] Eigen::Vector3d eigen(const cv::Point3f& point) {
  return Eigen::Vector3d{point.x, point.y, point.z};
}

// What a keyframe keeps: the filter's pose, and the left image's features
// that have a point by the pair.
struct Keyframe {
  std::int64_t stamp_ns{0};
  double path_m{0.0};
  VisualInertialClonePose pose;
  cv::Mat descriptors;
  std::vector<cv::Point2f> pixels;
  std::vector<cv::Point3f> points;
};

// One candidate's reading: how far it got and, when it is a closure, the
// graph's edge from the old keyframe to the new one.
struct Reading {
  const char* outcome{"few_matches"};
  std::size_t matches{0U};
  std::size_t inliers{0U};
  std::size_t off_plane{0U};
  double residual_ratio{0.0};
  Eigen::Vector3d shift_body{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d turn_body{Eigen::Matrix3d::Identity()};
  PoseGraphEdge edge;
  PoseGraphPose innovation;
  bool accepted{false};
};

} // namespace

struct StereoLoopClosure::Impl {
  explicit Impl(const StereoLoopClosureConfig& configuration)
      : config_{configuration},
        orb_{cv::ORB::create(kLoopFeatures)},
        matcher_{cv::NORM_HAMMING},
        camera_matrix_{configuration.focal_px,
                       0.0,
                       configuration.principal_x_px,
                       0.0,
                       configuration.focal_px,
                       configuration.principal_y_px,
                       0.0,
                       0.0,
                       1.0},
        camera_to_body_{configuration.left_camera.camera_to_body.toRotationMatrix()} {
  }

  // The depth's own error at a point this far, from the disparity's.
  [[nodiscard]] double depthError(const double depth_m) const {
    return kLoopDisparityErrorPx * depth_m * depth_m /
           (config_.focal_px * config_.baseline_m);
  }

  void extract(const cv::Mat& left, const cv::Mat& right, Keyframe& keyframe) const;

  [[nodiscard]] Reading read(const Keyframe& old, std::size_t old_index,
                             const Keyframe& current, std::size_t index) const;

  StereoLoopClosureConfig config_;
  cv::Ptr<cv::ORB> orb_;
  cv::BFMatcher matcher_;
  cv::Matx33d camera_matrix_;
  Eigen::Matrix3d camera_to_body_;
  std::vector<Keyframe> keyframes_;
  PoseGraph4Dof graph_;
  double path_m_{0.0};
  bool restarted_{false};
  std::uint64_t closures_{0U};
};

// ORB features of the left image, each followed into the right image and
// back (the tracker's test of a stereo match): the ones on their row with a
// disparity the pair resolves keep a point and a descriptor.
void StereoLoopClosure::Impl::extract(const cv::Mat& left, const cv::Mat& right,
                                      Keyframe& keyframe) const {
  constexpr double kRoundTripPx{1.0};
  constexpr double kRowErrorPx{1.5};
  constexpr double kMinimumDisparityPx{1.0};
  constexpr double kMaximumDisparityPx{256.0};
  std::vector<cv::KeyPoint> detected;
  orb_->detect(left, detected);
  if (detected.size() < kLoopMinimumMatches) {
    return;
  }
  std::vector<cv::Point2f> corners;
  corners.reserve(detected.size());
  for (const cv::KeyPoint& keypoint : detected) {
    corners.push_back(keypoint.pt);
  }
  const cv::Size window{21, 21};
  const cv::TermCriteria criteria{cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30,
                                  0.01};
  std::vector<cv::Point2f> there;
  std::vector<cv::Point2f> back;
  std::vector<unsigned char> found;
  std::vector<unsigned char> found_back;
  std::vector<float> error;
  cv::calcOpticalFlowPyrLK(left, right, corners, there, found, error, window, 4,
                           criteria);
  cv::calcOpticalFlowPyrLK(right, left, there, back, found_back, error, window, 4,
                           criteria);
  std::vector<cv::KeyPoint> kept;
  std::vector<cv::Point3f> points;
  for (std::size_t i = 0U; i < detected.size(); ++i) {
    const double disparity = static_cast<double>(corners[i].x) - there[i].x;
    if (found[i] == 0U || found_back[i] == 0U ||
        cv::norm(back[i] - corners[i]) > kRoundTripPx ||
        std::abs(there[i].y - corners[i].y) > kRowErrorPx ||
        disparity < kMinimumDisparityPx || disparity > kMaximumDisparityPx) {
      continue;
    }
    const double depth = config_.focal_px * config_.baseline_m / disparity;
    if (depth < kLoopMinimumDepthM || depth > kLoopMaximumDepthM) {
      continue;
    }
    cv::KeyPoint keypoint = detected[i];
    // The descriptor step may drop a keypoint by the border: its point is found by
    // this.
    keypoint.class_id = static_cast<int>(points.size());
    kept.push_back(keypoint);
    points.emplace_back(static_cast<float>((corners[i].x - config_.principal_x_px) /
                                           config_.focal_px * depth),
                        static_cast<float>((corners[i].y - config_.principal_y_px) /
                                           config_.focal_px * depth),
                        static_cast<float>(depth));
  }
  if (kept.size() < kLoopMinimumMatches) {
    return;
  }
  orb_->compute(left, kept, keyframe.descriptors);
  keyframe.pixels.reserve(kept.size());
  keyframe.points.reserve(kept.size());
  for (const cv::KeyPoint& keypoint : kept) {
    keyframe.pixels.push_back(keypoint.pt);
    keyframe.points.push_back(points[static_cast<std::size_t>(keypoint.class_id)]);
  }
}

Reading StereoLoopClosure::Impl::read(const Keyframe& old, const std::size_t old_index,
                                      const Keyframe& current,
                                      const std::size_t index) const {
  Reading reading;
  std::vector<std::vector<cv::DMatch>> nearest;
  matcher_.knnMatch(old.descriptors, current.descriptors, nearest, 2);
  std::vector<cv::Point3f> seen_before;
  std::vector<cv::Point2f> seen_now;
  std::vector<cv::Point3f> own;
  for (const std::vector<cv::DMatch>& pair : nearest) {
    if (pair.size() == 2U && pair[0].distance < kLoopMatchRatio * pair[1].distance) {
      seen_before.push_back(old.points[static_cast<std::size_t>(pair[0].queryIdx)]);
      seen_now.push_back(current.pixels[static_cast<std::size_t>(pair[0].trainIdx)]);
      own.push_back(current.points[static_cast<std::size_t>(pair[0].trainIdx)]);
    }
  }
  reading.matches = seen_before.size();
  if (reading.matches < kLoopMinimumMatches) {
    return reading;
  }
  reading.outcome = "few_inliers";
  // Which matches agree on one pose: PnP with RANSAC. The library's own
  // failures (an assertion on a degenerate set) are a reading without a pose.
  cv::Mat rotation_vector;
  cv::Mat translation_vector;
  std::vector<int> inliers;
  bool solved = false;
  try {
    solved =
        cv::solvePnPRansac(seen_before, seen_now, camera_matrix_, cv::noArray(),
                           rotation_vector, translation_vector, false,
                           kLoopRansacIterations, kLoopReprojectionPx, 0.99, inliers);
  } catch (const cv::Exception&) {
    solved = false;
  }
  reading.inliers = solved ? inliers.size() : 0U;
  if (reading.inliers < kLoopMinimumInliers) {
    return reading;
  }
  // The inliers' pose anew, in closed form from the two keyframes' own points
  // and then refined on the image: the library ends its RANSAC with a linear
  // fit of all inliers, which on points near one plane (a wall, the ground
  // under an open sky) answered with poses kilometres away (r4235's
  // readings). A point of the old camera in the new one: x_new = turn x_old
  // + shift.
  Eigen::Matrix3Xd before_points(3, static_cast<Eigen::Index>(inliers.size()));
  Eigen::Matrix3Xd now_points(3, static_cast<Eigen::Index>(inliers.size()));
  std::vector<cv::Point3f> inlier_points;
  std::vector<cv::Point2f> inlier_pixels;
  for (std::size_t i = 0U; i < inliers.size(); ++i) {
    const auto match = static_cast<std::size_t>(inliers[i]);
    before_points.col(static_cast<Eigen::Index>(i)) = eigen(seen_before[match]);
    now_points.col(static_cast<Eigen::Index>(i)) = eigen(own[match]);
    inlier_points.push_back(seen_before[match]);
    inlier_pixels.push_back(seen_now[match]);
  }
  const Eigen::Matrix4d aligned = Eigen::umeyama(before_points, now_points, false);
  cv::Matx33d turn_matrix;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      turn_matrix(row, column) = aligned(row, column);
    }
  }
  cv::Vec3d turn_vector;
  cv::Rodrigues(turn_matrix, turn_vector);
  cv::Vec3d shift_vector{aligned(0, 3), aligned(1, 3), aligned(2, 3)};
  try {
    cv::solvePnPRefineLM(inlier_points, inlier_pixels, camera_matrix_, cv::noArray(),
                         turn_vector, shift_vector);
  } catch (const cv::Exception&) {
    reading.inliers = 0U;
    return reading;
  }
  cv::Rodrigues(turn_vector, turn_matrix);
  Eigen::Matrix3d turn;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      turn(row, column) = turn_matrix(row, column);
    }
  }
  const Eigen::Vector3d shift{shift_vector[0], shift_vector[1], shift_vector[2]};
  // The new keyframe's own points against the old one's moved by the pose.
  reading.outcome = "residual";
  std::vector<double> ratios;
  ratios.reserve(inliers.size());
  for (Eigen::Index i = 0; i < before_points.cols(); ++i) {
    ratios.push_back((turn * before_points.col(i) + shift - now_points.col(i)).norm() /
                     (depthError(now_points(2, i)) + kLoopResidualSlackM));
  }
  const Eigen::MatrixXd cloud = before_points.transpose();
  std::nth_element(ratios.begin(),
                   ratios.begin() + static_cast<std::ptrdiff_t>(ratios.size() / 2U),
                   ratios.end());
  reading.residual_ratio = ratios[ratios.size() / 2U];
  if (!std::isfinite(reading.residual_ratio) || reading.residual_ratio > 1.0) {
    return reading;
  }
  // The inliers off their common plane (the header's note on repeated flat surfaces).
  reading.outcome = "planar";
  const Eigen::RowVector3d centre = cloud.colwise().mean();
  const Eigen::MatrixXd centred = cloud.rowwise() - centre;
  const Eigen::JacobiSVD<Eigen::MatrixXd> decomposition{centred, Eigen::ComputeThinV};
  const Eigen::Vector3d normal = decomposition.matrixV().col(2);
  for (Eigen::Index row = 0; row < centred.rows(); ++row) {
    if (std::abs(centred.row(row).dot(normal)) >
        depthError(cloud(row, 2)) + kLoopPlaneSlackM) {
      ++reading.off_plane;
    }
  }
  if (reading.off_plane < kLoopMinimumOffPlane) {
    return reading;
  }
  // The new body in the old body's frame, through the camera's mount.
  const Eigen::Matrix3d new_camera_in_old = turn.transpose();
  const Eigen::Vector3d new_camera_from_old = -turn.transpose() * shift;
  const Eigen::Vector3d& mount = config_.left_camera.position_body_m;
  reading.turn_body = camera_to_body_ * new_camera_in_old * camera_to_body_.transpose();
  reading.shift_body =
      camera_to_body_ * new_camera_from_old + mount - reading.turn_body * mount;
  // Roll and pitch by the closure against the filter's own at the new keyframe.
  reading.outcome = "attitude";
  const Eigen::Matrix3d old_attitude = old.pose.body_to_ned.toRotationMatrix();
  const Eigen::Matrix3d by_closure = old_attitude * reading.turn_body;
  const Eigen::Vector3d down_by_closure =
      by_closure.transpose() * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d down_by_filter =
      current.pose.body_to_ned.toRotationMatrix().transpose() *
      Eigen::Vector3d::UnitZ();
  if (std::acos(std::clamp(down_by_closure.dot(down_by_filter), -1.0, 1.0)) >
      kLoopAttitudeRad) {
    return reading;
  }
  // The edge in the old keyframe's heading frame, and what it disagrees by
  // with the estimate: within the drift the path between them allows.
  reading.outcome = "innovation";
  const double old_yaw = yawOf(old_attitude);
  reading.edge = PoseGraphEdge{.from = old_index,
                               .to = index,
                               .shift = aboutVertical(-old_yaw) * old_attitude *
                                        reading.shift_body,
                               .turn_rad = wrapped(yawOf(by_closure) - old_yaw),
                               .shift_sigma_m = kLoopClosureSigmaM,
                               .turn_sigma_rad = kLoopClosureTurnSigmaRad};
  reading.innovation = graph_.residual(reading.edge);
  if (reading.innovation.position.norm() >
          kLoopDriftBaseM + kLoopDriftPerPathM * (current.path_m - old.path_m) ||
      std::abs(reading.innovation.yaw_rad) > kLoopDriftHeadingRad) {
    return reading;
  }
  reading.outcome = "accepted";
  reading.accepted = true;
  return reading;
}

StereoLoopClosure::StereoLoopClosure(const StereoLoopClosureConfig& config)
    : impl_{std::make_unique<Impl>(config)} {
}

StereoLoopClosure::~StereoLoopClosure() = default;

bool StereoLoopClosure::due(const VisualInertialClonePose& pose) const {
  if (impl_->keyframes_.empty()) {
    return true;
  }
  const VisualInertialClonePose& last = impl_->keyframes_.back().pose;
  return (pose.position_ned_m - last.position_ned_m).norm() >= kLoopKeyframeStepM ||
         std::abs(wrapped(yawOf(pose.body_to_ned.toRotationMatrix()) -
                          yawOf(last.body_to_ned.toRotationMatrix()))) >=
             kLoopKeyframeTurnRad;
}

void StereoLoopClosure::filterRestarted() noexcept {
  impl_->restarted_ = true;
}

StereoLoopClosureReport
StereoLoopClosure::observe(const std::int64_t stamp_ns, const cv::Mat& left,
                           const cv::Mat& right, const VisualInertialClonePose& pose,
                           const MapFrameOffset& frame, const bool act) {
  const auto started = std::chrono::steady_clock::now();
  Impl& impl = *impl_;
  StereoLoopClosureReport report;
  report.stamp_ns = stamp_ns;
  report.target = frame;
  if (!impl.keyframes_.empty()) {
    impl.path_m_ +=
        (pose.position_ned_m - impl.keyframes_.back().pose.position_ned_m).norm();
  }
  Keyframe keyframe{.stamp_ns = stamp_ns,
                    .path_m = impl.path_m_,
                    .pose = pose,
                    .descriptors = {},
                    .pixels = {},
                    .points = {}};
  impl.extract(left, right, keyframe);
  report.features = keyframe.points.size();
  // The keyframe's node where the published frame puts it, and its edge from
  // the keyframe before: what lay between the two in that frame, the filter's
  // motion with the registration's corrections of item 40 (a closure's own
  // jump is in the node before already and not in the edge).
  const double yaw = yawOf(pose.body_to_ned.toRotationMatrix());
  const PoseGraphPose placed{frame.apply(pose.position_ned_m),
                             wrapped(frame.yaw_rad + yaw)};
  const std::size_t index = impl.graph_.addNode(placed);
  if (index > 0U) {
    const PoseGraphPose& before = impl.graph_.node(index - 1U);
    const Eigen::Vector3d step =
        aboutVertical(-before.yaw_rad) * (placed.position - before.position);
    impl.graph_.addEdge(PoseGraphEdge{
        .from = index - 1U,
        .to = index,
        .shift = step,
        .turn_rad = wrapped(placed.yaw_rad - before.yaw_rad),
        .shift_sigma_m = impl.restarted_ ? kLoopRestartSigmaM
                                         : kLoopOdometryBaseSigmaM +
                                               kLoopOdometryStepSigma * step.norm(),
        .turn_sigma_rad =
            impl.restarted_ ? kLoopRestartTurnSigmaRad : kLoopOdometryTurnSigmaRad});
    impl.restarted_ = false;
  }
  Reading best;
  std::size_t best_index = 0U;
  if (static_cast<std::size_t>(keyframe.descriptors.rows) >= kLoopMinimumMatches) {
    std::vector<std::pair<double, std::size_t>> near;
    for (std::size_t i = 0U; i < impl.keyframes_.size(); ++i) {
      const Keyframe& old = impl.keyframes_[i];
      const double path_between = keyframe.path_m - old.path_m;
      if (old.descriptors.empty() || stamp_ns - old.stamp_ns < kLoopCandidateAgeNs ||
          path_between < kLoopCandidatePathM) {
        continue;
      }
      const PoseGraphPose& node = impl.graph_.node(i);
      const double distance = (node.position - placed.position).norm();
      if (distance <= kLoopCandidateRadiusM + kLoopDriftBaseM +
                          kLoopDriftPerPathM * path_between &&
          std::abs(wrapped(node.yaw_rad - placed.yaw_rad)) <=
              kLoopCandidateHeadingRad) {
        near.emplace_back(distance, i);
      }
    }
    std::sort(near.begin(), near.end());
    near.resize(std::min(near.size(), kLoopCandidates));
    report.candidates = near.size();
    bool first = true;
    for (const auto& [distance, old_index] : near) {
      const Reading reading =
          impl.read(impl.keyframes_[old_index], old_index, keyframe, index);
      // The closure with the most inliers; without one, the reading with the most.
      if (first || (reading.accepted && !best.accepted) ||
          (reading.accepted == best.accepted && reading.inliers > best.inliers)) {
        best = reading;
        best_index = old_index;
        first = false;
      }
    }
  }
  if (report.candidates > 0U) {
    const Keyframe& old = impl.keyframes_[best_index];
    report.outcome = best.outcome;
    report.matched_stamp_ns = old.stamp_ns;
    report.matches = best.matches;
    report.inliers = best.inliers;
    report.off_plane = best.off_plane;
    report.residual_ratio = best.residual_ratio;
    report.path_between_m = keyframe.path_m - old.path_m;
    report.innovation_m = best.innovation.position.norm();
    report.innovation_yaw_rad = best.innovation.yaw_rad;
    report.shift_body = best.shift_body;
    report.turn_body = Eigen::Quaterniond{best.turn_body}.normalized();
  }
  impl.keyframes_.push_back(std::move(keyframe));
  if (best.accepted) {
    ++impl.closures_;
    report.accepted = true;
    if (act) {
      impl.graph_.addEdge(best.edge);
      impl.graph_.optimise(kLoopGraphIterations);
      // The offset that puts the filter's pose where the graph puts the keyframe.
      const PoseGraphPose& solved = impl.graph_.node(index);
      report.target.yaw_rad = wrapped(solved.yaw_rad - yaw);
      report.target.translation =
          solved.position - report.target.rotation() * pose.position_ned_m;
    }
  }
  report.keyframes = impl.keyframes_.size();
  report.closures = impl.closures_;
  report.recognition_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - started)
                              .count();
  return report;
}

std::string StereoLoopClosure::summary(const StereoLoopClosureReport& report) {
  std::array<char, 768> line{};
  std::snprintf(
      line.data(), line.size(),
      "VIO_LOOP_CLOSURE outcome=%s stamp_s=%.3f matched_stamp_s=%.3f keyframes=%zu "
      "features=%zu candidates=%zu matches=%zu inliers=%zu off_plane=%zu "
      "residual_ratio=%.3f path_between_m=%.1f innovation_m=%.3f "
      "innovation_yaw_deg=%.3f shift_body=(%.4f,%.4f,%.4f) "
      "turn_body=(%.6f,%.6f,%.6f,%.6f) target_yaw_deg=%.3f "
      "target_translation=(%.3f,%.3f,%.3f) recognition_ms=%.1f closures=%" PRIu64,
      report.outcome, 1.0e-9 * static_cast<double>(report.stamp_ns),
      1.0e-9 * static_cast<double>(report.matched_stamp_ns), report.keyframes,
      report.features, report.candidates, report.matches, report.inliers,
      report.off_plane, report.residual_ratio, report.path_between_m,
      report.innovation_m, report.innovation_yaw_rad * 180.0 / std::numbers::pi,
      report.shift_body.x(), report.shift_body.y(), report.shift_body.z(),
      report.turn_body.w(), report.turn_body.x(), report.turn_body.y(),
      report.turn_body.z(), report.target.yaw_rad * 180.0 / std::numbers::pi,
      report.target.translation.x(), report.target.translation.y(),
      report.target.translation.z(), report.recognition_ms, report.closures);
  return std::string{line.data()};
}

} // namespace drone_city_nav
