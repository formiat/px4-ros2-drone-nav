#pragma once
// Loop closure for the stereo estimate (roadmap item 41, stages 1 and 2): a
// place seen again is recognised from the keyframes the estimator keeps, the
// closure is one measurement with its residual, and a pose graph over the
// keyframes spreads its correction; what reaches the navigation is the frame
// offset of item 40 (stereo_map_registration.hpp), which the published pose
// follows at its bounded rates.
//
// A keyframe is laid every metre of path or fifteen degrees of heading: ORB
// features of the left rectified image that have a match in the right one
// (their descriptors and their points by the pair's disparity) and the
// filter's pose. Nothing else is remembered of a place, and nothing about a
// place is ever decided: a closure is accepted or dropped on its own
// geometry and on the estimate's drift bound, every time anew.
//
// The image side lives here with the tracker's, outside the estimator's
// library; the graph is the library's (pose_graph_4dof.hpp).

#include "drone_city_nav/visual_inertial_odometry.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "stereo_map_registration.hpp"

namespace cv {
class Mat;
} // namespace cv

namespace drone_city_nav {

// A keyframe every this far along the filter's path, or this much of heading.
inline constexpr double kLoopKeyframeStepM{1.0};
inline constexpr double kLoopKeyframeTurnRad{0.2618};
inline constexpr int kLoopFeatures{1000};
// A feature's point is kept within the depth the pair measures to a metre or
// so: at 0.2 m of baseline and 370 px a quarter pixel of disparity is 1.4 m
// at 20 m and 3 m at 30 m.
inline constexpr double kLoopMinimumDepthM{0.5};
inline constexpr double kLoopMaximumDepthM{30.0};
// A keyframe is a candidate when it is old (the window the filter still
// holds and the path just flown are not a loop), near by the estimate, and
// faced the same way: a view from within five metres, the drift bound added.
// The ten nearest are read: with the three nearest alone a tenth fewer
// keyframes found their closure (r4233: 134 against 156), and a place that
// answers from two keyframes is known for what it is only when both are read.
inline constexpr std::int64_t kLoopCandidateAgeNs{30'000'000'000};
inline constexpr double kLoopCandidatePathM{30.0};
inline constexpr double kLoopCandidateRadiusM{5.0};
inline constexpr double kLoopCandidateHeadingRad{0.7854};
inline constexpr std::size_t kLoopCandidates{10U};
// The estimate's own uncertainty, as measured and not as the filter states it
// (its position sigma is 0.07 m after 500 m, r4223, against drifts of metres):
// a closure farther from the estimate than item 40's registration reaches
// (kMapCorrectionReachM, 1.5 m) or five degrees of heading is dropped. With
// the registration acting the estimate stayed within 1.46 m of every right
// closure over the 1.5 km of r4233, and a repeated structure that is not flat
// (frames along a passage, 3.6 m apart: fourteen closures with 26 to 69
// inliers and up to 57 of them off their plane) answered 3.3 to 4.1 m from
// it. Where the estimate has drifted farther, no closure is taken: neither
// the right one nor the one a period away.
inline constexpr double kLoopInnovationM{kMapCorrectionReachM};
inline constexpr double kLoopInnovationHeadingRad{0.0873};
// The closures a keyframe reads from its candidates must put the vehicle in
// one place: two that pass every gate and lie farther apart than this are a
// place seen twice in the world, and none of them is taken.
inline constexpr double kLoopAgreementM{0.5};
// The descriptors: the nearest under 0.8 of the next nearest.
inline constexpr float kLoopMatchRatio{0.8F};
inline constexpr std::size_t kLoopMinimumMatches{12U};
// The pose: PnP with RANSAC of the old keyframe's points in the new one's
// image, two pixels, and no fewer inliers than this.
inline constexpr float kLoopReprojectionPx{2.0F};
inline constexpr int kLoopRansacIterations{200};
inline constexpr std::size_t kLoopMinimumInliers{25U};
// The new keyframe's own points against the old one's moved by the pose: the
// median residual within what a quarter pixel of disparity explains.
inline constexpr double kLoopDisparityErrorPx{0.25};
inline constexpr double kLoopResidualSlackM{0.05};
// A repeated flat surface (a facade, a paving) answers with a pose shifted
// along itself by its period: on r4233 and r4234 every grossly wrong closure
// (47 of them at 25 inliers and more: the turn right to a degree, the residual
// a centimetre or two, the shift 2.15 or 3.2 to 3.9 m off) had all its inliers
// in one plane, and the error lay in it. A closure needs this many inliers off
// their common plane by more than the depth's own error and this slack.
inline constexpr std::size_t kLoopMinimumOffPlane{8U};
inline constexpr double kLoopPlaneSlackM{0.10};
// Roll and pitch are the filter's by gravity at both keyframes: a closure
// that disagrees with them by more is a wrong one.
inline constexpr double kLoopAttitudeRad{0.0349};
// The graph's edges: the odometry between neighbours to a centimetre and a
// percent of the step and a tenth of a degree; a closure to a decimetre and
// half a degree (the accepted ones were within 0.18 m and 2 degrees of the
// truth on r4233 and r4234).
inline constexpr double kLoopOdometryBaseSigmaM{0.01};
inline constexpr double kLoopOdometryStepSigma{0.01};
inline constexpr double kLoopOdometryTurnSigmaRad{0.00175};
inline constexpr double kLoopClosureSigmaM{0.10};
inline constexpr double kLoopClosureTurnSigmaRad{0.0087};
// An odometry edge across a restart of the filter says little.
inline constexpr double kLoopRestartSigmaM{5.0};
inline constexpr double kLoopRestartTurnSigmaRad{0.175};
inline constexpr int kLoopGraphIterations{5};

struct StereoLoopClosureConfig {
  // The rectified pair's pinhole (specification K33) and its baseline.
  double focal_px{369.5041722813605};
  double principal_x_px{639.5};
  double principal_y_px{479.5};
  double baseline_m{0.2};
  VisualInertialCameraMount left_camera;
};

// One keyframe's reading, for the node's log line and the check against the
// truth (tools/loop_closures.py).
struct StereoLoopClosureReport {
  // none (no candidate), few_matches, few_inliers, residual, planar, attitude,
  // innovation, ambiguous (closures that disagree), or accepted: the best
  // candidate's.
  const char* outcome{"none"};
  std::int64_t stamp_ns{0};
  std::int64_t matched_stamp_ns{0};
  std::size_t keyframes{0U};
  std::size_t features{0U};
  std::size_t candidates{0U};
  std::size_t matches{0U};
  std::size_t inliers{0U};
  std::size_t off_plane{0U};
  double residual_ratio{0.0};
  double path_between_m{0.0};
  double innovation_m{0.0};
  double innovation_yaw_rad{0.0};
  // The closure's measurement: this keyframe's body in the matched one's body
  // frame (forward-right-down).
  Eigen::Vector3d shift_body{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond turn_body{Eigen::Quaterniond::Identity()};
  bool accepted{false};
  // The frame offset the graph asks for after an accepted closure.
  MapFrameOffset target;
  double recognition_ms{0.0};
  std::uint64_t closures{0U};
};

class StereoLoopClosure {
public:
  explicit StereoLoopClosure(const StereoLoopClosureConfig& config);
  ~StereoLoopClosure();
  StereoLoopClosure(const StereoLoopClosure&) = delete;
  StereoLoopClosure& operator=(const StereoLoopClosure&) = delete;

  // Whether the filter's pose is a keyframe's: far enough from the last one.
  [[nodiscard]] bool due(const VisualInertialClonePose& pose) const;

  // A keyframe: the rectified pair, the filter's pose at it and the frame
  // offset it is published under. With `act` an accepted closure enters the
  // graph and the report carries the offset the graph asks for; without it
  // (the measuring mode) the closure is read and nothing is moved.
  [[nodiscard]] StereoLoopClosureReport
  observe(std::int64_t stamp_ns, const cv::Mat& left, const cv::Mat& right,
          const VisualInertialClonePose& pose, const MapFrameOffset& frame, bool act);

  // The filter started again: the next odometry edge says little.
  void filterRestarted() noexcept;

  // The log line of a keyframe's reading.
  [[nodiscard]] static std::string summary(const StereoLoopClosureReport& report);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav
