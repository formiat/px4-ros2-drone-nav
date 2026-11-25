// The camera estimator's depth against its own long-lived map (roadmap items 19
// and 40): each depth frame registered against the map's old part, then laid
// into it (moved whole from visual_inertial_odometry_node.cpp on 2026-11-04,
// the source size contract); and the loop closures that move the same frame
// offset (roadmap item 41).
#include "visual_inertial_odometry_node_class.hpp"

namespace drone_city_nav::visual_inertial_odometry_node_internal {

// A depth frame as it arrives, on the depth's own thread: thinned and held
// for the frame thread. The returns are forward-left-up at the left camera.
void VisualInertialOdometryNode::onDepth(const sensor_msgs::msg::PointCloud2& cloud) {
  const std::int64_t stamp_ns =
      rclcpp::Time{cloud.header.stamp}.nanoseconds() + frame_stamp_offset_ns_;
  std::vector<Eigen::Vector3d> points_body =
      thinnedDepth(cloud, left_camera_to_body_, left_camera_body_);
  const std::scoped_lock lock{odometry_mutex_};
  last_depth_stamp_ns_ = std::max(last_depth_stamp_ns_, stamp_ns);
  if (points_body.empty()) {
    return;
  }
  pending_depth_.push_back(DepthFrame{stamp_ns, std::move(points_body)});
  // The frames stopped (the filter lost, a hole): the oldest go.
  while (pending_depth_.size() > kPendingDepthFrames) {
    pending_depth_.pop_front();
  }
}

// The depth frames stamped up to `stamp_ns`, in their order, each registered
// against the old part of the long-lived map when one is due and laid into
// it at its frame's pose as the filter holds it now. Called by the frame
// thread after a frame, under the filter's lock, for the depth a stated time
// behind that frame: taken as it arrived, a depth frame met the filter one
// frame further on in a flight than in its replay, was registered from
// another pose of its clone against another map, and the map's frame, which
// the published pose carries, stepped apart (r2936: the filter's own states
// equal to the last digit over 2423 frames, the published pose 0.03 m apart
// from the 280th second).
void VisualInertialOdometryNode::takeDepthUpTo(const std::int64_t stamp_ns) {
  while (!pending_depth_.empty() && pending_depth_.front().stamp_ns <= stamp_ns) {
    DepthFrame frame = std::move(pending_depth_.front());
    pending_depth_.pop_front();
    const std::optional<VisualInertialClonePose> pose =
        odometry_->clonePose(frame.stamp_ns);
    // The frame record's line of a depth frame (estimator_frame_log): what
    // arrived, whether the filter still held its pose, and the frame offset
    // after it. r4234's replay parted from its flight in that offset alone,
    // the filter's own frames equal to the last digit, and nothing said at
    // which depth frame.
    const char* registered = "none";
    if (frame_log_) {
      std::uint64_t hash = kDigestSeed;
      for (const Eigen::Vector3d& point : frame.points_body) {
        mix(hash, point.data(), 3U * sizeof(double));
      }
      RCLCPP_INFO(get_logger(),
                  "VIO_DEPTH stamp_s=%.3f points=%zu digest=%016" PRIx64 " clone=%d",
                  1.0e-9 * static_cast<double>(frame.stamp_ns),
                  frame.points_body.size(), hash, pose.has_value() ? 1 : 0);
    }
    if (!pose.has_value()) {
      map_registration_.countFrameWithoutClone();
      continue;
    }
    if (map_registration_.due(frame.stamp_ns)) {
      const StereoMapRegistrationReport report = map_registration_.registerFrame(
          frame.stamp_ns, frame.points_body, *pose, map_registration_act_);
      registered = report.outcome;
      // Every reading in the measuring mode (roadmap item 40), one a second otherwise.
      if (!map_registration_act_) {
        RCLCPP_INFO(get_logger(), "%s mode=measure",
                    StereoMapRegistration::summary(
                        report, map_registration_.framesWithoutClone())
                        .c_str());
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                             StereoMapRegistration::summary(
                                 report, map_registration_.framesWithoutClone())
                                 .c_str());
      }
    }
    map_registration_.lay(frame.stamp_ns, std::move(frame.points_body), *pose);
    if (frame_log_) {
      const MapFrameOffset target = map_registration_.target();
      RCLCPP_INFO(get_logger(),
                  "VIO_DEPTH_LAID stamp_s=%.3f registered=%s target=(%.5f,%.5f,%.5f) "
                  "target_yaw=%.7f map_points=%zu",
                  1.0e-9 * static_cast<double>(frame.stamp_ns), registered,
                  target.translation.x(), target.translation.y(),
                  target.translation.z(), target.yaw_rad,
                  map_registration_.mapPointCount());
    }
  }
}

// A keyframe's recognition takes a tenth of a second, and the frame thread
// holds the filter's lock: the keyframe is read on a thread of its own, from
// copies of the pair and of the frame's offset, one keyframe at a time, and
// nothing else touches the keyframes until its reading is taken.
void VisualInertialOdometryNode::startLoopReading(
    const std::int64_t stamp_ns, const std::array<cv::Mat, 2>& rectified_pair,
    const bool healthy) {
  if (loop_closure_ == nullptr || !healthy || loop_reading_.valid()) {
    return;
  }
  const std::optional<VisualInertialClonePose> pose = odometry_->clonePose(stamp_ns);
  if (!pose.has_value() || !loop_closure_->due(*pose)) {
    return;
  }
  loop_reading_frames_ = 0;
  loop_reading_ =
      std::async(std::launch::async, [this, stamp_ns, left = rectified_pair[0].clone(),
                                      right = rectified_pair[1].clone(), pose = *pose,
                                      frame = map_registration_.target()] {
        return loop_closure_->observe(stamp_ns, left, right, pose, frame,
                                      loop_closure_act_);
      });
}

// The reading of a keyframe, two frames after it whatever the thread's pace
// (a flight and its replay take it at the same frame; the frame thread waits
// for it only when it is later than that): logged when a place was tried,
// and an accepted closure's offset becomes the target the published frame
// follows (item 40's bounded turn and shift).
void VisualInertialOdometryNode::takeLoopReading() {
  constexpr int kReadingFrames{2};
  if (loop_reading_.valid() && ++loop_reading_frames_ >= kReadingFrames) {
    finishLoopReading();
  }
}

void VisualInertialOdometryNode::finishLoopReading() {
  const StereoLoopClosureReport report = loop_reading_.get();
  if (report.candidates > 0U) {
    RCLCPP_INFO(get_logger(), "%s mode=%s", StereoLoopClosure::summary(report).c_str(),
                loop_closure_act_ ? "act" : "measure");
  }
  if (report.accepted && loop_closure_act_) {
    map_registration_.retarget(report.target);
  }
}

// The filter starts again: a reading in flight is finished first, its
// keyframe was the old filter's.
void VisualInertialOdometryNode::loopFilterRestarted() {
  if (loop_closure_ == nullptr) {
    return;
  }
  if (loop_reading_.valid()) {
    finishLoopReading();
  }
  loop_closure_->filterRestarted();
}

} // namespace drone_city_nav::visual_inertial_odometry_node_internal
