// The camera estimator's depth against its own long-lived map (roadmap items 19
// and 40): each depth frame registered against the map's old part, then laid
// into it (moved whole from visual_inertial_odometry_node.cpp on 2026-11-04,
// the source size contract).
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
    if (!pose.has_value()) {
      map_registration_.countFrameWithoutClone();
      continue;
    }
    if (map_registration_.due(frame.stamp_ns)) {
      const StereoMapRegistrationReport report = map_registration_.registerFrame(
          frame.stamp_ns, frame.points_body, *pose, map_registration_act_);
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
  }
}

} // namespace drone_city_nav::visual_inertial_odometry_node_internal
