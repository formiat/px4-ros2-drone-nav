// The camera estimator's depth against its own long-lived map (roadmap items 19
// and 40): each depth frame registered against the map's old part, then laid
// into it (moved whole from visual_inertial_odometry_node.cpp on 2026-11-04,
// the source size contract).
#include "visual_inertial_odometry_node_class.hpp"

namespace drone_city_nav::visual_inertial_odometry_node_internal {

// One depth frame: registered against the old part of the long-lived map
// when one is due, then laid into it at the frame's pose as the filter now
// holds it. The returns are forward-left-up at the left camera.
void VisualInertialOdometryNode::onDepth(const sensor_msgs::msg::PointCloud2& cloud) {
  const std::int64_t stamp_ns =
      rclcpp::Time{cloud.header.stamp}.nanoseconds() + frame_stamp_offset_ns_;
  std::vector<Eigen::Vector3d> points_body =
      thinnedDepth(cloud, left_camera_to_body_, left_camera_body_);
  if (points_body.empty()) {
    return;
  }
  std::optional<VisualInertialClonePose> pose;
  {
    const std::scoped_lock lock{odometry_mutex_};
    pose = odometry_->clonePose(stamp_ns);
  }
  if (!pose.has_value()) {
    map_registration_.countFrameWithoutClone();
    return;
  }
  if (map_registration_.due(stamp_ns)) {
    const StereoMapRegistrationReport report = map_registration_.registerFrame(
        stamp_ns, points_body, *pose, map_registration_act_);
    // Every reading in the measuring mode (roadmap item 40), one a second otherwise.
    if (!map_registration_act_) {
      RCLCPP_INFO(
          get_logger(), "%s mode=measure",
          StereoMapRegistration::summary(report, map_registration_.framesWithoutClone())
              .c_str());
    } else {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000, "%s",
          StereoMapRegistration::summary(report, map_registration_.framesWithoutClone())
              .c_str());
    }
    const std::scoped_lock lock{odometry_mutex_};
    pose = odometry_->clonePose(stamp_ns);
  }
  if (!pose.has_value()) {
    return;
  }
  map_registration_.lay(stamp_ns, std::move(points_body), *pose);
}

} // namespace drone_city_nav::visual_inertial_odometry_node_internal
