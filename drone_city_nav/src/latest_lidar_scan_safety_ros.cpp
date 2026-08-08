#include "drone_city_nav/latest_lidar_scan_safety_ros.hpp"

#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <limits>
#include <string>

namespace drone_city_nav {

msg::LatestLidarSafetyScan makeLatestLidarSafetyScanMessage(
    const LatestLidarSafetyScanBuildResult& scan,
    const std_msgs::msg::Header& source_header, const std::string_view frame_id,
    const std::int64_t acquisition_stamp_ns, const std::uint64_t sequence,
    const std::uint64_t pose_generation) {
  msg::LatestLidarSafetyScan message;
  message.header = source_header;
  message.header.stamp = rclcpp::Time{acquisition_stamp_ns, RCL_ROS_TIME};
  message.header.frame_id = std::string{frame_id};
  message.sequence = sequence;
  message.pose_generation = pose_generation;
  message.frame_origin_map.x = scan.acquisition_body_frame.origin_map_m.x;
  message.frame_origin_map.y = scan.acquisition_body_frame.origin_map_m.y;
  message.frame_origin_map.z = scan.acquisition_body_frame.origin_map_m.z;
  message.body_x_axis_map.x = scan.acquisition_body_frame.x_axis_map.x;
  message.body_x_axis_map.y = scan.acquisition_body_frame.x_axis_map.y;
  message.body_x_axis_map.z = scan.acquisition_body_frame.x_axis_map.z;
  message.body_y_axis_map.x = scan.acquisition_body_frame.y_axis_map.x;
  message.body_y_axis_map.y = scan.acquisition_body_frame.y_axis_map.y;
  message.body_y_axis_map.z = scan.acquisition_body_frame.y_axis_map.z;
  message.body_z_axis_map.x = scan.acquisition_body_frame.z_axis_map.x;
  message.body_z_axis_map.y = scan.acquisition_body_frame.z_axis_map.y;
  message.body_z_axis_map.z = scan.acquisition_body_frame.z_axis_map.z;
  message.hit_points_body_frd.reserve(scan.hit_points_body_frd.size());
  for (const Point3& point : scan.hit_points_body_frd) {
    geometry_msgs::msg::Point32 output;
    output.x = static_cast<float>(point.x);
    output.y = static_cast<float>(point.y);
    output.z = static_cast<float>(point.z);
    message.hit_points_body_frd.push_back(output);
  }
  message.source_beam_count = static_cast<std::uint32_t>(std::min<std::size_t>(
      scan.source_beam_count, std::numeric_limits<std::uint32_t>::max()));
  message.invalid_beam_count = static_cast<std::uint32_t>(std::min<std::size_t>(
      scan.invalid_beam_count, std::numeric_limits<std::uint32_t>::max()));
  return message;
}

} // namespace drone_city_nav
