#pragma once

#include "drone_city_nav/gnss_trust.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "stereo_map_registration.hpp"

// The camera estimator node's numbers and helpers (visual_inertial_odometry_node.cpp).
namespace drone_city_nav::visual_inertial_odometry_node_internal {

// The trust in the satellite signal announced when it changes (roadmap item
// 39): the log line and the mission event the mission check reads.
inline void declareGnssTrust(const rclcpp::Logger& logger,
                             rclcpp::Publisher<std_msgs::msg::String>& events,
                             const GnssTrustVerdict& verdict, const double residual_m,
                             bool& declared) {
  if (verdict.trusted == declared) {
    return;
  }
  RCLCPP_WARN(logger, "GNSS_TRUST changed trusted=%s reason=%s residual_m=%.2f",
              verdict.trusted ? "true" : "false", std::string{verdict.reason}.c_str(),
              residual_m);
  std_msgs::msg::String event;
  event.data =
      std::string{"GNSS_TRUST trusted="} + (verdict.trusted ? "true" : "false");
  events.publish(event);
  declared = verdict.trusted;
}

// The vehicle stands still this many IMU samples before its pose is declared:
// roll, pitch and the gyroscope bias come from them.
inline constexpr std::uint64_t kRestSamplesBeforeInitialPose{100U};
inline constexpr std::int64_t kAutopilotOdometryPeriodNs{40'000'000};

// The optical frame (x right, y down, z forward) of a camera that looks along
// the body's x axis, in the body forward-right-down frame.
[[nodiscard]] inline Eigen::Matrix3d forwardCameraToBody() {
  Eigen::Matrix3d camera_to_body;
  camera_to_body << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  return camera_to_body;
}

// The surface returns within the confident depth, in the body frame, one
// per cell: the centroid of the cell's returns.
[[nodiscard]] inline std::vector<Eigen::Vector3d>
thinnedDepth(const sensor_msgs::msg::PointCloud2& cloud,
             const Eigen::Matrix3d& left_camera_to_body,
             const Eigen::Vector3d& left_camera_body) {
  struct Accumulator {
    Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
    std::size_t count{0U};
  };

  double cell_m = kMapCellM;
  std::vector<Eigen::Vector3d> body;
  body.reserve(cloud.width * cloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> x{cloud, "x"};
  sensor_msgs::PointCloud2ConstIterator<float> y{cloud, "y"};
  sensor_msgs::PointCloud2ConstIterator<float> z{cloud, "z"};
  sensor_msgs::PointCloud2ConstIterator<float> surface{cloud, "intensity"};
  for (; x != x.end(); ++x, ++y, ++z, ++surface) {
    const Eigen::Vector3d flu{*x, *y, *z};
    if (*surface < 0.5F || !flu.allFinite() || flu.norm() > kMapRangeM) {
      continue;
    }
    // The left camera's forward-left-up frame is its optical frame
    // (x right, y down, z forward) as (z, -x, -y).
    body.push_back(left_camera_to_body * Eigen::Vector3d{-flu.y(), -flu.z(), flu.x()} +
                   left_camera_body);
  }
  std::vector<Eigen::Vector3d> thinned;
  // A frame too dense for the budget is thinned coarser, not sampled, as
  // the lidar-inertial estimator thins its scans.
  do {
    std::unordered_map<std::int64_t, Accumulator> cells;
    for (const Eigen::Vector3d& point : body) {
      const auto index = [cell_m](const double value) {
        return static_cast<std::int64_t>(std::floor(value / cell_m)) & 0x1FFFFF;
      };
      Accumulator& cell =
          cells[(index(point.x()) << 42) | (index(point.y()) << 21) | index(point.z())];
      cell.sum += point;
      ++cell.count;
    }
    thinned.clear();
    thinned.reserve(cells.size());
    for (const auto& [key, cell] : cells) {
      thinned.push_back(cell.sum / static_cast<double>(cell.count));
    }
    cell_m *= 1.25;
  } while (thinned.size() > kMapMaximumScanPoints);
  return thinned;
}

} // namespace drone_city_nav::visual_inertial_odometry_node_internal
