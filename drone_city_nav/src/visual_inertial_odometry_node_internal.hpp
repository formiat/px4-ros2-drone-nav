#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

// The camera estimator node's numbers and helpers (visual_inertial_odometry_node.cpp).
namespace drone_city_nav::visual_inertial_odometry_node_internal {

// The vehicle stands still this many IMU samples before its pose is declared:
// roll, pitch and the gyroscope bias come from them.
inline constexpr std::uint64_t kRestSamplesBeforeInitialPose{100U};
inline constexpr std::int64_t kAutopilotOdometryPeriodNs{40'000'000};

// The long-lived map the depth is registered against where the vehicle has
// been before (roadmap item 19 measured the drift of a doubled path at 0.4
// to 4.1 m, r670 to r689, and r689 flew into the launch platform 4.1 m from
// where the estimate put it). Surfaces within the confident depth only, in
// the cells of the lidar-inertial estimator's map.
inline constexpr double kMapCellM{0.4};
inline constexpr std::size_t kMapPointsPerCell{6U};
inline constexpr double kMapRangeM{6.0};
inline constexpr std::size_t kMapMaximumScanPoints{3000U};
// Only what was mapped this long ago is registered against: the depth
// just laid down carries the same drift as the pose it is laid from.
inline constexpr std::int64_t kMapAgeNs{20'000'000'000};
// A registration every this long; the drift the map corrects accrues over
// minutes.
inline constexpr std::int64_t kMapRegistrationPeriodNs{500'000'000};
// What makes a registration a correction: most of the scan matched old
// cells, the fit is tight, and an axis carries information per matched
// point above the floor (the lidar-inertial estimator's gates).
inline constexpr double kMapMinimumMatchedFraction{0.3};
inline constexpr std::size_t kMapMinimumMatchedPoints{150U};
inline constexpr double kMapMaximumResidualRmsM{0.25};
inline constexpr double kMapMinimumInformationPerPoint{0.01};
// The correction is an offset from the filter's frame to the map's, kept
// outside the filter: the filter's position is certain to a few centimetres
// while its drift is metres (r691: clone sigma 0.05 m against innovations of
// 0.4 to 1.0 m), and a Kalman step against it passed only innovations under
// half a metre. A registration moves the target offset by this share of what
// it measured along its observed axes, by no more than the step, and a
// registration farther than the reach from its prior slid into another fit.
// The share is small against the registration's own scatter: at a half and a
// quarter metre per step, r709's target moved 0.2 to 0.35 m and its heading
// half a degree between registrations a second apart, where the drift it
// corrects accrues millimetres and hundredths of a degree per second.
inline constexpr double kMapCorrectionGain{0.2};
inline constexpr double kMapCorrectionStepM{0.1};
inline constexpr double kMapCorrectionReachM{1.5};
// The offset the autopilot sees follows the target at this rate: its fusion
// of the external position takes motion, and a jump it may refuse.
inline constexpr double kMapOffsetRateMps{0.2};
// The heading the filter holds drifts too, one to four degrees over the
// doubled path (r683 to r692), and a degree is 0.9 m over the 50 m of an
// open stretch where nothing within the depth range fixes the position. A
// registration whose depth fixes the heading, with the translation left
// free, moves the target heading by this share of what it measured, by no
// more than the step, and the published heading follows at the rate.
inline constexpr double kMapYawGain{0.2};
inline constexpr double kMapYawStepRad{0.0035};
inline constexpr double kMapYawReachRad{0.087};
inline constexpr double kMapMinimumYawInformationPerPoint{0.5};
inline constexpr double kMapYawRateRadps{0.0175};

// From the filter's frame to the map's: a turn about the vertical, then a
// shift.
struct MapFrameOffset {
  double yaw_rad{0.0};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};

  [[nodiscard]] Eigen::Matrix3d rotation() const {
    return Eigen::AngleAxisd{yaw_rad, Eigen::Vector3d::UnitZ()}.toRotationMatrix();
  }

  [[nodiscard]] Eigen::Vector3d apply(const Eigen::Vector3d& position) const {
    return rotation() * position + translation;
  }
};

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
