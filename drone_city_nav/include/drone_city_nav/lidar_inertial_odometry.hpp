#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

// Lidar-inertial odometry: where the vehicle is, from its own IMU and its
// own lidar, with no satellite, compass or simulator truth in the loop. The
// frame is the autopilot's local NED frame with the origin at the declared
// initial pose; the body is FRD, the lidar's points arrive in the body frame.
// The IMU propagates the state between scans and gives every scan its
// starting guess; each scan is registered point-to-plane against a sliding
// window of the scans before it, and the registered pose corrects the state.
// The submaps are the estimator's own and never the planner's obstacle
// memory: a map assembled from the pose under estimation would confirm that
// pose's error.

namespace drone_city_nav {

struct LidarInertialImuSample {
  std::int64_t stamp_ns{0};
  Eigen::Vector3d gyro_radps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_mps2{Eigen::Vector3d::Zero()};
};

struct LidarInertialOdometryConfig {
  // The scan is thinned to one point per cell of this size before
  // registration; the submap keeps its points in cells of the same size.
  double scan_voxel_m{0.4};
  double minimum_range_m{1.0};
  double maximum_range_m{30.0};
  // A scan point matches the nearest submap point within this distance.
  double maximum_correspondence_m{1.0};
  std::size_t maximum_iterations{10U};
  double convergence_translation_m{1.0e-3};
  double convergence_rotation_rad{1.0e-4};
  // Huber width of the point-to-plane residual.
  double robust_width_m{0.2};
  // A registered scan joins the submap once the vehicle has moved this far
  // or turned this much since the last one it inserted.
  double keyframe_translation_m{0.5};
  double keyframe_rotation_rad{0.17};
  std::size_t maximum_keyframes{40U};
  std::size_t maximum_points_per_cell{20U};
  // The registration is healthy when at least this share of the scan
  // matched, the residual stayed under this width and the translational
  // information along its weakest axis, per matched point, stayed above
  // this floor (a smooth facade or a bare corridor leaves an axis free).
  double minimum_matched_fraction{0.3};
  double maximum_residual_rms_m{0.5};
  double minimum_information_per_point{0.02};
  double gravity_mps2{9.80665};
  // The share of the attitude correction each scan feeds back into the
  // gyroscope bias.
  double gyro_bias_gain{0.05};
};

struct LidarInertialEstimate {
  std::int64_t stamp_ns{0};
  Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
  // Hamilton quaternion rotating the body FRD frame into NED.
  Eigen::Quaterniond body_to_ned{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity_ned_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d position_variance_m2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientation_variance_rad2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_variance_m2ps2{Eigen::Vector3d::Zero()};
  bool healthy{false};
  double matched_fraction{0.0};
  double residual_rms_m{0.0};
  double information_per_point{0.0};
  std::size_t iterations{0U};
  std::size_t scan_points{0U};
  std::size_t submap_points{0U};
  std::size_t keyframes{0U};
};

class LidarInertialOdometry {
public:
  explicit LidarInertialOdometry(const LidarInertialOdometryConfig& config);
  ~LidarInertialOdometry();
  LidarInertialOdometry(const LidarInertialOdometry&) = delete;
  LidarInertialOdometry& operator=(const LidarInertialOdometry&) = delete;

  // The declared initial pose: where the vehicle stands and which way it
  // faces, in NED, at the stamp. Roll and pitch come from the accelerometer
  // samples seen at rest before the first scan.
  void initialize(std::int64_t stamp_ns, const Eigen::Vector3d& position_ned_m,
                  double heading_rad);
  [[nodiscard]] bool initialized() const noexcept;

  void addImu(const LidarInertialImuSample& sample);

  // One scan, its points in the body FRD frame at `stamp_ns`; returns the
  // estimate at that stamp.
  [[nodiscard]] LidarInertialEstimate
  addScan(std::int64_t stamp_ns, const std::vector<Eigen::Vector3d>& points_body);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav
