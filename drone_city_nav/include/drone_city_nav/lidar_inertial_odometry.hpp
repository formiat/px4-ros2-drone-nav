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
  // A thinned scan larger than this keeps every k-th point: inside a
  // structure a 0.4 m thinning still leaves 11 000 points (r358), and the
  // registration has to fit the scan period.
  std::size_t maximum_scan_points{4000U};
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
  // Keyframes overlap, and every point of a cell is a candidate for every
  // query that reaches it; six keep the nearest search inside the scan
  // period where twenty did not (r357: 217 ms per scan at 300 000 points).
  std::size_t maximum_points_per_cell{6U};
  // When a scan fails to register from the propagated pose, it is tried
  // again from the last registered pose carried forward at its velocity,
  // with the correspondence distance widened by this factor; until a scan
  // registers again the position holds there and only the attitude follows
  // the gyroscope, so a lost scan cannot send the estimate running.
  double recovery_correspondence_factor{3.0};
  // The registration is healthy when at least this share of the scan
  // matched and the residual stayed under this width. A translational axis
  // whose information per matched point falls under the floor is
  // degenerate: a bare corridor leaves its own axis free, and along it the
  // registration is not believed; the IMU carries the motion there and the
  // variance reported for that axis is the degenerate one.
  double minimum_matched_fraction{0.3};
  double maximum_residual_rms_m{0.5};
  double minimum_information_per_point{0.02};
  double degenerate_axis_variance_m2{1.0};
  // The share of a scan's position correction, spread over the interval
  // since the last scan, that corrects the IMU-integrated velocity.
  double velocity_correction_gain{0.5};
  double gravity_mps2{9.80665};
  // The share of the rotation the IMU missed over a scan interval that is
  // attributed to the gyroscope bias, per scan, and the bias the estimator
  // will believe. r357 flew with 0.05 of the error per scan divided by the
  // interval, ten times this, and the heading ran away within ten seconds.
  double gyro_bias_gain{0.005};
  double maximum_gyro_bias_radps{0.05};
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
  // How far behind the scan the last IMU sample was when the scan was
  // registered; the prior is only as current as this.
  std::int64_t imu_lag_ns{0};
  double matched_fraction{0.0};
  double residual_rms_m{0.0};
  double information_per_point{0.0};
  // Translational axes the registration could not observe this scan.
  std::size_t degenerate_axes{0U};
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

  // One scan, its points in the body FRD frame at `stamp_ns`, on the same
  // clock as the IMU samples; returns the estimate at that stamp. The IMU
  // samples since the last scan are integrated up to the stamp for the
  // scan's starting guess, so a scan processed late still registers where
  // it was taken.
  [[nodiscard]] LidarInertialEstimate
  addScan(std::int64_t stamp_ns, const std::vector<Eigen::Vector3d>& points_body);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav
