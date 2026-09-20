#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

// Visual-inertial odometry: where the vehicle is, from its own IMU and its
// own stereo pair, with no satellite, compass, lidar or simulator truth in
// the loop. A stereo multi-state constraint Kalman filter: the state is the
// body pose, velocity and the two IMU biases with a sliding set of cloned
// body poses, one per frame; no point is ever a state. The IMU propagates the
// state and its covariance between frames. A feature is used once, when its
// track ends or outgrows the window: it is triangulated from every
// observation, its residuals are projected onto the left null space of its
// own Jacobian, so the point's error leaves the equations, and what remains
// constrains the cloned poses in one Kalman step. Jacobians are evaluated at
// the first estimate of every pose (FEJ): a filter that re-linearizes at each
// new estimate gains information along the global heading and position,
// which no camera observes, and reports a certainty it does not have.
//
// The frame is the autopilot's local NED frame with the origin at the
// declared initial pose; the body is FRD. The library reads Eigen and
// nothing else: the images are tracked outside it, and observations arrive
// as normalized image coordinates.

namespace drone_city_nav {

struct VisualInertialImuSample {
  std::int64_t stamp_ns{0};
  Eigen::Vector3d gyro_radps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_mps2{Eigen::Vector3d::Zero()};
};

// One feature seen by both cameras in one frame. The coordinates are
// normalized (x / z, y / z) in each camera's optical frame: x right, y down,
// z along the optical axis.
struct StereoFeatureObservation {
  std::uint64_t id{0U};
  Eigen::Vector2d left{Eigen::Vector2d::Zero()};
  Eigen::Vector2d right{Eigen::Vector2d::Zero()};
};

// Where a camera's optical frame sits in the body FRD frame.
struct VisualInertialCameraMount {
  Eigen::Quaterniond camera_to_body{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_body_m{Eigen::Vector3d::Zero()};
};

struct VisualInertialOdometryConfig {
  VisualInertialCameraMount left_camera;
  VisualInertialCameraMount right_camera;
  // Cloned poses kept; the oldest leaves when one more arrives.
  std::size_t maximum_clones{12U};
  // A track shorter than this many frames constrains nothing worth its
  // rows: one stereo frame alone only restates the fixed baseline.
  std::size_t minimum_track_frames{3U};
  // Features used in one update, the longest tracks first.
  std::size_t maximum_features_per_update{40U};
  // Standard deviation of one image coordinate, normalized: pixels over the
  // focal length.
  double observation_noise{2.7e-3};
  // A triangulated point is kept within these distances of every camera
  // that saw it.
  double minimum_depth_m{0.3};
  double maximum_depth_m{60.0};
  // A feature's projected residual is gated at this quantile of chi-square.
  double gate_normal_quantile{1.645};
  // Continuous-time IMU noise densities and bias random walks. The
  // accelerometer's is not the sensor's own: a tilt error of 0.2 degrees
  // leaks 0.03 m/s^2 of gravity into the horizontal acceleration. With the
  // sensor's 0.02 the filter believed the IMU over the pair, refused a
  // quarter of the features and measured every displacement 1.6 percent
  // short, on two recorded flights and both IMU streams; with 0.2 it refuses
  // 2 percent, the displacement ratio is 0.998, and the position error at
  // the goal of a 513 m flight is 0.7 m (r550).
  double gyro_noise_radps_sqrt_hz{1.0e-3};
  double accelerometer_noise_mps2_sqrt_hz{0.2};
  double gyro_bias_walk_radps2_sqrt_hz{2.0e-5};
  double accelerometer_bias_walk_mps3_sqrt_hz{1.0e-3};
  // Two samples farther apart than this are a hole in the stream (the
  // autopilot's IMU crosses a best-effort transport: r547 lost 0.52 s of it),
  // and over a hole the state's uncertainty grows by what the vehicle can do
  // unobserved: the recorded flights turn at up to 2.6 rad/s and accelerate
  // at up to 4 m/s^2.
  double maximum_imu_period_s{0.05};
  double unobserved_turn_rate_radps{2.6};
  double unobserved_acceleration_mps2{4.0};
  // The estimate is healthy while features corrected it within this long;
  // past it the IMU alone carries the state.
  double maximum_unaided_s{1.0};
  // What the declared initial pose and the alignment at rest are worth.
  double initial_tilt_sigma_rad{0.02};
  double initial_heading_sigma_rad{1.0e-3};
  double initial_position_sigma_m{1.0e-3};
  double initial_velocity_sigma_mps{0.05};
  double initial_gyro_bias_sigma_radps{5.0e-3};
  double initial_accelerometer_bias_sigma_mps2{0.1};
  double gravity_mps2{9.80665};
};

struct VisualInertialEstimate {
  std::int64_t stamp_ns{0};
  Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
  // Hamilton quaternion rotating the body FRD frame into NED.
  Eigen::Quaterniond body_to_ned{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity_ned_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias_radps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_bias_mps2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d position_variance_m2{Eigen::Vector3d::Zero()};
  // About the NED axes: the third is the heading.
  Eigen::Vector3d orientation_variance_rad2{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_variance_m2ps2{Eigen::Vector3d::Zero()};
  bool healthy{false};
  // How far behind the frame the last IMU sample was when the frame was
  // taken in; the propagation held the last sample over that interval.
  std::int64_t imu_lag_ns{0};
  std::size_t clones{0U};
  std::size_t tracked_features{0U};
  // Features this frame's update: offered, refused by triangulation,
  // refused by the gate, and used.
  std::size_t candidate_features{0U};
  std::size_t untriangulated_features{0U};
  std::size_t gated_features{0U};
  std::size_t used_features{0U};
  // Root mean square of the used residuals in observation deviations;
  // about one when the noise model fits.
  double residual_rms_sigma{0.0};
  // The standard deviation of the velocity along its least certain
  // direction. Features constrain the velocity in every direction they give
  // parallax in; along one they do not, only the accelerometer holds it and
  // this grows.
  double weakest_velocity_sigma_mps{0.0};
};

class VisualInertialOdometry {
public:
  explicit VisualInertialOdometry(const VisualInertialOdometryConfig& config);
  ~VisualInertialOdometry();
  VisualInertialOdometry(const VisualInertialOdometry&) = delete;
  VisualInertialOdometry& operator=(const VisualInertialOdometry&) = delete;

  // The declared initial pose: where the vehicle stands and which way it
  // faces, in NED, at the stamp. Roll, pitch and the gyroscope bias come from
  // the IMU samples seen at rest before it.
  void initialize(std::int64_t stamp_ns, const Eigen::Vector3d& position_ned_m,
                  double heading_rad);
  [[nodiscard]] bool initialized() const noexcept;

  void addImu(const VisualInertialImuSample& sample);

  // One frame of the pair at `stamp_ns`, on the IMU's clock. The IMU samples
  // up to the stamp must have been added: the frame's pose is cloned where
  // the propagation stands.
  [[nodiscard]] VisualInertialEstimate
  addFrame(std::int64_t stamp_ns,
           const std::vector<StereoFeatureObservation>& observations);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav
