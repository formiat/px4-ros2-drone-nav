#pragma once

// The visual-inertial estimator's private state and helpers, shared by the
// estimator's translation units (visual_inertial_odometry.cpp and the
// modules that update its state, visual_inertial_gnss.cpp): the source file
// stands at the repository's line limit, and a module that reads the state
// needs the definition. Included by them alone.

#include "drone_city_nav/visual_inertial_odometry.hpp"
#include "drone_city_nav/visual_inertial_void.hpp"

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

#include "visual_inertial_math.hpp"

namespace drone_city_nav {
namespace visual_inertial_detail {

using Matrix15 = Eigen::Matrix<double, 15, 15>;

// Error state of the IMU block: orientation (a rotation applied on the body
// side, R = R_hat * Exp(theta)), position, velocity, gyroscope bias,
// accelerometer bias. A clone carries orientation and position.
constexpr Eigen::Index kTheta{0};
constexpr Eigen::Index kPosition{3};
constexpr Eigen::Index kVelocity{6};
constexpr Eigen::Index kGyroBias{9};
constexpr Eigen::Index kAccelerometerBias{12};
constexpr Eigen::Index kImuStates{15};
constexpr Eigen::Index kCloneStates{6};

// One reading of the magnetometer, in the body frame (forward, right, down).
struct MagneticReading {
  std::int64_t stamp_ns{0};
  Eigen::Vector3d field_body{Eigen::Vector3d::Zero()};
};

struct Clone {
  std::uint64_t id{0U};
  std::int64_t stamp_ns{0};
  Eigen::Matrix3d body_to_ned{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  // The pose as it was cloned, before any update touched it: where every
  // Jacobian of this clone is evaluated.
  Eigen::Matrix3d first_body_to_ned{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d first_position{Eigen::Vector3d::Zero()};
};

struct TrackObservation {
  std::uint64_t clone_id{0U};
  Eigen::Vector2d left{Eigen::Vector2d::Zero()};
  Eigen::Vector2d right{Eigen::Vector2d::Zero()};
};

struct Camera {
  Eigen::Matrix3d body_to_camera{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d position_body{Eigen::Vector3d::Zero()};
};

struct FeatureRows {
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
};

} // namespace visual_inertial_detail

using namespace visual_inertial_detail; // NOLINT(google-build-using-namespace): the
                                        // estimator's own detail

struct VisualInertialOdometry::Impl {
  explicit Impl(const VisualInertialOdometryConfig& configuration)
      : config{configuration},
        cameras{Camera{configuration.left_camera.camera_to_body.normalized()
                           .toRotationMatrix()
                           .transpose(),
                       configuration.left_camera.position_body_m},
                Camera{configuration.right_camera.camera_to_body.normalized()
                           .toRotationMatrix()
                           .transpose(),
                       configuration.right_camera.position_body_m}},
        gravity{0.0, 0.0, configuration.gravity_mps2} {
  }

  VisualInertialOdometryConfig config;
  std::array<Camera, 2> cameras;
  Eigen::Vector3d gravity;

  bool initialized{false};
  // Set by a hole in the IMU stream longer than the unaided timeout; see
  // `maximum_unaided_s`.
  bool lost{false};
  std::int64_t stamp_ns{0};
  std::int64_t last_update_stamp_ns{0};
  // The height the features last gave and the barometer's then, and the
  // barometer's now: through a dark stretch the height follows the change
  // the barometer saw (roadmap item 17 stage 5).
  double held_height_ned_m{0.0};
  std::optional<double> held_barometric_height_m;
  std::optional<double> barometric_height_m;
  // The barometer's reading at the height the filter was declared at: the
  // height the barometer gives since is a prior on every frame (r2721:
  // under the imperfections profile the height drifted 1.2 m in 250 s).
  std::optional<double> barometric_zero_m;
  // The magnetometer's readings over the last second; the field's
  // horizontal direction in the world summed while the vehicle rests where it
  // was declared, and that place; the heading's zero, their mean, fixed when
  // the vehicle first moves.
  std::deque<MagneticReading> magnetic_readings;
  Eigen::Vector2d magnetic_zero_sum{Eigen::Vector2d::Zero()};
  std::optional<Eigen::Vector3d> magnetic_rest_position;
  std::optional<double> magnetic_zero_rad;
  // The last frame's prior, for the estimate.
  double magnetic_innovation_rad{0.0};
  std::int64_t magnetic_reading_age_ns{0};
  // The void (K46): the pair's judgment, the void's state and its time.
  VisualInertialVoid in_void;
  // The accelerometer across the rotor axis, integrated since the last frame.
  Eigen::Vector2d drag_reading_sum_mps{Eigen::Vector2d::Zero()};
  double drag_reading_s{0.0};
  VisualInertialEstimate last_estimate;
  Eigen::Matrix3d body_to_ned{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_bias{Eigen::Vector3d::Zero()};
  // The IMU pose and velocity as propagated, before the last update moved
  // them: the transition over the next interval is linearized there.
  Eigen::Matrix3d first_body_to_ned{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d first_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d first_velocity{Eigen::Vector3d::Zero()};
  Eigen::MatrixXd covariance;

  std::deque<Clone> clones;
  std::uint64_t next_clone_id{1U};
  std::unordered_map<std::uint64_t, std::vector<TrackObservation>> tracks;

  std::deque<VisualInertialImuSample> imu;
  std::optional<VisualInertialImuSample> last_imu;
  // Samples seen before the declared initial pose: the vehicle is at rest.
  Eigen::Vector3d rest_gyro_sum{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rest_gyro_square_sum{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rest_accelerometer_sum{Eigen::Vector3d::Zero()};
  std::size_t rest_samples{0U};
  // The samples received before the declared pose, summed at the
  // initialisation over those stamped on or before it alone: summed as they
  // arrived, the flight and its replay started from different rests (r2886).
  std::deque<VisualInertialImuSample> rest_imu;

  void propagateStep(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accelerometer,
                     double dt, Matrix15& transition, Matrix15& noise);
  std::int64_t propagateTo(std::int64_t target_ns);
  void augment(std::int64_t frame_stamp_ns);
  [[nodiscard]] const Clone* findClone(std::uint64_t id, Eigen::Index& index) const;
  [[nodiscard]] std::optional<Eigen::Vector3d>
  triangulate(const std::vector<TrackObservation>& observations) const;
  [[nodiscard]] std::optional<FeatureRows>
  featureRows(const std::vector<TrackObservation>& observations,
              const Eigen::Vector3d& point, bool& gated) const;
  void update(const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& residual);
  void marginalizeOldestClone();
  void judgeDeadReckoning(VisualInertialEstimate& estimate) const;
  void holdHeight();
  void barometricHeightPrior();
  void magneticHeadingPrior();
  void fuseRotorDrag();
};

} // namespace drone_city_nav
