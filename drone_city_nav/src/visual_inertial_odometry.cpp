#include "drone_city_nav/visual_inertial_odometry.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <optional>
#include <span>
#include <unordered_map>
#include <utility>

namespace drone_city_nav {
namespace {

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

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Matrix3d expSo3(const Eigen::Vector3d& rotation) {
  const double angle = rotation.norm();
  if (angle < 1.0e-9) {
    return Eigen::Matrix3d::Identity() + skew(rotation);
  }
  return Eigen::AngleAxisd{angle, rotation / angle}.toRotationMatrix();
}

Eigen::Matrix3d orthonormalized(const Eigen::Matrix3d& rotation) {
  return Eigen::Quaterniond{rotation}.normalized().toRotationMatrix();
}

// The quantile of chi-square with `degrees` degrees of freedom at the normal
// quantile `z` (Wilson and Hilferty).
double chiSquareQuantile(const Eigen::Index degrees, const double z) {
  const double k = static_cast<double>(degrees);
  const double term = 1.0 - 2.0 / (9.0 * k) + z * std::sqrt(2.0 / (9.0 * k));
  return k * term * term * term;
}

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

} // namespace

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
  std::int64_t stamp_ns{0};
  std::int64_t last_update_stamp_ns{0};
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
  Eigen::Vector3d rest_accelerometer_sum{Eigen::Vector3d::Zero()};
  std::size_t rest_samples{0U};

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
};

void VisualInertialOdometry::Impl::propagateStep(const Eigen::Vector3d& gyro,
                                                 const Eigen::Vector3d& accelerometer,
                                                 const double dt, Matrix15& transition,
                                                 Matrix15& noise) {
  const Eigen::Vector3d rate = gyro - gyro_bias;
  const Eigen::Vector3d specific_force = accelerometer - accelerometer_bias;
  const Eigen::Matrix3d half_turn = expSo3(0.5 * dt * rate);
  const Eigen::Vector3d acceleration =
      body_to_ned * half_turn * specific_force + gravity;
  const Eigen::Matrix3d next_body_to_ned = body_to_ned * half_turn * half_turn;
  const Eigen::Vector3d next_position =
      position + velocity * dt + 0.5 * acceleration * dt * dt;
  const Eigen::Vector3d next_velocity = velocity + acceleration * dt;

  // The transition is written in the differences of the states at the two
  // ends, the start taken at its first estimate: the directions the filter
  // cannot observe (a turn about gravity, a shift) then stay in the null
  // space of every transition and every measurement Jacobian together.
  const Eigen::Matrix3d turn = first_body_to_ned.transpose() * next_body_to_ned;
  const Eigen::Matrix3d force_to_ned = first_body_to_ned * half_turn;
  const Eigen::Matrix3d force_cross = skew(force_to_ned * specific_force);
  Matrix15 step = Matrix15::Identity();
  step.block<3, 3>(kTheta, kTheta) = turn.transpose();
  step.block<3, 3>(kTheta, kGyroBias) =
      -(Eigen::Matrix3d::Identity() - 0.5 * dt * skew(rate)) * dt;
  step.block<3, 3>(kPosition, kTheta) =
      -skew(next_position - first_position - first_velocity * dt -
            0.5 * gravity * dt * dt) *
      first_body_to_ned;
  step.block<3, 3>(kPosition, kVelocity) = Eigen::Matrix3d::Identity() * dt;
  step.block<3, 3>(kPosition, kGyroBias) =
      force_cross * force_to_ned * (dt * dt * dt / 6.0);
  step.block<3, 3>(kPosition, kAccelerometerBias) = -force_to_ned * (0.5 * dt * dt);
  step.block<3, 3>(kVelocity, kTheta) =
      -skew(next_velocity - first_velocity - gravity * dt) * first_body_to_ned;
  step.block<3, 3>(kVelocity, kGyroBias) = force_cross * force_to_ned * (0.5 * dt * dt);
  step.block<3, 3>(kVelocity, kAccelerometerBias) = -force_to_ned * dt;

  Matrix15 step_noise = Matrix15::Zero();
  const auto density = [dt](const double sigma) { return sigma * sigma * dt; };
  step_noise.block<3, 3>(kTheta, kTheta)
      .diagonal()
      .setConstant(density(config.gyro_noise_radps_sqrt_hz));
  step_noise.block<3, 3>(kVelocity, kVelocity)
      .diagonal()
      .setConstant(density(config.accelerometer_noise_mps2_sqrt_hz));
  step_noise.block<3, 3>(kPosition, kPosition)
      .diagonal()
      .setConstant(density(config.accelerometer_noise_mps2_sqrt_hz) * dt * dt / 3.0);
  step_noise.block<3, 3>(kGyroBias, kGyroBias)
      .diagonal()
      .setConstant(density(config.gyro_bias_walk_radps2_sqrt_hz));
  step_noise.block<3, 3>(kAccelerometerBias, kAccelerometerBias)
      .diagonal()
      .setConstant(density(config.accelerometer_bias_walk_mps3_sqrt_hz));

  transition = step * transition;
  noise = step * noise * step.transpose() + step_noise;
  body_to_ned = next_body_to_ned;
  position = next_position;
  velocity = next_velocity;
  // Inside an interval no update separates the estimate from its first
  // value: the next step is linearized where this one ended.
  first_body_to_ned = body_to_ned;
  first_position = position;
  first_velocity = velocity;
}

// Integrates the buffered samples up to the target and applies the interval's
// transition to the covariance once; returns how far short of the target the
// samples ended.
std::int64_t VisualInertialOdometry::Impl::propagateTo(const std::int64_t target_ns) {
  Matrix15 transition = Matrix15::Identity();
  Matrix15 noise = Matrix15::Zero();
  std::int64_t imu_lag_ns = 0;
  while (stamp_ns < target_ns) {
    if (!last_imu.has_value() && imu.empty()) {
      break;
    }
    // Midpoint of the two samples that bracket the step; past the last
    // sample, that sample is held.
    VisualInertialImuSample from = last_imu.value_or(imu.front());
    while (!imu.empty() && imu.front().stamp_ns <= stamp_ns) {
      from = imu.front();
      last_imu = from;
      imu.pop_front();
    }
    std::int64_t end_ns = target_ns;
    VisualInertialImuSample to = from;
    if (!imu.empty()) {
      to = imu.front();
      end_ns = std::min(target_ns, to.stamp_ns);
    } else {
      imu_lag_ns = target_ns - std::max(stamp_ns, from.stamp_ns);
    }
    const double dt = static_cast<double>(end_ns - stamp_ns) * 1.0e-9;
    if (dt > 0.0) {
      propagateStep(0.5 * (from.gyro_radps + to.gyro_radps),
                    0.5 * (from.accelerometer_mps2 + to.accelerometer_mps2), dt,
                    transition, noise);
      if (dt > config.maximum_imu_period_s) {
        // Samples were lost: what the vehicle did over the hole is unknown up
        // to what it can do, not up to the sensor's noise.
        const double turned = config.unobserved_turn_rate_radps * dt;
        const double gained = config.unobserved_acceleration_mps2 * dt;
        noise.block<3, 3>(kTheta, kTheta).diagonal().array() += turned * turned;
        noise.block<3, 3>(kVelocity, kVelocity).diagonal().array() += gained * gained;
        noise.block<3, 3>(kPosition, kPosition).diagonal().array() +=
            0.25 * gained * gained * dt * dt;
      }
    }
    stamp_ns = end_ns;
  }
  body_to_ned = orthonormalized(body_to_ned);
  first_body_to_ned = body_to_ned;
  const Eigen::Index size = covariance.rows();
  const Eigen::Index rest = size - kImuStates;
  covariance.topLeftCorner(kImuStates, kImuStates) =
      transition * covariance.topLeftCorner(kImuStates, kImuStates) *
          transition.transpose() +
      noise;
  if (rest > 0) {
    covariance.topRightCorner(kImuStates, rest) =
        transition * covariance.topRightCorner(kImuStates, rest);
    covariance.bottomLeftCorner(rest, kImuStates) =
        covariance.topRightCorner(kImuStates, rest).transpose();
  }
  return imu_lag_ns;
}

void VisualInertialOdometry::Impl::augment(const std::int64_t frame_stamp_ns) {
  clones.push_back(Clone{.id = next_clone_id++,
                         .stamp_ns = frame_stamp_ns,
                         .body_to_ned = body_to_ned,
                         .position = position,
                         .first_body_to_ned = body_to_ned,
                         .first_position = position});
  const Eigen::Index size = covariance.rows();
  Eigen::MatrixXd grown =
      Eigen::MatrixXd::Zero(size + kCloneStates, size + kCloneStates);
  grown.topLeftCorner(size, size) = covariance;
  // The clone is the IMU pose: its error is the first six IMU error states.
  grown.block(size, 0, kCloneStates, size) = covariance.topRows(kCloneStates);
  grown.block(0, size, size, kCloneStates) = covariance.leftCols(kCloneStates);
  grown.bottomRightCorner(kCloneStates, kCloneStates) =
      covariance.topLeftCorner(kCloneStates, kCloneStates);
  covariance = std::move(grown);
}

const Clone* VisualInertialOdometry::Impl::findClone(const std::uint64_t id,
                                                     Eigen::Index& index) const {
  for (std::size_t i = 0U; i < clones.size(); ++i) {
    if (clones[i].id == id) {
      index = kImuStates + kCloneStates * static_cast<Eigen::Index>(i);
      return &clones[i];
    }
  }
  return nullptr;
}

// The point every observation's ray passes nearest to, refined on the
// reprojection error; refused when it lies outside the depth bounds of a
// camera that saw it. How well it reprojects is not judged here: the cloned
// poses it was built from carry their own error, and only the gate, which
// knows that error, can tell a bad point from poses that need the correction
// this point brings. When the poses disagree so far that no such point
// exists, the newest frame's own pair places it: the baseline between the
// cameras is known whatever the poses are. A filter that refused instead
// never used a feature again after one half-second hole in the IMU stream
// (r547).
std::optional<Eigen::Vector3d> VisualInertialOdometry::Impl::triangulate(
    const std::vector<TrackObservation>& observations) const {
  struct View {
    Eigen::Matrix3d ned_to_camera;
    Eigen::Vector3d centre;
    Eigen::Vector2d measured;
  };

  std::vector<View> views;
  views.reserve(2U * observations.size());
  for (const TrackObservation& observation : observations) {
    Eigen::Index index = 0;
    const Clone* const clone = findClone(observation.clone_id, index);
    if (clone == nullptr) {
      continue;
    }
    for (std::size_t side = 0U; side < 2U; ++side) {
      const Camera& camera = cameras[side];
      views.push_back(
          View{.ned_to_camera = camera.body_to_camera * clone->body_to_ned.transpose(),
               .centre = clone->position + clone->body_to_ned * camera.position_body,
               .measured = side == 0U ? observation.left : observation.right});
    }
  }
  const auto solve =
      [this](const std::span<const View> used) -> std::optional<Eigen::Vector3d> {
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d right_side = Eigen::Vector3d::Zero();
    for (const View& view : used) {
      const Eigen::Vector3d ray =
          (view.ned_to_camera.transpose() *
           Eigen::Vector3d{view.measured.x(), view.measured.y(), 1.0})
              .normalized();
      const Eigen::Matrix3d off_ray =
          Eigen::Matrix3d::Identity() - ray * ray.transpose();
      normal += off_ray;
      right_side += off_ray * view.centre;
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> spectrum{normal};
    if (spectrum.eigenvalues().minCoeff() <
        1.0e-6 * spectrum.eigenvalues().maxCoeff()) {
      return std::nullopt;
    }
    Eigen::Vector3d point = normal.ldlt().solve(right_side);
    for (int iteration = 0; iteration < 5; ++iteration) {
      Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
      for (const View& view : used) {
        const Eigen::Vector3d in_camera = view.ned_to_camera * (point - view.centre);
        if (in_camera.z() < 1.0e-3) {
          return std::nullopt;
        }
        const double inverse_depth = 1.0 / in_camera.z();
        Eigen::Matrix<double, 2, 3> projection;
        projection << inverse_depth, 0.0,
            -in_camera.x() * inverse_depth * inverse_depth, 0.0, inverse_depth,
            -in_camera.y() * inverse_depth * inverse_depth;
        const Eigen::Matrix<double, 2, 3> jacobian = projection * view.ned_to_camera;
        hessian += jacobian.transpose() * jacobian;
        gradient += jacobian.transpose() *
                    (view.measured - in_camera.head<2>() * inverse_depth);
      }
      const Eigen::Vector3d step = hessian.ldlt().solve(gradient);
      point += step;
      if (step.norm() < 1.0e-4) {
        break;
      }
    }
    for (const View& view : used) {
      const double depth = (view.ned_to_camera * (point - view.centre)).z();
      if (!(depth >= config.minimum_depth_m && depth <= config.maximum_depth_m)) {
        return std::nullopt;
      }
    }
    return point.allFinite() ? std::optional<Eigen::Vector3d>{point} : std::nullopt;
  };
  if (views.size() < 4U) {
    return std::nullopt;
  }
  const std::optional<Eigen::Vector3d> from_all = solve(views);
  return from_all.has_value() ? from_all : solve(std::span<const View>{views}.last(2U));
}

// The feature's residuals and pose Jacobian with the point's own error
// projected out; refused when the projected residual fails the gate.
std::optional<FeatureRows> VisualInertialOdometry::Impl::featureRows(
    const std::vector<TrackObservation>& observations, const Eigen::Vector3d& point,
    bool& gated) const {
  gated = false;
  const Eigen::Index rows = 4 * static_cast<Eigen::Index>(observations.size());
  const Eigen::Index size = covariance.rows();
  Eigen::MatrixXd pose_jacobian = Eigen::MatrixXd::Zero(rows, size);
  Eigen::MatrixXd point_jacobian = Eigen::MatrixXd::Zero(rows, 3);
  Eigen::VectorXd residual = Eigen::VectorXd::Zero(rows);
  Eigen::Index row = 0;
  for (const TrackObservation& observation : observations) {
    Eigen::Index index = 0;
    const Clone* const clone = findClone(observation.clone_id, index);
    if (clone == nullptr) {
      return std::nullopt;
    }
    for (std::size_t side = 0U; side < 2U; ++side, row += 2) {
      const Camera& camera = cameras[side];
      const Eigen::Vector2d& measured =
          side == 0U ? observation.left : observation.right;
      // The residual at the current estimate, the Jacobian at the first.
      const Eigen::Vector3d in_camera =
          camera.body_to_camera *
          (clone->body_to_ned.transpose() * (point - clone->position) -
           camera.position_body);
      residual.segment<2>(row) = measured - in_camera.head<2>() / in_camera.z();
      const Eigen::Vector3d first_in_body =
          clone->first_body_to_ned.transpose() * (point - clone->first_position);
      const Eigen::Vector3d first_in_camera =
          camera.body_to_camera * (first_in_body - camera.position_body);
      if (first_in_camera.z() < 1.0e-3) {
        return std::nullopt;
      }
      const double inverse_depth = 1.0 / first_in_camera.z();
      Eigen::Matrix<double, 2, 3> projection;
      projection << inverse_depth, 0.0,
          -first_in_camera.x() * inverse_depth * inverse_depth, 0.0, inverse_depth,
          -first_in_camera.y() * inverse_depth * inverse_depth;
      const Eigen::Matrix<double, 2, 3> to_camera = projection * camera.body_to_camera;
      pose_jacobian.block<2, 3>(row, index + kTheta) = to_camera * skew(first_in_body);
      pose_jacobian.block<2, 3>(row, index + kPosition) =
          -to_camera * clone->first_body_to_ned.transpose();
      point_jacobian.block<2, 3>(row, 0) =
          to_camera * clone->first_body_to_ned.transpose();
    }
  }
  // Left null space of the point Jacobian: the last rows of Q transposed.
  const Eigen::HouseholderQR<Eigen::MatrixXd> decomposition{point_jacobian};
  const Eigen::MatrixXd rotated_jacobian =
      decomposition.householderQ().transpose() * pose_jacobian;
  const Eigen::VectorXd rotated_residual =
      decomposition.householderQ().transpose() * residual;
  FeatureRows projected{.jacobian = rotated_jacobian.bottomRows(rows - 3),
                        .residual = rotated_residual.tail(rows - 3)};
  Eigen::MatrixXd innovation =
      projected.jacobian * covariance * projected.jacobian.transpose();
  innovation.diagonal().array() += config.observation_noise * config.observation_noise;
  const double distance =
      projected.residual.dot(innovation.ldlt().solve(projected.residual));
  if (!(distance < chiSquareQuantile(rows - 3, config.gate_normal_quantile))) {
    gated = true;
    return std::nullopt;
  }
  return projected;
}

void VisualInertialOdometry::Impl::update(const Eigen::MatrixXd& stacked_jacobian,
                                          const Eigen::VectorXd& stacked_residual) {
  Eigen::MatrixXd jacobian = stacked_jacobian;
  Eigen::VectorXd residual = stacked_residual;
  const Eigen::Index size = covariance.rows();
  if (jacobian.rows() > size) {
    // More rows than states: the triangular factor carries all of them.
    const Eigen::HouseholderQR<Eigen::MatrixXd> decomposition{jacobian};
    const Eigen::VectorXd rotated = decomposition.householderQ().transpose() * residual;
    jacobian = decomposition.matrixQR().topRows(size).triangularView<Eigen::Upper>();
    residual = rotated.head(size);
  }
  const Eigen::MatrixXd cross = covariance * jacobian.transpose();
  Eigen::MatrixXd innovation = jacobian * cross;
  innovation.diagonal().array() += config.observation_noise * config.observation_noise;
  const Eigen::LLT<Eigen::MatrixXd> factor{innovation};
  if (factor.info() != Eigen::Success) {
    return;
  }
  const Eigen::MatrixXd gain = factor.solve(cross.transpose()).transpose();
  const Eigen::VectorXd correction = gain * residual;
  covariance -= gain * cross.transpose();
  covariance = 0.5 * (covariance + covariance.transpose()).eval();

  body_to_ned = orthonormalized(body_to_ned * expSo3(correction.segment<3>(kTheta)));
  position += correction.segment<3>(kPosition);
  velocity += correction.segment<3>(kVelocity);
  gyro_bias += correction.segment<3>(kGyroBias);
  accelerometer_bias += correction.segment<3>(kAccelerometerBias);
  for (std::size_t i = 0U; i < clones.size(); ++i) {
    const Eigen::Index index = kImuStates + kCloneStates * static_cast<Eigen::Index>(i);
    clones[i].body_to_ned = orthonormalized(
        clones[i].body_to_ned * expSo3(correction.segment<3>(index + kTheta)));
    clones[i].position += correction.segment<3>(index + kPosition);
  }
}

void VisualInertialOdometry::Impl::marginalizeOldestClone() {
  const std::uint64_t id = clones.front().id;
  clones.pop_front();
  const Eigen::Index size = covariance.rows();
  const Eigen::Index tail = size - kImuStates - kCloneStates;
  Eigen::MatrixXd kept{size - kCloneStates, size - kCloneStates};
  kept.topLeftCorner(kImuStates, kImuStates) =
      covariance.topLeftCorner(kImuStates, kImuStates);
  kept.topRightCorner(kImuStates, tail) = covariance.topRightCorner(kImuStates, tail);
  kept.bottomLeftCorner(tail, kImuStates) =
      covariance.bottomLeftCorner(tail, kImuStates);
  kept.bottomRightCorner(tail, tail) = covariance.bottomRightCorner(tail, tail);
  covariance = std::move(kept);
  for (auto track = tracks.begin(); track != tracks.end();) {
    std::erase_if(track->second, [id](const TrackObservation& observation) {
      return observation.clone_id == id;
    });
    track = track->second.empty() ? tracks.erase(track) : std::next(track);
  }
}

VisualInertialOdometry::VisualInertialOdometry(
    const VisualInertialOdometryConfig& config)
    : impl_{std::make_unique<Impl>(config)} {
}

VisualInertialOdometry::~VisualInertialOdometry() = default;

bool VisualInertialOdometry::initialized() const noexcept {
  return impl_->initialized;
}

void VisualInertialOdometry::initialize(const std::int64_t stamp_ns,
                                        const Eigen::Vector3d& position_ned_m,
                                        const double heading_rad) {
  Impl& state = *impl_;
  // At rest the accelerometer reads the reaction to gravity, up in the body
  // frame: roll and pitch are the rotation that takes it to NED's up.
  Eigen::Vector3d up_body{0.0, 0.0, -1.0};
  if (state.rest_samples > 0U) {
    up_body = (state.rest_accelerometer_sum / static_cast<double>(state.rest_samples))
                  .normalized();
    state.gyro_bias = state.rest_gyro_sum / static_cast<double>(state.rest_samples);
  }
  const Eigen::Matrix3d tilt =
      Eigen::Quaterniond::FromTwoVectors(up_body, Eigen::Vector3d{0.0, 0.0, -1.0})
          .toRotationMatrix();
  const double tilt_heading = std::atan2(tilt(1, 0), tilt(0, 0));
  state.body_to_ned =
      Eigen::AngleAxisd{heading_rad - tilt_heading, Eigen::Vector3d::UnitZ()}
          .toRotationMatrix() *
      tilt;
  state.position = position_ned_m;
  state.velocity.setZero();
  state.accelerometer_bias.setZero();
  state.first_body_to_ned = state.body_to_ned;
  state.first_position = state.position;
  state.first_velocity = state.velocity;
  state.stamp_ns = stamp_ns;
  state.last_update_stamp_ns = 0;
  state.clones.clear();
  state.tracks.clear();

  const VisualInertialOdometryConfig& config = state.config;
  const auto square = [](const double value) { return value * value; };
  state.covariance = Eigen::MatrixXd::Zero(kImuStates, kImuStates);
  // The orientation error lives in the body frame; the heading is the
  // component about NED's down.
  const Eigen::Vector3d down_body =
      state.body_to_ned.transpose() * Eigen::Vector3d::UnitZ();
  state.covariance.block<3, 3>(kTheta, kTheta) =
      square(config.initial_tilt_sigma_rad) *
          (Eigen::Matrix3d::Identity() - down_body * down_body.transpose()) +
      square(config.initial_heading_sigma_rad) * down_body * down_body.transpose();
  state.covariance.block<3, 3>(kPosition, kPosition)
      .diagonal()
      .setConstant(square(config.initial_position_sigma_m));
  state.covariance.block<3, 3>(kVelocity, kVelocity)
      .diagonal()
      .setConstant(square(config.initial_velocity_sigma_mps));
  state.covariance.block<3, 3>(kGyroBias, kGyroBias)
      .diagonal()
      .setConstant(square(config.initial_gyro_bias_sigma_radps));
  state.covariance.block<3, 3>(kAccelerometerBias, kAccelerometerBias)
      .diagonal()
      .setConstant(square(config.initial_accelerometer_bias_sigma_mps2));
  state.initialized = true;
}

void VisualInertialOdometry::addImu(const VisualInertialImuSample& sample) {
  Impl& state = *impl_;
  if (!state.initialized) {
    state.rest_gyro_sum += sample.gyro_radps;
    state.rest_accelerometer_sum += sample.accelerometer_mps2;
    ++state.rest_samples;
    state.last_imu = sample;
    return;
  }
  if (!state.imu.empty() && sample.stamp_ns <= state.imu.back().stamp_ns) {
    return;
  }
  state.imu.push_back(sample);
}

VisualInertialEstimate VisualInertialOdometry::addFrame(
    const std::int64_t stamp_ns,
    const std::vector<StereoFeatureObservation>& observations) {
  Impl& state = *impl_;
  VisualInertialEstimate estimate;
  estimate.stamp_ns = stamp_ns;
  if (!state.initialized || stamp_ns <= state.stamp_ns) {
    return estimate;
  }
  estimate.imu_lag_ns = state.propagateTo(stamp_ns);
  state.augment(stamp_ns);
  const std::uint64_t clone_id = state.clones.back().id;
  for (const StereoFeatureObservation& observation : observations) {
    state.tracks[observation.id].push_back(TrackObservation{
        .clone_id = clone_id, .left = observation.left, .right = observation.right});
  }

  // A feature is used when its track has ended, or when it has been seen
  // from the pose that is about to leave the window; either way all of its
  // observations go into one update and the track is closed.
  const bool window_full = state.clones.size() > state.config.maximum_clones;
  const std::uint64_t oldest_id = state.clones.front().id;
  std::vector<std::vector<TrackObservation>> closed;
  for (auto track = state.tracks.begin(); track != state.tracks.end();) {
    const bool ended = track->second.back().clone_id != clone_id;
    const bool leaving = window_full && track->second.front().clone_id == oldest_id;
    if (ended || leaving) {
      if (track->second.size() >= state.config.minimum_track_frames) {
        closed.push_back(std::move(track->second));
      }
      track = state.tracks.erase(track);
    } else {
      ++track;
    }
  }
  std::sort(closed.begin(), closed.end(),
            [](const auto& a, const auto& b) { return a.size() > b.size(); });
  if (closed.size() > state.config.maximum_features_per_update) {
    closed.resize(state.config.maximum_features_per_update);
  }
  estimate.candidate_features = closed.size();

  std::vector<FeatureRows> accepted;
  Eigen::Index total_rows = 0;
  for (const std::vector<TrackObservation>& track : closed) {
    const std::optional<Eigen::Vector3d> point = state.triangulate(track);
    if (!point.has_value()) {
      ++estimate.untriangulated_features;
      continue;
    }
    bool gated = false;
    std::optional<FeatureRows> rows = state.featureRows(track, *point, gated);
    if (!rows.has_value()) {
      estimate.gated_features += gated ? 1U : 0U;
      estimate.untriangulated_features += gated ? 0U : 1U;
      continue;
    }
    total_rows += rows->residual.size();
    accepted.push_back(std::move(*rows));
  }
  estimate.used_features = accepted.size();
  if (total_rows > 0) {
    Eigen::MatrixXd jacobian{total_rows, state.covariance.rows()};
    Eigen::VectorXd residual{total_rows};
    Eigen::Index row = 0;
    for (const FeatureRows& rows : accepted) {
      jacobian.middleRows(row, rows.residual.size()) = rows.jacobian;
      residual.segment(row, rows.residual.size()) = rows.residual;
      row += rows.residual.size();
    }
    estimate.residual_rms_sigma =
        std::sqrt(residual.squaredNorm() / static_cast<double>(total_rows)) /
        state.config.observation_noise;
    state.update(jacobian, residual);
    state.last_update_stamp_ns = stamp_ns;
  }
  estimate.healthy =
      state.last_update_stamp_ns > 0 &&
      static_cast<double>(stamp_ns - state.last_update_stamp_ns) * 1.0e-9 <=
          state.config.maximum_unaided_s &&
      static_cast<double>(estimate.imu_lag_ns) * 1.0e-9 <=
          state.config.maximum_imu_period_s;
  if (window_full) {
    state.marginalizeOldestClone();
  }

  estimate.position_ned_m = state.position;
  estimate.body_to_ned = Eigen::Quaterniond{state.body_to_ned};
  estimate.velocity_ned_mps = state.velocity;
  estimate.gyro_bias_radps = state.gyro_bias;
  estimate.accelerometer_bias_mps2 = state.accelerometer_bias;
  estimate.position_variance_m2 =
      state.covariance.block<3, 3>(kPosition, kPosition).diagonal();
  // The orientation error is carried in the body frame; reported in NED,
  // where the third component is the heading.
  estimate.orientation_variance_rad2 =
      (state.body_to_ned * state.covariance.block<3, 3>(kTheta, kTheta) *
       state.body_to_ned.transpose())
          .diagonal();
  estimate.velocity_variance_m2ps2 =
      state.covariance.block<3, 3>(kVelocity, kVelocity).diagonal();
  estimate.weakest_velocity_sigma_mps =
      std::sqrt(std::max(0.0,
                         Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>{
                             state.covariance.block<3, 3>(kVelocity, kVelocity)}
                             .eigenvalues()
                             .maxCoeff()));
  estimate.clones = state.clones.size();
  estimate.tracked_features = state.tracks.size();
  return estimate;
}

} // namespace drone_city_nav
