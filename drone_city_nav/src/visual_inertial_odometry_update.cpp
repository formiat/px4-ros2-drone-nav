#include "drone_city_nav/visual_inertial_odometry.hpp"
#include "drone_city_nav/visual_inertial_void.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <numbers>
#include <optional>
#include <span>
#include <unordered_map>
#include <utility>

#include "visual_inertial_odometry_impl.hpp"

namespace drone_city_nav {

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
      const Camera& camera = cameras.at(side);
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
      if (!std::isfinite(depth) || depth < config.minimum_depth_m ||
          depth > config.maximum_depth_m) {
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
      const Camera& camera = cameras.at(side);
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

// Past the unaided timeout the filter's height is the one the features last
// gave, moved by what the barometer saw since. The IMU alone drifted the
// height 2 to 8 m in 5 to 10 s (the recorded dark flight r779 replayed with
// its frames blanked), and the estimate that came out of a dark stretch
// jumped by metres when the features returned (r792); a height merely held
// still was not the vehicle's, which sank 1.4 m in 5 s of a hold, and the
// filter, told it stood still, bent its attitude and drifted 1.7 m sideways
// (r793). The barometer's 3 Pa of noise is 0.25 m, less once smoothed.
void VisualInertialOdometry::Impl::holdHeight() {
  constexpr double kHeldHeightSigmaM{0.3};
  const double weight = config.observation_noise / kHeldHeightSigmaM;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(1, covariance.rows());
  jacobian(0, kPosition + 2) = weight;
  Eigen::VectorXd residual{1};
  const double climbed_m =
      held_barometric_height_m.has_value() && barometric_height_m.has_value()
          ? *barometric_height_m - *held_barometric_height_m
          : 0.0;
  residual(0) = weight * (held_height_ned_m - climbed_m - position.z());
  update(jacobian, residual);
}

// The barometer's height since the declared pose as a prior on the filter's
// height, on every frame. The features measure the height against what they
// triangulated from the same drifting pose, so under the imperfections
// profile (the accelerometer's bias walk) the height drifted 1.2 m in 250 s
// and the vehicle climbed into an eave it held 0.8 m lower (r2721); the
// lidar's registration carries the same prior (K45). The sigma is the
// smoothed barometer's noise with room for the pressure's own drift.
void VisualInertialOdometry::Impl::barometricHeightPrior() {
  constexpr double kBarometricHeightSigmaM{0.5};
  if (!barometric_height_m.has_value()) {
    return;
  }
  if (!barometric_zero_m.has_value()) {
    barometric_zero_m = *barometric_height_m + position.z();
    return;
  }
  const double weight = config.observation_noise / kBarometricHeightSigmaM;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(1, covariance.rows());
  jacobian(0, kPosition + 2) = weight;
  Eigen::VectorXd residual{1};
  residual(0) = weight * (*barometric_zero_m - *barometric_height_m - position.z());
  update(jacobian, residual);
}

// The magnetometer's heading as a prior on the filter's yaw, on every frame:
// no feature observes the heading, and under the imperfections profile it
// drifted 0.2 to 2.3 degrees a flight, the frame turned 1.15 degrees and the
// goal missed by 2.07 m (r2842). The simulated sensor holds the field's
// horizontal direction in the world to 0.03 degrees (r2842, r2843: the
// truth's attitude interpolated to each reading, tools/magnetic_heading.py);
// the real vehicle's class of sensor, the IIS2MDC PX4's model is noted after,
// reads 3 mG RMS, 0.8 degrees across the 0.22 G of the horizontal field, and
// the filter's own tilt enters the heading twice over at the field's dip of
// 63 degrees: the sigma of a degree covers both. A heavier prior follows the
// tilt's error rather than holding the heading: at half a degree the heading
// ran to +2.9 degrees with the innovation at zero and the goal was missed by
// 2.14 m (r2850); at a degree the goal is met 0.73 to 1.00 m away under the
// profile (r2843, r2845, r2847, r2849). The field is taken at
// the frame's stamp: the reading nearest it, turned by the gyroscope through the time
// between (r2843 took the newest reading, whatever its age, and the heading wandered
// -0.69 to +0.28 degrees; r2844 waited for readings either side of the frame, and in
// lockstep the pair is processed before the magnetometer's next reading arrives:
// nothing was fused). The zero is the mean direction over the frames while the vehicle
// rests where it was declared, fixed when it first moves: a single reading's error
// stayed in the zero for the whole flight (r2847: the IIS2MDC's 0.8 degrees of
// noise a reading, the heading +1.69 degrees at the end). A reading more than
// 20 degrees off the zero is a disturbed field (iron near the sensor), left
// out reading by reading, never latched.
void VisualInertialOdometry::Impl::magneticHeadingPrior() {
  constexpr double kHeadingSigmaRad{1.0 * std::numbers::pi / 180.0};
  constexpr double kGateRad{20.0 * std::numbers::pi / 180.0};
  constexpr std::int64_t kNearestNs{50'000'000};
  if (!config.magnetic_heading) {
    return;
  }
  const auto nearest = std::ranges::min_element(
      magnetic_readings, {}, [this](const MagneticReading& reading) {
        return reading.stamp_ns > stamp_ns ? reading.stamp_ns - stamp_ns
                                           : stamp_ns - reading.stamp_ns;
      });
  if (nearest == magnetic_readings.end()) {
    return;
  }
  magnetic_reading_age_ns = stamp_ns - nearest->stamp_ns;
  if (std::abs(magnetic_reading_age_ns) > kNearestNs) {
    return;
  }
  // A field fixed in the world turns in the body against the body's rate.
  const Eigen::Vector3d rate = last_imu.has_value()
                                   ? Eigen::Vector3d{last_imu->gyro_radps - gyro_bias}
                                   : Eigen::Vector3d::Zero();
  const Eigen::Vector3d world =
      body_to_ned *
      (expSo3(-rate * (static_cast<double>(magnetic_reading_age_ns) * 1.0e-9)) *
       nearest->field_body);
  if (!(world.head<2>().squaredNorm() > 0.0)) {
    return;
  }
  if (!magnetic_zero_rad.has_value()) {
    constexpr double kRestM{0.3};
    constexpr double kRestMps{0.2};
    if (!magnetic_rest_position.has_value()) {
      magnetic_rest_position = position;
    }
    const Eigen::Vector2d direction = world.head<2>().normalized();
    if ((position - *magnetic_rest_position).norm() < kRestM &&
        velocity.norm() < kRestMps) {
      magnetic_zero_sum += direction;
      return;
    }
    if (!(magnetic_zero_sum.squaredNorm() > 0.0)) {
      magnetic_zero_sum = direction;
    }
    magnetic_zero_rad = std::atan2(magnetic_zero_sum.y(), magnetic_zero_sum.x());
  }
  const double heading = std::atan2(world.y(), world.x());
  const double innovation =
      std::remainder(*magnetic_zero_rad - heading, 2.0 * std::numbers::pi);
  magnetic_innovation_rad = innovation;
  if (std::abs(innovation) > kGateRad) {
    return;
  }
  // A heading, not a field: the turn about the world's vertical alone (the
  // correction is applied as body_to_ned * exp(theta), so that turn is the
  // vertical's row of body_to_ned). The field dips 64 degrees, and its full
  // Jacobian weighs the tilt twice the heading: the filter, sure of its
  // heading and less of its tilt, took the reading for a tilt and left the
  // heading to drift (4.2 degrees of 5.6 in the test). The tilt is the
  // gravity's to hold.
  const double weight = config.observation_noise / kHeadingSigmaRad;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(1, covariance.rows());
  jacobian.block<1, 3>(0, kTheta) = weight * body_to_ned.row(2);
  Eigen::VectorXd residual{1};
  residual(0) = weight * innovation;
  update(jacobian, residual);
}

// The mean reading across the rotor axis since the last frame against the
// drag of the body's velocity there and the accelerometer's bias. The model
// is the hover's: the rotors' speed moves the coefficient with the thrust,
// which the noise covers (0.036 m/s^2 of residual over the recorded flights).
// A reading beyond what the drag of any flight can be is not drag: the
// ground holding a body landed on a slope (r1194, tilted 12 degrees, read
// 2.3 m/s^2 of the ground's friction for as long as it lay there, which the
// model took for 21 m/s and ran the estimate 100 m away), or a collision.
// Such a reading is left out; the drag of the fastest flight (5 m/s) is 0.53.
void VisualInertialOdometry::Impl::fuseRotorDrag() {
  constexpr double kDragReadingSigmaMps2{0.05};
  constexpr double kMaximumDragReadingMps2{0.5};
  if (!(config.rotor_drag_1ps > 0.0) || !(drag_reading_s > 0.0)) {
    return;
  }
  const Eigen::Vector2d reading = drag_reading_sum_mps / drag_reading_s;
  drag_reading_sum_mps.setZero();
  drag_reading_s = 0.0;
  if ((reading - accelerometer_bias.head<2>()).norm() > kMaximumDragReadingMps2) {
    return;
  }
  const Eigen::Vector3d body_velocity = body_to_ned.transpose() * velocity;
  const double weight = config.observation_noise / kDragReadingSigmaMps2;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(2, covariance.rows());
  jacobian.block<2, 3>(0, kVelocity) =
      -weight * config.rotor_drag_1ps * body_to_ned.transpose().topRows<2>();
  jacobian.block<2, 3>(0, kTheta) =
      -weight * config.rotor_drag_1ps * skew(body_velocity).topRows<2>();
  jacobian.block<2, 3>(0, kAccelerometerBias) =
      weight * Eigen::Matrix3d::Identity().topRows<2>();
  const Eigen::VectorXd residual =
      weight * (reading + config.rotor_drag_1ps * body_velocity.head<2>() -
                accelerometer_bias.head<2>());
  update(jacobian, residual);
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

} // namespace drone_city_nav
