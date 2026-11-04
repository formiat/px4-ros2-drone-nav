#include "drone_city_nav/visual_inertial_odometry.hpp"

#include "drone_city_nav/visual_inertial_void.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <span>
#include <unordered_map>
#include <utility>

#include "visual_inertial_odometry_impl.hpp"

namespace drone_city_nav {

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
      // Two samples further apart than the unaided timeout bound a hole,
      // however short the steps that cross it: the node holds its frames
      // until the stream covers them and then feeds them a tenth of a second
      // apart, so no single step was ever longer than a second and a 2 s
      // hole was crossed on the average of its two ends, published healthy
      // and flown into the staging area four seconds later (r1258, r1260).
      if (static_cast<double>(to.stamp_ns - from.stamp_ns) * 1.0e-9 >
          config.maximum_unaided_s) {
        lost = true;
      }
    } else {
      // The lag is the stream's silence since its last sample, not since the
      // state's last step: measured from the state, a hole the frames kept
      // stepping across read as one frame period (replayed with the frames
      // flowing through a 4 s hole, the filter held its last sample to 218 m
      // of error without ever calling itself lost). Specification K28: a
      // stream silent longer than the unaided timeout is a hole whether or
      // not frames keep coming.
      imu_lag_ns = std::max<std::int64_t>(0, target_ns - from.stamp_ns);
      if (static_cast<double>(imu_lag_ns) * 1.0e-9 > config.maximum_unaided_s) {
        lost = true;
      }
    }
    const double dt = static_cast<double>(end_ns - stamp_ns) * 1.0e-9;
    if (dt > config.maximum_unaided_s) {
      lost = true;
    }
    if (dt > 0.0) {
      propagateStep(0.5 * (from.gyro_radps + to.gyro_radps),
                    0.5 * (from.accelerometer_mps2 + to.accelerometer_mps2), dt,
                    transition, noise);
      drag_reading_sum_mps +=
          0.5 * dt * (from.accelerometer_mps2 + to.accelerometer_mps2).head<2>();
      drag_reading_s += dt;
      // Samples were lost when the two that bracket the step are further
      // apart than the stream's period, not when the step itself is long:
      // the first step after the initialisation runs from the frame's stamp
      // to the sample after the last one read at rest, the frame's latency
      // and more, with no sample missing. Read as a hole, that step gave the
      // heading, which no feature observes, 9 degrees of uncertainty for the
      // whole flight, and every update walked it (r1670, r1673, r1674,
      // r1681: 2.3 to 3.5 m from B in truth; r1681's heading sigma 9.5
      // degrees at the first healthy frame, 7 at the end).
      if (static_cast<double>(to.stamp_ns - from.stamp_ns) * 1.0e-9 >
          config.maximum_imu_period_s) {
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

// Past the unaided timeout, an estimate otherwise sound is dead reckoning
// for `maximum_dead_reckoning_s` more (the ladder's second rung).
void VisualInertialOdometry::Impl::judgeDeadReckoning(
    VisualInertialEstimate& estimate) const {
  estimate.unaided_s =
      last_update_stamp_ns > 0
          ? static_cast<double>(estimate.stamp_ns - last_update_stamp_ns) * 1.0e-9
          : std::numeric_limits<double>::infinity();
  // The void's time (K46) is not the budget's: the frames were empty, not
  // blind.
  const bool sound =
      !estimate.healthy && !lost && last_update_stamp_ns > 0 &&
      static_cast<double>(estimate.imu_lag_ns) * 1.0e-9 <=
          config.maximum_imu_period_s &&
      estimate.weakest_velocity_sigma_mps <= config.maximum_velocity_sigma_mps;
  estimate.in_void = sound && in_void.inVoid();
  estimate.dead_reckoning =
      sound && !in_void.inVoid() &&
      estimate.unaided_s - in_void.seconds() <=
          config.maximum_unaided_s + config.maximum_dead_reckoning_s;
}

void VisualInertialOdometry::observeVoid(const std::int64_t stamp_ns,
                                         const bool void_frame) {
  impl_->in_void.observe(stamp_ns, void_frame);
}

VisualInertialOdometry::VisualInertialOdometry(
    const VisualInertialOdometryConfig& config)
    : impl_{std::make_unique<Impl>(config)} {
}

VisualInertialOdometry::~VisualInertialOdometry() = default;

bool VisualInertialOdometry::initialized() const noexcept {
  return impl_->initialized;
}

bool VisualInertialOdometry::lost() const noexcept {
  return impl_->initialized && impl_->lost;
}

void VisualInertialOdometry::reinitialize(const std::int64_t stamp_ns,
                                          const Eigen::Vector3d& position_ned_m,
                                          const Eigen::Vector3d& velocity_ned_mps,
                                          const Eigen::Quaterniond& body_to_ned) {
  Impl& state = *impl_;
  if (!state.initialized) {
    return;
  }
  // The autopilot's pose after its dead reckoning: a few decimetres and a
  // few degrees of tilt at most; the velocity it carried through the hole on
  // its own IMU, half a metre a second of it, which the first frames' tracks
  // correct (started with none at 1.5 m/s the filter gated its features and
  // never caught up). The heading is the autopilot's, held as the start
  // holds it: features do not observe it, and with a sigma of degrees the
  // first updates turned it by 4.8 degrees and the flight drifted 0.1 m/s
  // (r1299).
  constexpr double kPositionSigmaM{0.3};
  constexpr double kTiltSigmaRad{0.05};
  constexpr double kVelocitySigmaMps{0.5};
  const auto square = [](const double value) { return value * value; };
  state.body_to_ned = body_to_ned.normalized().toRotationMatrix();
  state.position = position_ned_m;
  state.velocity = velocity_ned_mps;
  state.first_body_to_ned = state.body_to_ned;
  state.first_position = state.position;
  state.first_velocity = state.velocity;
  state.stamp_ns = stamp_ns;
  // The autopilot's pose is the restart's fix: the estimate is healthy from
  // it for the unaided second, as after any update, and the autopilot has it
  // again before its own dead reckoning runs out (r1306: half a second
  // waiting for the features' first update after a 4 s hole and it had
  // given its position up).
  state.last_update_stamp_ns = stamp_ns;
  state.held_height_ned_m = position_ned_m.z();
  state.lost = false;
  state.clones.clear();
  state.tracks.clear();
  while (!state.imu.empty() && state.imu.front().stamp_ns <= stamp_ns) {
    state.last_imu = state.imu.front();
    state.imu.pop_front();
  }
  // The restart's moment has its own sample, the nearest reading known: the
  // autopilot's attitude may be a few milliseconds older than the stream's
  // first sample after the hole, and the sample from before the hole then
  // bracketed the hole again at the first step and the filter was lost at
  // once (r1309).
  std::optional<VisualInertialImuSample> sample =
      state.imu.empty() ? state.last_imu
                        : std::optional<VisualInertialImuSample>{state.imu.front()};
  if (sample.has_value()) {
    sample->stamp_ns = stamp_ns;
    state.last_imu = sample;
  }
  state.held_barometric_height_m = state.barometric_height_m;
  state.barometric_zero_m =
      state.barometric_height_m.has_value()
          ? std::optional<double>{*state.barometric_height_m + state.position.z()}
          : std::nullopt;
  state.drag_reading_sum_mps.setZero();
  state.drag_reading_s = 0.0;
  // The biases' block sits after the IMU states, before the clones' blocks.
  const Eigen::MatrixXd biases = state.covariance.block<6, 6>(kGyroBias, kGyroBias);
  state.covariance = Eigen::MatrixXd::Zero(kImuStates, kImuStates);
  const Eigen::Vector3d down_body =
      state.body_to_ned.transpose() * Eigen::Vector3d::UnitZ();
  state.covariance.block<3, 3>(kTheta, kTheta) =
      square(kTiltSigmaRad) *
          (Eigen::Matrix3d::Identity() - down_body * down_body.transpose()) +
      square(state.config.initial_heading_sigma_rad) * down_body *
          down_body.transpose();
  state.covariance.block<3, 3>(kPosition, kPosition)
      .diagonal()
      .setConstant(square(kPositionSigmaM));
  state.covariance.block<3, 3>(kVelocity, kVelocity)
      .diagonal()
      .setConstant(square(kVelocitySigmaMps));
  state.covariance.block<6, 6>(kGyroBias, kGyroBias) = biases;
}

void VisualInertialOdometry::initialize(const std::int64_t stamp_ns,
                                        const Eigen::Vector3d& position_ned_m,
                                        const double heading_rad) {
  Impl& state = *impl_;
  // At rest the accelerometer reads the reaction to gravity, up in the body
  // frame: roll and pitch are the rotation that takes it to NED's up.
  Eigen::Vector3d up_body{0.0, 0.0, -1.0};
  for (const VisualInertialImuSample& sample : state.rest_imu) {
    if (sample.stamp_ns > stamp_ns) {
      state.imu.push_back(sample);
      continue;
    }
    state.rest_gyro_sum += sample.gyro_radps;
    state.rest_gyro_square_sum += sample.gyro_radps.cwiseAbs2();
    state.rest_accelerometer_sum += sample.accelerometer_mps2;
    ++state.rest_samples;
  }
  state.rest_imu.clear();
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
  state.lost = false;
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
  // No sample at rest leaves the bias unknown to a consumer gyroscope's.
  double gyro_bias_sigma = 5.0e-3;
  if (state.rest_samples > 1U) {
    const double count = static_cast<double>(state.rest_samples);
    const Eigen::Vector3d variance =
        (state.rest_gyro_square_sum / count - (state.rest_gyro_sum / count).cwiseAbs2())
            .cwiseMax(0.0);
    gyro_bias_sigma = std::max(config.minimum_gyro_bias_sigma_radps,
                               std::sqrt(variance.maxCoeff() / count));
  }
  state.covariance.block<3, 3>(kGyroBias, kGyroBias)
      .diagonal()
      .setConstant(square(gyro_bias_sigma));
  state.covariance.block<3, 3>(kAccelerometerBias, kAccelerometerBias)
      .diagonal()
      .setConstant(square(config.initial_accelerometer_bias_sigma_mps2));
  state.barometric_zero_m =
      state.barometric_height_m.has_value()
          ? std::optional<double>{*state.barometric_height_m + state.position.z()}
          : std::nullopt;
  state.initialized = true;
}

void VisualInertialOdometry::addBarometricHeight(const double height_m) {
  if (std::isfinite(height_m)) {
    impl_->barometric_height_m = height_m;
  }
}

void VisualInertialOdometry::addMagneticField(const std::int64_t stamp_ns,
                                              const Eigen::Vector3d& field_body) {
  constexpr std::int64_t kKeptNs{1'000'000'000};
  std::deque<MagneticReading>& readings = impl_->magnetic_readings;
  if (!field_body.allFinite() || !(field_body.norm() > 0.0) ||
      (!readings.empty() && stamp_ns <= readings.back().stamp_ns)) {
    return;
  }
  readings.push_back(MagneticReading{.stamp_ns = stamp_ns, .field_body = field_body});
  while (readings.front().stamp_ns < stamp_ns - kKeptNs) {
    readings.pop_front();
  }
}

void VisualInertialOdometry::addImu(const VisualInertialImuSample& sample) {
  Impl& state = *impl_;
  if (!state.initialized) {
    if (state.rest_imu.empty() || sample.stamp_ns > state.rest_imu.back().stamp_ns) {
      state.rest_imu.push_back(sample);
    }
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
  const std::int64_t previous_frame_stamp_ns = state.stamp_ns;
  estimate.imu_lag_ns = state.propagateTo(stamp_ns);
  state.augment(stamp_ns);
  const std::uint64_t clone_id = state.clones.back().id;
  // The void (K46), kept by VisualInertialVoid.
  state.in_void.frame(stamp_ns, previous_frame_stamp_ns, state.last_update_stamp_ns,
                      observations.size() < state.config.minimum_frame_features);
  if (observations.size() >= state.config.minimum_frame_features) {
    for (const StereoFeatureObservation& observation : observations) {
      state.tracks[observation.id].push_back(TrackObservation{
          .clone_id = clone_id, .left = observation.left, .right = observation.right});
    }
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
    state.held_height_ned_m = state.position.z();
    state.held_barometric_height_m = state.barometric_height_m;
    state.in_void.updated();
  } else if (state.last_update_stamp_ns > 0 && !state.lost &&
             static_cast<double>(stamp_ns - state.last_update_stamp_ns) * 1.0e-9 >
                 state.config.maximum_unaided_s) {
    state.holdHeight();
  }
  state.magnetic_innovation_rad = std::numeric_limits<double>::quiet_NaN();
  if (!state.lost) {
    state.barometricHeightPrior();
    state.magneticHeadingPrior();
  }
  estimate.magnetic_innovation_rad = state.magnetic_innovation_rad;
  estimate.magnetic_reading_age_ns = state.magnetic_reading_age_ns;
  state.fuseRotorDrag();
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
  estimate.accelerometer_bias_variance_m2ps4 =
      state.covariance.block<3, 3>(kAccelerometerBias, kAccelerometerBias).diagonal();
  estimate.weakest_velocity_sigma_mps =
      std::sqrt(std::max(0.0,
                         Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>{
                             state.covariance.block<3, 3>(kVelocity, kVelocity)}
                             .eigenvalues()
                             .maxCoeff()));
  estimate.healthy =
      estimate.healthy && !state.lost &&
      estimate.weakest_velocity_sigma_mps <= state.config.maximum_velocity_sigma_mps;
  state.judgeDeadReckoning(estimate);
  // Dead reckoning spent is as lost as an IMU hole: the state is what the IMU
  // alone made of it. r820 lay landed in the dark with its unpublished state
  // drifting 35 m, came back healthy on seven features when the light
  // returned for a second, and the autopilot reset onto it and flew the
  // vehicle into the platform it lay on (specification K13).
  if (state.last_update_stamp_ns > 0 && !estimate.healthy && !estimate.dead_reckoning &&
      !estimate.in_void &&
      estimate.unaided_s - state.in_void.seconds() >
          state.config.maximum_unaided_s + state.config.maximum_dead_reckoning_s) {
    state.lost = true;
  }
  estimate.lost = state.lost;
  estimate.clones = state.clones.size();
  estimate.tracked_features = state.tracks.size();
  state.last_estimate = estimate;
  return estimate;
}

VisualInertialEstimate
VisualInertialOdometry::predicted(const std::int64_t stamp_ns) const {
  const Impl& state = *impl_;
  VisualInertialEstimate estimate = state.last_estimate;
  if (!state.initialized || stamp_ns <= state.stamp_ns || !state.last_imu.has_value()) {
    return estimate;
  }
  Eigen::Matrix3d body_to_ned = state.body_to_ned;
  Eigen::Vector3d position = state.position;
  Eigen::Vector3d velocity = state.velocity;
  std::int64_t at_ns = state.stamp_ns;
  VisualInertialImuSample from = *state.last_imu;
  const auto advance = [&](const VisualInertialImuSample& to,
                           const std::int64_t end_ns) {
    const double dt = static_cast<double>(end_ns - at_ns) * 1.0e-9;
    if (dt <= 0.0) {
      return;
    }
    const Eigen::Matrix3d half_turn =
        expSo3(0.5 * dt * (0.5 * (from.gyro_radps + to.gyro_radps) - state.gyro_bias));
    const Eigen::Vector3d acceleration =
        body_to_ned * half_turn *
            (0.5 * (from.accelerometer_mps2 + to.accelerometer_mps2) -
             state.accelerometer_bias) +
        state.gravity;
    position += velocity * dt + 0.5 * acceleration * dt * dt;
    velocity += acceleration * dt;
    body_to_ned = body_to_ned * half_turn * half_turn;
    at_ns = end_ns;
  };
  // Specification K28: a prediction across a hole of the stream is none. At
  // the hole's end the stream resumes before the frames held through it reach
  // the filter, and the pose predicted across it on the average of its two
  // ends went to the autopilot as dead reckoning, which moved the autopilot's
  // own estimate by 3.7 m (r1292).
  const auto unaided_ns =
      static_cast<std::int64_t>(state.config.maximum_unaided_s * 1.0e9);
  bool across_hole = false;
  for (const VisualInertialImuSample& sample : state.imu) {
    if (sample.stamp_ns > stamp_ns) {
      break;
    }
    across_hole = across_hole || sample.stamp_ns - from.stamp_ns > unaided_ns;
    advance(sample, sample.stamp_ns);
    from = sample;
  }
  across_hole = across_hole || stamp_ns - from.stamp_ns > unaided_ns;
  estimate.imu_lag_ns = stamp_ns - at_ns;
  advance(from, stamp_ns);
  estimate.stamp_ns = stamp_ns;
  estimate.position_ned_m = position;
  estimate.velocity_ned_mps = velocity;
  estimate.body_to_ned = Eigen::Quaterniond{orthonormalized(body_to_ned)};
  estimate.healthy =
      estimate.healthy &&
      static_cast<double>(stamp_ns - state.last_update_stamp_ns) * 1.0e-9 <=
          state.config.maximum_unaided_s &&
      static_cast<double>(estimate.imu_lag_ns) * 1.0e-9 <=
          state.config.maximum_imu_period_s;
  state.judgeDeadReckoning(estimate);
  if (across_hole) {
    estimate.healthy = false;
    estimate.dead_reckoning = false;
    estimate.lost = true;
  }
  return estimate;
}

std::optional<VisualInertialClonePose>
VisualInertialOdometry::clonePose(const std::int64_t stamp_ns) const {
  for (const Clone& clone : impl_->clones) {
    if (clone.stamp_ns == stamp_ns) {
      return VisualInertialClonePose{
          .position_ned_m = clone.position,
          .body_to_ned = Eigen::Quaterniond{clone.body_to_ned}.normalized()};
    }
  }
  return std::nullopt;
}

} // namespace drone_city_nav
