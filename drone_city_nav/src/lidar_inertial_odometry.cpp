#include "drone_city_nav/lidar_inertial_odometry.hpp"

#include "drone_city_nav/point_plane_map_3d.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {

namespace {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

// The measurement variance of an axis the registration could not observe.
constexpr double kUnobservedVarianceM2{1.0e6};

struct CellKey {
  std::int32_t x{0};
  std::int32_t y{0};
  std::int32_t z{0};
  [[nodiscard]] bool operator==(const CellKey&) const noexcept = default;
};

struct CellKeyHash {
  [[nodiscard]] std::size_t operator()(const CellKey& key) const noexcept {
    const auto x = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.x));
    const auto y = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.y));
    const auto z = static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.z));
    return static_cast<std::size_t>(x * 73856093ULL ^ y * 19349663ULL ^
                                    z * 83492791ULL);
  }
};

[[nodiscard]] CellKey cellOf(const Eigen::Vector3d& point,
                             const double size_m) noexcept {
  return CellKey{static_cast<std::int32_t>(std::floor(point.x() / size_m)),
                 static_cast<std::int32_t>(std::floor(point.y() / size_m)),
                 static_cast<std::int32_t>(std::floor(point.z() / size_m))};
}

// The rotation of a small angle vector.
[[nodiscard]] Eigen::Quaterniond expSmallAngle(const Eigen::Vector3d& theta) noexcept {
  const double angle = theta.norm();
  if (angle < 1.0e-12) {
    return Eigen::Quaterniond{1.0, 0.5 * theta.x(), 0.5 * theta.y(), 0.5 * theta.z()}
        .normalized();
  }
  return Eigen::Quaterniond{Eigen::AngleAxisd{angle, theta / angle}};
}

[[nodiscard]] Eigen::Vector3d logRotation(const Eigen::Quaterniond& q) noexcept {
  const Eigen::AngleAxisd axis{q.normalized()};
  double angle = axis.angle();
  if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  return angle * axis.axis();
}

struct Keyframe {
  std::uint64_t id{0U};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  std::vector<Eigen::Vector3d> points_world;
};

// A sliding window of keyframes over the point-plane map: the oldest
// keyframe leaves first and takes its own points with it.
class Submap {
public:
  explicit Submap(const LidarInertialOdometryConfig& config)
      : config_(config),
        map_(PointPlaneMapConfig3D{.cell_m = config.scan_voxel_m,
                                   .maximum_points_per_cell =
                                       config.maximum_points_per_cell}) {
  }

  [[nodiscard]] bool empty() const noexcept {
    return map_.empty();
  }

  [[nodiscard]] std::size_t pointCount() const noexcept {
    return map_.pointCount();
  }

  [[nodiscard]] std::size_t keyframeCount() const noexcept {
    return keyframes_.size();
  }

  [[nodiscard]] PointPlaneMap3D& map() noexcept {
    return map_;
  }

  void insert(Keyframe keyframe) {
    // The oldest keyframe leaves first, so the cells it frees are refilled by
    // the keyframe arriving now rather than standing empty until the next.
    while (!keyframes_.empty() && keyframes_.size() >= config_.maximum_keyframes) {
      const Keyframe oldest = std::move(keyframes_.front());
      keyframes_.pop_front();
      map_.remove(oldest.points_world, oldest.id);
    }
    keyframe.id = ++last_keyframe_id_;
    map_.insert(keyframe.points_world, keyframe.id, 0);
    keyframes_.push_back(std::move(keyframe));
  }

private:
  const LidarInertialOdometryConfig& config_;
  PointPlaneMap3D map_;
  std::deque<Keyframe> keyframes_;
  std::uint64_t last_keyframe_id_{0U};
};

// One point per cell, the cell's centroid, within the range band.
[[nodiscard]] std::vector<Eigen::Vector3d>
thinScan(const std::vector<Eigen::Vector3d>& points,
         const LidarInertialOdometryConfig& config) {
  struct Accumulator {
    Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
    std::size_t count{0U};
  };

  std::unordered_map<CellKey, Accumulator, CellKeyHash> cells;
  const double minimum = config.minimum_range_m * config.minimum_range_m;
  const double maximum = config.maximum_range_m * config.maximum_range_m;
  for (const Eigen::Vector3d& point : points) {
    if (!point.allFinite()) {
      continue;
    }
    const double range = point.squaredNorm();
    if (range < minimum || range > maximum) {
      continue;
    }
    Accumulator& accumulator = cells[cellOf(point, config.scan_voxel_m)];
    accumulator.sum += point;
    ++accumulator.count;
  }
  std::vector<Eigen::Vector3d> thinned;
  thinned.reserve(cells.size());
  for (const auto& [key, accumulator] : cells) {
    thinned.push_back(accumulator.sum / static_cast<double>(accumulator.count));
  }
  return thinned;
}

// The registration of the thinned scan against the submap from the prior
// pose. The correspondence search reads the submap's own configured distance.
[[nodiscard]] PointPlaneRegistration3D
registerScan(Submap& submap, const std::vector<Eigen::Vector3d>& points_body,
             const Eigen::Vector3d& prior_position,
             const Eigen::Quaterniond& prior_rotation,
             const LidarInertialOdometryConfig& config, const double correspondence_m) {
  return registerPointsToPlanes(
      submap.map(), points_body, prior_position, prior_rotation,
      PointPlaneRegistrationConfig3D{
          .maximum_correspondence_m = correspondence_m,
          .maximum_iterations = config.maximum_iterations,
          .convergence_translation_m = config.convergence_translation_m,
          .convergence_rotation_rad = config.convergence_rotation_rad,
          .robust_width_m = config.robust_width_m,
          .born_by_ns = std::numeric_limits<std::int64_t>::max(),
      });
}

} // namespace

struct LidarInertialOdometry::Impl {
  explicit Impl(const LidarInertialOdometryConfig& config_in)
      : config(config_in),
        submap(config) {
  }

  LidarInertialOdometryConfig config;
  Submap submap;
  bool initialized{false};
  bool attitude_levelled{false};
  double heading_rad{0.0};
  // Accelerometer and gyroscope samples seen before the first scan, for the
  // level and the bias the vehicle shows at rest.
  Eigen::Vector3d rest_accelerometer_sum{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rest_gyro_sum{Eigen::Vector3d::Zero()};
  std::size_t rest_samples{0U};
  // The state at the last scan: what every later IMU sample is integrated
  // from, so a scan's starting guess is the state at the scan's own stamp.
  std::int64_t stamp_ns{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  std::deque<LidarInertialImuSample> imu;
  std::int64_t last_imu_stamp_ns{0};
  Eigen::Vector3d last_keyframe_position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond last_keyframe_rotation{Eigen::Quaterniond::Identity()};
  bool has_keyframe{false};
  // The last scan that registered: where the estimate returns to when a
  // scan fails, carried forward at that velocity.
  bool has_registered{false};
  bool holding{false};
  std::int64_t registered_stamp_ns{0};
  Eigen::Vector3d registered_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d registered_velocity{Eigen::Vector3d::Zero()};
  // The uncertainty of position and velocity at the last registered scan.
  Matrix6d covariance{Matrix6d::Identity() * 0.01};

  struct Propagated {
    std::int64_t stamp_ns{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  };

  // The state integrated through the IMU samples up to `target_stamp_ns`.
  // While holding, only the attitude follows the gyroscope. The interval
  // ends at the target itself: the stretch after the last sample before it
  // is integrated with the sample that spans it, or with the last sample
  // when none has arrived yet. Stopping at that last sample dropped up to a
  // transport period of every scan interval, and on r381 the velocity made
  // up for the missing motion at 1.12 times the scans' own and scattered
  // 0.55 m/s rms about it.
  [[nodiscard]] Propagated propagateTo(const std::int64_t target_stamp_ns) const {
    Propagated state{.stamp_ns = stamp_ns,
                     .position = position,
                     .rotation = rotation,
                     .velocity = velocity};
    const auto integrate = [this, &state](const LidarInertialImuSample& sample,
                                          const std::int64_t until_ns) {
      const double dt = 1.0e-9 * static_cast<double>(until_ns - state.stamp_ns);
      if (dt <= 0.5) {
        const Eigen::Vector3d rate = sample.gyro_radps - gyro_bias;
        state.rotation = (state.rotation * expSmallAngle(rate * dt)).normalized();
        if (!holding) {
          const Eigen::Vector3d acceleration =
              state.rotation * sample.accelerometer_mps2 +
              Eigen::Vector3d{0.0, 0.0, config.gravity_mps2};
          state.position += state.velocity * dt + 0.5 * acceleration * dt * dt;
          state.velocity += acceleration * dt;
        }
      }
      state.stamp_ns = until_ns;
    };
    const LidarInertialImuSample* last = nullptr;
    for (const LidarInertialImuSample& sample : imu) {
      if (state.stamp_ns >= target_stamp_ns) {
        break;
      }
      if (sample.stamp_ns <= state.stamp_ns) {
        last = &sample;
        continue;
      }
      integrate(sample, std::min(sample.stamp_ns, target_stamp_ns));
      last = &sample;
    }
    if (last != nullptr && state.stamp_ns < target_stamp_ns) {
      integrate(*last, target_stamp_ns);
    }
    return state;
  }

  void dropImuUpTo(const std::int64_t target_stamp_ns) {
    while (!imu.empty() && imu.front().stamp_ns <= target_stamp_ns) {
      imu.pop_front();
    }
  }

  [[nodiscard]] Eigen::Vector3d carriedPosition(const std::int64_t at_stamp_ns) const {
    const double dt =
        std::max(0.0, 1.0e-9 * static_cast<double>(at_stamp_ns - registered_stamp_ns));
    return registered_position + registered_velocity * std::min(dt, 2.0);
  }

  // The level from the accelerometer at rest: the measured specific force
  // is minus gravity in the body frame, so the rotation that takes it to
  // straight up gives roll and pitch; the heading is the declared one.
  void levelFromRest() {
    if (rest_samples == 0U) {
      rotation =
          Eigen::Quaterniond{Eigen::AngleAxisd{heading_rad, Eigen::Vector3d::UnitZ()}};
      attitude_levelled = true;
      return;
    }
    const Eigen::Vector3d body_up =
        (rest_accelerometer_sum / static_cast<double>(rest_samples)).normalized();
    const Eigen::Vector3d ned_up{0.0, 0.0, -1.0};
    const Eigen::Quaterniond tilt = Eigen::Quaterniond::FromTwoVectors(body_up, ned_up);
    const Eigen::Quaterniond heading{
        Eigen::AngleAxisd{heading_rad, Eigen::Vector3d::UnitZ()}};
    rotation = (heading * tilt).normalized();
    gyro_bias = rest_gyro_sum / static_cast<double>(rest_samples);
    attitude_levelled = true;
  }
};

LidarInertialOdometry::LidarInertialOdometry(const LidarInertialOdometryConfig& config)
    : impl_(std::make_unique<Impl>(config)) {
}

LidarInertialOdometry::~LidarInertialOdometry() = default;

void LidarInertialOdometry::initialize(const std::int64_t stamp_ns,
                                       const Eigen::Vector3d& position_ned_m,
                                       const double heading_rad) {
  impl_->initialized = true;
  impl_->attitude_levelled = false;
  impl_->stamp_ns = stamp_ns;
  impl_->position = position_ned_m;
  impl_->heading_rad = heading_rad;
  impl_->velocity.setZero();
  impl_->rotation =
      Eigen::Quaterniond{Eigen::AngleAxisd{heading_rad, Eigen::Vector3d::UnitZ()}};
  impl_->imu.clear();
}

bool LidarInertialOdometry::initialized() const noexcept {
  return impl_->initialized;
}

void LidarInertialOdometry::addImu(const LidarInertialImuSample& sample) {
  Impl& impl = *impl_;
  if (!impl.initialized) {
    return;
  }
  impl.last_imu_stamp_ns = std::max(impl.last_imu_stamp_ns, sample.stamp_ns);
  if (!impl.attitude_levelled) {
    impl.rest_accelerometer_sum += sample.accelerometer_mps2;
    impl.rest_gyro_sum += sample.gyro_radps;
    ++impl.rest_samples;
    return;
  }
  impl.imu.push_back(sample);
  // Five seconds of samples is more than any scan gap the estimator bridges.
  while (!impl.imu.empty() &&
         sample.stamp_ns - impl.imu.front().stamp_ns > 5'000'000'000LL) {
    impl.imu.pop_front();
  }
}

LidarInertialEstimate
LidarInertialOdometry::addScan(const std::int64_t stamp_ns,
                               const std::vector<Eigen::Vector3d>& points_body) {
  Impl& impl = *impl_;
  LidarInertialEstimate estimate;
  estimate.stamp_ns = stamp_ns;
  if (!impl.initialized) {
    return estimate;
  }
  if (!impl.attitude_levelled) {
    impl.levelFromRest();
    impl.stamp_ns = stamp_ns;
  }
  // A scan too large for the period is thinned coarser, not sampled: a
  // sampled scan leaves holes in the submap where a normal cannot be fitted.
  LidarInertialOdometryConfig thinning = impl.config;
  std::vector<Eigen::Vector3d> thinned = thinScan(points_body, thinning);
  while (impl.config.maximum_scan_points > 0U &&
         thinned.size() > impl.config.maximum_scan_points) {
    thinning.scan_voxel_m *= 1.25;
    thinned = thinScan(points_body, thinning);
  }
  estimate.scan_points = thinned.size();
  estimate.imu_lag_ns = stamp_ns - impl.last_imu_stamp_ns;

  // The starting guess: the last scan's state carried through the IMU to
  // this scan's stamp, or, after a lost scan, the last registered pose
  // carried at its velocity.
  const Impl::Propagated propagated = impl.propagateTo(stamp_ns);
  const Eigen::Quaterniond prior_rotation = propagated.rotation;
  const Eigen::Vector3d prior_position = impl.holding && impl.has_registered
                                             ? impl.carriedPosition(stamp_ns)
                                             : propagated.position;
  const double interval_s =
      impl.has_registered && stamp_ns > impl.registered_stamp_ns
          ? 1.0e-9 * static_cast<double>(stamp_ns - impl.registered_stamp_ns)
          : 0.0;
  bool healthy = false;
  Eigen::Vector3d corrected_position = prior_position;
  Eigen::Quaterniond corrected_rotation = prior_rotation;
  // Where the scan itself registered: along the axes the registration
  // observed its own position, along the others the filter's. The submap is
  // built at this pose, so the map follows what the scans saw and not the
  // filter's blend of them with the IMU.
  Eigen::Vector3d keyframe_position = prior_position;
  Eigen::Vector3d corrected_velocity = propagated.velocity;
  if (impl.submap.empty()) {
    healthy = !thinned.empty();
    estimate.matched_fraction = healthy ? 1.0 : 0.0;
  } else {
    const auto assess = [&impl](const PointPlaneRegistration3D& registration) {
      return registration.converged &&
             registration.matched_fraction >= impl.config.minimum_matched_fraction &&
             registration.residual_rms_m <= impl.config.maximum_residual_rms_m;
    };
    PointPlaneRegistration3D registration =
        registerScan(impl.submap, thinned, prior_position, prior_rotation, impl.config,
                     impl.config.maximum_correspondence_m);
    healthy = assess(registration);
    if (!healthy && impl.has_registered) {
      LidarInertialOdometryConfig wide = impl.config;
      wide.maximum_correspondence_m *= impl.config.recovery_correspondence_factor;
      // The search keeps the configured distance, as it always has: the
      // widened configuration reaches the registration's other settings only.
      const PointPlaneRegistration3D recovered =
          registerScan(impl.submap, thinned, impl.carriedPosition(stamp_ns),
                       prior_rotation, wide, impl.config.maximum_correspondence_m);
      if (assess(recovered)) {
        registration = recovered;
        healthy = true;
      }
    }
    estimate.matched_fraction = registration.matched_fraction;
    estimate.residual_rms_m = registration.residual_rms_m;
    estimate.information_per_point = registration.information_per_point;
    estimate.iterations = registration.iterations;
    if (healthy) {
      // The measurement: the registered position, with the variance the
      // registration's information gives along each axis it observed and
      // the degenerate variance along an axis it could not.
      const Eigen::Matrix3d translational =
          registration.information.bottomRightCorner<3, 3>();
      const double matched_points = std::max(
          1.0, registration.matched_fraction * static_cast<double>(thinned.size()));
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver{translational};
      const double residual_variance =
          std::max(1.0e-4, registration.residual_rms_m * registration.residual_rms_m);
      // The prior: the IMU's motion since the last registered scan, with the
      // uncertainty white acceleration noise adds over that interval.
      const double interval = std::max(0.0, interval_s);
      Matrix6d transition = Matrix6d::Identity();
      transition.topRightCorner<3, 3>() = interval * Eigen::Matrix3d::Identity();
      const double acceleration_variance =
          impl.config.acceleration_noise_mps2 * impl.config.acceleration_noise_mps2;
      Matrix6d process = Matrix6d::Zero();
      process.topLeftCorner<3, 3>() = acceleration_variance * std::pow(interval, 4) /
                                      4.0 * Eigen::Matrix3d::Identity();
      process.topRightCorner<3, 3>() = acceleration_variance * std::pow(interval, 3) /
                                       2.0 * Eigen::Matrix3d::Identity();
      process.bottomLeftCorner<3, 3>() = process.topRightCorner<3, 3>();
      process.bottomRightCorner<3, 3>() =
          acceleration_variance * interval * interval * Eigen::Matrix3d::Identity();
      const Matrix6d prior_covariance =
          transition * impl.covariance * transition.transpose() + process;
      Eigen::Matrix3d measurement_covariance = Eigen::Matrix3d::Zero();
      Eigen::Matrix3d observed = Eigen::Matrix3d::Zero();
      for (int axis = 0; axis < 3; ++axis) {
        const Eigen::Vector3d direction = solver.eigenvectors().col(axis);
        const double information = solver.eigenvalues()(axis);
        // An axis the registration could not observe carries no measurement:
        // the registration slides freely along it, and any finite variance
        // there lets that slide pull the prior along. Neither does an axis
        // whose registered position lies beyond the gate of the prior and
        // the measurement's own spread: a registration that slid along a
        // weak axis into another fit.
        double variance = kUnobservedVarianceM2;
        if (information / matched_points < impl.config.minimum_information_per_point) {
          ++estimate.degenerate_axes;
        } else {
          const double observed_variance =
              std::max(residual_variance / information,
                       impl.config.minimum_position_variance_m2);
          const double along = direction.dot(registration.position - prior_position);
          const double spread =
              direction.dot(prior_covariance.topLeftCorner<3, 3>() * direction) +
              observed_variance;
          if (along * along > impl.config.innovation_gate_sigma *
                                  impl.config.innovation_gate_sigma * spread) {
            ++estimate.gated_axes;
          } else {
            variance = observed_variance;
            observed += direction * direction.transpose();
          }
        }
        measurement_covariance += variance * direction * direction.transpose();
      }
      const Eigen::Vector3d prior_velocity =
          impl.holding ? impl.registered_velocity : propagated.velocity;
      // The Kalman step on the position measurement.
      const Eigen::Matrix3d innovation_covariance =
          prior_covariance.topLeftCorner<3, 3>() + measurement_covariance;
      const Eigen::Matrix<double, 6, 3> gain =
          prior_covariance.leftCols<3>() * innovation_covariance.inverse();
      const Eigen::Vector3d innovation = registration.position - prior_position;
      corrected_position = prior_position + gain.topRows<3>() * innovation;
      corrected_velocity = prior_velocity + gain.bottomRows<3>() * innovation;
      keyframe_position =
          corrected_position + observed * (registration.position - corrected_position);
      Matrix6d update = Matrix6d::Identity();
      update.leftCols<3>() -= gain;
      impl.covariance = update * prior_covariance;
      impl.covariance = 0.5 * (impl.covariance + impl.covariance.transpose());
      const Eigen::Vector3d correction = corrected_position - prior_position;
      const double speed = prior_velocity.norm();
      estimate.correction_along_track_m =
          speed > 0.1 ? correction.dot(prior_velocity) / speed : 0.0;
      corrected_rotation = registration.rotation;
      estimate.position_variance_m2 =
          impl.covariance.topLeftCorner<3, 3>().diagonal().cwiseMax(1.0e-4);
      estimate.velocity_variance_m2ps2 =
          impl.covariance.bottomRightCorner<3, 3>().diagonal().cwiseMax(1.0e-4);
      const Eigen::Matrix3d rotational = registration.information.topLeftCorner<3, 3>();
      const Eigen::Matrix3d orientation_covariance =
          residual_variance *
          (rotational + 1.0e-6 * Eigen::Matrix3d::Identity()).inverse();
      estimate.orientation_variance_rad2 =
          orientation_covariance.diagonal().cwiseMax(1.0e-6).cwiseMin(0.1);
      // A share of the rotation the IMU missed over the interval goes to
      // the gyroscope bias.
      const Eigen::Vector3d attitude_error =
          logRotation(prior_rotation.conjugate() * registration.rotation);
      if (interval_s > 0.0) {
        impl.gyro_bias += impl.config.gyro_bias_gain * attitude_error / interval_s;
        const double bias_norm = impl.gyro_bias.norm();
        if (bias_norm > impl.config.maximum_gyro_bias_radps) {
          impl.gyro_bias *= impl.config.maximum_gyro_bias_radps / bias_norm;
        }
      }
    }
  }
  if (healthy) {
    impl.stamp_ns = stamp_ns;
    impl.position = corrected_position;
    impl.rotation = corrected_rotation;
    impl.velocity = corrected_velocity;
    impl.dropImuUpTo(stamp_ns);
    const bool keyframe_due =
        !impl.has_keyframe ||
        (impl.position - impl.last_keyframe_position).norm() >=
            impl.config.keyframe_translation_m ||
        logRotation(impl.last_keyframe_rotation.conjugate() * impl.rotation).norm() >=
            impl.config.keyframe_rotation_rad;
    if (keyframe_due) {
      Keyframe keyframe;
      keyframe.position = keyframe_position;
      keyframe.rotation = impl.rotation;
      keyframe.points_world.reserve(thinned.size());
      const Eigen::Matrix3d rotation = impl.rotation.toRotationMatrix();
      for (const Eigen::Vector3d& point : thinned) {
        keyframe.points_world.push_back(rotation * point + keyframe_position);
      }
      impl.submap.insert(std::move(keyframe));
      impl.last_keyframe_position = impl.position;
      impl.last_keyframe_rotation = impl.rotation;
      impl.has_keyframe = true;
    }
    impl.has_registered = true;
    impl.holding = false;
    impl.registered_stamp_ns = stamp_ns;
    impl.registered_position = impl.position;
    impl.registered_velocity = impl.velocity;
  } else if (impl.has_registered) {
    // Hold at the carried pose: the attitude keeps following the gyroscope,
    // the position waits for a scan that registers.
    impl.holding = true;
    impl.stamp_ns = stamp_ns;
    impl.position = prior_position;
    impl.rotation = prior_rotation;
    impl.velocity = impl.registered_velocity;
    impl.dropImuUpTo(stamp_ns);
  }
  estimate.healthy = healthy;
  estimate.position_ned_m = impl.position;
  estimate.body_to_ned = impl.rotation;
  estimate.velocity_ned_mps = impl.velocity;
  estimate.submap_points = impl.submap.pointCount();
  estimate.keyframes = impl.submap.keyframeCount();
  return estimate;
}

} // namespace drone_city_nav
