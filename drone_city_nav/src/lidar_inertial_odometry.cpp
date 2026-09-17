#include "drone_city_nav/lidar_inertial_odometry.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

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

// One cell of the submap: its points and the plane through them, fitted
// over the cell and its neighbours once enough points are there.
struct SubmapCell {
  std::vector<Eigen::Vector3d> points;
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
  bool normal_valid{false};
  bool normal_stale{true};
};

struct Keyframe {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  std::vector<Eigen::Vector3d> points_world;
};

class Submap {
public:
  explicit Submap(const LidarInertialOdometryConfig& config)
      : config_(config) {
  }

  [[nodiscard]] bool empty() const noexcept {
    return cells_.empty();
  }

  [[nodiscard]] std::size_t pointCount() const noexcept {
    return point_count_;
  }

  [[nodiscard]] std::size_t keyframeCount() const noexcept {
    return keyframes_.size();
  }

  void insert(Keyframe keyframe) {
    for (const Eigen::Vector3d& point : keyframe.points_world) {
      SubmapCell& cell = cells_[cellOf(point, config_.scan_voxel_m)];
      if (cell.points.size() >= config_.maximum_points_per_cell) {
        continue;
      }
      cell.points.push_back(point);
      cell.normal_stale = true;
      ++point_count_;
      markNeighboursStale(point);
    }
    keyframes_.push_back(std::move(keyframe));
    while (keyframes_.size() > config_.maximum_keyframes) {
      rebuild();
    }
  }

  // The nearest submap point to `query` with its cell's plane normal.
  [[nodiscard]] bool nearest(const Eigen::Vector3d& query, Eigen::Vector3d& point,
                             Eigen::Vector3d& normal) {
    const CellKey center = cellOf(query, config_.scan_voxel_m);
    const double limit =
        config_.maximum_correspondence_m * config_.maximum_correspondence_m;
    double best = limit;
    SubmapCell* best_cell = nullptr;
    for (std::int32_t dx = -1; dx <= 1; ++dx) {
      for (std::int32_t dy = -1; dy <= 1; ++dy) {
        for (std::int32_t dz = -1; dz <= 1; ++dz) {
          const auto found =
              cells_.find(CellKey{center.x + dx, center.y + dy, center.z + dz});
          if (found == cells_.end()) {
            continue;
          }
          for (const Eigen::Vector3d& candidate : found->second.points) {
            const double distance = (candidate - query).squaredNorm();
            if (distance < best) {
              best = distance;
              point = candidate;
              best_cell = &found->second;
            }
          }
        }
      }
    }
    if (best_cell == nullptr) {
      return false;
    }
    if (best_cell->normal_stale) {
      fitNormal(*best_cell, point);
    }
    normal = best_cell->normal;
    return best_cell->normal_valid;
  }

private:
  void markNeighboursStale(const Eigen::Vector3d& point) {
    const CellKey center = cellOf(point, config_.scan_voxel_m);
    for (std::int32_t dx = -2; dx <= 2; ++dx) {
      for (std::int32_t dy = -2; dy <= 2; ++dy) {
        for (std::int32_t dz = -2; dz <= 2; ++dz) {
          const auto found =
              cells_.find(CellKey{center.x + dx, center.y + dy, center.z + dz});
          if (found != cells_.end()) {
            found->second.normal_stale = true;
          }
        }
      }
    }
  }

  // The plane through the points of the cell and its two rings of
  // neighbours: the smallest principal axis, valid once the points are many
  // and flat enough. Two rings, because a scan thinned coarser than the
  // cell leaves one ring with too few points for a plane.
  void fitNormal(SubmapCell& cell, const Eigen::Vector3d& around) {
    const CellKey center = cellOf(around, config_.scan_voxel_m);
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    std::size_t count = 0U;
    std::vector<const Eigen::Vector3d*> neighbours;
    for (std::int32_t dx = -2; dx <= 2; ++dx) {
      for (std::int32_t dy = -2; dy <= 2; ++dy) {
        for (std::int32_t dz = -2; dz <= 2; ++dz) {
          const auto found =
              cells_.find(CellKey{center.x + dx, center.y + dy, center.z + dz});
          if (found == cells_.end()) {
            continue;
          }
          for (const Eigen::Vector3d& point : found->second.points) {
            neighbours.push_back(&point);
            mean += point;
            ++count;
          }
        }
      }
    }
    cell.normal_stale = false;
    cell.normal_valid = false;
    if (count < 6U) {
      return;
    }
    mean /= static_cast<double>(count);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const Eigen::Vector3d* point : neighbours) {
      const Eigen::Vector3d offset = *point - mean;
      covariance += offset * offset.transpose();
    }
    covariance /= static_cast<double>(count);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver{covariance};
    const Eigen::Vector3d values = solver.eigenvalues();
    // A plane: the smallest spread is well below the middle one.
    if (!(values(0) < 0.25 * values(1))) {
      return;
    }
    cell.normal = solver.eigenvectors().col(0).normalized();
    cell.normal_valid = true;
  }

  void rebuild() {
    keyframes_.pop_front();
    cells_.clear();
    point_count_ = 0U;
    for (const Keyframe& keyframe : keyframes_) {
      for (const Eigen::Vector3d& point : keyframe.points_world) {
        SubmapCell& cell = cells_[cellOf(point, config_.scan_voxel_m)];
        if (cell.points.size() < config_.maximum_points_per_cell) {
          cell.points.push_back(point);
          ++point_count_;
        }
      }
    }
    for (auto& [key, cell] : cells_) {
      cell.normal_stale = true;
    }
  }

  const LidarInertialOdometryConfig& config_;
  std::unordered_map<CellKey, SubmapCell, CellKeyHash> cells_;
  std::deque<Keyframe> keyframes_;
  std::size_t point_count_{0U};
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

struct RegistrationResult {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Matrix6d information{Matrix6d::Zero()};
  double matched_fraction{0.0};
  double residual_rms_m{0.0};
  double information_per_point{0.0};
  std::size_t iterations{0U};
  bool converged{false};
};

// Point-to-plane registration of the thinned scan against the submap from
// the prior pose: Gauss-Newton over the left perturbation [rotation;
// translation] of the world pose with a Huber weight on each residual.
[[nodiscard]] RegistrationResult
registerScan(Submap& submap, const std::vector<Eigen::Vector3d>& points_body,
             const Eigen::Vector3d& prior_position,
             const Eigen::Quaterniond& prior_rotation,
             const LidarInertialOdometryConfig& config) {
  RegistrationResult result;
  result.position = prior_position;
  result.rotation = prior_rotation;
  if (points_body.empty()) {
    return result;
  }
  Eigen::Vector3d map_point;
  Eigen::Vector3d normal;
  for (std::size_t iteration = 0U; iteration < config.maximum_iterations; ++iteration) {
    Matrix6d hessian = Matrix6d::Zero();
    Vector6d gradient = Vector6d::Zero();
    double weighted_square = 0.0;
    std::size_t matched = 0U;
    const Eigen::Matrix3d rotation = result.rotation.toRotationMatrix();
    for (const Eigen::Vector3d& point_body : points_body) {
      const Eigen::Vector3d rotated = rotation * point_body;
      const Eigen::Vector3d point_world = rotated + result.position;
      if (!submap.nearest(point_world, map_point, normal)) {
        continue;
      }
      const double residual = normal.dot(point_world - map_point);
      const double magnitude = std::abs(residual);
      const double weight =
          magnitude <= config.robust_width_m ? 1.0 : config.robust_width_m / magnitude;
      Vector6d jacobian;
      jacobian.head<3>() = rotated.cross(normal);
      jacobian.tail<3>() = normal;
      hessian += weight * jacobian * jacobian.transpose();
      gradient += weight * residual * jacobian;
      weighted_square += weight * residual * residual;
      ++matched;
    }
    result.iterations = iteration + 1U;
    result.matched_fraction =
        static_cast<double>(matched) / static_cast<double>(points_body.size());
    result.residual_rms_m =
        matched > 0U ? std::sqrt(weighted_square / static_cast<double>(matched)) : 0.0;
    result.information = hessian;
    if (matched < 6U) {
      return result;
    }
    // A touch of damping keeps a weakly observed axis from running away.
    const Matrix6d damped = hessian + 1.0e-6 * Matrix6d::Identity();
    const Vector6d delta = damped.ldlt().solve(-gradient);
    if (!delta.allFinite()) {
      return result;
    }
    result.rotation = (expSmallAngle(delta.head<3>()) * result.rotation).normalized();
    result.position += delta.tail<3>();
    if (delta.tail<3>().norm() < config.convergence_translation_m &&
        delta.head<3>().norm() < config.convergence_rotation_rad) {
      result.converged = true;
      break;
    }
  }
  if (result.iterations > 0U && !result.converged) {
    // The last step was applied; the fit is what its own step size says.
    result.converged = true;
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver{
      result.information.bottomRightCorner<3, 3>()};
  const double matched_points =
      result.matched_fraction * static_cast<double>(points_body.size());
  result.information_per_point =
      matched_points > 0.0 ? solver.eigenvalues()(0) / matched_points : 0.0;
  return result;
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

  struct Propagated {
    std::int64_t stamp_ns{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  };

  // The state integrated through the IMU samples up to `target_stamp_ns`.
  // While holding, only the attitude follows the gyroscope.
  [[nodiscard]] Propagated propagateTo(const std::int64_t target_stamp_ns) const {
    Propagated state{.stamp_ns = stamp_ns,
                     .position = position,
                     .rotation = rotation,
                     .velocity = velocity};
    for (const LidarInertialImuSample& sample : imu) {
      if (sample.stamp_ns <= state.stamp_ns) {
        continue;
      }
      if (sample.stamp_ns > target_stamp_ns) {
        break;
      }
      const double dt = 1.0e-9 * static_cast<double>(sample.stamp_ns - state.stamp_ns);
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
      state.stamp_ns = sample.stamp_ns;
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
  bool healthy = false;
  Eigen::Vector3d corrected_position = prior_position;
  Eigen::Quaterniond corrected_rotation = prior_rotation;
  if (impl.submap.empty()) {
    healthy = !thinned.empty();
    estimate.matched_fraction = healthy ? 1.0 : 0.0;
  } else {
    const auto assess = [&impl](const RegistrationResult& registration) {
      return registration.converged &&
             registration.matched_fraction >= impl.config.minimum_matched_fraction &&
             registration.residual_rms_m <= impl.config.maximum_residual_rms_m &&
             registration.information_per_point >=
                 impl.config.minimum_information_per_point;
    };
    RegistrationResult registration =
        registerScan(impl.submap, thinned, prior_position, prior_rotation, impl.config);
    healthy = assess(registration);
    if (!healthy && impl.has_registered) {
      LidarInertialOdometryConfig wide = impl.config;
      wide.maximum_correspondence_m *= impl.config.recovery_correspondence_factor;
      const RegistrationResult recovered = registerScan(
          impl.submap, thinned, impl.carriedPosition(stamp_ns), prior_rotation, wide);
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
      corrected_position = registration.position;
      corrected_rotation = registration.rotation;
      // A share of the rotation the IMU missed over the interval goes to
      // the gyroscope bias.
      const Eigen::Vector3d attitude_error =
          logRotation(prior_rotation.conjugate() * registration.rotation);
      if (impl.has_registered && stamp_ns > impl.registered_stamp_ns) {
        const double dt =
            1.0e-9 * static_cast<double>(stamp_ns - impl.registered_stamp_ns);
        impl.gyro_bias += impl.config.gyro_bias_gain * attitude_error / dt;
        const double bias_norm = impl.gyro_bias.norm();
        if (bias_norm > impl.config.maximum_gyro_bias_radps) {
          impl.gyro_bias *= impl.config.maximum_gyro_bias_radps / bias_norm;
        }
      }
      const Eigen::Matrix3d translational =
          registration.information.bottomRightCorner<3, 3>();
      const Eigen::Matrix3d rotational = registration.information.topLeftCorner<3, 3>();
      const double residual_variance =
          std::max(1.0e-4, registration.residual_rms_m * registration.residual_rms_m);
      const Eigen::Matrix3d position_covariance =
          residual_variance *
          (translational + 1.0e-6 * Eigen::Matrix3d::Identity()).inverse();
      const Eigen::Matrix3d orientation_covariance =
          residual_variance *
          (rotational + 1.0e-6 * Eigen::Matrix3d::Identity()).inverse();
      estimate.position_variance_m2 =
          position_covariance.diagonal().cwiseMax(1.0e-4).cwiseMin(1.0);
      estimate.orientation_variance_rad2 =
          orientation_covariance.diagonal().cwiseMax(1.0e-6).cwiseMin(0.1);
    }
  }
  if (healthy) {
    // The velocity is the registered motion since the last registered scan.
    if (impl.has_registered && stamp_ns > impl.registered_stamp_ns) {
      const double dt =
          1.0e-9 * static_cast<double>(stamp_ns - impl.registered_stamp_ns);
      impl.velocity = (corrected_position - impl.registered_position) / dt;
    }
    impl.stamp_ns = stamp_ns;
    impl.position = corrected_position;
    impl.rotation = corrected_rotation;
    impl.dropImuUpTo(stamp_ns);
    const bool keyframe_due =
        !impl.has_keyframe ||
        (impl.position - impl.last_keyframe_position).norm() >=
            impl.config.keyframe_translation_m ||
        logRotation(impl.last_keyframe_rotation.conjugate() * impl.rotation).norm() >=
            impl.config.keyframe_rotation_rad;
    if (keyframe_due) {
      Keyframe keyframe;
      keyframe.position = impl.position;
      keyframe.rotation = impl.rotation;
      keyframe.points_world.reserve(thinned.size());
      const Eigen::Matrix3d rotation = impl.rotation.toRotationMatrix();
      for (const Eigen::Vector3d& point : thinned) {
        keyframe.points_world.push_back(rotation * point + impl.position);
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
  estimate.velocity_variance_m2ps2 = estimate.position_variance_m2 * 4.0;
  estimate.submap_points = impl.submap.pointCount();
  estimate.keyframes = impl.submap.keyframeCount();
  return estimate;
}

} // namespace drone_city_nav
