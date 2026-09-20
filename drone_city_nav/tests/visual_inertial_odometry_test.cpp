#include "drone_city_nav/visual_inertial_odometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr double kGravity{9.80665};
constexpr std::int64_t kImuPeriodNs{4'000'000};
constexpr std::int64_t kFramePeriodNs{100'000'000};

// The optical frame (x right, y down, z forward) in the body FRD frame.
Eigen::Quaterniond forwardCamera() {
  Eigen::Matrix3d camera_to_body;
  camera_to_body << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  return Eigen::Quaterniond{camera_to_body};
}

VisualInertialOdometryConfig testConfig() {
  VisualInertialOdometryConfig config;
  config.left_camera = {.camera_to_body = forwardCamera(),
                        .position_body_m = {0.3, -0.1, 0.0}};
  config.right_camera = {.camera_to_body = forwardCamera(),
                         .position_body_m = {0.3, 0.1, 0.0}};
  return config;
}

// A level vehicle that leaves rest onto a circle of 6 m radius, facing along
// its velocity and reaching 1.5 m/s, with a slow climb and descent: every
// axis is excited, and the declared initial pose is a pose at rest.
struct Motion {
  static constexpr double kRadius{6.0};
  static constexpr double kRate{0.25};
  static constexpr double kRampS{2.0};
  static constexpr double kClimb{0.5};
  static constexpr double kClimbRate{0.4};

  [[nodiscard]] static double angle(const double t) {
    return kRate * (t - kRampS * (1.0 - std::exp(-t / kRampS)));
  }

  [[nodiscard]] static double angleRate(const double t) {
    return kRate * (1.0 - std::exp(-t / kRampS));
  }

  [[nodiscard]] static Eigen::Vector3d position(const double t) {
    return {kRadius * std::sin(angle(t)), kRadius * (1.0 - std::cos(angle(t))),
            -kClimb * (1.0 - std::cos(kClimbRate * t))};
  }

  [[nodiscard]] static Eigen::Vector3d acceleration(const double t) {
    const double rate = angleRate(t);
    const double rate_change = kRate / kRampS * std::exp(-t / kRampS);
    return {
        kRadius * (rate_change * std::cos(angle(t)) - rate * rate * std::sin(angle(t))),
        kRadius * (rate_change * std::sin(angle(t)) + rate * rate * std::cos(angle(t))),
        -kClimb * kClimbRate * kClimbRate * std::cos(kClimbRate * t)};
  }

  [[nodiscard]] static Eigen::Matrix3d bodyToNed(const double t) {
    return Eigen::AngleAxisd{angle(t), Eigen::Vector3d::UnitZ()}.toRotationMatrix();
  }

  [[nodiscard]] static VisualInertialImuSample imu(const std::int64_t stamp_ns,
                                                   const Eigen::Vector3d& gyro_bias) {
    const double t = static_cast<double>(stamp_ns) * 1.0e-9;
    return {.stamp_ns = stamp_ns,
            .gyro_radps = Eigen::Vector3d{0.0, 0.0, angleRate(t)} + gyro_bias,
            .accelerometer_mps2 =
                bodyToNed(t).transpose() *
                (acceleration(t) - Eigen::Vector3d{0.0, 0.0, kGravity})};
  }
};

std::vector<Eigen::Vector3d> landmarks(std::mt19937& generator) {
  // Walls of a 40 m box around the circle: every heading sees some of them.
  std::uniform_real_distribution<double> along{-20.0, 20.0};
  std::uniform_real_distribution<double> height{-6.0, 2.0};
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 400; ++i) {
    const double a = along(generator);
    const double z = height(generator);
    switch (i % 4) {
      case 0:
        points.emplace_back(a, 26.0, z);
        break;
      case 1:
        points.emplace_back(a, -14.0, z);
        break;
      case 2:
        points.emplace_back(20.0, a + 6.0, z);
        break;
      default:
        points.emplace_back(-20.0, a + 6.0, z);
        break;
    }
  }
  return points;
}

std::vector<StereoFeatureObservation>
observe(const VisualInertialOdometryConfig& config, const double t,
        const std::vector<Eigen::Vector3d>& points, const double noise,
        std::mt19937& generator) {
  std::normal_distribution<double> pixel{0.0, 1.0};
  const Eigen::Matrix3d body_to_ned = Motion::bodyToNed(t);
  const Eigen::Vector3d position = Motion::position(t);
  std::vector<StereoFeatureObservation> observations;
  for (std::size_t id = 0U; id < points.size(); ++id) {
    const Eigen::Vector3d in_body = body_to_ned.transpose() * (points[id] - position);
    StereoFeatureObservation observation{.id = id + 1U};
    bool visible = true;
    for (int side = 0; side < 2; ++side) {
      const VisualInertialCameraMount& mount =
          side == 0 ? config.left_camera : config.right_camera;
      const Eigen::Vector3d in_camera =
          mount.camera_to_body.conjugate() * (in_body - mount.position_body_m);
      if (in_camera.z() < 1.0 || std::abs(in_camera.x() / in_camera.z()) > 1.5 ||
          std::abs(in_camera.y() / in_camera.z()) > 1.1) {
        visible = false;
        break;
      }
      const Eigen::Vector2d projected =
          in_camera.head<2>() / in_camera.z() +
          noise * Eigen::Vector2d{pixel(generator), pixel(generator)};
      (side == 0 ? observation.left : observation.right) = projected;
    }
    if (visible) {
      observations.push_back(observation);
    }
  }
  return observations;
}

struct Flight {
  VisualInertialEstimate last;
  double maximum_position_error_m{0.0};
  double minimum_heading_variance_rad2{1.0e9};
  std::size_t gated{0U};
  std::size_t used{0U};
};

Flight fly(const VisualInertialOdometryConfig& config, const double duration_s,
           const Eigen::Vector3d& gyro_bias, const bool with_features,
           const bool with_outlier) {
  std::mt19937 generator{7U};
  const std::vector<Eigen::Vector3d> points = landmarks(generator);
  VisualInertialOdometry odometry{config};
  odometry.initialize(0, Motion::position(0.0), 0.0);
  Flight flight;
  const auto frames = static_cast<std::int64_t>(duration_s * 1.0e9) / kFramePeriodNs;
  std::int64_t imu_stamp = 0;
  for (std::int64_t frame = 1; frame <= frames; ++frame) {
    const std::int64_t stamp = frame * kFramePeriodNs;
    for (; imu_stamp <= stamp; imu_stamp += kImuPeriodNs) {
      odometry.addImu(Motion::imu(imu_stamp, gyro_bias));
    }
    const double t = static_cast<double>(stamp) * 1.0e-9;
    std::vector<StereoFeatureObservation> observations;
    if (with_features) {
      observations = observe(config, t, points, config.observation_noise, generator);
    }
    if (with_outlier && !observations.empty()) {
      // A point that slides across the image: nothing fixed in the world.
      StereoFeatureObservation moving = observations.front();
      moving.id = 100'000U;
      moving.left.x() += 0.02 * static_cast<double>(frame % 20);
      moving.right.x() += 0.02 * static_cast<double>(frame % 20);
      observations.push_back(moving);
    }
    flight.last = odometry.addFrame(stamp, observations);
    flight.maximum_position_error_m =
        std::max(flight.maximum_position_error_m,
                 (flight.last.position_ned_m - Motion::position(t)).norm());
    flight.minimum_heading_variance_rad2 =
        std::min(flight.minimum_heading_variance_rad2,
                 flight.last.orientation_variance_rad2.z());
    flight.gated += flight.last.gated_features;
    flight.used += flight.last.used_features;
  }
  return flight;
}

TEST(VisualInertialOdometry, TheImuAloneCarriesTheMotionAndItsUncertaintyGrows) {
  const Flight flight = fly(testConfig(), 4.0, Eigen::Vector3d::Zero(), false, false);
  EXPECT_LT(flight.maximum_position_error_m, 0.02);
  EXPECT_EQ(flight.last.used_features, 0U);
  EXPECT_FALSE(flight.last.healthy);
  EXPECT_GT(flight.last.position_variance_m2.x(), 1.0e-4);
}

TEST(VisualInertialOdometry, TheWindowKeepsItsDeclaredNumberOfClones) {
  VisualInertialOdometryConfig config = testConfig();
  config.maximum_clones = 6U;
  const Flight flight = fly(config, 3.0, Eigen::Vector3d::Zero(), true, false);
  EXPECT_EQ(flight.last.clones, 6U);
  EXPECT_GT(flight.used, 0U);
}

TEST(VisualInertialOdometry, FeaturesHoldThePositionAgainstAGyroscopeBias) {
  const Eigen::Vector3d bias{0.004, -0.003, 0.005};
  const Flight blind = fly(testConfig(), 30.0, bias, false, false);
  const Flight sighted = fly(testConfig(), 30.0, bias, true, false);
  EXPECT_GT(blind.maximum_position_error_m, 3.0);
  EXPECT_LT(sighted.maximum_position_error_m, 0.3);
  EXPECT_TRUE(sighted.last.healthy);
  EXPECT_LT(sighted.last.weakest_velocity_sigma_mps,
            0.2 * blind.last.weakest_velocity_sigma_mps);
  EXPECT_NEAR(sighted.last.gyro_bias_radps.z(), bias.z(), 1.5e-3);
  EXPECT_LT(sighted.last.residual_rms_sigma, 1.5);
}

TEST(VisualInertialOdometry, APointThatMovesInTheWorldIsGatedOut) {
  const Flight clean = fly(testConfig(), 10.0, Eigen::Vector3d::Zero(), true, false);
  const Flight flight = fly(testConfig(), 10.0, Eigen::Vector3d::Zero(), true, true);
  EXPECT_GT(flight.gated, clean.gated);
  EXPECT_LT(flight.maximum_position_error_m, clean.maximum_position_error_m + 0.03);
}

TEST(VisualInertialOdometry, TheHeadingIsNeverLearnedFromFeatures) {
  // No camera observes the rotation about gravity or the position of the
  // whole scene; with first-estimate Jacobians the filter's certainty about
  // them never improves on what was declared.
  const VisualInertialOdometryConfig config = testConfig();
  const Flight flight = fly(config, 30.0, Eigen::Vector3d::Zero(), true, false);
  const double declared =
      config.initial_heading_sigma_rad * config.initial_heading_sigma_rad;
  EXPECT_GE(flight.minimum_heading_variance_rad2, 0.999 * declared);
  EXPECT_GE(flight.last.position_variance_m2.minCoeff(),
            0.999 * config.initial_position_sigma_m * config.initial_position_sigma_m);
}

TEST(VisualInertialOdometry, AFrameBeforeTheDeclaredPoseIsNoEstimate) {
  VisualInertialOdometry odometry{testConfig()};
  EXPECT_FALSE(odometry.initialized());
  const VisualInertialEstimate estimate = odometry.addFrame(kFramePeriodNs, {});
  EXPECT_EQ(estimate.clones, 0U);
}

} // namespace
} // namespace drone_city_nav
