#include "drone_city_nav/visual_inertial_odometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <optional>
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
  // The synthetic motion is a point's, not a multirotor's: it has no drag.
  config.rotor_drag_1ps = 0.0;
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

// A hole in the IMU stream and the frames, as a stalled host leaves one:
// nothing between its two ends reaches the filter.
struct Hole {
  double from_s{0.0};
  double to_s{0.0};

  [[nodiscard]] bool contains(const std::int64_t stamp_ns) const noexcept {
    const double t = static_cast<double>(stamp_ns) * 1.0e-9;
    return t > from_s && t < to_s;
  }
};

Flight fly(const VisualInertialOdometryConfig& config, const double duration_s,
           const Eigen::Vector3d& gyro_bias, const bool with_features,
           const bool with_outlier, const Hole hole = {}, const Hole dark = {},
           const Hole sparse = {}, const Hole void_frames = {}) {
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
      if (!hole.contains(imu_stamp)) {
        odometry.addImu(Motion::imu(imu_stamp, gyro_bias));
      }
    }
    if (hole.contains(stamp)) {
      continue;
    }
    const double t = static_cast<double>(stamp) * 1.0e-9;
    std::vector<StereoFeatureObservation> observations;
    if (with_features && !dark.contains(stamp)) {
      observations = observe(config, t, points, config.observation_noise, generator);
    }
    if (sparse.contains(stamp) && observations.size() > 6U) {
      observations.resize(6U);
    }
    if (with_outlier && !observations.empty()) {
      // A point that slides across the image: nothing fixed in the world.
      StereoFeatureObservation moving = observations.front();
      moving.id = 100'000U;
      moving.left.x() += 0.02 * static_cast<double>(frame % 20);
      moving.right.x() += 0.02 * static_cast<double>(frame % 20);
      observations.push_back(moving);
    }
    if (void_frames.contains(stamp)) {
      odometry.observeVoid(stamp, true);
    } else if (void_frames.to_s > 0.0) {
      odometry.observeVoid(stamp, false);
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

TEST(VisualInertialOdometry, AHoleTheFilterCannotBridgeLeavesItUnhealthy) {
  // r720, r723: a hole of seconds, and the estimate that came out of it was
  // published as healthy. A hole within the unaided timeout is bridged; one
  // beyond it leaves the estimate unhealthy for the rest of the flight,
  // however well the features resume.
  const VisualInertialOdometryConfig config = testConfig();
  const Flight bridged =
      fly(config, 12.0, Eigen::Vector3d::Zero(), true, false, Hole{5.0, 5.5});
  const Flight lost = fly(config, 12.0, Eigen::Vector3d::Zero(), true, false,
                          Hole{5.0, 5.0 + config.maximum_unaided_s + 0.5});
  EXPECT_TRUE(bridged.last.healthy);
  EXPECT_GT(lost.last.used_features + lost.used, 0U);
  EXPECT_FALSE(lost.last.healthy);
}

TEST(VisualInertialOdometry, ADarkStretchIsDeadReckoningForAStatedTime) {
  // Roadmap item 17 stage 5: frames that arrive with nothing in them. Past
  // the unaided timeout the estimate is not healthy, but it is declared dead
  // reckoning while its velocity stays within the bound (about 7 s here), and
  // never past the stated time.
  const VisualInertialOdometryConfig config = testConfig();
  const Flight short_dark =
      fly(config, 8.0, Eigen::Vector3d::Zero(), true, false, {}, Hole{5.0, 9.0});
  EXPECT_FALSE(short_dark.last.healthy);
  EXPECT_TRUE(short_dark.last.dead_reckoning);
  EXPECT_NEAR(short_dark.last.unaided_s, 3.0, 0.2);
  const Flight long_dark = fly(
      config, 5.0 + config.maximum_unaided_s + config.maximum_dead_reckoning_s + 1.0,
      Eigen::Vector3d::Zero(), true, false, {}, Hole{5.0, 60.0});
  EXPECT_FALSE(long_dark.last.healthy);
  EXPECT_FALSE(long_dark.last.dead_reckoning);
  // Spent, it is lost for the flight: frames that return do not bring it
  // back healthy (r820 came back 35 m off).
  const Flight dark_then_seen = fly(
      config, 5.0 + config.maximum_unaided_s + config.maximum_dead_reckoning_s + 6.0,
      Eigen::Vector3d::Zero(), true, false, {},
      Hole{5.0,
           5.0 + config.maximum_unaided_s + config.maximum_dead_reckoning_s + 1.0});
  EXPECT_FALSE(dark_then_seen.last.healthy);
  const Flight sighted = fly(config, 12.0, Eigen::Vector3d::Zero(), true, false);
  EXPECT_TRUE(sighted.last.healthy);
  EXPECT_FALSE(sighted.last.dead_reckoning);
}

TEST(VisualInertialOdometry, TheVoidIsNotDeadReckoningAndSpendsNoBudget) {
  // Roadmap item 31 (K46): frames with nothing in them while the pair judges
  // each a void frame. Past the unaided timeout the estimate is in the void,
  // not dead reckoning, for as long as the void lasts, 30 s here, three
  // times the dead-reckoning budget; it is never lost; and when the features
  // return the estimate continues and is healthy again. The synthetic motion
  // has no drag to bound the velocity by (K21), so the velocity's own bound,
  // tested above, is lifted here: the void's time is the question.
  VisualInertialOdometryConfig config = testConfig();
  config.maximum_velocity_sigma_mps = 100.0;
  const Flight in_void = fly(config, 35.0, Eigen::Vector3d::Zero(), true, false, {},
                             Hole{5.0, 60.0}, {}, Hole{4.0, 60.0});
  EXPECT_FALSE(in_void.last.healthy);
  EXPECT_TRUE(in_void.last.in_void);
  EXPECT_FALSE(in_void.last.dead_reckoning);
  EXPECT_FALSE(in_void.last.lost);
  EXPECT_NEAR(in_void.last.unaided_s, 30.0, 0.2);
  const Flight seen_again = fly(config, 45.0, Eigen::Vector3d::Zero(), true, false, {},
                                Hole{5.0, 35.0}, {}, Hole{4.0, 35.5});
  EXPECT_TRUE(seen_again.last.healthy);
  EXPECT_FALSE(seen_again.last.in_void);
  EXPECT_FALSE(seen_again.last.lost);
}

TEST(VisualInertialOdometry, ALightFailingInTheVoidIsDeadReckoningFromThatMoment) {
  // The void of 15 s, then the witness gone with the frames still dark (the
  // light failed): dead reckoning from there with its whole budget (1 + 10 s),
  // so at 8 s after the witness went the estimate is dead reckoning, and past
  // the budget it is lost as before (K13).
  VisualInertialOdometryConfig config = testConfig();
  config.maximum_velocity_sigma_mps = 100.0;
  const Flight reckoning = fly(config, 28.0, Eigen::Vector3d::Zero(), true, false, {},
                               Hole{5.0, 60.0}, {}, Hole{4.0, 20.0});
  EXPECT_FALSE(reckoning.last.in_void);
  EXPECT_TRUE(reckoning.last.dead_reckoning);
  EXPECT_FALSE(reckoning.last.lost);
  const Flight spent = fly(config, 33.0, Eigen::Vector3d::Zero(), true, false, {},
                           Hole{5.0, 60.0}, {}, Hole{4.0, 20.0});
  EXPECT_FALSE(spent.last.dead_reckoning);
  EXPECT_TRUE(spent.last.lost);
}

TEST(VisualInertialOdometry, AHandfulOfFeaturesIsNoAid) {
  // r933: six points of the imager's noise read as health to a landed
  // vehicle. A frame with a handful of features is a dark one.
  const VisualInertialOdometryConfig config = testConfig();
  const Flight sparse =
      fly(config, 8.0, Eigen::Vector3d::Zero(), true, false, {}, {}, Hole{5.0, 9.0});
  EXPECT_FALSE(sparse.last.healthy);
  EXPECT_TRUE(sparse.last.dead_reckoning);
}

TEST(VisualInertialOdometry, TheRotorsDragBoundsADarkHoversDrift) {
  // A hover with nothing to see and an accelerometer 0.05 m/s^2 off across
  // the rotor axis: integrated, the bias is 2.5 m in 10 s; a hovering
  // multirotor's reading there is its drag, nothing while it stands, so the
  // drag's fusion reads the offset as the bias it is (r927, r932).
  const auto hover = [](const double rotor_drag_1ps) {
    VisualInertialOdometryConfig config = testConfig();
    config.rotor_drag_1ps = rotor_drag_1ps;
    VisualInertialOdometry odometry{config};
    odometry.initialize(0, Eigen::Vector3d::Zero(), 0.0);
    VisualInertialEstimate estimate;
    std::int64_t imu_stamp = 0;
    for (std::int64_t frame = 1; frame <= 100; ++frame) {
      const std::int64_t stamp = frame * kFramePeriodNs;
      for (; imu_stamp <= stamp; imu_stamp += kImuPeriodNs) {
        odometry.addImu(VisualInertialImuSample{
            .stamp_ns = imu_stamp,
            .gyro_radps = Eigen::Vector3d::Zero(),
            .accelerometer_mps2 = {0.05, 0.0, -config.gravity_mps2}});
      }
      estimate = odometry.addFrame(stamp, {});
    }
    return estimate.position_ned_m.norm();
  };
  EXPECT_GT(hover(0.0), 2.0);
  EXPECT_LT(hover(0.106), 0.5);
}

TEST(VisualInertialOdometry, TheGroundHoldingATiltedBodyIsNoDrag) {
  // A body landed on a slope in the dark, 12 degrees from level: the ground's
  // friction holds it, and the accelerometer reads 2 m/s^2 across the rotor
  // axis for as long as it lies there. The drag model took that for a
  // velocity of 21 m/s (r1194) and ran the estimate 100 m away in 4 s; a
  // reading beyond any flight's drag is left out, and the IMU alone keeps
  // the resting body where it is.
  VisualInertialOdometryConfig config = testConfig();
  config.rotor_drag_1ps = 0.106;
  const double tilt = 12.0 * M_PI / 180.0;
  const Eigen::Vector3d resting{config.gravity_mps2 * std::sin(tilt), 0.0,
                                -config.gravity_mps2 * std::cos(tilt)};
  VisualInertialOdometry odometry{config};
  for (std::int64_t stamp = -400'000'000; stamp < 0; stamp += kImuPeriodNs) {
    odometry.addImu(VisualInertialImuSample{.stamp_ns = stamp,
                                            .gyro_radps = Eigen::Vector3d::Zero(),
                                            .accelerometer_mps2 = resting});
  }
  odometry.initialize(0, Eigen::Vector3d::Zero(), 0.0);
  VisualInertialEstimate estimate;
  std::int64_t imu_stamp = 0;
  for (std::int64_t frame = 1; frame <= 40; ++frame) {
    const std::int64_t stamp = frame * kFramePeriodNs;
    for (; imu_stamp <= stamp; imu_stamp += kImuPeriodNs) {
      odometry.addImu(VisualInertialImuSample{.stamp_ns = imu_stamp,
                                              .gyro_radps = Eigen::Vector3d::Zero(),
                                              .accelerometer_mps2 = resting});
    }
    estimate = odometry.addFrame(stamp, {});
  }
  EXPECT_LT(estimate.velocity_ned_mps.norm(), 0.5);
  EXPECT_LT(estimate.position_ned_m.norm(), 1.0);
}

TEST(VisualInertialOdometry, AnUncertainVelocityIsNotHealthy) {
  // The flying filter holds its least certain velocity within 0.2 m/s; a
  // bound below what features give it makes even a sighted flight unhealthy.
  VisualInertialOdometryConfig config = testConfig();
  const Flight sighted = fly(config, 10.0, Eigen::Vector3d::Zero(), true, false);
  ASSERT_TRUE(sighted.last.healthy);
  config.maximum_velocity_sigma_mps = 0.5 * sighted.last.weakest_velocity_sigma_mps;
  const Flight bounded = fly(config, 10.0, Eigen::Vector3d::Zero(), true, false);
  EXPECT_FALSE(bounded.last.healthy);
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

// The node declares the pose on a frame older than the stream's last sample
// by the frame's latency: the step from the frame's stamp to the sample after
// the ones read at rest is no hole, nothing in it went unobserved. Read as one
// it gave the heading, which no feature observes, 9.5 degrees of uncertainty
// for the whole flight (r1681), and every update walked it.
TEST(VisualInertialOdometry, TheFirstStepAfterTheDeclaredPoseIsNoHole) {
  const VisualInertialOdometryConfig config = testConfig();
  VisualInertialOdometry odometry{config};
  constexpr std::int64_t kLatencyNs{70'000'000};
  std::int64_t imu_stamp = 0;
  for (; imu_stamp <= kFramePeriodNs + kLatencyNs; imu_stamp += kImuPeriodNs) {
    odometry.addImu(Motion::imu(imu_stamp, Eigen::Vector3d::Zero()));
  }
  odometry.initialize(kFramePeriodNs, Motion::position(0.0), 0.0);
  for (; imu_stamp <= 2 * kFramePeriodNs; imu_stamp += kImuPeriodNs) {
    odometry.addImu(Motion::imu(imu_stamp, Eigen::Vector3d::Zero()));
  }
  const VisualInertialEstimate estimate = odometry.addFrame(2 * kFramePeriodNs, {});
  const double declared =
      config.initial_heading_sigma_rad * config.initial_heading_sigma_rad;
  EXPECT_LT(estimate.orientation_variance_rad2.z(), 1.01 * declared);
  EXPECT_LT(estimate.weakest_velocity_sigma_mps,
            2.0 * config.initial_velocity_sigma_mps);
}

TEST(VisualInertialOdometry, ThePoseBetweenFramesFollowsTheImu) {
  const VisualInertialOdometryConfig config = testConfig();
  std::mt19937 generator{7U};
  const std::vector<Eigen::Vector3d> points = landmarks(generator);
  VisualInertialOdometry odometry{config};
  odometry.initialize(0, Motion::position(0.0), 0.0);
  std::int64_t imu_stamp = 0;
  for (std::int64_t frame = 1; frame <= 100; ++frame) {
    const std::int64_t stamp = frame * kFramePeriodNs;
    for (; imu_stamp <= stamp; imu_stamp += kImuPeriodNs) {
      odometry.addImu(Motion::imu(imu_stamp, Eigen::Vector3d::Zero()));
    }
    static_cast<void>(
        odometry.addFrame(stamp, observe(config, static_cast<double>(stamp) * 1.0e-9,
                                         points, config.observation_noise, generator)));
  }
  // Eighty milliseconds past the last frame, with the IMU that far ahead.
  const std::int64_t ahead = 100 * kFramePeriodNs + 80'000'000;
  for (; imu_stamp <= ahead; imu_stamp += kImuPeriodNs) {
    odometry.addImu(Motion::imu(imu_stamp, Eigen::Vector3d::Zero()));
  }
  const VisualInertialEstimate between = odometry.predicted(ahead);
  const double t = static_cast<double>(ahead) * 1.0e-9;
  EXPECT_EQ(between.stamp_ns, ahead);
  EXPECT_TRUE(between.healthy);
  EXPECT_LT((between.position_ned_m - Motion::position(t)).norm(), 0.1);
  // The vehicle moved 0.12 m over those 80 ms: the pose is not the frame's.
  EXPECT_GT((between.position_ned_m - Motion::position(t - 0.08)).norm(), 0.05);
}

TEST(VisualInertialOdometry, AFramesCloneIsFoundByItsStampWhileItIsKept) {
  VisualInertialOdometryConfig config = testConfig();
  config.maximum_clones = 3U;
  VisualInertialOdometry odometry{config};
  odometry.initialize(0, Motion::position(0.0), 0.0);
  std::int64_t imu_stamp = 0;
  for (std::int64_t frame = 1; frame <= 5; ++frame) {
    const std::int64_t stamp = frame * kFramePeriodNs;
    for (; imu_stamp <= stamp; imu_stamp += kImuPeriodNs) {
      odometry.addImu(Motion::imu(imu_stamp, Eigen::Vector3d::Zero()));
    }
    static_cast<void>(odometry.addFrame(stamp, {}));
  }
  const std::optional<VisualInertialClonePose> last =
      odometry.clonePose(5 * kFramePeriodNs);
  ASSERT_TRUE(last.has_value());
  EXPECT_LT((last->position_ned_m - Motion::position(0.5)).norm(), 0.02);
  EXPECT_FALSE(odometry.clonePose(5 * kFramePeriodNs + 1).has_value());
  // Three clones are kept: the first frame's has left.
  EXPECT_FALSE(odometry.clonePose(kFramePeriodNs).has_value());
}

TEST(VisualInertialOdometry, AFrameBeforeTheDeclaredPoseIsNoEstimate) {
  VisualInertialOdometry odometry{testConfig()};
  EXPECT_FALSE(odometry.initialized());
  const VisualInertialEstimate estimate = odometry.addFrame(kFramePeriodNs, {});
  EXPECT_EQ(estimate.clones, 0U);
}

// Specification K28: the frames the node held through a hole reach the
// filter once the stream covers them, a frame period apart; the hole is one
// all the same (r1258, r1260 crossed 2 s that way, healthy).
TEST(VisualInertialOdometry, FramesHeldThroughAHoleDoNotBridgeIt) {
  const VisualInertialOdometryConfig config = testConfig();
  std::mt19937 generator{11U};
  const std::vector<Eigen::Vector3d> points = landmarks(generator);
  VisualInertialOdometry odometry{config};
  odometry.initialize(0, Motion::position(0.0), 0.0);
  const Hole hole{4.0, 4.0 + config.maximum_unaided_s + 1.0};
  std::int64_t imu_stamp = 0;
  const auto feedImuTo = [&](const std::int64_t until_ns) {
    for (; imu_stamp <= until_ns; imu_stamp += kImuPeriodNs) {
      if (!hole.contains(imu_stamp)) {
        odometry.addImu(Motion::imu(imu_stamp, Eigen::Vector3d::Zero()));
      }
    }
  };
  const auto frameAt = [&](const std::int64_t stamp) {
    const double t = static_cast<double>(stamp) * 1.0e-9;
    return odometry.addFrame(
        stamp, observe(config, t, points, config.observation_noise, generator));
  };
  std::int64_t stamp = kFramePeriodNs;
  for (; static_cast<double>(stamp) * 1.0e-9 < hole.from_s; stamp += kFramePeriodNs) {
    feedImuTo(stamp);
    static_cast<void>(frameAt(stamp));
  }
  ASSERT_FALSE(odometry.lost());
  // The stream resumes; the frames held through the hole follow. Before they
  // do, the pose predicted across the hole is no estimate at all (r1292).
  feedImuTo(static_cast<std::int64_t>((hole.to_s + 0.5) * 1.0e9));
  const VisualInertialEstimate across =
      odometry.predicted(static_cast<std::int64_t>((hole.to_s + 0.2) * 1.0e9));
  EXPECT_FALSE(across.healthy);
  EXPECT_FALSE(across.dead_reckoning);
  EXPECT_TRUE(across.lost);
  VisualInertialEstimate last;
  for (; static_cast<double>(stamp) * 1.0e-9 <= hole.to_s + 0.5;
       stamp += kFramePeriodNs) {
    last = frameAt(stamp);
  }
  EXPECT_TRUE(odometry.lost());
  EXPECT_FALSE(last.healthy);
}

// Specification K28: after a hole the filter cannot bridge, started again at
// a pose and velocity the autopilot would give, a few decimetres and tenths
// of a metre a second off, it is healthy again once features correct it,
// holds the velocity, and carries the prior's offset and no more.
TEST(VisualInertialOdometry, StartedAgainAfterAHoleTheFilterRecovers) {
  const VisualInertialOdometryConfig config = testConfig();
  std::mt19937 generator{7U};
  const std::vector<Eigen::Vector3d> points = landmarks(generator);
  VisualInertialOdometry odometry{config};
  odometry.initialize(0, Motion::position(0.0), 0.0);
  const Hole hole{5.0, 5.0 + config.maximum_unaided_s + 0.5};
  const auto frames = static_cast<std::int64_t>(12.0 * 1.0e9) / kFramePeriodNs;
  std::int64_t imu_stamp = 0;
  bool restarted = false;
  std::int64_t restarted_ns = 0;
  std::int64_t healthy_again_ns = 0;
  VisualInertialEstimate last;
  bool lost_after_restart = false;
  for (std::int64_t frame = 1; frame <= frames; ++frame) {
    const std::int64_t stamp = frame * kFramePeriodNs;
    for (; imu_stamp <= stamp; imu_stamp += kImuPeriodNs) {
      if (hole.contains(imu_stamp)) {
        continue;
      }
      odometry.addImu(Motion::imu(imu_stamp, Eigen::Vector3d::Zero()));
      // As the node starts it: on the stream's first sample after the hole,
      // at the autopilot's attitude's moment, which may be a few
      // milliseconds older than that sample (r1309).
      if (!restarted && static_cast<double>(imu_stamp) * 1.0e-9 >= hole.to_s) {
        restarted_ns = imu_stamp - 2'000'000;
        const double t = static_cast<double>(restarted_ns) * 1.0e-9;
        const Eigen::Vector3d velocity =
            (Motion::position(t + 0.01) - Motion::position(t - 0.01)) / 0.02;
        odometry.reinitialize(restarted_ns,
                              Motion::position(t) + Eigen::Vector3d{0.3, -0.2, 0.1},
                              velocity + Eigen::Vector3d{0.2, -0.1, 0.1},
                              Eigen::Quaterniond{Motion::bodyToNed(t)});
        restarted = true;
      }
    }
    // The node drops the frames held through the hole.
    if (hole.contains(stamp)) {
      continue;
    }
    const double t = static_cast<double>(stamp) * 1.0e-9;
    last = odometry.addFrame(
        stamp, observe(config, t, points, config.observation_noise, generator));
    if (restarted && last.healthy && healthy_again_ns == 0) {
      healthy_again_ns = stamp;
    }
    lost_after_restart = lost_after_restart || (restarted && last.lost);
  }
  ASSERT_TRUE(restarted);
  // Healthy from the next frame on, the restart's pose its fix (r1306: the
  // features' first update came too late for the autopilot).
  EXPECT_GT(healthy_again_ns, 0);
  EXPECT_LE(healthy_again_ns, restarted_ns + kFramePeriodNs);
  EXPECT_FALSE(lost_after_restart);
  EXPECT_TRUE(last.healthy);
  EXPECT_FALSE(last.lost);
  // The prior's own offset (0.37 m) stays, no feature can tell it; the
  // filter adds no more than a decimetre over the next five seconds.
  EXPECT_LT((last.position_ned_m - Motion::position(12.0)).norm(), 0.5);
  const Eigen::Vector3d velocity =
      (Motion::position(12.01) - Motion::position(11.99)) / 0.02;
  EXPECT_LT((last.velocity_ned_mps - velocity).norm(), 0.3);
  // The heading stays the one it was started with (r1299: turned 4.8 degrees).
  const Eigen::Matrix3d truth_turn = Motion::bodyToNed(12.0);
  const Eigen::Matrix3d estimate_turn = last.body_to_ned.toRotationMatrix();
  const double heading_error = std::atan2(estimate_turn(1, 0), estimate_turn(0, 0)) -
                               std::atan2(truth_turn(1, 0), truth_turn(0, 0));
  EXPECT_LT(std::abs(std::remainder(heading_error, 2.0 * M_PI)), 0.01);
}

} // namespace
} // namespace drone_city_nav
