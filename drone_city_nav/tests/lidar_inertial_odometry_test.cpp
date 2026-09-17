#include "drone_city_nav/lidar_inertial_odometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace drone_city_nav {
namespace {

// A room in NED: walls at x and y of +-10 m, the floor at z = 0 and the
// ceiling at z = -6, sampled every quarter metre.
[[nodiscard]] std::vector<Eigen::Vector3d> roomPoints() {
  std::vector<Eigen::Vector3d> points;
  for (double a = -10.0; a <= 10.0; a += 0.25) {
    for (double b = -6.0; b <= 0.0; b += 0.25) {
      points.emplace_back(10.0, a, b);
      points.emplace_back(-10.0, a, b);
      points.emplace_back(a, 10.0, b);
      points.emplace_back(a, -10.0, b);
    }
    for (double c = -10.0; c <= 10.0; c += 0.25) {
      points.emplace_back(a, c, 0.0);
      points.emplace_back(a, c, -6.0);
    }
  }
  return points;
}

// The room as the lidar sees it from a pose: every point in the body frame.
[[nodiscard]] std::vector<Eigen::Vector3d>
scanFrom(const std::vector<Eigen::Vector3d>& room, const Eigen::Vector3d& position,
         const Eigen::Quaterniond& body_to_ned) {
  std::vector<Eigen::Vector3d> scan;
  scan.reserve(room.size());
  const Eigen::Quaterniond ned_to_body = body_to_ned.conjugate();
  for (const Eigen::Vector3d& point : room) {
    scan.push_back(ned_to_body * (point - position));
  }
  return scan;
}

constexpr std::int64_t kSecond{1'000'000'000LL};

// At rest the accelerometer reads minus gravity in the body frame.
[[nodiscard]] LidarInertialImuSample
restSample(const std::int64_t stamp_ns, const Eigen::Quaterniond& body_to_ned,
           const Eigen::Vector3d& gyro = Eigen::Vector3d::Zero()) {
  return LidarInertialImuSample{.stamp_ns = stamp_ns,
                                .gyro_radps = gyro,
                                .accelerometer_mps2 =
                                    body_to_ned.conjugate() *
                                    Eigen::Vector3d{0.0, 0.0, -9.80665}};
}

TEST(LidarInertialOdometryTest, TracksAStraightFlightThroughARoom) {
  const std::vector<Eigen::Vector3d> room = roomPoints();
  LidarInertialOdometry odometry{LidarInertialOdometryConfig{}};
  const Eigen::Vector3d start{-3.0, 1.0, -2.0};
  const Eigen::Quaterniond level = Eigen::Quaterniond::Identity();
  odometry.initialize(0, start, 0.0);
  for (int i = 1; i <= 50; ++i) {
    odometry.addImu(restSample(i * kSecond / 100, level));
  }
  // Constant 1 m/s along x: the IMU sees only gravity, the scans see the walls
  // move; the estimate has to follow the scans.
  LidarInertialEstimate estimate;
  const std::int64_t first_scan = kSecond / 2;
  for (int k = 0; k <= 30; ++k) {
    const std::int64_t stamp = first_scan + k * kSecond / 10;
    const double seconds = 1.0e-9 * static_cast<double>(stamp - first_scan);
    const Eigen::Vector3d truth = start + Eigen::Vector3d{seconds, 0.0, 0.0};
    if (k > 0) {
      for (int i = 1; i <= 10; ++i) {
        odometry.addImu(restSample(stamp - kSecond / 10 + i * kSecond / 100, level));
      }
    }
    estimate = odometry.addScan(stamp, scanFrom(room, truth, level));
    ASSERT_TRUE(estimate.healthy)
        << "scan " << k << " matched " << estimate.matched_fraction << " residual "
        << estimate.residual_rms_m;
    EXPECT_LT((estimate.position_ned_m - truth).norm(), 0.15)
        << "scan " << k << " at " << estimate.position_ned_m.transpose();
  }
  EXPECT_LT((estimate.position_ned_m - (start + Eigen::Vector3d{3.0, 0.0, 0.0})).norm(),
            0.1);
  EXPECT_NEAR(estimate.velocity_ned_mps.x(), 1.0, 0.2);
  EXPECT_GT(estimate.keyframes, 3U);
  EXPECT_GT(estimate.information_per_point,
            LidarInertialOdometryConfig{}.minimum_information_per_point);
}

TEST(LidarInertialOdometryTest, TracksAYawTurnFromTheGyroscopeAndTheWalls) {
  const std::vector<Eigen::Vector3d> room = roomPoints();
  LidarInertialOdometry odometry{LidarInertialOdometryConfig{}};
  const Eigen::Vector3d position{0.0, 0.0, -2.0};
  odometry.initialize(0, position, 0.0);
  for (int i = 1; i <= 50; ++i) {
    odometry.addImu(restSample(i * kSecond / 100, Eigen::Quaterniond::Identity()));
  }
  const double rate = 0.3;
  LidarInertialEstimate estimate;
  const std::int64_t first_scan = kSecond / 2;
  for (int k = 0; k <= 20; ++k) {
    const std::int64_t stamp = first_scan + k * kSecond / 10;
    const double seconds = 1.0e-9 * static_cast<double>(stamp - first_scan);
    const Eigen::Quaterniond truth{
        Eigen::AngleAxisd{rate * seconds, Eigen::Vector3d::UnitZ()}};
    if (k > 0) {
      for (int i = 1; i <= 10; ++i) {
        const std::int64_t imu_stamp = stamp - kSecond / 10 + i * kSecond / 100;
        const double imu_seconds = 1.0e-9 * static_cast<double>(imu_stamp - first_scan);
        const Eigen::Quaterniond attitude{
            Eigen::AngleAxisd{rate * imu_seconds, Eigen::Vector3d::UnitZ()}};
        odometry.addImu(
            restSample(imu_stamp, attitude, Eigen::Vector3d{0.0, 0.0, rate}));
      }
    }
    estimate = odometry.addScan(stamp, scanFrom(room, position, truth));
    ASSERT_TRUE(estimate.healthy) << "scan " << k;
  }
  const double yaw =
      std::atan2(2.0 * (estimate.body_to_ned.w() * estimate.body_to_ned.z()),
                 1.0 - 2.0 * estimate.body_to_ned.z() * estimate.body_to_ned.z());
  EXPECT_NEAR(yaw, rate * 2.0, 0.03);
  EXPECT_LT((estimate.position_ned_m - position).norm(), 0.1);
}

// A corridor whose end walls are out of range leaves its own axis free: the
// registration observes nothing along it, says so, and the IMU carries the
// motion there. Flown at 3 m/s with a true IMU the estimate stays within
// half a metre over fifteen seconds; before this the position froze at the
// first scan's velocity and fell forty metres behind.
TEST(LidarInertialOdometryTest, ACorridorLeavesItsAxisToTheImu) {
  std::vector<Eigen::Vector3d> corridor;
  for (double x = -40.0; x <= 40.0; x += 0.25) {
    for (double z = -4.0; z <= 0.0; z += 0.25) {
      corridor.emplace_back(x, 3.0, z);
      corridor.emplace_back(x, -3.0, z);
    }
    for (double y = -3.0; y <= 3.0; y += 0.25) {
      corridor.emplace_back(x, y, 0.0);
      corridor.emplace_back(x, y, -4.0);
    }
  }
  LidarInertialOdometry odometry{LidarInertialOdometryConfig{}};
  const Eigen::Vector3d start{-30.0, 0.0, -2.0};
  const Eigen::Quaterniond level = Eigen::Quaterniond::Identity();
  odometry.initialize(0, start, 0.0);
  for (int i = 1; i <= 100; ++i) {
    odometry.addImu(restSample(i * kSecond / 100, level));
  }
  const double speed = 3.0;
  const std::int64_t first_scan = kSecond;
  LidarInertialEstimate estimate;
  std::size_t degenerate_scans = 0U;
  for (int k = 0; k <= 150; ++k) {
    const std::int64_t stamp = first_scan + k * kSecond / 10;
    const double seconds = 1.0e-9 * static_cast<double>(stamp - first_scan);
    // Two seconds of constant acceleration, then cruise.
    const double along =
        seconds < 2.0 ? 0.25 * speed * seconds * seconds : speed * (seconds - 1.0);
    if (k > 0) {
      for (int i = 1; i <= 10; ++i) {
        const std::int64_t imu_stamp = stamp - kSecond / 10 + i * kSecond / 100;
        const double imu_seconds = 1.0e-9 * static_cast<double>(imu_stamp - first_scan);
        LidarInertialImuSample sample = restSample(imu_stamp, level);
        sample.accelerometer_mps2.x() += imu_seconds < 2.0 ? speed / 2.0 : 0.0;
        odometry.addImu(sample);
      }
    }
    const Eigen::Vector3d truth = start + Eigen::Vector3d{along, 0.0, 0.0};
    std::vector<Eigen::Vector3d> scan;
    for (const Eigen::Vector3d& point : corridor) {
      if ((point - truth).norm() <= 30.0) {
        scan.push_back(point - truth);
      }
    }
    estimate = odometry.addScan(stamp, scan);
    ASSERT_TRUE(estimate.healthy) << "scan " << k;
    degenerate_scans += estimate.degenerate_axes > 0U ? 1U : 0U;
    EXPECT_LT((estimate.position_ned_m - truth).norm(), 0.6) << "scan " << k;
  }
  EXPECT_GT(degenerate_scans, 100U);
  EXPECT_NEAR(estimate.velocity_ned_mps.x(), speed, 0.2);
  EXPECT_GE(estimate.position_variance_m2.x(), 0.5);
  EXPECT_LT(estimate.position_variance_m2.y(), 0.1);
}

// A scan that matches nothing does not send the estimate away: the
// position holds at the last registered pose while the IMU keeps the
// attitude, and the next scan that fits registers again from there.
TEST(LidarInertialOdometryTest, ALostScanHoldsThePositionUntilOneRegistersAgain) {
  const std::vector<Eigen::Vector3d> room = roomPoints();
  LidarInertialOdometry odometry{LidarInertialOdometryConfig{}};
  const Eigen::Vector3d position{1.0, -2.0, -2.0};
  const Eigen::Quaterniond level = Eigen::Quaterniond::Identity();
  odometry.initialize(0, position, 0.0);
  for (int i = 1; i <= 50; ++i) {
    odometry.addImu(restSample(i * kSecond / 100, level));
  }
  std::int64_t stamp = kSecond / 2;
  for (int k = 0; k < 5; ++k, stamp += kSecond / 10) {
    ASSERT_TRUE(odometry.addScan(stamp, scanFrom(room, position, level)).healthy);
    for (int i = 1; i <= 10; ++i) {
      odometry.addImu(restSample(stamp + i * kSecond / 100, level));
    }
  }
  // Two seconds of scans from nowhere near the room, with an accelerometer
  // that reads a spurious push the whole time.
  const std::vector<Eigen::Vector3d> nowhere{{100.0, 100.0, 100.0},
                                             {101.0, 100.0, 100.0},
                                             {100.0, 101.0, 100.0},
                                             {100.0, 100.0, 101.0}};
  for (int k = 0; k < 20; ++k, stamp += kSecond / 10) {
    const LidarInertialEstimate lost = odometry.addScan(stamp, nowhere);
    EXPECT_FALSE(lost.healthy);
    EXPECT_LT((lost.position_ned_m - position).norm(), 0.2) << "scan " << k;
    for (int i = 1; i <= 10; ++i) {
      LidarInertialImuSample sample = restSample(stamp + i * kSecond / 100, level);
      sample.accelerometer_mps2 += Eigen::Vector3d{2.0, 0.0, 0.0};
      odometry.addImu(sample);
    }
  }
  const LidarInertialEstimate back =
      odometry.addScan(stamp, scanFrom(room, position, level));
  EXPECT_TRUE(back.healthy);
  EXPECT_LT((back.position_ned_m - position).norm(), 0.15);
}

TEST(LidarInertialOdometryTest,
     AnEmptyScanIsNotHealthyAndAnUninitializedEstimatorSaysSo) {
  LidarInertialOdometry odometry{LidarInertialOdometryConfig{}};
  EXPECT_FALSE(odometry.initialized());
  EXPECT_FALSE(odometry.addScan(kSecond, {}).healthy);
  odometry.initialize(0, Eigen::Vector3d::Zero(), 0.0);
  EXPECT_TRUE(odometry.initialized());
  EXPECT_FALSE(odometry.addScan(kSecond, {}).healthy);
}

} // namespace
} // namespace drone_city_nav
