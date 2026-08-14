#include "drone_city_nav/radar_cadence.hpp"
#include "drone_city_nav/radar_model.hpp"
#include "drone_city_nav/radar_target_tracker.hpp"
#include "drone_city_nav/radar_visibility.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] TimedVehicleState vehicleState(const Point3 position, const Vec3 velocity,
                                             const std::int64_t stamp_ns,
                                             const double heading_rad = 0.0) {
  return TimedVehicleState{
      .position = position,
      .velocity = velocity,
      .stamp_ns = stamp_ns,
      .heading_rad = heading_rad,
      .position_valid = true,
      .velocity_valid = true,
      .heading_valid = true,
      .armed = true,
      .airborne = true,
      .navigation_ready = true,
  };
}

TEST(RadarModel, PublishesOnlyRelativeSphericalMeasurement) {
  const TimedVehicleState radar =
      vehicleState(Point3{0.0, 0.0, 2.0}, Vec3{1.0, 0.0, 0.0}, 1'000'000'000LL,
                   std::numbers::pi / 2.0);
  const TimedVehicleState target =
      vehicleState(Point3{10.0, 0.0, 2.0}, Vec3{5.0, 2.0, 0.0}, 1'000'000'000LL);

  const auto detection = simulateIdealRadarDetection(radar, target, 7U);

  if (!detection.has_value()) {
    FAIL() << "ideal radar detection was unexpectedly rejected";
    return;
  }
  const RadarDetectionSample& detection_value = *detection;
  EXPECT_EQ(detection_value.detection_id, 7U);
  EXPECT_NEAR(detection_value.range_m, 10.0, 1.0e-9);
  EXPECT_NEAR(detection_value.azimuth_rad, -std::numbers::pi / 2.0, 1.0e-9);
  EXPECT_NEAR(detection_value.elevation_rad, 0.0, 1.0e-9);
  EXPECT_NEAR(detection_value.radial_velocity_mps, 4.0, 1.0e-9);

  const auto reconstructed = radarDetectionPositionMap(radar, detection_value);
  if (!reconstructed.has_value()) {
    FAIL() << "ideal radar position was unexpectedly rejected";
    return;
  }
  const Point3& reconstructed_value = *reconstructed;
  EXPECT_NEAR(reconstructed_value.x, target.position.x, 1.0e-9);
  EXPECT_NEAR(reconstructed_value.y, target.position.y, 1.0e-9);
  EXPECT_NEAR(reconstructed_value.z, target.position.z, 1.0e-9);
}

TEST(RadarCadence, IsDeterministicCorrelatedAndBounded) {
  const RadarCadenceConfig config{
      .minimum_interval_s = 0.1,
      .maximum_interval_s = 3.0,
      .initial_interval_s = 0.1,
      .maximum_step_s = 0.25,
      .step_correlation = 0.85,
      .track_interval_s = 0.05,
      .random_seed = 1234U,
  };
  CorrelatedRadarCadence first{config};
  CorrelatedRadarCadence second{config};
  double previous = first.nextIntervalSeconds();
  EXPECT_DOUBLE_EQ(previous, 0.1);
  EXPECT_DOUBLE_EQ(previous, second.nextIntervalSeconds());
  bool varied = false;
  for (std::size_t index = 0U; index < 500U; ++index) {
    const double interval = first.nextIntervalSeconds();
    EXPECT_DOUBLE_EQ(interval, second.nextIntervalSeconds());
    EXPECT_GE(interval, config.minimum_interval_s);
    EXPECT_LE(interval, config.maximum_interval_s);
    EXPECT_LE(std::abs(interval - previous), config.maximum_step_s + 1.0e-12);
    varied = varied || std::abs(interval - previous) > 1.0e-6;
    previous = interval;
  }
  EXPECT_TRUE(varied);
}

TEST(RadarCadence, UsesExplicitFastTrackModeWithoutRangePolicy) {
  CorrelatedRadarCadence cadence;

  EXPECT_DOUBLE_EQ(cadence.nextIntervalSeconds(true), 0.05);
  EXPECT_DOUBLE_EQ(cadence.nextIntervalSeconds(true), 0.05);
  EXPECT_GT(cadence.nextIntervalSeconds(false), 0.0);
}

TEST(RadarVisibility, UsesPhysicalOccupancyWithoutArtificialInflation) {
  OccupancyGrid3D occupancy{GridBounds3D{
      .origin_x = 0.0,
      .origin_y = 0.0,
      .origin_z = 0.0,
      .resolution_m = 1.0,
      .width_cells = 20,
      .height_cells = 20,
      .depth_cells = 10,
  }};
  occupancy.setOccupied(GridIndex3D{.x = 5, .y = 5, .z = 5});

  EXPECT_EQ(radarLineOfSightStatus(occupancy, Point3{1.5, 5.5, 5.5},
                                   Point3{9.5, 5.5, 5.5}, 0.25),
            RadarVisibilityStatus::kOccluded);
  EXPECT_EQ(radarLineOfSightStatus(occupancy, Point3{1.5, 4.5, 5.5},
                                   Point3{9.5, 4.5, 5.5}, 0.25),
            RadarVisibilityStatus::kVisible);
  EXPECT_EQ(radarLineOfSightStatus(occupancy, Point3{-1.0, 4.5, 5.5},
                                   Point3{9.5, 4.5, 5.5}, 0.25),
            RadarVisibilityStatus::kOutsideWorld);
}

TEST(RadarOwnshipHistory, InterpolatesPositionVelocityAndWrappedHeading) {
  RadarOwnshipHistory history;
  ASSERT_TRUE(history.add(
      vehicleState(Point3{0.0, 0.0, 1.0}, Vec3{2.0, 0.0, 0.0}, 1'000'000'000LL, 3.0)));
  ASSERT_TRUE(history.add(
      vehicleState(Point3{2.0, 2.0, 3.0}, Vec3{2.0, 2.0, 0.0}, 2'000'000'000LL, -3.0)));

  const auto sampled = history.sample(1'500'000'000LL);

  if (!sampled.has_value()) {
    FAIL() << "interpolated ownship state was unexpectedly unavailable";
    return;
  }
  const TimedVehicleState& sampled_value = *sampled;
  EXPECT_NEAR(sampled_value.position.x, 1.0, 1.0e-9);
  EXPECT_NEAR(sampled_value.position.y, 1.0, 1.0e-9);
  EXPECT_NEAR(sampled_value.velocity.y, 1.0, 1.0e-9);
  EXPECT_NEAR(std::abs(sampled_value.heading_rad), std::numbers::pi, 1.0e-9);
}

TEST(RadarTargetTracker, InitializesDirectThenEstimatesVelocityAtVariableDt) {
  RadarTargetTracker tracker;
  const auto measurement = [](const TimedVehicleState& ownship,
                              const TimedVehicleState& target) {
    const auto value = simulateIdealRadarDetection(ownship, target, 1U);
    EXPECT_TRUE(value.has_value());
    return value.value_or(RadarDetectionSample{});
  };

  const TimedVehicleState ownship1 = vehicleState(Point3{}, Vec3{}, 1'000'000'000LL);
  const TimedVehicleState target1 =
      vehicleState(Point3{10.0, 2.0, 5.0}, Vec3{2.0, 1.0, 0.0}, 1'000'000'000LL);
  const RadarTrackEstimate first =
      tracker.update(ownship1, measurement(ownship1, target1), ownship1.stamp_ns, 1U);
  ASSERT_TRUE(first.position_valid);
  EXPECT_FALSE(first.velocity_valid);

  const TimedVehicleState ownship2 = vehicleState(Point3{}, Vec3{}, 1'100'000'000LL);
  const TimedVehicleState target2 =
      vehicleState(Point3{10.2, 2.1, 5.0}, target1.velocity, 1'100'000'000LL);
  const RadarTrackEstimate second =
      tracker.update(ownship2, measurement(ownship2, target2), ownship2.stamp_ns, 2U);
  ASSERT_TRUE(second.velocity_valid);
  EXPECT_NEAR(second.velocity.x, 2.0, 1.0e-9);
  EXPECT_NEAR(second.velocity.y, 1.0, 1.0e-9);

  const TimedVehicleState ownship3 = vehicleState(Point3{}, Vec3{}, 4'100'000'000LL);
  const TimedVehicleState target3 =
      vehicleState(Point3{16.2, 5.1, 5.0}, target1.velocity, 4'100'000'000LL);
  const RadarTrackEstimate third =
      tracker.update(ownship3, measurement(ownship3, target3), ownship3.stamp_ns, 3U);
  ASSERT_TRUE(third.velocity_valid);
  EXPECT_NEAR(third.position.x, target3.position.x, 1.0e-9);
  EXPECT_NEAR(third.velocity.x, 2.0, 1.0e-9);
  EXPECT_NEAR(third.velocity.y, 1.0, 1.0e-9);
}

TEST(RadarTargetTracker, FiltersTangentialInnovationWithConstantVelocityModel) {
  RadarTargetTracker tracker{RadarTargetTrackerConfig{
      .maximum_update_interval_s = 4.0,
      .position_correction_gain = 1.0,
      .velocity_correction_gain = 0.25,
      .high_rate_velocity_correction_gain = 1.0,
      .maximum_ownship_stamp_error_s = 0.05,
      .track_id = 1U,
  }};
  const auto update = [&tracker](const TimedVehicleState& ownship,
                                 const TimedVehicleState& target,
                                 const std::uint64_t sequence) {
    const auto measurement = simulateIdealRadarDetection(ownship, target, 1U);
    EXPECT_TRUE(measurement.has_value());
    return tracker.update(ownship, measurement.value_or(RadarDetectionSample{}),
                          ownship.stamp_ns, sequence);
  };

  const TimedVehicleState ownship1 = vehicleState(Point3{}, Vec3{}, 1'000'000'000LL);
  const TimedVehicleState ownship2 = vehicleState(Point3{}, Vec3{}, 2'000'000'000LL);
  const TimedVehicleState ownship3 = vehicleState(Point3{}, Vec3{}, 3'000'000'000LL);
  const Vec3 target_velocity{0.0, 1.0, 0.0};
  ASSERT_TRUE(
      update(ownship1,
             vehicleState(Point3{100.0, 0.0, 0.0}, target_velocity, ownship1.stamp_ns),
             1U)
          .position_valid);
  ASSERT_TRUE(
      update(ownship2,
             vehicleState(Point3{100.0, 1.0, 0.0}, target_velocity, ownship2.stamp_ns),
             2U)
          .velocity_valid);

  const RadarTrackEstimate filtered = update(
      ownship3,
      vehicleState(Point3{100.0, 3.0, 0.0}, target_velocity, ownship3.stamp_ns), 3U);

  ASSERT_TRUE(filtered.velocity_valid);
  EXPECT_GT(filtered.velocity.y, 1.2);
  EXPECT_LT(filtered.velocity.y, 1.3);
  EXPECT_LT(filtered.velocity.y, 2.0);
}

TEST(RadarTargetTracker, AppliesFullVelocityCorrectionInTrackMode) {
  RadarTargetTracker tracker{RadarTargetTrackerConfig{
      .maximum_update_interval_s = 4.0,
      .position_correction_gain = 1.0,
      .velocity_correction_gain = 0.5,
      .high_rate_velocity_correction_gain = 1.0,
      .maximum_ownship_stamp_error_s = 0.05,
      .track_id = 1U,
  }};
  const TimedVehicleState ownship1 = vehicleState(Point3{}, Vec3{}, 1'000'000'000LL);
  const TimedVehicleState ownship2 = vehicleState(Point3{}, Vec3{}, 2'000'000'000LL);
  const TimedVehicleState ownship3 = vehicleState(Point3{}, Vec3{}, 3'000'000'000LL);
  const auto update =
      [&tracker](const TimedVehicleState& ownship, const TimedVehicleState& target,
                 const std::uint64_t sequence, const RadarTrackerUpdateMode mode) {
        const auto measurement = simulateIdealRadarDetection(ownship, target, 1U);
        EXPECT_TRUE(measurement.has_value());
        return tracker.update(ownship, measurement.value_or(RadarDetectionSample{}),
                              ownship.stamp_ns, sequence, mode);
      };

  ASSERT_TRUE(update(ownship1,
                     vehicleState(Point3{10.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0},
                                  ownship1.stamp_ns),
                     1U, RadarTrackerUpdateMode::kSearch)
                  .position_valid);
  ASSERT_TRUE(update(ownship2,
                     vehicleState(Point3{11.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0},
                                  ownship2.stamp_ns),
                     2U, RadarTrackerUpdateMode::kSearch)
                  .velocity_valid);
  const RadarTrackEstimate turned = update(
      ownship3,
      vehicleState(Point3{11.0, 1.0, 0.0}, Vec3{0.0, 1.0, 0.0}, ownship3.stamp_ns), 3U,
      RadarTrackerUpdateMode::kTrack);

  ASSERT_TRUE(turned.velocity_valid);
  EXPECT_DOUBLE_EQ(turned.velocity_correction_gain, 1.0);
  EXPECT_NEAR(turned.velocity.x, 0.0, 1.0e-9);
  EXPECT_NEAR(turned.velocity.y, 1.0, 1.0e-9);
}

} // namespace
} // namespace drone_city_nav
