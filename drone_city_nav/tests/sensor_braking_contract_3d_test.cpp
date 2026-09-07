#include "drone_city_nav/sensor_braking_contract_3d.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace drone_city_nav {
namespace {

TEST(SensorBrakingContract3DTest,
     MaximumSpeedSaturatesTheCompleteDetectionRangeInequality) {
  const SensorBrakingContract3D contract{
      .guaranteed_detection_range_m = 30.0,
      .maximum_evidence_age_s = 1.0,
      .physical_margin_m = 3.0,
      .maximum_horizontal_acceleration_mps2 = 5.0,
      .maximum_vertical_acceleration_mps2 = 5.0,
      .maximum_control_jerk_mps3 = 12.0,
  };
  const StoppingCapability capability{
      .maximum_commanded_horizontal_deceleration_mps2 = 4.0,
      .guaranteed_horizontal_deceleration_mps2 = 4.0,
      .guaranteed_vertical_deceleration_mps2 = 2.0,
      .reaction_latency_s = 0.1,
  };

  const double maximum_speed_mps =
      sensorBrakingMaximumSpeedMps(contract, capability, 20.0);
  const SensorBrakingAssessment3D boundary =
      assessSensorBrakingContract3D(contract, capability, maximum_speed_mps);

  ASSERT_GT(maximum_speed_mps, 0.0);
  ASSERT_LT(maximum_speed_mps, 20.0);
  EXPECT_TRUE(boundary.accepted());
  EXPECT_NEAR(boundary.total_latency_s, 1.1, 1.0e-12);
  EXPECT_NEAR(boundary.latency_distance_m, maximum_speed_mps * 1.1, 1.0e-12);
  EXPECT_GT(boundary.stopping_distance_m, maximum_speed_mps * maximum_speed_mps / 4.0);
  EXPECT_NEAR(boundary.required_detection_range_m,
              boundary.guaranteed_detection_range_m, 1.0e-10);
  EXPECT_FALSE(
      assessSensorBrakingContract3D(contract, capability, maximum_speed_mps + 1.0e-3)
          .accepted());
}

TEST(SensorBrakingContract3DTest, EvidenceFreshnessBudgetReducesSafeSpeed) {
  SensorBrakingContract3D fresh_contract;
  fresh_contract.maximum_evidence_age_s = 0.1;
  SensorBrakingContract3D old_contract = fresh_contract;
  old_contract.maximum_evidence_age_s = 1.0;
  const StoppingCapability capability;

  const double fresh_limit_mps =
      sensorBrakingMaximumSpeedMps(fresh_contract, capability, 20.0);
  const double old_limit_mps =
      sensorBrakingMaximumSpeedMps(old_contract, capability, 20.0);

  EXPECT_GT(fresh_limit_mps, old_limit_mps);
}

TEST(SensorBrakingContract3DTest, ProductionContractAdmitsCruiseButNotAbsoluteLimit) {
  const SensorBrakingContract3D contract{
      .guaranteed_detection_range_m = 30.0,
      .maximum_evidence_age_s = 1.0,
      .physical_margin_m = 3.0,
      .maximum_horizontal_acceleration_mps2 = 4.0,
      .maximum_vertical_acceleration_mps2 = 4.0,
      .maximum_control_jerk_mps3 = 12.0,
  };
  const StoppingCapability capability{
      .maximum_commanded_horizontal_deceleration_mps2 = 4.0,
      .guaranteed_horizontal_deceleration_mps2 = 4.0,
      .guaranteed_vertical_deceleration_mps2 = 2.0,
      .reaction_latency_s = 0.1,
  };

  const double limit_mps = sensorBrakingMaximumSpeedMps(contract, capability, 10.0);

  EXPECT_GT(limit_mps, 5.0);
  EXPECT_LT(limit_mps, 10.0);
  EXPECT_TRUE(assessSensorBrakingContract3D(contract, capability, 5.0).accepted());
  EXPECT_FALSE(assessSensorBrakingContract3D(contract, capability, 10.0).accepted());
}

TEST(SensorBrakingContract3DTest,
     WeakestThreeDimensionalGuaranteedDecelerationGoverns) {
  const SensorBrakingContract3D contract;
  StoppingCapability weak_vertical;
  weak_vertical.guaranteed_vertical_deceleration_mps2 = 1.0;
  StoppingCapability balanced = weak_vertical;
  balanced.guaranteed_vertical_deceleration_mps2 =
      balanced.guaranteed_horizontal_deceleration_mps2;

  const double weak_limit_mps =
      sensorBrakingMaximumSpeedMps(contract, weak_vertical, 20.0);
  const double balanced_limit_mps =
      sensorBrakingMaximumSpeedMps(contract, balanced, 20.0);

  EXPECT_GT(balanced_limit_mps, weak_limit_mps);
}

TEST(SensorBrakingContract3DTest,
     WorstForwardAccelerationAndJerkLimitedReversalRemainConservative) {
  SensorBrakingContract3D nominal;
  nominal.maximum_horizontal_acceleration_mps2 = 1.0;
  nominal.maximum_vertical_acceleration_mps2 = 1.0;
  nominal.maximum_control_jerk_mps3 = 20.0;
  SensorBrakingContract3D stronger_forward_acceleration = nominal;
  stronger_forward_acceleration.maximum_horizontal_acceleration_mps2 = 6.0;
  stronger_forward_acceleration.maximum_vertical_acceleration_mps2 = 6.0;
  SensorBrakingContract3D slower_reversal = nominal;
  slower_reversal.maximum_control_jerk_mps3 = 2.0;
  const StoppingCapability capability;

  const double nominal_limit_mps =
      sensorBrakingMaximumSpeedMps(nominal, capability, 20.0);
  const double acceleration_limit_mps =
      sensorBrakingMaximumSpeedMps(stronger_forward_acceleration, capability, 20.0);
  const double jerk_limit_mps =
      sensorBrakingMaximumSpeedMps(slower_reversal, capability, 20.0);

  EXPECT_GT(nominal_limit_mps, acceleration_limit_mps);
  EXPECT_GT(nominal_limit_mps, jerk_limit_mps);
}

TEST(SensorBrakingContract3DTest, LevelFlightAnswersToTheHorizontalAxisAlone) {
  // Along a level direction the guaranteed deceleration is the horizontal one
  // and the acceleration still carried is the horizontal one; a pure climb
  // answers to the weaker vertical deceleration. The worst direction admits
  // no more than either, and the cap every rollout obeys is that worst case.
  const SensorBrakingContract3D contract{
      .guaranteed_detection_range_m = 30.0,
      .maximum_evidence_age_s = 0.25,
      .physical_margin_m = 3.0,
      .maximum_horizontal_acceleration_mps2 = 4.0,
      .maximum_vertical_acceleration_mps2 = 4.0,
      .maximum_control_jerk_mps3 = 12.0,
  };
  const StoppingCapability capability{
      .maximum_commanded_horizontal_deceleration_mps2 = 4.0,
      .guaranteed_horizontal_deceleration_mps2 = 4.0,
      .guaranteed_vertical_deceleration_mps2 = 2.0,
      .reaction_latency_s = 0.1,
  };

  const double level_mps =
      sensorBrakingMaximumSpeedMps(contract, capability, 20.0, Vec3{1.0, 0.0, 0.0});
  const double climb_mps =
      sensorBrakingMaximumSpeedMps(contract, capability, 20.0, Vec3{0.0, 0.0, 1.0});
  const double worst_mps = sensorBrakingMaximumSpeedMps(contract, capability, 20.0);

  EXPECT_GT(level_mps, climb_mps);
  EXPECT_LE(worst_mps, climb_mps + 1.0e-9);
  EXPECT_LE(worst_mps, level_mps + 1.0e-9);
  EXPECT_GT(worst_mps, 5.0);
  // The stopping distance along the level direction is the horizontal axis
  // law, decelerating at 4 m/s^2 after accelerating at 4 m/s^2.
  const SensorBrakingAssessment3D level =
      assessSensorBrakingContract3D(contract, capability, 5.0, Vec3{1.0, 0.0, 0.0});
  const SensorBrakingAssessment3D climb =
      assessSensorBrakingContract3D(contract, capability, 5.0, Vec3{0.0, 0.0, 1.0});
  EXPECT_LT(level.stopping_distance_m, climb.stopping_distance_m);
}

TEST(SensorBrakingContract3DTest, EachDirectionAnswersToItsOwnAxes) {
  // Level flight is bounded by the horizontal axis alone and a climb by the
  // vertical one: weakening the vertical deceleration slows the climb and not
  // level flight, and raising the horizontal acceleration touches level flight
  // and not the climb. The vector sum of both accelerations used to be paired
  // with the weakest deceleration of either axis for every direction, so a
  // stronger horizontal acceleration slowed the contract in level flight.
  SensorBrakingContract3D contract;
  contract.maximum_horizontal_acceleration_mps2 = 4.0;
  contract.maximum_vertical_acceleration_mps2 = 4.0;
  StoppingCapability capability;
  capability.guaranteed_horizontal_deceleration_mps2 = 4.0;
  capability.maximum_commanded_horizontal_deceleration_mps2 = 4.0;
  capability.guaranteed_vertical_deceleration_mps2 = 2.0;
  StoppingCapability weaker_vertical = capability;
  weaker_vertical.guaranteed_vertical_deceleration_mps2 = 1.0;
  SensorBrakingContract3D stronger_horizontal = contract;
  stronger_horizontal.maximum_horizontal_acceleration_mps2 = 8.0;
  const Vec3 level{1.0, 0.0, 0.0};
  const Vec3 climb{0.0, 0.0, 1.0};

  EXPECT_NEAR(sensorBrakingMaximumSpeedMps(contract, weaker_vertical, 20.0, level),
              sensorBrakingMaximumSpeedMps(contract, capability, 20.0, level), 1.0e-9);
  EXPECT_LT(sensorBrakingMaximumSpeedMps(contract, weaker_vertical, 20.0, climb),
            sensorBrakingMaximumSpeedMps(contract, capability, 20.0, climb));
  EXPECT_NEAR(
      sensorBrakingMaximumSpeedMps(stronger_horizontal, capability, 20.0, climb),
      sensorBrakingMaximumSpeedMps(contract, capability, 20.0, climb), 1.0e-9);
  EXPECT_NE(sensorBrakingMaximumSpeedMps(stronger_horizontal, capability, 20.0, level),
            sensorBrakingMaximumSpeedMps(contract, capability, 20.0, level));
}

TEST(SensorBrakingContract3DTest, InvalidContractFailsClosed) {
  SensorBrakingContract3D invalid;
  invalid.physical_margin_m = invalid.guaranteed_detection_range_m;
  const StoppingCapability capability;

  const SensorBrakingAssessment3D assessment =
      assessSensorBrakingContract3D(invalid, capability, 1.0);

  EXPECT_FALSE(sensorBrakingContract3DIsValid(invalid, capability));
  EXPECT_DOUBLE_EQ(sensorBrakingMaximumSpeedMps(invalid, capability, 20.0), 0.0);
  EXPECT_FALSE(assessment.valid);
  EXPECT_FALSE(assessment.accepted());
}

} // namespace
} // namespace drone_city_nav
