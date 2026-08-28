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
      .maximum_forward_acceleration_mps2 = 5.0,
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
      .maximum_forward_acceleration_mps2 = std::hypot(4.0, 4.0),
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
  nominal.maximum_forward_acceleration_mps2 = 1.0;
  nominal.maximum_control_jerk_mps3 = 20.0;
  SensorBrakingContract3D stronger_forward_acceleration = nominal;
  stronger_forward_acceleration.maximum_forward_acceleration_mps2 = 6.0;
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
