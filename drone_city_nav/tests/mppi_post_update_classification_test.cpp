#include "drone_city_nav/mppi/mppi_post_update_classification.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace drone_city_nav::mppi {
namespace {

[[nodiscard]] MppiEligibleRiskContract eligibleContract() {
  return MppiEligibleRiskContract{
      .available = true,
      .tier = RiskTier::kPlanning,
      .best_critical_exposure_m = 0.0F,
      .best_planning_exposure_m = 2.0F,
      .critical_exposure_tolerance_m = 0.5F,
      .planning_exposure_tolerance_m = 1.0F,
      .weight_sum = 12.0F,
  };
}

[[nodiscard]] MppiPostUpdateObservation safeObservation() {
  return MppiPostUpdateObservation{
      .tier = RiskTier::kPlanning,
      .raw_collision = false,
      .known_solid_collision = false,
      .critical_exposure_m = 0.5F,
      .planning_exposure_m = 3.0F,
  };
}

TEST(MppiPostUpdateClassification, PreservesEligibleContractAtToleranceLimits) {
  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(eligibleContract(), safeObservation());

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kPreserved);
  EXPECT_TRUE(result.contract_preserved);
  EXPECT_FLOAT_EQ(result.critical_exposure_limit_m, 0.5F);
  EXPECT_FLOAT_EQ(result.planning_exposure_limit_m, 3.0F);
}

TEST(MppiPostUpdateClassification, ReportsMissingEligibleRollout) {
  MppiEligibleRiskContract contract = eligibleContract();
  contract.available = false;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(contract, safeObservation());

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kNoEligibleRollout);
  EXPECT_FALSE(result.contract_preserved);
}

TEST(MppiPostUpdateClassification, ReportsInvalidContractMetrics) {
  MppiEligibleRiskContract contract = eligibleContract();
  contract.best_planning_exposure_m = std::numeric_limits<float>::quiet_NaN();

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(contract, safeObservation());

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kInvalidMetrics);
}

TEST(MppiPostUpdateClassification, ReportsRawCollision) {
  MppiPostUpdateObservation observation = safeObservation();
  observation.raw_collision = true;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(eligibleContract(), observation);

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kRawCollision);
}

TEST(MppiPostUpdateClassification, ReportsKnownSolidCollision) {
  MppiPostUpdateObservation observation = safeObservation();
  observation.known_solid_collision = true;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(eligibleContract(), observation);

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kKnownSolidCollision);
}

TEST(MppiPostUpdateClassification, ReportsRiskTierDegradation) {
  MppiPostUpdateObservation observation = safeObservation();
  observation.tier = RiskTier::kCritical;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(eligibleContract(), observation);

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kRiskTierDegraded);
}

TEST(MppiPostUpdateClassification, ReportsCriticalExposureViolation) {
  MppiPostUpdateObservation observation = safeObservation();
  observation.critical_exposure_m = 0.51F;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(eligibleContract(), observation);

  EXPECT_EQ(result.classification,
            MppiPostUpdateClassification::kCriticalExposureExceeded);
}

TEST(MppiPostUpdateClassification, ReportsPlanningExposureViolation) {
  MppiPostUpdateObservation observation = safeObservation();
  observation.planning_exposure_m = 3.01F;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(eligibleContract(), observation);

  EXPECT_EQ(result.classification,
            MppiPostUpdateClassification::kPlanningExposureExceeded);
}

} // namespace
} // namespace drone_city_nav::mppi
