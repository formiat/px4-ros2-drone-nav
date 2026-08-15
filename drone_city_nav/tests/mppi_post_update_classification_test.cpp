#include "drone_city_nav/mppi/mppi_post_update_classification.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav::mppi {
namespace {

[[nodiscard]] MppiFeasibilityContract feasibleContract() {
  return MppiFeasibilityContract{
      .available = true,
      .weight_sum = 12.0F,
  };
}

[[nodiscard]] MppiPostUpdateObservation safeObservation() {
  return MppiPostUpdateObservation{
      .raw_collision = false,
      .known_solid_collision = false,
  };
}

TEST(MppiPostUpdateClassification, AcceptsPhysicallyFeasibleSequence) {
  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(feasibleContract(), safeObservation());

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kPreserved);
  EXPECT_TRUE(result.executable);
}

TEST(MppiPostUpdateClassification, ReportsMissingFeasibleRollout) {
  MppiFeasibilityContract contract = feasibleContract();
  contract.available = false;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(contract, safeObservation());

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kNoFeasibleRollout);
  EXPECT_FALSE(result.executable);
}

TEST(MppiPostUpdateClassification, ReportsInvalidContractMetrics) {
  MppiFeasibilityContract contract = feasibleContract();
  contract.weight_sum = 0.0F;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(contract, safeObservation());

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kInvalidMetrics);
}

TEST(MppiPostUpdateClassification, ReportsRawCollision) {
  MppiPostUpdateObservation observation = safeObservation();
  observation.raw_collision = true;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(feasibleContract(), observation);

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kRawCollision);
}

TEST(MppiPostUpdateClassification, ReportsAltitudeEnvelopeViolation) {
  MppiPostUpdateObservation observation = safeObservation();
  observation.altitude_envelope_violation = true;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(feasibleContract(), observation);

  EXPECT_EQ(result.classification,
            MppiPostUpdateClassification::kAltitudeEnvelopeViolation);
  EXPECT_FALSE(result.executable);
}

TEST(MppiPostUpdateClassification, ReportsKnownSolidCollision) {
  MppiPostUpdateObservation observation = safeObservation();
  observation.known_solid_collision = true;

  const MppiPostUpdateClassificationResult result =
      classifyMppiPostUpdate(feasibleContract(), observation);

  EXPECT_EQ(result.classification, MppiPostUpdateClassification::kKnownSolidCollision);
}

} // namespace
} // namespace drone_city_nav::mppi
