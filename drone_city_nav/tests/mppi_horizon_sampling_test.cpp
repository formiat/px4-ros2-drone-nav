#include "drone_city_nav/mppi/mppi_horizon_sampling.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav::mppi {
namespace {

TEST(MppiHorizonSamplingTest, KeepsNearHorizonAtFullRate) {
  const HorizonSamplingConfig config{2.0F, 2U};
  for (std::size_t step = 0U; step < 40U; ++step) {
    const HorizonCostSample sample = horizonCostSample(step, 120U, 0.05F, 0.4F, config);
    EXPECT_TRUE(sample.evaluate);
    EXPECT_EQ(sample.represented_steps, 1U);
  }
}

TEST(MppiHorizonSamplingTest, SamplesFarHorizonAtConfiguredStride) {
  const HorizonSamplingConfig config{2.0F, 2U};
  EXPECT_FALSE(horizonCostSample(40U, 120U, 0.05F, 0.4F, config).evaluate);
  const HorizonCostSample sample = horizonCostSample(41U, 120U, 0.05F, 0.4F, config);
  EXPECT_TRUE(sample.evaluate);
  EXPECT_EQ(sample.represented_steps, 2U);
}

TEST(MppiHorizonSamplingTest, AlwaysSamplesFinalPartialInterval) {
  const HorizonSamplingConfig config{2.0F, 2U};
  const HorizonCostSample sample = horizonCostSample(42U, 43U, 0.05F, 0.4F, config);
  EXPECT_TRUE(sample.evaluate);
  EXPECT_EQ(sample.represented_steps, 1U);
}

TEST(MppiHorizonSamplingTest, HeadProgressWindowExtendsFullRateRegion) {
  const HorizonSamplingConfig config{2.0F, 2U};
  EXPECT_TRUE(horizonCostSample(59U, 120U, 0.05F, 3.0F, config).evaluate);
  EXPECT_FALSE(horizonCostSample(60U, 120U, 0.05F, 3.0F, config).evaluate);
}

TEST(MppiHorizonSamplingTest, UnitStrideDisablesFarSampling) {
  const HorizonSamplingConfig config{0.0F, 1U};
  const HorizonCostSample sample = horizonCostSample(79U, 80U, 0.05F, 0.4F, config);
  EXPECT_TRUE(sample.evaluate);
  EXPECT_EQ(sample.represented_steps, 1U);
}

TEST(MppiHorizonSamplingTest, RejectsZeroStride) {
  EXPECT_FALSE(horizonSamplingConfigIsValid(HorizonSamplingConfig{2.0F, 0U}));
}

} // namespace
} // namespace drone_city_nav::mppi
