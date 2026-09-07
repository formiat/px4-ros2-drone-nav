#include "drone_city_nav/mppi/mppi_control_arbitration.hpp"
#include "drone_city_nav/mppi/mppi_temperature_regulator.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav::mppi {
namespace {

TEST(MppiTemperatureRegulatorTest, EffectiveSampleFractionCountsTheWeightThatCarries) {
  // One sample carrying everything: one out of the population.
  EXPECT_FLOAT_EQ(effectiveSampleFraction(1.0F, 1.0F, 100U), 0.01F);
  // A hundred equal weights: the whole population carries.
  EXPECT_FLOAT_EQ(effectiveSampleFraction(100.0F, 100.0F, 100U), 1.0F);
  // Degenerate inputs measure nothing.
  EXPECT_FLOAT_EQ(effectiveSampleFraction(0.0F, 0.0F, 100U), 0.0F);
  EXPECT_FLOAT_EQ(effectiveSampleFraction(1.0F, 1.0F, 0U), 0.0F);
}

TEST(MppiTemperatureRegulatorTest, TheTemperatureMovesTowardTheTargetAndSettles) {
  const MppiTemperatureRegulatorConfig config{
      .minimum_temperature = 8.0F,
      .target_effective_sample_fraction = 0.07F,
      .maximum_growth = 1000.0F,
      .maximum_step = 2.0F,
  };

  // Collapsed weights: the temperature rises.
  EXPECT_GT(regulatedTemperature(config, 8.0F, 0.001F), 8.0F);
  // Too flat: it falls, but never below the floor.
  EXPECT_LT(regulatedTemperature(config, 100.0F, 0.9F), 100.0F);
  EXPECT_GE(regulatedTemperature(config, 8.0F, 0.9F), 8.0F);
  // On target: it stays.
  EXPECT_FLOAT_EQ(regulatedTemperature(config, 50.0F, 0.07F), 50.0F);
  // One tick cannot move it further than the step allows.
  EXPECT_LE(regulatedTemperature(config, 50.0F, 1.0e-6F), 50.0F * 2.0F);
  EXPECT_GE(regulatedTemperature(config, 50.0F, 1.0F), 50.0F / 2.0F);
  // The ceiling holds.
  EXPECT_LE(regulatedTemperature(config, 8000.0F, 1.0e-6F), 8.0F * 1000.0F);
}

TEST(MppiTemperatureRegulatorTest, ADisabledRegulatorPinsTheFloor) {
  const MppiTemperatureRegulatorConfig disabled{
      .minimum_temperature = 8.0F,
      .target_effective_sample_fraction = 0.0F,
  };
  EXPECT_FLOAT_EQ(regulatedTemperature(disabled, 500.0F, 0.5F), 8.0F);
}

TEST(MppiControlArbitrationTest, TakingTheUpdateOverIsEarnedAndHandingItBackIsNot) {
  MppiControlArbitrationState state;
  const MppiControlArbitrationInput preferable{.candidate_preferable = true,
                                               .switch_ticks = 3U};
  const MppiControlArbitrationInput not_preferable{.candidate_preferable = false,
                                                   .switch_ticks = 3U};

  EXPECT_FALSE(stickyRouteCandidatePreference(state, preferable));
  EXPECT_FALSE(stickyRouteCandidatePreference(state, preferable));
  EXPECT_TRUE(stickyRouteCandidatePreference(state, preferable));

  // Once it owns the update, a preferable candidate keeps it without waiting
  // again.
  state.previous_selection = MppiControlSelection::kRouteDirectedCandidate;
  EXPECT_TRUE(stickyRouteCandidatePreference(state, preferable));
  // Losing the preference hands the update back at once.
  EXPECT_FALSE(stickyRouteCandidatePreference(state, not_preferable));
  // And the count starts over.
  state.previous_selection = MppiControlSelection::kWeightedUpdate;
  EXPECT_FALSE(stickyRouteCandidatePreference(state, preferable));
  EXPECT_FALSE(stickyRouteCandidatePreference(state, preferable));
  EXPECT_TRUE(stickyRouteCandidatePreference(state, preferable));
}

} // namespace
} // namespace drone_city_nav::mppi
