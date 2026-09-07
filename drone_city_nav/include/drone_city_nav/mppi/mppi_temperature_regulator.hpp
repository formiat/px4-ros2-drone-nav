#pragma once

#include <cstddef>

namespace drone_city_nav::mppi {

// Keeps the softmax spread over a target share of the population.
//
// A fixed temperature against a population spread over thousands of cost units
// collapses the weights onto whichever sample happens to be best, and the
// weighted update degenerates into "best of N random". Scaling the temperature
// by a share of the mean cost excess tracks the spread's scale but not how many
// samples actually carry weight, which is the quantity that matters.
//
// The effective sample size, (sum w)^2 / sum w^2, is that quantity. It is
// measured from the weights the tick just produced and the temperature is
// nudged toward the target for the next one, so the loop settles within a few
// ticks and a single odd tick cannot swing it far.
struct MppiTemperatureRegulatorConfig {
  // Floor the temperature never goes below.
  float minimum_temperature{8.0F};
  // Share of the feasible population the softmax should spread over. Zero
  // disables the regulator and pins the temperature to the floor.
  float target_effective_sample_fraction{0.07F};
  // How far above the floor the regulator may take the temperature.
  float maximum_growth{1000.0F};
  // Largest factor one tick may change the temperature by, in either
  // direction.
  float maximum_step{2.0F};
};

// The share of `feasible_count` rollouts that carried weight. Zero when the
// weights are degenerate or the population is empty.
[[nodiscard]] float effectiveSampleFraction(float weight_sum, float weight_square_sum,
                                            std::size_t feasible_count) noexcept;

// The temperature for the next tick given the one used and the sample fraction
// it achieved.
[[nodiscard]] float regulatedTemperature(const MppiTemperatureRegulatorConfig& config,
                                         float temperature,
                                         float effective_sample_fraction) noexcept;

} // namespace drone_city_nav::mppi
