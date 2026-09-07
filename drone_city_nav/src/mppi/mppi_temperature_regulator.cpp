#include "drone_city_nav/mppi/mppi_temperature_regulator.hpp"

#include <algorithm>
#include <cmath>

namespace drone_city_nav::mppi {

float effectiveSampleFraction(const float weight_sum, const float weight_square_sum,
                              const std::size_t feasible_count) noexcept {
  if (feasible_count == 0U || !std::isfinite(weight_sum) ||
      !std::isfinite(weight_square_sum) || !(weight_square_sum > 0.0F) ||
      !(weight_sum > 0.0F)) {
    return 0.0F;
  }
  const float fraction = (weight_sum * weight_sum) /
                         (weight_square_sum * static_cast<float>(feasible_count));
  return std::clamp(fraction, 0.0F, 1.0F);
}

float regulatedTemperature(const MppiTemperatureRegulatorConfig& config,
                           const float temperature,
                           const float effective_sample_fraction) noexcept {
  const float floor_temperature = std::max(config.minimum_temperature, 0.0F);
  if (!(config.target_effective_sample_fraction > 0.0F) ||
      !(config.maximum_growth >= 1.0F) || !(config.maximum_step > 1.0F) ||
      !std::isfinite(temperature) || !(temperature > 0.0F)) {
    return floor_temperature;
  }
  const float ceiling = floor_temperature * config.maximum_growth;
  if (!(effective_sample_fraction > 0.0F)) {
    // The weights carry no measurable spread at all: raise the temperature by
    // a full step so the next tick has something to measure.
    return std::clamp(temperature * config.maximum_step, floor_temperature, ceiling);
  }
  // Under an exponential family the sample fraction moves roughly with the
  // square of the temperature, so the square root of the ratio is the step
  // that lands on the target without overshooting it.
  const float ratio =
      config.target_effective_sample_fraction / effective_sample_fraction;
  const float step =
      std::clamp(std::sqrt(ratio), 1.0F / config.maximum_step, config.maximum_step);
  return std::clamp(temperature * step, floor_temperature, ceiling);
}

} // namespace drone_city_nav::mppi
