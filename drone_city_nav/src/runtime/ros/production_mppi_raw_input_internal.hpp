#pragma once

#include "drone_city_nav/producer_epoch_admission.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace drone_city_nav::production_mppi_raw_input_detail {

[[nodiscard]] inline ProducerEpochAdmissionConfig
producerEpochConfig(const double maximum_esdf_age_ms,
                    const double stale_esdf_execution_window_ms) noexcept {
  constexpr double kNanosecondsPerMillisecond{1.0e6};
  const double maximum_age_ms = maximum_esdf_age_ms + stale_esdf_execution_window_ms;
  if (!std::isfinite(maximum_age_ms) || maximum_age_ms <= 0.0) {
    return ProducerEpochAdmissionConfig{
        .maximum_observation_age_ns = 0,
        .maximum_confirmation_interval_ns = 0,
    };
  }
  constexpr std::int64_t kMaximumSafeAgeNanoseconds =
      std::numeric_limits<std::int64_t>::max() / 4;
  const double bounded_nanoseconds =
      std::min(maximum_age_ms * kNanosecondsPerMillisecond,
               static_cast<double>(kMaximumSafeAgeNanoseconds));
  const std::int64_t maximum_age_ns = static_cast<std::int64_t>(bounded_nanoseconds);
  return ProducerEpochAdmissionConfig{
      .maximum_observation_age_ns = maximum_age_ns,
      .maximum_confirmation_interval_ns = maximum_age_ns,
  };
}

} // namespace drone_city_nav::production_mppi_raw_input_detail
