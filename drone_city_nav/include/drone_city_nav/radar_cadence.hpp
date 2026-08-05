#pragma once

#include <cstdint>
#include <random>

namespace drone_city_nav {

struct RadarCadenceConfig {
  double minimum_interval_s{0.1};
  double maximum_interval_s{3.0};
  double initial_interval_s{0.1};
  double maximum_step_s{0.25};
  double step_correlation{0.85};
  std::uint64_t random_seed{42U};
};

class CorrelatedRadarCadence final {
public:
  explicit CorrelatedRadarCadence(const RadarCadenceConfig& config = {});

  [[nodiscard]] double nextIntervalSeconds();

  [[nodiscard]] double currentIntervalSeconds() const noexcept {
    return current_interval_s_;
  }

private:
  RadarCadenceConfig config_{};
  std::mt19937_64 random_engine_;
  std::uniform_real_distribution<double> noise_distribution_{-1.0, 1.0};
  double current_interval_s_{0.1};
  double current_step_s_{0.0};
};

} // namespace drone_city_nav
