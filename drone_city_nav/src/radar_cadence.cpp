#include "drone_city_nav/radar_cadence.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_city_nav {

CorrelatedRadarCadence::CorrelatedRadarCadence(const RadarCadenceConfig& config)
    : config_{config},
      random_engine_{config.random_seed},
      current_interval_s_{config.initial_interval_s} {
  const double interval_range = config_.maximum_interval_s - config_.minimum_interval_s;
  if (!(config_.minimum_interval_s > 0.0) || !(interval_range > 0.0) ||
      config_.initial_interval_s < config_.minimum_interval_s ||
      config_.initial_interval_s > config_.maximum_interval_s ||
      !(config_.maximum_step_s > 0.0) || config_.maximum_step_s > interval_range ||
      config_.step_correlation < 0.0 || config_.step_correlation >= 1.0 ||
      !(config_.track_interval_s > 0.0)) {
    throw std::invalid_argument{"invalid radar cadence configuration"};
  }
  current_step_s_ = std::min(config_.maximum_step_s, interval_range * 0.1);
}

double CorrelatedRadarCadence::nextIntervalSeconds() {
  const double result = current_interval_s_;
  const double innovation =
      noise_distribution_(random_engine_) * config_.maximum_step_s;
  current_step_s_ = std::clamp(config_.step_correlation * current_step_s_ +
                                   (1.0 - config_.step_correlation) * innovation,
                               -config_.maximum_step_s, config_.maximum_step_s);
  double candidate = current_interval_s_ + current_step_s_;
  if (candidate > config_.maximum_interval_s) {
    candidate = config_.maximum_interval_s - (candidate - config_.maximum_interval_s);
    current_step_s_ = -std::abs(current_step_s_);
  } else if (candidate < config_.minimum_interval_s) {
    candidate = config_.minimum_interval_s + (config_.minimum_interval_s - candidate);
    current_step_s_ = std::abs(current_step_s_);
  }
  current_interval_s_ =
      std::clamp(candidate, config_.minimum_interval_s, config_.maximum_interval_s);
  return result;
}

double CorrelatedRadarCadence::nextIntervalSeconds(const bool track_mode) {
  return track_mode ? config_.track_interval_s : nextIntervalSeconds();
}

} // namespace drone_city_nav
