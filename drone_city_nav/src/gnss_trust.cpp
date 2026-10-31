#include "drone_city_nav/gnss_trust.hpp"

#include <cmath>

namespace drone_city_nav {

GnssTrustVerdict GnssTrust::read(const GnssFixReading& reading) noexcept {
  GnssTrustVerdict verdict;
  const auto fails = [](const double value, const double gate) {
    return !std::isfinite(value) || value > gate;
  };
  if (!reading.valid) {
    verdict.reason = "no_fix";
  } else if (fails(reading.clock_step_s, config_.clock_step_gate_s) ||
             reading.clock_step_s < 0.0) {
    verdict.reason = "clock";
  } else if (fails(reading.position_step_m, config_.jump_gate_m)) {
    verdict.reason = "jump";
  } else if (fails(reading.self_disagreement_mps, config_.self_gate_mps)) {
    verdict.reason = "self";
  } else if (fails(reading.position_residual_normalised, config_.position_gate)) {
    verdict.reason = "position";
  } else if (fails(reading.velocity_residual_normalised, config_.velocity_gate)) {
    verdict.reason = "velocity";
  } else {
    verdict.agreeing = true;
  }
  if (!verdict.agreeing) {
    agreeing_ = false;
    trusted_ = false;
    return verdict;
  }
  if (!agreeing_) {
    agreeing_ = true;
    agreement_since_ns_ = reading.stamp_ns;
  }
  verdict.agreeing_s =
      static_cast<double>(reading.stamp_ns - agreement_since_ns_) * 1.0e-9;
  trusted_ = verdict.agreeing_s >= config_.window_s;
  verdict.trusted = trusted_;
  return verdict;
}

} // namespace drone_city_nav
