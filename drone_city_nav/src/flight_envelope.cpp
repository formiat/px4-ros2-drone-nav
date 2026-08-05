#include "drone_city_nav/flight_envelope.hpp"

#include <algorithm>

namespace drone_city_nav {

FlightEnvelopeStatus
evaluateFlightEnvelopeAltitude(const double z_m,
                               const FlightEnvelopeConfig& config) noexcept {
  if (!std::isfinite(config.minimum_target_z_m) ||
      !std::isfinite(config.maximum_target_z_m) ||
      !(config.maximum_target_z_m > config.minimum_target_z_m)) {
    return FlightEnvelopeStatus::kInvalidConfiguration;
  }
  if (!std::isfinite(z_m)) {
    return FlightEnvelopeStatus::kNonFiniteAltitude;
  }
  if (z_m < config.minimum_target_z_m) {
    return FlightEnvelopeStatus::kBelowMinimum;
  }
  if (z_m >= config.maximum_target_z_m) {
    return FlightEnvelopeStatus::kAtOrAboveMaximum;
  }
  return FlightEnvelopeStatus::kValid;
}

bool insideFlightEnvelope(const double z_m,
                          const FlightEnvelopeConfig& config) noexcept {
  return evaluateFlightEnvelopeAltitude(z_m, config) == FlightEnvelopeStatus::kValid;
}

bool insideFlightEnvelope(const Point3& point,
                          const FlightEnvelopeConfig& config) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         insideFlightEnvelope(point.z, config);
}

std::optional<double>
clampToFlightEnvelope(const double z_m, const FlightEnvelopeConfig& config) noexcept {
  if (!std::isfinite(z_m) ||
      evaluateFlightEnvelopeAltitude(config.minimum_target_z_m, config) !=
          FlightEnvelopeStatus::kValid) {
    return std::nullopt;
  }
  const double maximum_valid_z =
      std::nextafter(config.maximum_target_z_m, config.minimum_target_z_m);
  return std::clamp(z_m, config.minimum_target_z_m, maximum_valid_z);
}

bool segmentInsideFlightEnvelope(const Point3& first, const Point3& second,
                                 const FlightEnvelopeConfig& config) noexcept {
  return insideFlightEnvelope(first, config) && insideFlightEnvelope(second, config);
}

const char* flightEnvelopeStatusName(const FlightEnvelopeStatus status) noexcept {
  switch (status) {
    case FlightEnvelopeStatus::kValid:
      return "valid";
    case FlightEnvelopeStatus::kInvalidConfiguration:
      return "invalid_configuration";
    case FlightEnvelopeStatus::kNonFiniteAltitude:
      return "non_finite_altitude";
    case FlightEnvelopeStatus::kBelowMinimum:
      return "below_minimum";
    case FlightEnvelopeStatus::kAtOrAboveMaximum:
      return "at_or_above_maximum";
  }
  return "unknown";
}

} // namespace drone_city_nav
