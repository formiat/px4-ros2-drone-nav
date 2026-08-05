#pragma once

#include "drone_city_nav/types.hpp"

#include <cmath>
#include <cstdint>

namespace drone_city_nav {

struct FlightEnvelopeConfig {
  double minimum_target_z_m{1.0};
  double maximum_target_z_m{32.0};
};

enum class FlightEnvelopeStatus : std::uint8_t {
  kValid,
  kInvalidConfiguration,
  kNonFiniteAltitude,
  kBelowMinimum,
  kAtOrAboveMaximum,
};

[[nodiscard]] FlightEnvelopeStatus
evaluateFlightEnvelopeAltitude(double z_m, const FlightEnvelopeConfig& config) noexcept;

[[nodiscard]] bool insideFlightEnvelope(double z_m,
                                        const FlightEnvelopeConfig& config) noexcept;

[[nodiscard]] bool insideFlightEnvelope(const Point3& point,
                                        const FlightEnvelopeConfig& config) noexcept;

[[nodiscard]] bool
segmentInsideFlightEnvelope(const Point3& first, const Point3& second,
                            const FlightEnvelopeConfig& config) noexcept;

[[nodiscard]] const char*
flightEnvelopeStatusName(FlightEnvelopeStatus status) noexcept;

} // namespace drone_city_nav
