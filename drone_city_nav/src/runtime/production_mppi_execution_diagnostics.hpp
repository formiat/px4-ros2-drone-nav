#pragma once

#include <string>

namespace drone_city_nav {

struct ProductionMppiExecutionPublication;
struct RollingRouteTelemetryObservation3D;

namespace detail {

[[nodiscard]] std::string
executionInfoFields(const ProductionMppiExecutionPublication& execution);

[[nodiscard]] std::string
executionJsonFields(const ProductionMppiExecutionPublication& execution);

[[nodiscard]] std::string
rollingRouteInfoFields(const RollingRouteTelemetryObservation3D& observation);

[[nodiscard]] std::string
rollingRouteJsonFields(const RollingRouteTelemetryObservation3D& observation);

} // namespace detail
} // namespace drone_city_nav
