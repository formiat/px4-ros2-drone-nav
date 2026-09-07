#pragma once

#include "drone_city_nav/executed_horizon_clearance_3d.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"

#include <optional>
#include <string>

namespace drone_city_nav {

struct ProductionMppiExecutionPublication;
struct RollingRouteTelemetryObservation3D;

namespace detail {

// The clearance the speed policy answered to, as JSON fields: the first
// constrained sample, the minimum, and every constrained sample in path order.
[[nodiscard]] std::string executedHorizonClearanceJsonFields(
    const std::optional<ExecutedHorizonClearance3D>& clearance);

// One rollout's weighted cost terms as a JSON object.
[[nodiscard]] std::string rolloutCostTermsJson(const mppi::RolloutCostTerms& terms);

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
