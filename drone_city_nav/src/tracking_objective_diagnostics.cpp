#include "tracking_objective_diagnostics.hpp"

#include "drone_city_nav/mppi_debug_markers.hpp"

#include <algorithm>
#include <sstream>

#include "production_mppi_node.hpp"

namespace drone_city_nav::detail {

std::string
trackingObjectiveJsonFields(const ProductionNavigationObjective* navigation_objective,
                            const Point3& resolved_position,
                            const std::int64_t now_ns) {
  const ProductionTrackingObjective* objective =
      navigation_objective != nullptr && navigation_objective->tracking.has_value()
          ? &navigation_objective->tracking.value()
          : nullptr;
  const ProductionTrackingObjective empty_objective;
  const ProductionTrackingObjective& data =
      objective != nullptr ? *objective : empty_objective;
  const bool active = objective != nullptr;
  std::ostringstream output;
  output << ",\"tracking_objective_active\":" << (active ? "true" : "false")
         << ",\"tracking_guidance_mode\":\""
         << (active ? interceptGuidanceModeName(data.guidance_mode) : "none") << '"'
         << ",\"tracking_resolution\":\""
         << (active ? trackingObjectiveResolutionStatusName(data.resolution_status)
                    : "none")
         << '"' << ",\"tracking_prediction_horizon_s\":" << data.prediction_horizon_s
         << ",\"tracking_observation_stamp_ns\":" << data.observation_stamp_ns
         << ",\"tracking_observation_age_ms\":"
         << (active ? static_cast<double>(std::max<std::int64_t>(
                          0, now_ns - data.observation_stamp_ns)) /
                          1.0e6
                    : 0.0)
         << ",\"tracking_resolved_fraction\":" << data.resolved_fraction
         << ",\"tracking_observed_x_m\":" << data.observed_position.x
         << ",\"tracking_observed_y_m\":" << data.observed_position.y
         << ",\"tracking_observed_z_m\":" << data.observed_position.z
         << ",\"tracking_velocity_x_mps\":" << data.observed_velocity.x
         << ",\"tracking_velocity_y_mps\":" << data.observed_velocity.y
         << ",\"tracking_velocity_z_mps\":" << data.observed_velocity.z
         << ",\"tracking_predicted_x_m\":" << data.unconstrained_predicted_position.x
         << ",\"tracking_predicted_y_m\":" << data.unconstrained_predicted_position.y
         << ",\"tracking_predicted_z_m\":" << data.unconstrained_predicted_position.z
         << ",\"tracking_resolved_x_m\":" << resolved_position.x
         << ",\"tracking_resolved_y_m\":" << resolved_position.y
         << ",\"tracking_resolved_z_m\":" << resolved_position.z;
  return output.str();
}

void populateTrackingObjectiveMarkers(
    const ProductionNavigationObjective* navigation_objective,
    MppiDebugMarkerInput& marker_input) {
  if (navigation_objective == nullptr || !navigation_objective->tracking.has_value()) {
    return;
  }
  const ProductionTrackingObjective& tracking = navigation_objective->tracking.value();
  marker_input.tracking_objective_active = true;
  marker_input.observed_tracking_target = tracking.observed_position;
  marker_input.predicted_tracking_target = tracking.unconstrained_predicted_position;
  marker_input.resolved_tracking_target = navigation_objective->goal;
}

} // namespace drone_city_nav::detail
