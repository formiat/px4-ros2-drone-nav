#include "production_mppi_noncooperative_diagnostics.hpp"

#include <iomanip>
#include <sstream>

#include "production_mppi_node.hpp"

namespace drone_city_nav::detail {
namespace {

struct PrimaryThreatFields {
  std::uint64_t local_track_id{0U};
  double range_m{-1.0};
  double closing_speed_mps{-1.0};
  double time_to_closest_approach_s{-1.0};
  double closest_approach_distance_m{-1.0};
  const char* reason{"none"};
};

[[nodiscard]] PrimaryThreatFields
primaryThreatFields(const NonCooperativeAvoidanceUpdate& update) noexcept {
  if (!update.primary_threat) {
    return {};
  }
  const NonCooperativeClosestApproach& threat = *update.primary_threat;
  return PrimaryThreatFields{
      .local_track_id = threat.local_track_id,
      .range_m = threat.current_range_m,
      .closing_speed_mps = threat.closing_speed_mps,
      .time_to_closest_approach_s = threat.time_to_closest_approach_s,
      .closest_approach_distance_m = threat.closest_approach_distance_m,
      .reason = nonCooperativeThreatReasonName(threat.reason),
  };
}

void appendFields(std::ostringstream& output,
                  const ProductionMppiNonCooperativeUpdate& noncooperative,
                  const mppi::MppiTickResult& result, const bool json) {
  const NonCooperativeAvoidanceUpdate& update = noncooperative.avoidance;
  const PrimaryThreatFields threat = primaryThreatFields(update);
  const double radar_age_ms =
      update.maximum_radar_age_s >= 0.0 ? update.maximum_radar_age_s * 1000.0 : -1.0;
  if (json) {
    output << ",\"noncooperative_avoidance_enabled\":"
           << (noncooperative.enabled ? "true" : "false")
           << ",\"noncooperative_source_scan_sequence\":"
           << noncooperative.source_scan_sequence
           << ",\"noncooperative_transport_age_ms\":" << noncooperative.transport_age_ms
           << ",\"noncooperative_received_track_count\":" << update.received_track_count
           << ",\"noncooperative_fresh_track_count\":" << update.fresh_track_count
           << ",\"noncooperative_radar_age_ms\":" << radar_age_ms
           << ",\"noncooperative_primary_track_id\":" << threat.local_track_id
           << ",\"noncooperative_current_range_m\":" << threat.range_m
           << ",\"noncooperative_closing_speed_mps\":" << threat.closing_speed_mps
           << ",\"noncooperative_tcpa_s\":" << threat.time_to_closest_approach_s
           << ",\"noncooperative_dcpa_m\":" << threat.closest_approach_distance_m
           << ",\"noncooperative_threat_reason\":\"" << threat.reason << '"'
           << ",\"noncooperative_lifecycle_state\":\""
           << nonCooperativeAvoidanceLifecycleStateName(update.lifecycle_state) << '"'
           << ",\"noncooperative_lifecycle_generation\":" << update.lifecycle_generation
           << ",\"noncooperative_active\":" << (update.active ? "true" : "false")
           << ",\"noncooperative_acquisition_reseeded\":"
           << (result.noncooperative_acquisition_reseeded ? "true" : "false")
           << ",\"noncooperative_release_reseeded\":"
           << (result.noncooperative_release_reseeded ? "true" : "false")
           << ",\"noncooperative_acquisition_available\":"
           << (result.noncooperative_acquisition_available ? "true" : "false")
           << ",\"noncooperative_acquisition_candidate_index\":"
           << result.noncooperative_acquisition_candidate_index
           << ",\"noncooperative_selected_maneuver\":\""
           << mppi::nonCooperativeManeuverName(
                  result.noncooperative_acquisition_maneuver)
           << '"' << ",\"noncooperative_acquisition_minimum_separation_m\":"
           << result.noncooperative_acquisition_minimum_separation_m
           << ",\"noncooperative_separation_gain_m\":"
           << result.noncooperative_acquisition_separation_gain_m
           << ",\"noncooperative_acquisition_head_progress_m\":"
           << result.noncooperative_acquisition_head_progress_m
           << ",\"noncooperative_acquisition_terminal_progress_m\":"
           << result.noncooperative_acquisition_terminal_progress_m
           << ",\"noncooperative_anticipation_cost\":"
           << result.dynamic_aircraft_anticipation_cost
           << ",\"noncooperative_survival_cost\":"
           << result.dynamic_aircraft_survival_cost
           << ",\"noncooperative_survival_cost_ratio\":"
           << result.dynamic_aircraft_survival_cost_ratio
           << ",\"noncooperative_dynamic_aircraft_count\":"
           << (noncooperative.enabled ? result.dynamic_aircraft_count : 0U);
    return;
  }

  output << " noncooperative_avoidance_enabled="
         << (noncooperative.enabled ? "true" : "false")
         << " noncooperative_source_scan_sequence="
         << noncooperative.source_scan_sequence
         << " noncooperative_transport_age_ms=" << noncooperative.transport_age_ms
         << " noncooperative_received_track_count=" << update.received_track_count
         << " noncooperative_fresh_track_count=" << update.fresh_track_count
         << " noncooperative_radar_age_ms=" << radar_age_ms
         << " noncooperative_primary_track_id=" << threat.local_track_id
         << " noncooperative_current_range_m=" << threat.range_m
         << " noncooperative_closing_speed_mps=" << threat.closing_speed_mps
         << " noncooperative_tcpa_s=" << threat.time_to_closest_approach_s
         << " noncooperative_dcpa_m=" << threat.closest_approach_distance_m
         << " noncooperative_threat_reason=" << threat.reason
         << " noncooperative_lifecycle_state="
         << nonCooperativeAvoidanceLifecycleStateName(update.lifecycle_state)
         << " noncooperative_lifecycle_generation=" << update.lifecycle_generation
         << " noncooperative_active=" << (update.active ? "true" : "false")
         << " noncooperative_acquisition_reseeded="
         << (result.noncooperative_acquisition_reseeded ? "true" : "false")
         << " noncooperative_release_reseeded="
         << (result.noncooperative_release_reseeded ? "true" : "false")
         << " noncooperative_acquisition_available="
         << (result.noncooperative_acquisition_available ? "true" : "false")
         << " noncooperative_selected_maneuver="
         << mppi::nonCooperativeManeuverName(result.noncooperative_acquisition_maneuver)
         << " noncooperative_acquisition_minimum_separation_m="
         << result.noncooperative_acquisition_minimum_separation_m
         << " noncooperative_separation_gain_m="
         << result.noncooperative_acquisition_separation_gain_m
         << " noncooperative_anticipation_cost="
         << result.dynamic_aircraft_anticipation_cost
         << " noncooperative_survival_cost=" << result.dynamic_aircraft_survival_cost
         << " noncooperative_survival_cost_ratio="
         << result.dynamic_aircraft_survival_cost_ratio;
}

} // namespace

std::string
nonCooperativeInfoFields(const ProductionMppiNonCooperativeUpdate& noncooperative,
                         const mppi::MppiTickResult& result) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(3);
  appendFields(output, noncooperative, result, false);
  return output.str();
}

std::string
nonCooperativeJsonFields(const ProductionMppiNonCooperativeUpdate& noncooperative,
                         const mppi::MppiTickResult& result) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(3);
  appendFields(output, noncooperative, result, true);
  return output.str();
}

} // namespace drone_city_nav::detail
