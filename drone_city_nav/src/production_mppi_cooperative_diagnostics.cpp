#include "production_mppi_cooperative_diagnostics.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

#include "production_mppi_node.hpp"

namespace drone_city_nav::detail {
namespace {

[[nodiscard]] float finiteOrNegative(const float value) noexcept {
  return std::isfinite(value) ? value : -1.0F;
}

void appendFields(std::ostringstream& output,
                  const ProductionMppiCooperativeUpdate& cooperative,
                  const mppi::MppiTickResult& result, const bool json) {
  if (json) {
    output << ",\"cooperative_command_generation\":" << cooperative.command_generation
           << ",\"cooperative_command_age_ms\":" << cooperative.command_age_ms
           << ",\"cooperative_adapter_status\":\""
           << cooperativeMppiAdapterStatusName(cooperative.mppi.status) << '"'
           << ",\"cooperative_peer_count\":" << cooperative.mppi.dynamic_aircraft.size()
           << ",\"cooperative_candidates_injected\":"
           << (result.cooperative_candidates_injected ? "true" : "false")
           << ",\"cooperative_acquisition_reseeded\":"
           << (result.cooperative_acquisition_reseeded ? "true" : "false")
           << ",\"cooperative_release_reseeded\":"
           << (result.cooperative_release_reseeded ? "true" : "false")
           << ",\"cooperative_acquisition_available\":"
           << (result.cooperative_acquisition_available ? "true" : "false")
           << ",\"cooperative_acquisition_positive_progress\":"
           << (result.cooperative_acquisition_positive_progress ? "true" : "false")
           << ",\"cooperative_acquisition_backward_fallback\":"
           << (result.cooperative_acquisition_backward_fallback ? "true" : "false")
           << ",\"cooperative_acquisition_candidate_index\":"
           << result.cooperative_acquisition_candidate_index
           << ",\"cooperative_acquisition_head_progress_m\":"
           << result.cooperative_acquisition_head_progress_m
           << ",\"cooperative_acquisition_terminal_progress_m\":"
           << result.cooperative_acquisition_terminal_progress_m
           << ",\"cooperative_acquisition_separation_gain_m\":"
           << result.cooperative_acquisition_separation_gain_m
           << ",\"cooperative_minimum_peer_separation_m\":"
           << finiteOrNegative(result.minimum_peer_separation_m)
           << ",\"cooperative_peer_separation_cost\":" << result.peer_separation_cost
           << ",\"cooperative_space_time_plan_active\":"
           << (cooperative.mppi.space_time_plan_active ? "true" : "false")
           << ",\"cooperative_space_time_lateral_offset_m\":"
           << cooperative.mppi.space_time_lateral_offset_m
           << ",\"cooperative_space_time_vertical_offset_m\":"
           << cooperative.mppi.space_time_vertical_offset_m
           << ",\"cooperative_space_time_shift_s\":"
           << cooperative.mppi.space_time_shift_s
           << ",\"cooperative_space_time_predicted_minimum_separation_m\":"
           << cooperative.mppi.space_time_predicted_minimum_separation_m
           << ",\"cooperative_space_time_integrated_shortfall_m2_s\":"
           << cooperative.mppi.space_time_integrated_shortfall_m2_s
           << ",\"cooperative_space_time_evaluated_candidate_count\":"
           << cooperative.mppi.space_time_evaluated_candidate_count
           << ",\"cooperative_passage_phase\":\""
           << cooperativePassagePhaseName(cooperative.passage.phase) << '"'
           << ",\"cooperative_passage_lateral_offset_m\":"
           << cooperative.passage.lateral_offset_m
           << ",\"cooperative_passage_minimum_lateral_offset_m\":"
           << cooperative.passage.minimum_lateral_offset_m
           << ",\"cooperative_passage_maximum_lateral_offset_m\":"
           << cooperative.passage.maximum_lateral_offset_m
           << ",\"cooperative_yield_status\":\""
           << cooperativePassageYieldStatusName(cooperative.yield.status) << '"'
           << ",\"cooperative_yield_active\":"
           << (cooperative.yield.active ? "true" : "false")
           << ",\"cooperative_yield_hold\":"
           << (cooperative.yield.hold_at_entry ? "true" : "false")
           << ",\"cooperative_yield_hold_station_m\":"
           << cooperative.yield.hold_station_m
           << ",\"cooperative_yield_maximum_speed_mps\":"
           << cooperative.yield.maximum_speed_mps
           << ",\"cooperative_yield_entry_not_before_ns\":"
           << cooperative.yield.entry_not_before_ns;
    return;
  }

  output << " cooperative_command_generation=" << cooperative.command_generation
         << " cooperative_command_age_ms=" << cooperative.command_age_ms
         << " cooperative_adapter_status="
         << cooperativeMppiAdapterStatusName(cooperative.mppi.status)
         << " cooperative_peer_count=" << cooperative.mppi.dynamic_aircraft.size()
         << " cooperative_candidates_injected="
         << (result.cooperative_candidates_injected ? "true" : "false")
         << " cooperative_acquisition_reseeded="
         << (result.cooperative_acquisition_reseeded ? "true" : "false")
         << " cooperative_release_reseeded="
         << (result.cooperative_release_reseeded ? "true" : "false")
         << " cooperative_acquisition_available="
         << (result.cooperative_acquisition_available ? "true" : "false")
         << " cooperative_acquisition_positive_progress="
         << (result.cooperative_acquisition_positive_progress ? "true" : "false")
         << " cooperative_acquisition_backward_fallback="
         << (result.cooperative_acquisition_backward_fallback ? "true" : "false")
         << " cooperative_acquisition_candidate_index="
         << result.cooperative_acquisition_candidate_index
         << " cooperative_acquisition_head_progress_m="
         << result.cooperative_acquisition_head_progress_m
         << " cooperative_acquisition_terminal_progress_m="
         << result.cooperative_acquisition_terminal_progress_m
         << " cooperative_acquisition_separation_gain_m="
         << result.cooperative_acquisition_separation_gain_m
         << " cooperative_minimum_peer_separation_m="
         << finiteOrNegative(result.minimum_peer_separation_m)
         << " cooperative_peer_separation_cost=" << result.peer_separation_cost
         << " cooperative_space_time_plan_active="
         << (cooperative.mppi.space_time_plan_active ? "true" : "false")
         << " cooperative_space_time_lateral_offset_m="
         << cooperative.mppi.space_time_lateral_offset_m
         << " cooperative_space_time_vertical_offset_m="
         << cooperative.mppi.space_time_vertical_offset_m
         << " cooperative_space_time_shift_s=" << cooperative.mppi.space_time_shift_s
         << " cooperative_space_time_predicted_minimum_separation_m="
         << cooperative.mppi.space_time_predicted_minimum_separation_m
         << " cooperative_space_time_integrated_shortfall_m2_s="
         << cooperative.mppi.space_time_integrated_shortfall_m2_s
         << " cooperative_space_time_evaluated_candidate_count="
         << cooperative.mppi.space_time_evaluated_candidate_count
         << " cooperative_passage_phase="
         << cooperativePassagePhaseName(cooperative.passage.phase)
         << " cooperative_passage_lateral_offset_m="
         << cooperative.passage.lateral_offset_m
         << " cooperative_passage_offset_interval_m=["
         << cooperative.passage.minimum_lateral_offset_m << ','
         << cooperative.passage.maximum_lateral_offset_m
         << "] cooperative_yield_status="
         << cooperativePassageYieldStatusName(cooperative.yield.status)
         << " cooperative_yield_active="
         << (cooperative.yield.active ? "true" : "false") << " cooperative_yield_hold="
         << (cooperative.yield.hold_at_entry ? "true" : "false");
}

} // namespace

std::string cooperativeInfoFields(const ProductionMppiCooperativeUpdate& cooperative,
                                  const mppi::MppiTickResult& result) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(3);
  appendFields(output, cooperative, result, false);
  return output.str();
}

std::string cooperativeJsonFields(const ProductionMppiCooperativeUpdate& cooperative,
                                  const mppi::MppiTickResult& result) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(3);
  appendFields(output, cooperative, result, true);
  return output.str();
}

} // namespace drone_city_nav::detail
