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
           << ",\"cooperative_peer_count\":" << result.cooperative_peer_count
           << ",\"cooperative_candidates_injected\":"
           << (result.cooperative_candidates_injected ? "true" : "false")
           << ",\"cooperative_minimum_peer_separation_m\":"
           << finiteOrNegative(result.minimum_peer_separation_m)
           << ",\"cooperative_peer_separation_cost\":" << result.peer_separation_cost
           << ",\"cooperative_channel_phase\":\""
           << cooperativeChannelPhaseName(cooperative.channel.phase) << '"'
           << ",\"cooperative_channel_lane_index\":" << cooperative.channel.lane_index
           << ",\"cooperative_channel_lane_count\":" << cooperative.channel.lane_count
           << ",\"cooperative_yield_status\":\""
           << cooperativeChannelYieldStatusName(cooperative.yield.status) << '"'
           << ",\"cooperative_yield_active\":"
           << (cooperative.yield.active ? "true" : "false")
           << ",\"cooperative_yield_hold\":"
           << (cooperative.yield.hold_at_entry ? "true" : "false")
           << ",\"cooperative_yield_hold_station_m\":"
           << cooperative.yield.hold_station_m
           << ",\"cooperative_yield_maximum_speed_mps\":"
           << cooperative.yield.maximum_speed_mps;
    return;
  }

  output << " cooperative_command_generation=" << cooperative.command_generation
         << " cooperative_command_age_ms=" << cooperative.command_age_ms
         << " cooperative_adapter_status="
         << cooperativeMppiAdapterStatusName(cooperative.mppi.status)
         << " cooperative_peer_count=" << result.cooperative_peer_count
         << " cooperative_candidates_injected="
         << (result.cooperative_candidates_injected ? "true" : "false")
         << " cooperative_minimum_peer_separation_m="
         << finiteOrNegative(result.minimum_peer_separation_m)
         << " cooperative_peer_separation_cost=" << result.peer_separation_cost
         << " cooperative_channel_phase="
         << cooperativeChannelPhaseName(cooperative.channel.phase)
         << " cooperative_channel_lane=" << cooperative.channel.lane_index << '/'
         << cooperative.channel.lane_count << " cooperative_yield_status="
         << cooperativeChannelYieldStatusName(cooperative.yield.status)
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
