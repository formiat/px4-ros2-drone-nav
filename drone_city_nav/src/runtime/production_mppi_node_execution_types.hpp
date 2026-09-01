#pragma once

#include "drone_city_nav/cooperative_mppi_adapter.hpp"
#include "drone_city_nav/cooperative_passage_execution.hpp"
#include "drone_city_nav/cooperative_traffic.hpp"
#include "drone_city_nav/noncooperative_collision_avoidance.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "production_planner_search_transaction_3d.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"
#include "route_execution_selection_3d.hpp"

namespace drone_city_nav {

struct ProductionMppiStability {
  double first_control_delta{0.0};
  double position_rms_m{0.0};
  double position_max_m{0.0};
  double terminal_shift_m{0.0};
  bool valid{false};
};

struct ProductionMppiPredictionError {
  double position_m{0.0};
  double velocity_mps{0.0};
  double yaw_rad{0.0};
  bool valid{false};
};

struct ProductionMppiCooperativeCommand {
  CooperativeManeuverCommandData data;
  std::int64_t receive_stamp_ns{0};
};

struct ProductionMppiCooperativeUpdate {
  CooperativeMppiAdapterResult mppi{};
  CooperativePassageUse passage{};
  CooperativePassageYieldDecision yield{};
  std::uint64_t command_generation{0U};
  double command_age_ms{-1.0};
};

struct ProductionMppiNonCooperativeTracks {
  std::vector<NonCooperativeAircraftTrack> tracks;
  std::uint64_t source_scan_sequence{0U};
  std::int64_t receive_stamp_ns{0};
};

struct ProductionMppiNonCooperativeUpdate {
  NonCooperativeAvoidanceUpdate avoidance{};
  std::uint64_t source_scan_sequence{0U};
  double transport_age_ms{-1.0};
  bool enabled{false};
};

} // namespace drone_city_nav
