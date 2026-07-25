#pragma once

#include "drone_city_nav/occupancy_grid.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace drone_city_nav {

enum class InflationEscapeNeed {
  kNotNeeded,
  kNeeded,
  kStartOccupied,
  kOutsideGrid,
  kNoReachableExit,
};

enum class DirectedInflationEscapeState {
  kInactive,
  kStarted,
  kActive,
  kCompleted,
  kFailed,
};

struct DirectedInflationEscapeConfig {
  bool enabled{true};
  double tunnel_width_m{5.0};
  double max_length_m{25.0};
  double exit_depth_m{2.0};
  double inflation_exposure_cost_weight{1.0};
  double occupied_clearance_cost_weight{10.0};
  double mission_egress_distance_m{7.0};
  std::size_t stable_exit_cycles{3U};
};

struct DirectedInflationEscapeResult {
  InflationEscapeNeed need{InflationEscapeNeed::kNotNeeded};
  DirectedInflationEscapeState state{DirectedInflationEscapeState::kInactive};
  bool applied{false};
  bool connected{false};
  bool centerline_blocked{false};
  bool episode_off_centerline{false};
  bool episode_target_too_far{false};
  bool mission_egress_available{false};
  std::uint64_t episode_generation{0U};
  Point2 start{};
  Point2 target{};
  double centerline_length_m{0.0};
  std::size_t cells_considered{0U};
  std::size_t stable_exit_cycles{0U};
  LocalInflationRelaxationStats relaxation{};
  std::vector<Point2> centerline;
};

class DirectedInflationEscapePlanner {
public:
  [[nodiscard]] DirectedInflationEscapeResult
  update(const OccupancyGrid2D& original_grid, Point2 current_position,
         Point2 mission_goal, const DirectedInflationEscapeConfig& config);

  void reset() noexcept;

private:
  struct Episode {
    std::uint64_t generation{0U};
    Point2 start{};
    Point2 target{};
    double centerline_length_m{0.0};
    std::size_t stable_exit_cycles{0U};
    std::vector<Point2> centerline;
  };

  Episode episode_{};
  std::uint64_t next_generation_{1U};
};

[[nodiscard]] LocalInflationRelaxationStats
applyDirectedInflationEscape(OccupancyGrid2D& planning_grid,
                             const DirectedInflationEscapeResult& escape,
                             double tunnel_width_m);

[[nodiscard]] std::string_view
inflationEscapeNeedName(InflationEscapeNeed need) noexcept;

[[nodiscard]] std::string_view
directedInflationEscapeStateName(DirectedInflationEscapeState state) noexcept;

} // namespace drone_city_nav
