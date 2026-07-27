#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"

#include <span>

namespace drone_city_nav {

struct PassageRouteSelectionConfig {
  double activation_distance_m{45.0};
  double lateral_margin_m{0.5};
  double minimum_normal_alignment{0.35};
};

[[nodiscard]] bool guideCrossesPassageAhead(const mppi::State& state,
                                            std::span<const Point2> guide,
                                            const PassageOpening& opening,
                                            const PassageRouteSelectionConfig& config);

} // namespace drone_city_nav
