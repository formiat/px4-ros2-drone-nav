#include "drone_city_nav/passage_traversal_monitor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool crossSectionContains(const PassageMargins& margins) noexcept {
  return margins.lateral_m >= 0.0 && margins.vertical_m >= 0.0;
}

[[nodiscard]] PassageMinimumSample
minimumSample(const KnownPassageOpeningWorldPoint& world,
              const KnownPassageOpeningLocalPoint& local,
              const PassageMargins& margins) noexcept {
  return PassageMinimumSample{
      .world = world,
      .local = local,
      .margins = margins,
      .valid = true,
  };
}

} // namespace

PassageTraversalMonitor::PassageTraversalMonitor(PassageOpening opening,
                                                 KnownPassageOpeningFrame frame,
                                                 PassageTraversalMonitorConfig config)
    : opening_{std::move(opening)},
      frame_{frame},
      config_{config} {
  if (!std::isfinite(config_.crossing_hysteresis_m) ||
      config_.crossing_hysteresis_m < 0.0) {
    config_.crossing_hysteresis_m = 0.25;
  }
}

PassageTraversalUpdate
PassageTraversalMonitor::update(const KnownPassageOpeningWorldPoint& point) noexcept {
  const KnownPassageOpeningLocalPoint local =
      knownPassageOpeningLocalPoint(point, frame_);
  const PassageMargins margins = passageMargins(local, opening_, frame_);
  const bool inside = margins.boundary_margin_m >= 0.0;
  PassageTraversalUpdate update{
      .local = local,
      .margins = margins,
      .inside = inside,
  };

  if (inside) {
    ++metrics_.samples_inside;
    metrics_.min_lateral_clearance_m =
        std::min(metrics_.min_lateral_clearance_m, margins.lateral_m);
    metrics_.min_vertical_clearance_m =
        std::min(metrics_.min_vertical_clearance_m, margins.vertical_m);
    metrics_.min_depth_margin_m =
        std::min(metrics_.min_depth_margin_m, margins.depth_m);
    metrics_.min_local_depth_m = std::min(metrics_.min_local_depth_m, local.u_m);
    metrics_.max_local_depth_m = std::max(metrics_.max_local_depth_m, local.u_m);
    if (margins.wall_clearance_m < metrics_.min_wall_clearance_m) {
      metrics_.min_wall_clearance_m = margins.wall_clearance_m;
      metrics_.min_wall_sample = minimumSample(point, local, margins);
    }
    if (margins.boundary_margin_m < metrics_.min_boundary_margin_m) {
      metrics_.min_boundary_margin_m = margins.boundary_margin_m;
      metrics_.min_boundary_sample = minimumSample(point, local, margins);
    }
    if (!metrics_.entered) {
      metrics_.entered = true;
      update.entered_now = true;
    }
    attempt_inside_observed_ = true;
  }

  if (metrics_.completed) {
    return update;
  }

  const double entry_plane_m = -frame_.half_depth_m;
  const double exit_plane_m = frame_.half_depth_m;
  const double hysteresis_m = config_.crossing_hysteresis_m;
  if (local.u_m <= entry_plane_m - hysteresis_m) {
    entry_armed_ = true;
    attempt_entry_crossed_ = false;
    attempt_inside_observed_ = false;
    attempt_invalid_ = false;
    return update;
  }
  if (!entry_armed_) {
    return update;
  }

  const bool cross_section_contains = crossSectionContains(margins);
  if (!attempt_entry_crossed_ && local.u_m >= entry_plane_m + hysteresis_m) {
    attempt_entry_crossed_ = true;
    metrics_.entry_crossed = true;
    if (!cross_section_contains) {
      attempt_invalid_ = true;
    }
  }
  if (attempt_entry_crossed_ && local.u_m < exit_plane_m + hysteresis_m &&
      !cross_section_contains) {
    attempt_invalid_ = true;
  }
  if (attempt_entry_crossed_ && local.u_m >= exit_plane_m + hysteresis_m) {
    if (!attempt_invalid_ && attempt_inside_observed_ && cross_section_contains) {
      metrics_.exit_crossed = true;
      metrics_.completed = true;
      update.completed_now = true;
    }
    entry_armed_ = false;
  }
  return update;
}

const PassageOpening& PassageTraversalMonitor::opening() const noexcept {
  return opening_;
}

const KnownPassageOpeningFrame& PassageTraversalMonitor::frame() const noexcept {
  return frame_;
}

const PassageTraversalMetrics& PassageTraversalMonitor::metrics() const noexcept {
  return metrics_;
}

PassageMargins passageMargins(const KnownPassageOpeningLocalPoint& local,
                              const PassageOpening& opening,
                              const KnownPassageOpeningFrame& frame) noexcept {
  const std::array boundary_margins{
      std::pair{frame.half_depth_m + local.u_m, PassageBoundary::kDepthEntry},
      std::pair{frame.half_depth_m - local.u_m, PassageBoundary::kDepthExit},
      std::pair{frame.half_width_m + local.v_m, PassageBoundary::kLateralNegative},
      std::pair{frame.half_width_m - local.v_m, PassageBoundary::kLateralPositive},
      std::pair{local.z_m - opening.min_z_m, PassageBoundary::kVerticalLower},
      std::pair{opening.max_z_m - local.z_m, PassageBoundary::kVerticalUpper},
  };
  const auto* const nearest = std::min_element(
      boundary_margins.begin(), boundary_margins.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
  const auto* const nearest_wall = std::min_element(
      boundary_margins.begin() + 2, boundary_margins.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
  return PassageMargins{
      .depth_m = std::min(boundary_margins[0].first, boundary_margins[1].first),
      .lateral_m = std::min(boundary_margins[2].first, boundary_margins[3].first),
      .vertical_m = std::min(boundary_margins[4].first, boundary_margins[5].first),
      .wall_clearance_m = nearest_wall->first,
      .boundary_margin_m = nearest->first,
      .nearest_wall_boundary = nearest_wall->second,
      .nearest_boundary = nearest->second,
  };
}

const char* passageBoundaryName(const PassageBoundary boundary) noexcept {
  switch (boundary) {
    case PassageBoundary::kDepthEntry:
      return "depth_entry";
    case PassageBoundary::kDepthExit:
      return "depth_exit";
    case PassageBoundary::kLateralNegative:
      return "lateral_negative";
    case PassageBoundary::kLateralPositive:
      return "lateral_positive";
    case PassageBoundary::kVerticalLower:
      return "vertical_lower";
    case PassageBoundary::kVerticalUpper:
      return "vertical_upper";
  }
  return "unknown";
}

} // namespace drone_city_nav
