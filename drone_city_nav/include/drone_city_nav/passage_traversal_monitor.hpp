#pragma once

#include "drone_city_nav/known_passage_geometry.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>

namespace drone_city_nav {

enum class PassageBoundary : std::uint8_t {
  kDepthEntry,
  kDepthExit,
  kLateralNegative,
  kLateralPositive,
  kVerticalLower,
  kVerticalUpper,
};

struct PassageMargins {
  double depth_m{std::numeric_limits<double>::infinity()};
  double lateral_m{std::numeric_limits<double>::infinity()};
  double vertical_m{std::numeric_limits<double>::infinity()};
  double wall_clearance_m{std::numeric_limits<double>::infinity()};
  double boundary_margin_m{std::numeric_limits<double>::infinity()};
  PassageBoundary nearest_wall_boundary{PassageBoundary::kLateralNegative};
  PassageBoundary nearest_boundary{PassageBoundary::kDepthEntry};
};

struct PassageMinimumSample {
  KnownPassageOpeningWorldPoint world{};
  KnownPassageOpeningLocalPoint local{};
  PassageMargins margins{};
  bool valid{false};
};

struct PassageTraversalMetrics {
  bool entered{false};
  bool entry_crossed{false};
  bool exit_crossed{false};
  bool completed{false};
  std::size_t samples_inside{0U};
  double min_lateral_clearance_m{std::numeric_limits<double>::infinity()};
  double min_vertical_clearance_m{std::numeric_limits<double>::infinity()};
  double min_wall_clearance_m{std::numeric_limits<double>::infinity()};
  double min_depth_margin_m{std::numeric_limits<double>::infinity()};
  double min_boundary_margin_m{std::numeric_limits<double>::infinity()};
  double min_local_depth_m{std::numeric_limits<double>::infinity()};
  double max_local_depth_m{-std::numeric_limits<double>::infinity()};
  PassageMinimumSample min_wall_sample{};
  PassageMinimumSample min_boundary_sample{};
};

struct PassageTraversalUpdate {
  KnownPassageOpeningLocalPoint local{};
  PassageMargins margins{};
  bool inside{false};
  bool entered_now{false};
  bool completed_now{false};
};

struct PassageTraversalMonitorConfig {
  double crossing_hysteresis_m{0.25};
};

class PassageTraversalMonitor {
public:
  PassageTraversalMonitor(PassageOpening opening, KnownPassageOpeningFrame frame,
                          PassageTraversalMonitorConfig config = {});

  [[nodiscard]] PassageTraversalUpdate
  update(const KnownPassageOpeningWorldPoint& point) noexcept;

  [[nodiscard]] const PassageOpening& opening() const noexcept;
  [[nodiscard]] const KnownPassageOpeningFrame& frame() const noexcept;
  [[nodiscard]] const PassageTraversalMetrics& metrics() const noexcept;

private:
  PassageOpening opening_{};
  KnownPassageOpeningFrame frame_{};
  PassageTraversalMonitorConfig config_{};
  PassageTraversalMetrics metrics_{};
  bool entry_armed_{false};
  bool attempt_entry_crossed_{false};
  bool attempt_inside_observed_{false};
  bool attempt_invalid_{false};
};

[[nodiscard]] PassageMargins
passageMargins(const KnownPassageOpeningLocalPoint& local,
               const PassageOpening& opening,
               const KnownPassageOpeningFrame& frame) noexcept;

[[nodiscard]] const char* passageBoundaryName(PassageBoundary boundary) noexcept;

} // namespace drone_city_nav
