#pragma once

#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace drone_city_nav {

enum class PassageCoordinatorPhase {
  kInactive,
  kApproach,
  kStationaryVerticalAlignment,
  kTraversal,
  kPartialFromInside,
  kInvalidOpening,
};

struct PassageCoordinatorConfig {
  double vertical_clearance_margin_m{1.0};
  double vertical_capture_hysteresis_m{0.25};
  double preferred_z_capture_tolerance_m{0.5};
  double maximum_capture_vertical_speed_mps{0.5};
  std::size_t capture_stable_cycles{3U};
  std::size_t retention_violation_cycles{3U};
  double lateral_alignment_tolerance_m{2.0};
  double approach_alignment_speed_mps{3.0};
  double approach_staging_distance_m{2.0};
  double alignment_time_margin_s{0.5};
  double minimum_stationary_trigger_distance_m{2.0};
  double maximum_vertical_acceleration_mps2{4.0};
  double maximum_vertical_speed_mps{5.0};
  double maximum_horizontal_braking_acceleration_mps2{4.0};
  double reaction_latency_s{0.1};
};

struct PassageCoordinatorInput {
  mppi::State state{};
  const PassageOpening* selected_opening{nullptr};
  double approach_speed_mps{0.0};
  double passage_speed_limit_mps{0.0};
};

struct PassageCoordinatorResult {
  PassageCoordinatorPhase phase{PassageCoordinatorPhase::kInactive};
  std::optional<mppi::PassageConstraint> constraint;
  std::string opening_id;
  bool active{false};
  bool hold_xy{false};
  bool approach_alignment_active{false};
  bool vertical_ready{false};
  Point2 hold_position{};
  Point2 approach_target{};
  double preferred_z_m{0.0};
  double vertical_error_m{0.0};
  double lateral_error_m{0.0};
  double approach_reference_speed_mps{0.0};
  double distance_to_entry_m{0.0};
  std::size_t capture_stable_cycles{0U};
  std::size_t retention_violation_cycles{0U};
  double required_alignment_time_s{0.0};
  double required_stopping_distance_m{0.0};
  double required_alignment_distance_m{0.0};
};

class PassageCoordinator {
public:
  explicit PassageCoordinator(const PassageCoordinatorConfig& config = {});

  [[nodiscard]] PassageCoordinatorResult update(const PassageCoordinatorInput& input);

  void reset() noexcept;

private:
  PassageCoordinatorConfig config_{};
  std::optional<PassageOpening> active_opening_;
  double travel_sign_{1.0};
  bool vertical_alignment_active_{false};
  bool traversal_latched_{false};
  bool partial_from_inside_{false};
  std::size_t capture_stable_cycles_{0U};
  std::size_t retention_violation_cycles_{0U};
  Point2 hold_position_{};
  std::optional<Point2> approach_target_;
  double preferred_z_m_{0.0};
};

[[nodiscard]] const char*
passageCoordinatorPhaseName(PassageCoordinatorPhase phase) noexcept;

} // namespace drone_city_nav
