#pragma once

#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/semantic_portal_route.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace drone_city_nav {

enum class PassageCoordinatorPhase {
  kInactive,
  kUpcoming,
  kVerticalAlignment,
  kReady,
  kTraversal,
  kCleared,
  kInvalidRouteEvent,
};

struct PassageCoordinatorConfig {
  double vertical_clearance_margin_m{1.0};
  double vertical_capture_hysteresis_m{0.25};
  double preferred_z_capture_tolerance_m{0.5};
  double maximum_capture_vertical_speed_mps{0.5};
  std::size_t capture_stable_cycles{3U};
  std::size_t retention_violation_cycles{3U};
  double alignment_time_margin_s{1.5};
  double stationary_hold_clearance_m{2.0};
  double minimum_continuous_speed_mps{0.5};
  double prediction_time_step_s{0.02};
  double maximum_vertical_acceleration_mps2{4.0};
  double maximum_vertical_speed_mps{5.0};
  double maximum_horizontal_braking_acceleration_mps2{4.0};
  double reaction_latency_s{0.1};
  double exit_station_hysteresis_m{0.5};
};

struct PassageCoordinatorInput {
  mppi::State state{};
  std::shared_ptr<const SemanticPortalRoute> route;
  double route_station_m{0.0};
  double normal_flight_z_m{0.0};
  double approach_speed_mps{0.0};
};

struct PassageCoordinatorResult {
  PassageCoordinatorPhase phase{PassageCoordinatorPhase::kInactive};
  std::optional<mppi::PassageConstraint> constraint;
  std::string portal_id;
  std::uint64_t route_generation{0U};
  std::size_t route_event_index{0U};
  bool active{false};
  bool hold_xy{false};
  bool vertical_ready{false};
  bool traversal_predicted_safe{false};
  bool speed_limit_active{false};
  bool entry_plane_crossed{false};
  bool exit_plane_crossed{false};
  Point2 hold_position{};
  double preferred_z_m{0.0};
  double z_reference_m{0.0};
  double vertical_error_m{0.0};
  double distance_to_entry_m{0.0};
  std::size_t capture_stable_cycles{0U};
  std::size_t retention_violation_cycles{0U};
  double required_alignment_time_s{0.0};
  double required_stopping_distance_m{0.0};
  double required_alignment_distance_m{0.0};
  double effective_approach_station_m{0.0};
  double alignment_completion_station_m{0.0};
  double effective_speed_limit_mps{0.0};
  double stationary_hold_station_m{0.0};
  double predicted_entry_z_m{0.0};
  double predicted_entry_vz_mps{0.0};
  double predicted_exit_z_m{0.0};
  double predicted_exit_vz_mps{0.0};
  double predicted_minimum_z_m{0.0};
  double predicted_maximum_z_m{0.0};
  double entry_plane_signed_distance_m{0.0};
  double exit_plane_signed_distance_m{0.0};
};

class PassageCoordinator {
public:
  explicit PassageCoordinator(const PassageCoordinatorConfig& config = {});

  [[nodiscard]] PassageCoordinatorResult update(const PassageCoordinatorInput& input);

  void reset() noexcept;

private:
  enum class VerticalState {
    kUncaptured,
    kAlignment,
    kReady,
  };

  [[nodiscard]] bool
  continueTraversalForRoute(const PassageCoordinatorInput& input) noexcept;
  void resetForRoute(std::uint64_t generation) noexcept;
  void resetEventState() noexcept;

  PassageCoordinatorConfig config_{};
  std::uint64_t route_generation_{0U};
  std::size_t next_event_index_{0U};
  bool route_seen_{false};
  bool event_initialized_{false};
  bool position_seen_{false};
  bool entry_plane_crossed_{false};
  bool exit_plane_crossed_{false};
  VerticalState vertical_state_{VerticalState::kUncaptured};
  std::size_t capture_stable_cycles_{0U};
  std::size_t retention_violation_cycles_{0U};
  std::string active_portal_id_;
  int active_traversal_direction_{0};
  Point2 hold_position_{};
  double preferred_z_m_{0.0};
  double effective_approach_station_m_{0.0};
  double alignment_completion_station_m_{0.0};
  double stationary_hold_station_m_{0.0};
  double previous_entry_plane_distance_m_{0.0};
  double previous_exit_plane_distance_m_{0.0};
};

[[nodiscard]] const char*
passageCoordinatorPhaseName(PassageCoordinatorPhase phase) noexcept;

} // namespace drone_city_nav
