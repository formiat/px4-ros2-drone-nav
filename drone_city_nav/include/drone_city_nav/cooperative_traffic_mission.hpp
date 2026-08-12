#pragma once

#include "drone_city_nav/intercept_mission.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <span>
#include <utility>
#include <vector>

namespace drone_city_nav {

struct CooperativeGoalHoldConfig {
  double goal_tolerance_m{2.0};
  double hold_position_tolerance_m{2.0};
  double maximum_speed_mps{0.8};
  double confirmation_duration_s{1.0};
};

struct CooperativeGoalHoldUpdate {
  bool confirmed{false};
  bool newly_confirmed{false};
  double goal_distance_m{0.0};
  double hold_position_error_m{0.0};
  double speed_mps{0.0};
};

class CooperativeGoalHoldConfirmation final {
public:
  explicit CooperativeGoalHoldConfirmation(
      const CooperativeGoalHoldConfig& config = {});

  [[nodiscard]] CooperativeGoalHoldUpdate
  update(const TimedVehicleState& state, const Point3& goal,
         const std::optional<Point3>& active_hold_position);

  void reset() noexcept;

private:
  CooperativeGoalHoldConfig config_{};
  std::optional<std::int64_t> stable_since_ns_;
  bool confirmed_{false};
};

struct CooperativeSeparationConfig {
  double desired_minimum_separation_m{5.0};
  double release_separation_m{7.0};
  double maximum_continuity_gap_s{0.25};
};

struct CooperativePairSeparationUpdate {
  std::size_t first_index{0U};
  std::size_t second_index{0U};
  SweptVehicleSeparation separation{};
  bool desired_violation_active{false};
  bool newly_entered_desired_violation{false};
  bool newly_released_desired_violation{false};
};

struct CooperativeSeparationUpdate {
  double minimum_separation_m{0.0};
  std::optional<std::size_t> first_minimum_index;
  std::optional<std::size_t> second_minimum_index;
  std::size_t active_desired_violation_count{0U};
  std::uint64_t desired_violation_event_count{0U};
  std::vector<CooperativePairSeparationUpdate> pairs;
};

class CooperativeSeparationMonitor final {
public:
  CooperativeSeparationMonitor(std::size_t vehicle_count,
                               const CooperativeSeparationConfig& config = {});

  [[nodiscard]] CooperativeSeparationUpdate
  update(std::span<const TimedVehicleState> states);

  void resetTemporalContinuity() noexcept;

  [[nodiscard]] double minimumObservedSeparationM() const noexcept {
    return minimum_observed_separation_m_;
  }

  [[nodiscard]] std::uint64_t desiredViolationEventCount() const noexcept {
    return desired_violation_event_count_;
  }

private:
  using Pair = std::pair<std::size_t, std::size_t>;

  CooperativeSeparationConfig config_{};
  std::vector<std::optional<TimedVehicleState>> previous_states_;
  std::set<Pair> active_desired_violations_;
  double minimum_observed_separation_m_{0.0};
  std::uint64_t desired_violation_event_count_{0U};
};

} // namespace drone_city_nav
