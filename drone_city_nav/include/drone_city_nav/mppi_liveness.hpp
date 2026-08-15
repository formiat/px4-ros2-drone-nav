#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"

#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct MppiLivenessConfig {
  bool enabled{true};
  double observation_window_s{1.0};
  double minimum_actual_displacement_m{0.5};
  double minimum_predicted_terminal_progress_m{5.0};
};

enum class MppiLivenessState : std::uint8_t {
  kInactive,
  kMonitoring,
  kMoving,
  kReseedRequested,
};

struct MppiLivenessObservation {
  std::int64_t stamp_ns{0};
  mppi::State actual_state{};
  bool controller_active{false};
  double predicted_head_progress_m{0.0};
  double predicted_terminal_progress_m{0.0};
  std::uint64_t route_generation{0U};
  double route_station_m{0.0};
  bool route_station_valid{false};
};

struct MppiLivenessResult {
  MppiLivenessState state{MppiLivenessState::kInactive};
  bool reseed_requested{false};
  double observation_age_s{0.0};
  double actual_displacement_m{0.0};
  double actual_route_progress_m{0.0};
  bool used_route_progress{false};
  double actual_speed_mps{0.0};
  double predicted_head_progress_m{0.0};
  double predicted_terminal_progress_m{0.0};
  std::uint64_t reseed_generation{0U};
};

class MppiLivenessSupervisor {
public:
  explicit MppiLivenessSupervisor(const MppiLivenessConfig& config = {});

  [[nodiscard]] MppiLivenessResult evaluate(const MppiLivenessObservation& observation);
  void reset() noexcept;

private:
  struct Anchor {
    std::int64_t stamp_ns{0};
    mppi::State state{};
    std::uint64_t route_generation{0U};
    double route_station_m{0.0};
    bool route_station_valid{false};
  };

  MppiLivenessConfig config_;
  std::optional<Anchor> anchor_;
  std::uint64_t reseed_generation_{0U};
};

[[nodiscard]] const char* mppiLivenessStateName(MppiLivenessState state) noexcept;

} // namespace drone_city_nav
