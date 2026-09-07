#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace drone_city_nav {

struct MppiLivenessConfig {
  // Recovery heuristics are opt-in. Normal navigation must not reseed solely
  // because a short observation window appears to show insufficient progress.
  bool enabled{false};
  double observation_window_s{1.0};
  double minimum_actual_displacement_m{0.5};
  // Net displacement that counts as movement even when it delivers no route
  // progress: a vehicle carrying out a lateral or vertical manoeuvre around an
  // obstacle is flying, not stuck, and replacing its optimised sequence with a
  // route connector would cut the manoeuvre short. A wobble in place stays
  // below it, so oscillation is still caught.
  double minimum_offroute_displacement_m{2.0};
  // How many consecutive windows without progress a reseed needs. One window
  // is a measurement; two are a stall.
  std::size_t stalled_windows_before_reseed{2U};
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
  std::uint64_t route_generation{0U};
  double route_station_m{0.0};
  bool route_station_valid{false};
  // Unit route tangent where the vehicle projects. Displacement along it is
  // progress the station coordinate misses whenever the route is replaced
  // under a moving vehicle and its station restarts from a new geometry.
  Vec3 route_tangent{};
  bool route_tangent_valid{false};
};

struct MppiLivenessResult {
  MppiLivenessState state{MppiLivenessState::kInactive};
  bool reseed_requested{false};
  bool recovery_active{false};
  double observation_age_s{0.0};
  double actual_displacement_m{0.0};
  double actual_route_progress_m{0.0};
  // Displacement projected on the route tangent held at the anchor: what the
  // vehicle covered along the route it was following when the window opened.
  double tangential_progress_m{0.0};
  // The progress the verdict was taken on, whichever measure delivered it.
  double useful_progress_m{0.0};
  std::size_t stalled_windows{0U};
  bool used_route_progress{false};
  double actual_speed_mps{0.0};
  double predicted_head_progress_m{0.0};
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
    Vec3 route_tangent{};
    bool route_tangent_valid{false};
  };

  [[nodiscard]] Anchor anchorFrom(const MppiLivenessObservation& observation,
                                  bool route_progress_available) const noexcept;

  MppiLivenessConfig config_;
  std::optional<Anchor> anchor_;
  std::uint64_t reseed_generation_{0U};
  std::size_t stalled_windows_{0U};
  bool recovery_active_{false};
};

[[nodiscard]] const char* mppiLivenessStateName(MppiLivenessState state) noexcept;

} // namespace drone_city_nav
