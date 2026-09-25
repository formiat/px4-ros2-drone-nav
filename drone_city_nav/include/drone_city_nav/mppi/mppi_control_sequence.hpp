#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace drone_city_nav::mppi {

inline constexpr std::size_t kCooperativeManeuverCandidateCount{6U};
inline constexpr std::size_t kCooperativeAcquisitionCandidateCount{6U};

[[nodiscard]] Control interpolateControl(const Control& first, const Control& second,
                                         float ratio) noexcept;

[[nodiscard]] std::vector<Control>
shiftControlSequence(std::span<const Control> controls, float dt_s, double elapsed_s);

void limitControlSequence(std::span<Control> controls, const DynamicsConfig& dynamics,
                          Control previous_applied_control,
                          float first_control_interval_s) noexcept;

[[nodiscard]] std::vector<Control> buildGuideDirectedNominalSeed(
    const State& initial, const State& target, std::span<const RouteSample3D> route,
    float initial_route_station_m, float reference_speed_mps,
    const DynamicsConfig& dynamics, std::size_t steps, Control previous_applied_control,
    const StoppingCapability& stopping_capability = {});

[[nodiscard]] std::vector<Control> buildFiniteRouteDirectedSeed(
    const State& initial, const State& target, std::span<const RouteSample3D> route,
    float initial_route_station_m, float reference_speed_mps,
    const DynamicsConfig& dynamics, std::size_t steps, Control previous_applied_control,
    const StoppingCapability& stopping_capability = {});

[[nodiscard]] std::vector<Control> buildCooperativeSeparationAcquisitionCandidates(
    const State& initial, const State& target, std::span<const RouteSample3D> route,
    float initial_route_station_m, float reference_speed_mps,
    const CooperativeSeparationAcquisition& acquisition, const DynamicsConfig& dynamics,
    const CooperativeConfig& cooperative, std::size_t steps,
    Control previous_applied_control, float first_control_interval_s,
    const StoppingCapability& stopping_capability);

[[nodiscard]] std::vector<Control> buildCooperativeManeuverCandidates(
    const State& initial, const State& target, std::span<const Control> nominal,
    const DynamicsConfig& dynamics, const CooperativeConfig& cooperative,
    Control previous_applied_control, float first_control_interval_s);

[[nodiscard]] std::optional<float>
projectForwardRouteStation(std::span<const RouteSample3D> route, const State& state,
                           float minimum_station_m) noexcept;

[[nodiscard]] RiskTier maximumRequiredRiskTier(std::span<const RouteSample3D> route,
                                               float begin_station_m,
                                               float end_station_m) noexcept;

// The gaze of a vehicle whose sensor looks forward: the yaw channel of
// `controls` is rewritten so that the heading follows the horizontal direction
// the horizon moves in over `lookahead_s`, as fast as the yaw dynamics allow
// and arriving without overshoot, and the yaw states of `horizon` are
// re-integrated to match. Where the horizon moves less than
// `minimum_displacement_m` over the lookahead the motion names no direction:
// the heading turns to `rest_heading_rad`, where the route leaves, and is held
// where there is none. The translation is untouched: the yaw channel does not
// act on it. Returns the decision taken at the first step, for diagnostics.
GazeDecision applyGazeYawControls(std::span<Control> controls, std::span<State> horizon,
                                  const DynamicsConfig& dynamics, float lookahead_s,
                                  float minimum_displacement_m,
                                  std::optional<float> rest_heading_rad);

// The heading of the route where the vehicle stands on it: what a vehicle at
// rest has to face before the speed law lets it leave along a route it has
// not looked along. Absent where the tangent's horizontal share is below
// `minimum_horizontal_share`.
[[nodiscard]] std::optional<float>
gazeRestHeading(const std::optional<RouteReference>& route,
                float minimum_horizontal_share);

} // namespace drone_city_nav::mppi
