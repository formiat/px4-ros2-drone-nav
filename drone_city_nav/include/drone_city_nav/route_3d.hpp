#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace drone_city_nav {

struct RouteSample3D {
  Point3 position{};
  Vec3 tangent{};
  double station_m{0.0};
  double reference_speed_mps{0.0};
  mppi::RiskTier required_risk_tier{mppi::RiskTier::kPreferred};
};

struct RouteProjection3D {
  bool valid{false};
  double station_m{0.0};
  double remaining_m{0.0};
  double distance_m{0.0};
  Point3 point{};
};

struct RouteEnvelopeSample {
  double station_m{0.0};
  double lateral_free_left_m{0.0};
  double lateral_free_right_m{0.0};
  double min_z_m{0.0};
  double max_z_m{0.0};
  double minimum_clearance_m{0.0};
  double reference_z_m{0.0};
  double reference_speed_mps{0.0};
};

struct ConstrainedRouteSpan {
  std::string channel_id;
  std::uint64_t route_generation{0U};
  int direction_sign{0};
  double begin_station_m{0.0};
  double end_station_m{0.0};
  std::vector<RouteEnvelopeSample> envelope;
};

struct SelectedChannelTraversal {
  std::string channel_id;
  int direction_sign{0};
  double begin_station_m{0.0};
  double end_station_m{0.0};
  double min_z_m{0.0};
  double max_z_m{0.0};
  double width_m{0.0};
  double height_m{0.0};
  double minimum_clearance_m{0.0};
  double speed_limit_mps{0.0};
};

struct RouteEnvelopeConfig {
  double sample_step_m{0.5};
  double maximum_probe_distance_m{12.0};
  double constrained_lateral_width_m{8.0};
  double constrained_vertical_height_m{8.0};
  double minimum_span_length_m{2.0};
  double unconstrained_speed_mps{20.0};
  double constrained_speed_mps{10.0};
};

enum class ConstrainedRoutePhase : std::uint8_t {
  kUnavailable,
  kUnconstrained,
  kApproach,
  kTraversal,
  kDeparture,
};

struct ConstrainedRouteObservation {
  ConstrainedRoutePhase phase{ConstrainedRoutePhase::kUnavailable};
  std::uint64_t route_generation{0U};
  std::size_t span_index{0U};
  std::size_t span_count{0U};
  bool span_available{false};
  std::string channel_id;
  int direction_sign{0};
  bool within_vertical_window{false};
  double station_m{0.0};
  double begin_station_m{0.0};
  double end_station_m{0.0};
  double distance_to_entry_m{0.0};
  double distance_to_exit_m{0.0};
  Point3 entry_position{};
  Point3 exit_position{};
  double reference_z_m{0.0};
  double min_z_m{0.0};
  double max_z_m{0.0};
  double lateral_free_left_m{0.0};
  double lateral_free_right_m{0.0};
  double lateral_width_m{0.0};
  double vertical_height_m{0.0};
  double vertical_error_m{0.0};
  double cross_track_error_m{0.0};
  double reference_speed_mps{0.0};
  double actual_horizontal_speed_mps{0.0};
  double actual_vertical_speed_mps{0.0};
  double actual_z_m{0.0};
  bool lateral_constrained{false};
  bool vertical_constrained{false};
};

struct ConstrainedRouteControlConfig {
  double maximum_vertical_acceleration_mps2{4.0};
  double maximum_vertical_speed_mps{5.0};
  double alignment_distance_buffer_m{5.0};
  double stationary_hold_distance_m{2.0};
  double vertical_capture_margin_m{0.5};
  double vertical_capture_speed_mps{0.75};
};

struct ConstrainedRouteControl {
  bool active{false};
  bool vertical_ready{false};
  bool hold_xy{false};
  double required_alignment_time_s{0.0};
  double alignment_start_distance_m{0.0};
  double reference_z_m{0.0};
  double speed_limit_mps{0.0};
};

class ConstrainedRouteCoordinator {
public:
  [[nodiscard]] ConstrainedRouteControl
  update(const ConstrainedRouteObservation& observation, double unconstrained_speed_mps,
         const ConstrainedRouteControlConfig& config) noexcept;
  void reset() noexcept;

private:
  std::uint64_t route_generation_{0U};
  std::size_t span_index_{0U};
  bool vertical_ready_latched_{false};
};

[[nodiscard]] std::string_view
constrainedRoutePhaseName(ConstrainedRoutePhase phase) noexcept;

[[nodiscard]] ConstrainedRouteObservation
observeConstrainedRoute(std::span<const RouteSample3D> route,
                        std::span<const ConstrainedRouteSpan> spans,
                        std::uint64_t route_generation, double current_station_m,
                        const Point3& actual_position, const Vec3& actual_velocity,
                        const RouteEnvelopeConfig& config, double event_distance_m);

[[nodiscard]] std::vector<RouteSample3D> sampleRoute3D(std::span<const Point3> points,
                                                       double sample_step_m,
                                                       double reference_speed_mps);

[[nodiscard]] RouteSample3D sampleRoute3DAtStation(std::span<const RouteSample3D> route,
                                                   double station_m) noexcept;

[[nodiscard]] std::uint64_t
routeFingerprint(std::span<const RouteSample3D> route,
                 std::span<const SelectedChannelTraversal> traversals = {}) noexcept;

[[nodiscard]] std::uint64_t routeFingerprint(std::span<const Point2> route) noexcept;

[[nodiscard]] bool assignRouteRiskTiers(std::span<RouteSample3D> route,
                                        const mppi::EsdfGrid& grid,
                                        std::span<const float> esdf_m,
                                        double critical_distance_m,
                                        double preferred_distance_m) noexcept;

[[nodiscard]] RouteProjection3D
projectOntoRoute3D(std::span<const RouteSample3D> route, const Point3& position,
                   double minimum_station_m = 0.0) noexcept;

[[nodiscard]] std::vector<ConstrainedRouteSpan>
makeConstrainedRouteSpans(std::span<const RouteSample3D> route,
                          std::span<const SelectedChannelTraversal> traversals,
                          std::uint64_t route_generation,
                          const RouteEnvelopeConfig& config);

[[nodiscard]] bool validateConstrainedRouteSpans(
    std::span<const RouteSample3D> route, std::span<const ConstrainedRouteSpan> spans,
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m) noexcept;

} // namespace drone_city_nav
