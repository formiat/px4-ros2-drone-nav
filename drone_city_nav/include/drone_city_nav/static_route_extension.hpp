#pragma once

#include "drone_city_nav/active_global_guide.hpp"
#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <span>
#include <string_view>

namespace drone_city_nav {

struct StaticRouteExtensionConfig {
  double minimum_remaining_m{45.0};
  double latency_margin_s{0.5};
  double maximum_latency_s{8.0};
  double minimum_retry_progress_m{15.0};
  double minimum_retry_interval_s{1.0};
  double minimum_endpoint_improvement_m{5.0};
};

struct StaticRouteExtensionObservation {
  std::uint64_t route_generation{0U};
  double route_station_m{0.0};
  double route_remaining_m{0.0};
  double horizontal_speed_mps{0.0};
  double guide_search_latency_ms{0.0};
  double esdf_build_latency_ms{0.0};
  bool route_reaches_mission_goal{false};
  bool next_planning_goal_inside_esdf{true};
  bool request_in_flight{false};
  std::uint64_t last_request_generation{0U};
  double last_request_station_m{0.0};
  std::int64_t request_stamp_ns{0};
  std::int64_t last_request_stamp_ns{0};
};

struct StaticRouteExtensionDecision {
  bool request_extension{false};
  bool request_roi_refresh{false};
  double extension_trigger_remaining_m{0.0};
  double roi_refresh_trigger_remaining_m{0.0};
};

class StaticRouteReplanGate {
public:
  [[nodiscard]] bool tryBegin(std::uint64_t route_generation) noexcept;
  void finish(std::uint64_t route_generation) noexcept;
  [[nodiscard]] bool inFlight() const noexcept;
  [[nodiscard]] std::uint64_t generation() const noexcept;

private:
  std::uint64_t generation_{0U};
};

enum class StaticRouteCandidateStatus : std::uint8_t {
  kAccepted,
  kEmpty,
  kOutsideEsdf,
  kInvalidEsdf,
  kRawCollision,
  kInvalidChannelSpan,
  kNoEndpointImprovement,
};

struct StaticRouteCandidateValidation {
  StaticRouteCandidateStatus status{StaticRouteCandidateStatus::kEmpty};
  double endpoint_improvement_m{0.0};
  bool accepted{false};
};

[[nodiscard]] StaticRouteExtensionDecision evaluateStaticRouteExtension(
    const StaticRouteExtensionConfig& config,
    const StaticRouteExtensionObservation& observation) noexcept;

[[nodiscard]] bool
deferStaticRouteReleaseDuringExtension(bool request_in_flight,
                                       GlobalGuideReleaseReason reason) noexcept;

[[nodiscard]] Point3 staticRoutePlanningGoal(const Point3& start,
                                             const Point3& mission_goal,
                                             double planning_distance_m) noexcept;

[[nodiscard]] bool staticRoutePointInsideEsdf(const mppi::EsdfGrid& grid,
                                              const Point3& point) noexcept;

[[nodiscard]] StaticRouteCandidateValidation validateStaticRouteCandidate(
    std::span<const RouteSample3D> active_route,
    std::span<const RouteSample3D> candidate_route, const mppi::EsdfGrid& grid,
    std::span<const float> esdf_m, const Point3& mission_goal,
    double minimum_endpoint_improvement_m, bool reaches_mission_goal,
    bool required_continuation = false) noexcept;

[[nodiscard]] std::string_view
staticRouteCandidateStatusName(StaticRouteCandidateStatus status) noexcept;

} // namespace drone_city_nav
