#pragma once

#include <cstdint>
#include <string>

namespace drone_city_nav {

struct Point3;
struct MppiDebugMarkerInput;
struct MppiSpeedPolicyResult;
struct ProductionNavigationObjective;
struct ProductionMppiExecutionPublication;

namespace mppi {
struct MppiTickInput;
struct MppiTickResult;
} // namespace mppi

namespace detail {

struct TrackingPursuitDiagnostics {
  double actual_speed_mps{0.0};
  double commanded_speed_mps{0.0};
  double target_separation_m{-1.0};
  double closing_speed_mps{-1.0};
  double radar_age_ms{-1.0};
  double selected_prediction_fraction{0.0};
  std::uint8_t radar_cadence_reason{0U};
  bool observed_target_visible{false};
  bool predicted_intercept_path_clear{false};
  bool direct_interception_active{false};
};

[[nodiscard]] TrackingPursuitDiagnostics
trackingPursuitDiagnostics(const ProductionNavigationObjective* navigation_objective,
                           const mppi::MppiTickInput& input,
                           const ProductionMppiExecutionPublication& execution);

[[nodiscard]] std::string
trackingPursuitInfoFields(const TrackingPursuitDiagnostics& diagnostics,
                          const MppiSpeedPolicyResult& speed_policy,
                          const mppi::MppiTickResult& result);

[[nodiscard]] std::string
trackingPursuitJsonFields(const TrackingPursuitDiagnostics& diagnostics,
                          const MppiSpeedPolicyResult& speed_policy,
                          const mppi::MppiTickResult& result);

[[nodiscard]] std::string
trackingObjectiveJsonFields(const ProductionNavigationObjective* navigation_objective,
                            const Point3& resolved_position, std::int64_t now_ns);

void populateTrackingObjectiveMarkers(
    const ProductionNavigationObjective* navigation_objective,
    MppiDebugMarkerInput& marker_input);

} // namespace detail
} // namespace drone_city_nav
