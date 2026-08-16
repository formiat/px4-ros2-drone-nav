#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/passage_ids.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <optional>
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

struct PassageTraversalSegmentSpan {
  PassageSegmentId passage_segment_id;
  double begin_station_m{0.0};
  double end_station_m{0.0};

  [[nodiscard]] bool
  operator==(const PassageTraversalSegmentSpan&) const noexcept = default;
};

struct ConstrainedRouteSpan {
  PassageTraversalId passage_traversal_id;
  std::uint64_t route_generation{0U};
  int direction_sign{0};
  double begin_station_m{0.0};
  double end_station_m{0.0};
  std::vector<RouteEnvelopeSample> envelope;
  std::vector<PassageTraversalSegmentSpan> segment_spans;
};

struct SelectedPassageTraversal {
  PassageTraversalId passage_traversal_id;
  int direction_sign{0};
  double begin_station_m{0.0};
  double end_station_m{0.0};
  double min_z_m{0.0};
  double max_z_m{0.0};
  double width_m{0.0};
  double height_m{0.0};
  double minimum_clearance_m{0.0};
  double speed_limit_mps{0.0};
  std::vector<PassageTraversalSegmentSpan> segment_spans;
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
  PassageTraversalId passage_traversal_id;
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
  std::vector<PassageTraversalSegmentSpan> segment_spans;
};

enum class PassageTraversalEvidenceStatus : std::uint8_t {
  kEntered,
  kCompleted,
  kAborted,
};

enum class PassageTraversalEvidenceReason : std::uint8_t {
  kEntryBoundaryCrossed,
  kExitBoundaryCrossed,
  kRouteChanged,
  kObservationLost,
};

struct PassageTraversalEvidenceEvent {
  PassageTraversalEvidenceStatus status{PassageTraversalEvidenceStatus::kEntered};
  PassageTraversalEvidenceReason reason{
      PassageTraversalEvidenceReason::kEntryBoundaryCrossed};
  std::uint64_t sequence{0U};
  PassageTraversalId passage_traversal_id;
  std::uint64_t route_generation{0U};
  std::size_t span_index{0U};
  std::size_t traversal_observation_count{0U};
  std::int64_t event_stamp_ns{0};
  double duration_s{0.0};
  double station_m{0.0};
  double begin_station_m{0.0};
  double end_station_m{0.0};
  double maximum_cross_track_error_m{0.0};
  double maximum_absolute_vertical_error_m{0.0};
  Point3 actual_position{};
  bool vertical_window_preserved{true};
};

class PassageTraversalEvidenceTracker {
public:
  [[nodiscard]] std::vector<PassageTraversalEvidenceEvent>
  update(const ConstrainedRouteObservation& observation, const Point3& actual_position,
         std::int64_t now_ns);
  void reset() noexcept;

private:
  struct ActiveTraversal {
    PassageTraversalId passage_traversal_id;
    std::uint64_t route_generation{0U};
    std::size_t span_index{0U};
    std::size_t observation_count{0U};
    std::int64_t entry_stamp_ns{0};
    double begin_station_m{0.0};
    double end_station_m{0.0};
    double maximum_cross_track_error_m{0.0};
    double maximum_absolute_vertical_error_m{0.0};
    bool vertical_window_preserved{true};
  };

  [[nodiscard]] PassageTraversalEvidenceEvent
  makeEvent(const ActiveTraversal& active, PassageTraversalEvidenceStatus status,
            PassageTraversalEvidenceReason reason,
            const ConstrainedRouteObservation& observation,
            const Point3& actual_position, std::int64_t now_ns);

  std::optional<ActiveTraversal> active_;
  std::uint64_t event_sequence_{0U};
};

struct PassageGeometryObservation {
  PassageTraversalId passage_traversal_id;
  bool within_corridor{false};
  double station_m{0.0};
  double traversal_length_m{0.0};
  double cross_track_error_m{0.0};
};

struct PassageGeometryEvidenceConfig {
  double entry_capture_distance_m{2.0};
  double exit_capture_distance_m{2.0};
  double minimum_progress_fraction{0.70};
  double observation_timeout_s{2.0};
};

struct PassageGeometryEvidenceEvent {
  PassageTraversalEvidenceStatus status{PassageTraversalEvidenceStatus::kEntered};
  PassageTraversalEvidenceReason reason{
      PassageTraversalEvidenceReason::kEntryBoundaryCrossed};
  std::uint64_t sequence{0U};
  PassageTraversalId passage_traversal_id;
  std::size_t observation_count{0U};
  std::int64_t event_stamp_ns{0};
  double duration_s{0.0};
  double station_m{0.0};
  double traversal_length_m{0.0};
  double maximum_station_m{0.0};
  double maximum_cross_track_error_m{0.0};
  Point3 actual_position{};
};

class PassageGeometryEvidenceTracker {
public:
  [[nodiscard]] std::vector<PassageGeometryEvidenceEvent>
  update(std::span<const PassageGeometryObservation> observations,
         const Point3& actual_position, std::int64_t now_ns,
         const PassageGeometryEvidenceConfig& config);
  void reset() noexcept;

private:
  struct ActiveTraversal {
    PassageTraversalId passage_traversal_id;
    std::size_t observation_count{0U};
    std::int64_t entry_stamp_ns{0};
    std::int64_t last_observation_stamp_ns{0};
    double traversal_length_m{0.0};
    double maximum_station_m{0.0};
    double maximum_cross_track_error_m{0.0};
  };

  [[nodiscard]] PassageGeometryEvidenceEvent
  makeEvent(const ActiveTraversal& active, PassageTraversalEvidenceStatus status,
            PassageTraversalEvidenceReason reason, double station_m,
            const Point3& actual_position, std::int64_t now_ns);

  std::optional<ActiveTraversal> active_;
  std::uint64_t event_sequence_{0U};
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

[[nodiscard]] std::string_view
passageTraversalEvidenceStatusName(PassageTraversalEvidenceStatus status) noexcept;

[[nodiscard]] std::string_view
passageTraversalEvidenceReasonName(PassageTraversalEvidenceReason reason) noexcept;

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
                 std::span<const SelectedPassageTraversal> traversals = {}) noexcept;

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
                          std::span<const SelectedPassageTraversal> traversals,
                          std::uint64_t route_generation,
                          const RouteEnvelopeConfig& config);

[[nodiscard]] bool validateConstrainedRouteSpans(
    std::span<const RouteSample3D> route, std::span<const ConstrainedRouteSpan> spans,
    const mppi::EsdfGrid& grid, std::span<const float> esdf_m) noexcept;

} // namespace drone_city_nav
