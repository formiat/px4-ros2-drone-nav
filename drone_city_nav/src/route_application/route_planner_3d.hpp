#pragma once

#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "production_planner_search_transaction_3d.hpp"

namespace drone_city_nav {

struct RoutePlannerVehicleState3D {
  Point3 position{};
  Vec3 velocity{};
  bool valid{false};
  // Whether the executor is flying a route of its own; see
  // PersistentPlannerRequest3D::vehicle_route_available.
  bool route_available{true};
};

struct RoutePlannerConfig3D {
  PersistentPlannerConfig3D planner{};
  StaticRouteExtensionConfig extension{};
  double route_sampling_step_m{0.5};
  double cruise_speed_mps{10.0};
};

struct RouteSearchCandidate3D {
  Point3 search_start{};
  Vec3 search_velocity{};
  RouteInstanceId3D search_base_route_instance_id{};
  std::optional<double> search_base_stitch_station_m;
  RouteIntent3D intent{};
  SegmentEvidence3D evidence{};
  SpatialRouteCandidate3D spatial_route{};
  PlannerInputStatus3D planner_input_status{PlannerInputStatus3D::kInvalidInput};
  SearchProgress3D planner_progress{SearchProgress3D::kInvalidated};
  PlannerTelemetry3D planner_telemetry{};
  std::vector<RouteSample3D> route;
};

struct RoutePlannerSession3D {
  PersistentPlannerRequest3D request{};
  // Consecutive continuations that kept the session world while the planner
  // was still repairing the previous change; bounds how long a search may lag
  // behind the newest evidence.
  std::uint32_t deferred_world_refreshes{0U};
  Point3 mission_goal{};
  Point3 search_start{};
  Vec3 search_velocity{};
  RouteInstanceId3D search_base_route_instance_id{};
  std::optional<double> search_base_stitch_station_m;
  RouteIntent3D intent{};
};

enum class RoutePlannerUpdateStatus3D : std::uint8_t {
  kUpdated,
  kInvalidRequest,
  kStitchBaseUnavailable,
  kStitchBeyondCertifiedRoute,
};

[[nodiscard]] std::string_view
routePlannerUpdateStatus3DName(RoutePlannerUpdateStatus3D status) noexcept;

struct RoutePlannerUpdate3D {
  std::optional<RouteSearchCandidate3D> improved_incumbent;
  std::shared_ptr<const RoutePlannerSession3D> planner_session;
  PlannerDispatch3D dispatch{};
  PlannerInputStatus3D planner_input_status{PlannerInputStatus3D::kInvalidInput};
  SearchProgress3D planner_progress{SearchProgress3D::kInvalidated};
  PlannerTelemetry3D planner_telemetry{};
  RoutePlannerUpdateStatus3D status{RoutePlannerUpdateStatus3D::kInvalidRequest};
  double attempted_stitch_station_m{0.0};
  double certified_route_end_station_m{0.0};
  // A blocked replacement asked to stitch found its overlap reaching the
  // block and was searched from the vehicle instead.
  bool stitch_fallback_to_vehicle{false};
  bool planner_invoked{false};
  double search_ms{0.0};
};

class RoutePlanner3D final {
public:
  explicit RoutePlanner3D(const RoutePlannerConfig3D& config);

  RoutePlanner3D(const RoutePlanner3D&) = delete;
  RoutePlanner3D& operator=(const RoutePlanner3D&) = delete;
  RoutePlanner3D(RoutePlanner3D&&) = delete;
  RoutePlanner3D& operator=(RoutePlanner3D&&) = delete;

  // A continuation with `renew_consumer_session` opens a new consumer session
  // on the same persistent search: the planner delivers its incumbent again
  // on that update, whatever it delivered to the previous session.
  [[nodiscard]] RoutePlannerUpdate3D
  update(const PlannerSearchTransaction3D& transaction,
         const RoutePlannerVehicleState3D& vehicle_state,
         std::shared_ptr<const RoutePlannerSession3D> continuation_session = nullptr,
         bool renew_consumer_session = false);

  void reset() noexcept;

private:
  RoutePlannerConfig3D config_{};
  PersistentDStarLitePlanner3D planner_;
  std::uint64_t next_session_id_{0U};
};

} // namespace drone_city_nav
