#pragma once

#include "drone_city_nav/cooperative_passage_route.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/mppi/route_risk_adapter_3d.hpp"
#include "drone_city_nav/passage_volume.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/static_route_geometry.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstdint>
#include <memory>
#include <optional>

#include "production_mppi_raw_world.hpp"
#include "production_planner_search_transaction_3d.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"
#include "route_planner_3d.hpp"

namespace drone_city_nav {

class BoundedWorkerPool;

struct RouteMaterializerConfig3D {
  RouteEnvelopeConfig route_envelope{};
  FutureRouteConnectorConfig3D future_route_connector{};
  StaticRouteGeometryConfig route_geometry{};
  SweptFootprintConfig physical_footprint{};
  FlightEnvelopeConfig flight_envelope{};
  StaticRouteExtensionConfig route_extension{};
  PassageVolumeConfig passage_volume{};
  CooperativePassageRouteConfig cooperative_passage_route{};
  double critical_distance_m{1.5};
  double preferred_distance_m{4.0};
  BoundedWorkerPool* worker_pool{nullptr};
  bool cooperative_traffic_enabled{false};
};

// One immutable materialization transaction. The exact planner transaction,
// candidate, active route, and latest raw world remain owned for the complete
// operation and cannot be replaced by concurrent resident publications.
struct RouteMaterializationRequest3D {
  std::shared_ptr<const PlannerSearchTransaction3D> transaction;
  ProductionWorldBuildTelemetry3D world_telemetry{};
  Point3 current_position{};
  RouteSearchCandidate3D candidate{};
  std::uint64_t candidate_generation{0U};
  std::shared_ptr<const CertifiedRouteSuffix3D> active_route;
  std::shared_ptr<const ProductionMppiRawWorld3D> activation_raw_world;

  [[nodiscard]] bool valid() const noexcept {
    return transaction != nullptr && transaction->valid() && candidate_generation != 0U;
  }
};

struct ProductionRouteMaterialization3D {
  MaterializedRoute3D route{};
  ProductionRoutePipelineTelemetry3D telemetry{};
  StaticRouteCandidateValidation validation{};
  StaticRouteReplacementPolicy replacement_policy{
      StaticRouteReplacementPolicy::kRequireEndpointImprovement};
  std::optional<RouteRiskTierAssignmentResult3D> geometry_optimization_fallback;
};

class RouteMaterializer3D final {
public:
  explicit RouteMaterializer3D(const RouteMaterializerConfig3D& config);

  RouteMaterializer3D(const RouteMaterializer3D&) = delete;
  RouteMaterializer3D& operator=(const RouteMaterializer3D&) = delete;
  RouteMaterializer3D(RouteMaterializer3D&&) = delete;
  RouteMaterializer3D& operator=(RouteMaterializer3D&&) = delete;

  [[nodiscard]] ProductionRouteMaterialization3D
  materialize(RouteMaterializationRequest3D request) const;

private:
  RouteMaterializerConfig3D config_{};
};

} // namespace drone_city_nav
