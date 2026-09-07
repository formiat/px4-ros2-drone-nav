#pragma once

#include "drone_city_nav/execution_supervisor_3d.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/route_progress_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "production_mppi_execution_control.hpp"
#include "production_mppi_node_types.hpp"
#include "production_mppi_raw_world.hpp"
#include "route_execution_selection_3d.hpp"

namespace drone_city_nav {

struct RouteExecutionSelectorConfig3D {
  SweptFootprintConfig physical_footprint{};
  FlightEnvelopeConfig flight_envelope{};
  RouteTrackingPolicy3D route_tracking{};
  bool route_cross_track_constraints_enabled{false};
  bool route_tracking_tube_constraints_enabled{false};
  // How far ahead along the followed route the latest lidar scan is checked
  // for hits the persistent memory has not integrated yet. Zero disables it.
  // The distance the vehicle needs to react to an obstacle at its absolute
  // speed limit is the natural value: a hit farther away bounds nothing yet.
  double latest_lidar_route_lookahead_m{0.0};
};

struct RouteExecutionSelectorRequest3D {
  const WorldSnapshot3D* world{nullptr};
  const ProductionNavigationObjective* objective{nullptr};
  ProductionMppiNavigation navigation{};
  std::shared_ptr<const VersionedExecutionInput3D> execution_input;
  std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world;
  std::shared_ptr<const VersionedLatestLidarEvidence3D> latest_lidar_evidence;
  std::int64_t validation_stamp_ns{0};
  std::uint64_t minimum_tracking_sample_sequence{0U};
  std::uint64_t physically_invalidated_through_generation{0U};
  std::optional<DirectTrackingOwnerIdentity3D> direct_tracking_identity;
  bool observed_3d_world{false};

  [[nodiscard]] bool valid() const noexcept {
    return world != nullptr;
  }
};

enum class RouteExecutionSelectorEffectKind3D : std::uint8_t {
  kRequestRouteRelease,
  kHandlePhysicalTrajectoryCollision,
};

struct RouteExecutionSelectorEffect3D {
  RouteExecutionSelectorEffectKind3D kind{
      RouteExecutionSelectorEffectKind3D::kRequestRouteRelease};
  RouteReleaseReason3D release_reason{RouteReleaseReason3D::kNoActiveRoute};
  ProductionMppiResidentObstacleDisposition obstacle_disposition{
      ProductionMppiResidentObstacleDisposition::kClear};
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
  std::uint64_t route_generation{0U};
};

struct RouteExecutionSelectorResult3D {
  ProductionRouteExecutionSelection3D selection{};
  std::vector<RouteExecutionSelectorEffect3D> effects;
};

// Selects and advances the exact execution route snapshot without depending on
// ROS or applying node-side publication/replan effects.
class RouteExecutionSelector3D final {
public:
  RouteExecutionSelector3D(ExecutionSupervisor3D& execution_supervisor,
                           const RouteExecutionSelectorConfig3D& config);

  [[nodiscard]] RouteExecutionSelectorResult3D
  select(const RouteExecutionSelectorRequest3D& request);

private:
  // Station of the first route sample within the lookahead that the latest
  // scan's hits touch, or nullopt when the window is clear.
  [[nodiscard]] std::optional<double>
  latestLidarBlockedStation(const CertifiedRouteSuffix3D& route,
                            const RouteProjection3D& projection,
                            const VersionedLatestLidarEvidence3D& latest_lidar,
                            const VersionedObservedRawWorld3D* contact_world);

  // The latest lidar window is one point-cloud sweep of the route ahead; a
  // scan and a route geometry that did not change give the same answer, so
  // the answer is kept until either does.
  struct LatestLidarWindowCache3D {
    std::uint64_t lidar_producer_instance_id{0U};
    std::uint64_t lidar_sequence{0U};
    std::uint64_t route_generation{0U};
    std::uint64_t geometry_revision{0U};
    std::optional<double> blocked_station_m;
    bool valid{false};
  };

  ExecutionSupervisor3D& execution_supervisor_;
  RouteExecutionSelectorConfig3D config_{};
  LatestLidarWindowCache3D latest_lidar_window_cache_{};
};

} // namespace drone_city_nav
