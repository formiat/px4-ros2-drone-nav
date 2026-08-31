#pragma once

#include "drone_city_nav/execution_route_model_3d.hpp"
#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <memory>

#include "production_route_pipeline_artifacts_3d.hpp"

namespace drone_city_nav {

struct RouteTrajectoryCompilerConfig3D {
  TrajectoryCompilerConfig3D trajectory{};
  PassageVolumeConfig passage_volume{};
};

// One immutable route-compilation transaction. The materialized route and
// optional observed raw owner keep every referenced world/resource alive until
// the exact initial state has been sealed into the compiled trajectory.
struct RouteTrajectoryCompilationRequest3D {
  MaterializedRoute3D materialized{};
  VehicleState3D exact_initial_state{};
  RouteEndpointSemantics3D endpoint_semantics{RouteEndpointSemantics3D::kContinuation};
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world;
};

class RouteTrajectoryCompiler3D final {
public:
  explicit RouteTrajectoryCompiler3D(const RouteTrajectoryCompilerConfig3D& config);

  RouteTrajectoryCompiler3D(const RouteTrajectoryCompiler3D&) = delete;
  RouteTrajectoryCompiler3D& operator=(const RouteTrajectoryCompiler3D&) = delete;
  RouteTrajectoryCompiler3D(RouteTrajectoryCompiler3D&&) = delete;
  RouteTrajectoryCompiler3D& operator=(RouteTrajectoryCompiler3D&&) = delete;

  [[nodiscard]] TrajectoryCompilationResult3D
  compile(const RouteTrajectoryCompilationRequest3D& request) const;

private:
  RouteTrajectoryCompilerConfig3D config_{};
};

} // namespace drone_city_nav
