#include "drone_city_nav/execution_route_snapshot_3d.hpp"
#include "drone_city_nav/route_execution_manager_3d.hpp"

bool executionRouteCompatibilityHeadersCompile() {
  return drone_city_nav::ExecutionPlan3D{}.routeGenerationHighWater() == 0U;
}
